from heapq import heappush, heappop
from collections import namedtuple, defaultdict

from stripstream.algorithms.focused.utils import AbstractConstant, Concrete
from stripstream.utils import INF, argmin

AtomN = namedtuple('AtomN', ['c', 'l', 'op'])
StreamN = namedtuple('StreamN', ['c', 'l']) # TODO - maybe put the condition stuff here?

# TODO - set additive heuristic
# TODO - plan using separate objects for each input. Could iteratively increase the size of this. Need many objects then

def compute_costs(state, operators, fn=sum):
  cond_to_op = defaultdict(list) # TODO - store this in memory
  for op in operators:
    op._conditions = op.conditions + [None]
    op._remaining = len(op._conditions)
    for atom in op._conditions:
      cond_to_op[atom].append(op)

  a_costs = {lit: AtomN(0, 0, None) for lit in state | {None}}
  s_costs = {}
  queue = [(pair.c, atom) for atom, pair in a_costs.iteritems()]
  processed = set()
  while len(queue) != 0:
    _, atom = heappop(queue)
    if atom in processed: continue
    processed.add(atom)
    for op in cond_to_op[atom]:
      op._remaining -= 1
      if op._remaining == 0:
        s_costs[op] = StreamN(fn(a_costs[lit].c for lit in op._conditions),
                           max(a_costs[lit].l for lit in op._conditions))
        l_cost, l_level = s_costs[op].c + op.cost, s_costs[op].l + 1
        for effect in op.effects:
          if effect not in processed and l_cost < a_costs.get(effect, INF):
            a_costs[effect] = AtomN(l_cost, l_level, op)
            heappush(queue, (l_cost, effect))
  return a_costs, s_costs

##################################################

def linearize_backpointers(plan):
  # TODO - just do this (but produce a plan)
  pass

def get_layers(o_costs):
  num_layers = max(pair.l for pair in o_costs.values()) + 1
  layers = [[] for _ in range(num_layers)]
  for value, (_, level) in o_costs.iteritems():
    layers[level].append(value)
  return layers

# TODO - discounted cost if it uses an already achieved subgoal
# NOTE - the problem I am having is that the current formulation might choose a different block when sampling a traj
# NOTE - want to linearize in DFS order unless otherwise noted

# TODO - make a DAG then topologically sort it
# TODO - bias sorting to put some things (such as expensive base motion plans) towards the end of the plan
# - This way the greedy algorithm will only produce them if everything else is solid

# TODO - maybe I just greedily choose the operator which achieve the most goals (only considering goals in which it is equal to or below)
# TODO - just choose one supporter per condition and then automatically quit (so I don't even attempt to resample something)

def cost_heuristic(o, o_costs, *args):
  return o_costs[o].c

def discounted_cost_heuristic(o, o_costs, subgoals, marked, level):
  return (o_costs[o].c, -sum(g not in marked and g in o.effects for g in subgoals[level]))

##################################################

# TODO - ground using different selectors
# TODO - sort by place along operator in order to bias

def extract_plan(goals, a_costs, s_costs):
  operator_layers = get_layers(s_costs)
  num_goal_layers = max(a_costs[atom].l for atom in goals) + 1
  subgoals = [set() for _ in range(num_goal_layers)]
  relaxed_plan = [[] for _ in range(num_goal_layers - 1)]
  for lit in goals:
    subgoals[a_costs[lit].l].add(lit)
  achievers = {}

  for level in reversed(range(1, num_goal_layers)):
    #for lit in subgoals[level]:
    marked = set()
    for lit in sorted(subgoals[level], key=lambda g: g.predicate == Concrete): # Moves the ground preconditions to the end
      if lit in marked:
        continue
      heuristic = lambda o: discounted_cost_heuristic(o, s_costs, subgoals, marked, level) # cost_heuristic | discounted_cost_heuristic
      easiest_operator = argmin(heuristic,
          (o for o in operator_layers[level-1] if lit in o.effects))
      #easiest_operator = argmin(heuristic,
      #    (o for o, p in operator_costs.iteritems() if p.level < level and literal in o.effects)) # Only useful if discounted_cost_heuristic
      relaxed_plan[level-1].append(easiest_operator)
      for condition in easiest_operator.conditions:
        if not (a_costs[condition].l == level-1 and condition in marked): # Problem was that it was undoing future goals
          subgoals[a_costs[condition].l].add(condition)
      for effect in easiest_operator.effects:
        marked.add(effect) # As a result of the linearization of each level in a forward order
        achievers[effect] = easiest_operator # Earlier achievers will overwrite later ones
  return relaxed_plan, achievers

##################################################

# TODO - I could also greedily prune any unnecessary operators out of the relaxed plan
# TODO - greedy algorithm that removes things in order of time cost
def prune_unnecessary(plan, subgoals):
  pass

def print_plan(plan, subgoals):
  for i in range(len(plan)):
    print 'Level', i
    print subgoals[i]
    print plan[i]
    print

##################################################

# Outputs can be any existing variable
# Single variable for each type
# Each generator has it's own variables
# NOTE - I should definitely (at least implicitly) add the concrete effect

# TODO - create a separate action and object for the output of each block; i.e. can include discrete variables here
# TODO - create a "stream instance" for each action
# TODO - can make multiple copies of objects that each just satisfy one thing (i.e. multiple poses and grasps)

class StreamInstance(object):
  def __init__(self, lifted, args):
    self.lifted = lifted
    self.args = tuple(args)
    cs = lifted.cond_stream
    self.inp_args = self.args[:len(cs.inputs)]
    self.out_args = self.args[len(cs.inputs):len(cs.inputs)+len(cs.outputs)]
    param_map = dict(zip(self.lifted.parameters, args))
    self.conditions = [condition.instantiate(param_map) for condition in lifted.conditions]
    self.effects = [effect.instantiate(param_map) for effect in lifted.effects]
    self.cost = lifted.cost if lifted.cost is not None else 0
  @property
  def name(self):
    return self.lifted.name
  @property
  def out_consts(self):
    return self.lifted.out_consts
  def __eq__(self, other): # TODO - get rid of this
    return type(self) == type(other) and self.lifted == other.lifted and self.args == other.args
  def __ne__(self, other):
    return not(self == other)
  def __hash__(self):
    return hash((self.__class__, self.lifted, self.args))
  def __repr__(self):
    return self.name + '(' + ','.join(repr(arg) for arg in self.args) + ')'

#class StreamAction(STRIPSAction): # NOTE - not using this class in order to assert STRIPS preconditions and effects
class StreamAction(object):
  template = '_ab_%s_%s'
  def __init__(self, cs, unit=False):
    self.cond_stream = cs
    cs.action = self
    self.parameters = cs.inputs
    bound_parameters = set(p for a in cs.conditions for p in a.args)
    self.conditions = cs.conditions + [Concrete(param) for param in cs.inputs if param not in bound_parameters]
    name = cs.n # cs.n | cs.name
    self.out_consts = [AbstractConstant(self.template%(name, out.name[1:]), out.type) for out in cs.outputs]
    param_map = dict(zip(cs.outputs, self.out_consts))
    self.effects = [effect.instantiate(param_map) for effect in cs.effects] + \
                   [Concrete(param_map[param]) for param in cs.outputs]
    self.cost = cs.cost if not unit else 1
  def instantiate(self, args):
    return StreamInstance(self, args)
  @property
  def name(self):
    return self.cond_stream.name
  def __repr__(self):
    return self.name + '(' + ','.join(repr(param) for param in self.parameters) + ')'
  def __eq__(self, other): # TODO - get rid of this
    return type(self) == type(other) and self.cond_stream == other.cond_stream
  def __ne__(self, other):
    return not(self == other)
  def __hash__(self):
    return hash((self.__class__, self.cond_stream))