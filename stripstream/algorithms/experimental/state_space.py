from collections import deque
from time import time

from stripstream.algorithms.experimental.prune_streams import useful_at_state, get_streams, state_fluent_bindings

from stripstream.utils import first
from stripstream.priority_queue import FILOPriorityQueue
from stripstream.algorithms.universe import Universe
from stripstream.utils import INF
from stripstream.algorithms.plan import plan_cost, print_plan_stats
from stripstream.algorithms.incremental.incremental_planner import call_stream
from stripstream.pddl.operators import Action, STRIPSAction
from stripstream.algorithms.experimental.simultaneous import get_plan, get_vertex_keys, get_operator_key
from operator import itemgetter
from collections import defaultdict
from stripstream.pddl.logic.atoms import Atom
from stripstream.pddl.logic.connectives import Not

class Vertex(object):
  def __init__(self, fluents, universe):
    self.fluents = frozenset(fluents)
    #self.derived = set() # TODO - support axioms
    #self.axioms = set()
    self.edges = {}
    self.parent = None
    self.cost = 0
    self.distance = 0
    self.pops = 0
    self.op_index = 0
    self.stream_index = 0

    # TODO - map subsets of cs.streams to the streams
    self.streams = deque()
    self.cond_streams = dict()
    useful_atoms, useful_types = useful_at_state(self.fluents, universe)
    for cs, bindings in get_streams(useful_atoms, useful_types, universe):
      bind_map = dict(bindings)
      if set(cs.inputs) == set(bind_map.keys()):
        stream = cs(tuple(bind_map[param] for param in cs.inputs))
        if not stream.enumerated:
          self.streams.append(stream)
      else:
        if cs not in self.cond_streams:
          self.cond_streams[cs] = []
        self.cond_streams[cs].append(bind_map)
    # TODO - removed streams in order to implement lazy strategies

    self.action_hashes = defaultdict(list)
    for action in universe.actions:
      self.action_hashes[action] = []
      for key in get_vertex_keys(self, action, universe):
        self.action_hashes[action].append(hash(key))

    #self.action_bindings = {}
    #for action in universe.actions: # TODO - share with useful_at_state
    #  self.action_bindings[action] = list(state_fluent_bindings(self.fluents, action, universe))

  #def atoms(self, stream_atoms):
  #  #return self.fluents | self.derived | stream_atoms
  #  return self.fluents | stream_atoms
  def relax(self, op, pv): # TODO - dynamically update cost
    self.cost = pv.cost + (op.cost if op.cost is not None else 0)
    self.distance = pv.distance + 1
    self.parent = (op, pv)
  def is_useful(self, stream):
    # TODO - could also just store useful_atoms, useful_types and implicitly check
    if stream.cond_stream in self.cond_streams:
      for bind_map in self.cond_streams[stream.cond_stream]:
        param_map = dict(zip(stream.cond_stream.inputs, stream.inputs))
        if all(bind_map[p] == param_map[p] for p in bind_map):
          return True
    return False
  def __eq__(self, other):
    return type(self) == type(other) and self.fluents == other.fluents
  def __ne__(self, other):
    return not self == other
  def __hash__(self):
    return hash((self.__class__, self.fluents))
  def __repr__(self):
    return self.__class__.__name__ + repr(tuple(self.fluents))
  __str__ = __repr__

##################################################

# NOTE - do we only want to use the generators that achieve specific chosen values or do we allow them to be anything
# TODO - maybe I could think of this as the relaxed planning on each state for real
# TODO - only makes samples for actions on a relaxed plan or something

# TODO - I could thing of this as a relaxed plan for achieving some of the things on the plan?
# TODO - I can use this method to prune things that can't possibly achieve goals

def satisfies_goal(v, universe):
  assert isinstance(universe.goal_formula, Atom)
  return universe.goal_formula in v.fluents
  #return universe.goal_formula.holds(v.atoms(universe.initial_stream_atoms()), universe.type_to_objects)

def applicable_action(action, v, universe):
  assert isinstance(action.lifted, STRIPSAction)
  for literal in action.condition.formulas:
    if isinstance(literal, Atom):
      atom = literal
      if not (atom in v.fluents or atom in universe.stream_atoms):
        return False
    elif isinstance(literal, Not):
      atom = literal.formula
      if atom in v.fluents or atom in universe.stream_atoms:
        return False
    else:
      raise ValueError(literal)
  return True
  #return all(atom in v.fluents or atom in universe.stream_atoms for atom in action.pos_conditions()) and \
  #       not any(atom in v.fluents or atom in universe.stream_atoms for atom in action.neg_conditions())
  #state = v.atoms(universe.initial_stream_atoms()) # Can compute this once per state
  #return action.is_applicable(state, universe.type_to_objects)

def apply_action(action, v, universe):
  #assert isinstance(action.lifted, STRIPSAction)
  #add = set(filter(lambda a: isinstance(a, Atom), action.effect.formulas))
  #delete = set(map(lambda a: a.formula, filter(lambda a: isinstance(a, Not), action.effect.formulas)))
  #return (v.fluents | (add-delete)) - (delete-add) # TODO - single action condition/effect?
  return action.apply(v.fluents, universe.type_to_objects)

def return_plan(v, universe):
  plan = []
  for action, args in get_plan(v):
    if not hasattr(action.original, 'is_internal'):
      plan.append((action.original, args[:len(action.original.parameters)]))
  universe.print_statistics()
  if plan is not None and universe.verbose:
    print_plan_stats(plan, universe)
  return plan, universe

# TODO - only update states in which the actions are applicable

def get_actions(vertex, action_map):
  for action in vertex.action_hashes:
    for key in vertex.action_hashes[action]:
      for instance in action_map[key]:
        yield instance

def get_new_actions(vertex, universe):
  #print len(universe.new_instances) - vertex.op_index # NOTE - this grows linearly
  op_index = vertex.op_index
  vertex.op_index = len(universe.new_instances)
  for i in range(op_index, len(universe.new_instances)):
    yield universe.new_instances[i]

##################################################

def progression(problem, max_time=INF, max_iterations=INF, verbose=True,
                optimal=False, cost_fn=plan_cost, allow_intersect=False,
                priority=lambda _: 0, recurrence=1, useful_actions=True, frequency=10):
  problem.replace_axioms()
  problem.convert_goal(axiom=False) # Otherwise need to include the goal in useful_at_state
  problem.to_strips()
  # TODO - do STRIPS version of this with a quick lookup

  universe = Universe(problem, use_ground=False, make_action_instances=True, verbose=verbose)
  root = Vertex(universe.initial_fluents(), universe)
  if satisfies_goal(root, universe):
    return return_plan(root, universe)

  vertices = {root}
  queue = deque([root])
  last_print = time()
  action_map = defaultdict(list)
  while queue and time() - universe.start_time < max_time and universe.iterations < max_iterations:
    cv = queue.popleft(); cv.pops += 1
    if recurrence is not None and (cv.pops - 1) % recurrence != 0: queue.append(cv); continue
    if time() - last_print >= 1:
      last_print = time()
      print 'Iteration: %d | Time: %s | Calls: %s | Search Time: %s | States: %s | Queue: %s | ' \
            'Actions: %s | Streams: %s'%(universe.iterations, round(time() - universe.start_time, 3),
                                         universe.calls, round(universe.search_time, 3),
                                         len(vertices), len(queue), len(universe.new_instances),
                                         len(universe.new_streams))
    universe.iterations += 1
    # TODO - could also not call streams but expand any new edges (fractional recurrence?)
    # TODO - make a partially instantiated cond-stream and update instances of them

    action_index = len(universe.new_instances)
    if cv.cond_streams:
      for i in range(cv.stream_index, len(universe.new_streams)):
        stream = universe.new_streams[i]
        if useful_actions and cv.is_useful(stream): # TODO - why is this slower when I add "not stream.enumerated"
          cv.streams.append(stream)
      cv.stream_index = len(universe.new_streams)
    for _ in range(min(len(cv.streams), frequency)):
      stream = cv.streams.popleft()
      if not stream.enumerated:
        call_stream(universe, stream) # Only processed if there exists a successful call?
        if not stream.enumerated:
          cv.streams.append(stream)

    for i in range(action_index, len(universe.new_instances)):
      instance = universe.new_instances[i]
      key = hash(get_operator_key(instance, universe))
      action_map[key].append(instance)

    t0 = time()
    #for action in get_new_actions(cv, universe):
    for action in get_actions(cv, action_map):
      assert isinstance(action.lifted, Action)
      if action not in cv.edges and applicable_action(action, cv, universe):
        nv = Vertex(apply_action(action, cv, universe), universe)
        if nv not in vertices:
          nv.relax(action, cv)
          vertices.add(nv)
          if satisfies_goal(nv, universe):
            return return_plan(nv, universe)
          queue.append(nv)
        cv.edges[action] = nv
    if recurrence is not None and cv.streams:
      queue.append(cv)
    universe.search_time += time() - t0
  return None, universe

##################################################

# TODO - find a way to integrate this with my existing search infrastructure
# TODO - needs to be a recurrent search

def greedy_search(problem, max_time=INF, max_iterations=INF, verbose=True,
                optimal=False, cost_fn=plan_cost, allow_intersect=False, priority=lambda _: 0):
  raise NotImplementedError()
  """
  universe = Universe(problem, use_ground=False, make_action_instances=True)
  print problem.cond_streams
  root = Vertex(universe.initial_fluents(), universe)
  vertices = {root}

  if universe.goal_formula.holds(root.atoms(universe.initial_stream_atoms()), universe.type_to_objects):
    return get_plan(root), universe
  queue = FILOPriorityQueue([(priority(root), root)])
  while not queue.empty() and time() - universe.start_time < max_time and universe.iterations < max_iterations:
    cv = queue.pop()
    universe.iterations += 1

    successors = list(cv.unexplored()) + [cv]
    gv = first(lambda v: v.contained(goal), successors[:-1])
    if gv is not None: return get_plan(gv), universe
    for v in successors:
      if v.generate():
        queue.push(priority(v), v)
  return None, universe
  """

##################################################

def regression():
  # Only call streams that could lead to valid predecessor actions
  # Use partial bindings of objects
  raise NotImplementedError()

##################################################

def hbf():
  # Call streams until relaxed plan feasible (as long as below a time limit)
  # Lazy heuristic evaluation
  # Can either either do the tree
  # Or lazy re-planning for the heuristic
  raise NotImplementedError()

def lazy_hbf():
  # Lazy re-planning for the heuristic
  raise NotImplementedError()

##################################################

# TODO - include shortcut action in metric spaces that is automatically able to move to a goal
# TODO - heuristics and search that ignore static predicates
# NOTE - this is like the semantic attachments strategy

def blind_heuristic(*args):
  return 0

def goal_cost_heuristic():
  raise NotImplementedError()

def goal_dist_heuristic():
  # Uses metrics defined on object types
  raise NotImplementedError()

# TODO - use a heuristic that removes static predicates and keeps the discrete elements with abstract constants

def optimistic_heuristic():
  # Relaxes stream conditions and solves the problem optimally
  raise NotImplementedError()

def optimistic_ff_heuristic():
  # Relaxes stream conditions and performs the FF heuristic
  raise NotImplementedError()
