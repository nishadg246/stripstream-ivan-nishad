from time import time
from collections import deque
from collections import defaultdict, namedtuple
from heapq import heappush, heappop

from stripstream.pddl.logic.atoms import Atom
from stripstream.pddl.operators import ActionInstance
from stripstream.utils import irange, INF, set_union
from stripstream.algorithms.instantiation import parameter_product2
from stripstream.algorithms.universe import Universe
from stripstream.algorithms.focused.utils import AbstractConstant, make_abstract_constant, Concrete, is_concrete, \
    constant_from_concrete, has_abstract_inputs, replace_abstract_constants, partition_values, get_target_atoms, \
    get_stream_functions, add_stream_cost, print_status
from stripstream.algorithms.incremental.incremental_planner import DEFAULT_SEARCH, call_stream
from stripstream.pddl.utils import get_value, convert_plan
from stripstream.algorithms.plan import plan_cost, substitute_axioms
from stripstream.pddl.objects import Object
from stripstream.priority_queue import PriorityQueue

def focused_debug(universe):
  for type in universe.type_to_objects:
    print type, len(universe.type_to_objects[type]), \
      map(get_value, filter(lambda o: not isinstance(o, AbstractConstant), universe.type_to_objects[type]))[:10]
  raw_input('Continue?\n')

def abstract_focused_debug(universe):
  print 'Streams:', len(universe.new_streams)
  for type, constants in universe.type_to_objects.iteritems():
    print type, len(constants), constants
  raw_input('Continue?\n')

def is_blocked(stream, universe):
  return stream in universe.perm_blocked or stream in universe.temp_blocked

def block_stream(stream, universe):
  if stream.enumerated:
    universe.perm_blocked.add(stream)
    universe.enumerated += 1
  else:
    universe.temp_blocked.add(stream)

##################################################

def is_real_stream(stream, universe):
  return all(inp in universe.real_objects for inp in stream.inputs) and \
         all(con in universe.real_initial_atoms for con in stream.conditions)

def call_real_stream(stream, universe, **kwargs):
  assert is_real_stream(stream, universe)
  if stream.enumerated:
    #print stream
    #print stream.history.keys()
    return [] # NOTE - happens when bound arguments is an existing stream
  values = call_stream(universe, stream, **kwargs)
  if stream.enumerated:
    universe.enumerated += 1
  for value in values:
    if isinstance(value, Atom) and value not in values:
      values += value.args
  for value in values:
    if isinstance(value, Atom):
      universe.real_initial_atoms.add(value)
      assert not any(isinstance(arg, AbstractConstant) for arg in value.args)
      universe.real_objects.update(value.args)
    elif isinstance(value, Object):
      assert not isinstance(value, AbstractConstant)
      universe.real_objects.add(value)
  block_stream(stream, universe)
  return values

# TODO - maybe do this until you reach a cycle
# NOTE - don't need to worry about shared interaction when sampled because will have separate inputs
def call_abstract_stream(universe, stream, max_level): # max_level in [-1, 0, 1, ..., INF]
  if stream.cond_stream.max_level is not None:
    max_level = stream.cond_stream.max_level
  stream.include = False
  if stream.cond_stream.eager and not has_abstract_inputs(stream):
    if not stream.enumerated: # NOTE - can happen when the stream is called in the bindings
      call_real_stream(stream, universe)
    assert stream.enumerated
    return
  if not stream.cond_stream.plannable or (stream.cond_stream.cost == INF):
    return
  stream.include = True
  level = max([0] + [inp.level if isinstance(inp, AbstractConstant) else 0 for inp in stream.inputs]) # TODO - add 1?
  if level <= max_level:
    outputs = [make_abstract_constant(out.type) for out in stream.cond_stream.outputs]
    for out in outputs:
      out.level = level + 1
      out.stream = stream
    stream.shared = False
  else:
    outputs = stream.cond_stream.abs_outputs
    stream.shared = True
  stream.level = level
  stream.abs_outputs = outputs
  stream.abs_effects = stream.cond_stream.instantiate_effects(stream.inputs, outputs)
  for out in outputs:
    universe.add_object(out)
  for atom in stream.abs_effects:
    if isinstance(atom, Atom):
      universe.add_initial_atom(atom)
    #elif isinstance(value, Initialize):
    #  universe.add_perm_initialize(value)
    else:
      raise ValueError(atom)

##################################################

def add_streams(universe):
  for i in range(universe.queue_index, len(universe.new_streams)):
    universe.queue.append(universe.new_streams[i])
  universe.queue_index = len(universe.new_streams)

def make_streams(universe, max_level, max_time=INF, frequency=INF):
  # TODO - can add static objects to the universe and maintain them or can wait
  add_streams(universe)
  local_calls = 0
  universe.new_problem = False
  while universe.queue and (not universe.new_problem or (local_calls < frequency)): # TODO - can do local_calls here as well
    if max_time < (time() - universe.start_time):
      return False
    stream = universe.queue.popleft()
    assert all(con in universe.initial_atoms for con in stream.instantiate_conditions())
    call_abstract_stream(universe, stream, max_level)
    add_streams(universe) # TODO - I could also call once all promised
    if not stream.cond_stream.eager or not stream.called:
      local_calls += 1

##################################################

# TODO - streams currently don't have hash functions
#from stream_search import compute_costs

AtomN = namedtuple('AtomN', ['cost', 'level', 'stream'])
StreamN = namedtuple('StreamN', ['cost', 'level'])

def determine_reachable(universe, op=sum, unit=False):
  state = universe.real_initial_atoms
  state.update(map(Concrete, universe.real_objects))
  state.add(None)
  unprocessed = defaultdict(list) # TODO - store this in memory
  for stream in universe.new_streams:
    if stream.include and not is_blocked(stream, universe):
      stream._conditions = stream.conditions + [Concrete(inp) for inp in stream.inputs] + [None]
      stream._effects = stream.abs_effects + [Concrete(out) for out in stream.abs_outputs]
      stream._remaining = len(stream._conditions)
      for atom in stream._conditions:
        unprocessed[atom].append(stream)

  atom_nodes = {literal: AtomN(0, 0, None) for literal in state}
  queue = [(pair.cost, atom) for atom, pair in atom_nodes.iteritems()] # TODO - do deque version of this
  while queue:
    _, atom = heappop(queue)
    if atom not in unprocessed:
      continue
    for stream in unprocessed[atom]:
      stream._remaining -= 1
      if stream._remaining == 0:
        stream._cost = op(atom_nodes[con].cost for con in stream._conditions) + (stream.cond_stream.cost if not unit else 1)
        stream._level = max(atom_nodes[con].level for con in stream._conditions) + 1
        for effect in stream._effects:
          if effect not in atom_nodes or stream._cost < atom_nodes[effect].cost:
            atom_nodes[effect] = AtomN(stream._cost, stream._level, stream)
            heappush(queue, (stream._cost, effect))
    del unprocessed[atom]
  del atom_nodes[None]
  return atom_nodes

##################################################

# NOTE - could think about greedily minimizing the cost if atoms have multiple effects but most tests have one effect
def extract_streams(atom_nodes, goals):
  new_goals = goals.copy() # TODO - maybe I should just add the concrete preconditions?
  streams = set()
  orders = set() # TODO - could make a None node that is a precondition for everything
  queue = deque(new_goals)
  while queue:
    goal = queue.popleft()
    stream = atom_nodes[goal].stream
    if stream is not None:
      if stream not in streams:
        stream.targets = set()
        streams.add(stream)
        for atom in stream._conditions:
          if atom is not None:
            if atom_nodes[atom].stream is not None:
              orders.add((atom_nodes[atom].stream, stream))
            if atom not in new_goals:
              new_goals.add(atom)
              queue.append(atom)
      stream.targets.add(goal)
  #visualize_order(streams, orders)
  return topological_sort(streams, orders)

##################################################

# NOTE - difference between evaluation cost and likelihood of success

#from stripstream.algorithms.focused.bind_abstract import get_ordering

# TODO - option to stop on a certain level

def topological_sort(streams, edges):
  incoming_edges = defaultdict(set)
  outgoing_edges = defaultdict(set)
  for start, end in edges:
    incoming_edges[end].add(start)
    outgoing_edges[start].add(end)
  #priority_fn = lambda stream: 0 # TODO - use cost or level here?
  #priority_fn = lambda stream: stream.cost
  priority_fn = lambda stream: stream.cond_stream.order
  #priority_fn = lambda stream: stream.level # TODO - can also do distance in graph
  #priority_fn = lambda stream: stream._cost
  #priority_fn = lambda stream: stream._level
  #priority_fn = lambda stream: stream.cond_stream.prob # Given a fixed plan, schedule most risky first. Penalty for failure is all prior.
  # But might still want to put cheapest first in case a failure? What if high cost and high chance of failure along with low cost and medium chance?
  # TODO - dynamic programming version of this that avoids trying all orderings?
  # TODO - sort by first use along the plan

  ordering = []
  queue = PriorityQueue()
  for stream in streams:
    if not incoming_edges[stream]:
      queue.push(priority_fn(stream), stream)
  while not queue.empty():
    stream1 = queue.pop()
    ordering.append(stream1)
    for stream2 in outgoing_edges[stream1]:
      incoming_edges[stream2].remove(stream1)
      if not incoming_edges[stream2]:
        queue.push(priority_fn(stream2), stream2)
  return ordering

##################################################

# TODO - pass current bindings as well
# NOTE - could also just make these things temporary attributes of universe anyways

def bind_stream_outputs(stream, inputs, universe, history=True, **kwargs): # TODO - consider the history here?
  if inputs is None:
    return
  bound_stream = stream.cond_stream(inputs)
  if not is_real_stream(bound_stream, universe):
    return
  values = call_real_stream(bound_stream, universe, **kwargs)
  bound_map = dict(zip(stream.inputs, inputs))
  targets = [effect.instantiate(bound_map) for effect in stream.targets]
  target_atoms, target_objects = partition_values(targets)
  target_parameters = filter(lambda o: isinstance(o, AbstractConstant), target_objects)
  produced_atoms, produced_objects = partition_values(values)
  if not target_parameters:
    return iter([{}]) if target_atoms <= produced_atoms else iter([])
  if universe.verbose:
    print
    print bound_stream
    print values
    print target_atoms, target_objects
    print target_parameters
    print produced_atoms, produced_objects

  return parameter_product2(target_parameters, produced_objects, target_atoms, produced_atoms)

##################################################

# TODO - how many times should you call shared per iteration?
# TODO - should I generate values if using an abstract constant achievable by several things?

def first_bindings(order, universe, shared=True):
  if not order: return {}
  for stream in order:
    if (not stream.shared or shared) and not has_abstract_inputs(stream):
      _ = call_real_stream(stream, universe)
      return None # TODO - return binding if appropriate
  raise ValueError(order)

def first_level_bindings(order, universe, greedy=True, shared=True):
  if not order: return {}
  called = 0
  for stream in order:
    if (not stream.shared or shared) and not has_abstract_inputs(stream):
      _ = call_real_stream(stream, universe)
      called += 1
      # TODO - fail if values doesn't contain the desired thing
      # NOTE - only want to break if doesn't produce NEEDED outputs/effects (not all of them)
  assert called
  return None # TODO - return binding if appropriate

# TODO - try the separation of objects

def single_bindings(order, universe, greedy=True, shared=True):
  failure = False
  bindings = {}
  for stream in order:
    local_failure = True
    if shared or not stream.shared:
      inputs = replace_abstract_constants(stream.inputs, bindings)
      outputs = next(bind_stream_outputs(stream, inputs, universe), None)
      if outputs is not None: # NOTE - by checking stream validity before applying, we allow for previously bound things
        local_failure = False
        for abs_const, real_const in outputs.iteritems():
          assert stream.shared or abs_const not in bindings
          if bindings.get(abs_const, real_const) == real_const:
            bindings[abs_const] = real_const
          else:
            local_failure = True
    failure |= local_failure
    if greedy and failure:
      break
  return bindings if not failure else None

# Bias choice to handle all unachieved predicates with involve outputs

# TODO - what happens if we have multiple checks, we can avoid calling them if already achieved
# TODO - just do this for goals, or
# TODO - do this for things down the road
# TODO - pass in the binding of outputs to abstract

def get_dependent_streams(stream, order, bindings, allow_abstract=False):
  copied_bindings = bindings.copy()
  for abs_out, param_out in zip(stream.abs_outputs, stream.cond_stream.outputs):
    copied_bindings[abs_out] = param_out
  related = set()
  for other in order:
    if stream != other:
      intersect = set(stream.abs_outputs) & set(other.inputs)
      if intersect:
        bound_other = other.cond_stream(map(lambda const: copied_bindings.get(const, const), other.inputs))
        if allow_abstract or all(not isinstance(inp, AbstractConstant) for inp in set(bound_other.inputs)-intersect):
          related.add(bound_other)
  # TODO - if allow_abstract then do this recursively
  return related

def bind_atom(atom, bindings):
  return atom.predicate(*map(lambda const: bindings.get(const, const), atom.args))

# NOTE - could even pass in targets
def get_dependent_goals(stream, order, bindings, allow_abstract=False):
  if not stream.abs_outputs:
    bound_targets = {bind_atom(goal, bindings) for goal in stream.targets}
    return bound_targets
    #return set(filter(lambda a: a not in universe.real_initial_atoms))
  goals = set_union(other.targets for other in order) # NOTE - could also just pass in goals
  copied_bindings = bindings.copy()
  for abs_out, param_out in zip(stream.abs_outputs, stream.cond_stream.outputs):
    copied_bindings[abs_out] = param_out
  related = set()
  for goal in goals:
    if not is_concrete(goal):
      intersect = set(stream.abs_outputs) & set(goal.args)
      if intersect:
        bound_goal = bind_atom(goal, copied_bindings)
        if allow_abstract or all(not isinstance(inp, AbstractConstant) for inp in set(bound_goal.args)-intersect):
          related.add(bound_goal)
  # TODO - if allow_abstract then do this recursively with any abstract constants present
  return related

def dfs_bindings(order, universe, bindings={}, greedy=True, shared=True):
  if not order:
    return bindings
  stream = order[0]
  if shared or not stream.shared:
    inputs = replace_abstract_constants(stream.inputs, bindings)
    #dependent_goals = get_dependent_streams(stream, order, bindings)
    dependent_atoms = get_dependent_goals(stream, order, bindings)
    outputs_list = list(bind_stream_outputs(stream, inputs, universe, dependent_atoms=dependent_atoms))
    for outputs in outputs_list:
      new_bindings = bindings.copy() # TODO - what if this is the same as bindings?
      local_failure = False
      for abs_const, real_const in outputs.iteritems():
        assert stream.shared or abs_const not in new_bindings
        if new_bindings.get(abs_const, real_const) == real_const:
          new_bindings[abs_const] = real_const
        else:
          local_failure = True
      result_bindings = dfs_bindings(order[1:], universe, bindings=new_bindings, greedy=greedy, shared=shared)
      if not local_failure and result_bindings is not None:
        return result_bindings
    if not outputs_list and not greedy:
      dfs_bindings(order[1:], universe, bindings=bindings, greedy=greedy, shared=shared)
  elif not greedy:
    dfs_bindings(order[1:], universe, bindings=bindings, greedy=greedy, shared=shared)
  return None

##################################################

# TODO - should I check that the path of bindings is all instantiated or just that the things on the plan are
# TODO - checking validity by ensuring all the instantiated goals are in real_initial_atoms now
# This would involve ensuring that all parameters are bound, replacing values, and checking that all predicates are added to the real pool
#for goal in goals:
#  goal.instantiate(...)
#  pass

def is_real_solution(universe, plan):
  type_to_objects = defaultdict(list)
  for obj in universe.real_objects:
    type_to_objects[obj.type].append(obj)
  instances = [action.instantiate(args) for action, args in plan]
  conditions = [instance.condition for instance in instances]
  state = universe.real_initial_atoms
  for condition, instance in zip(conditions, instances) + [(universe.goal_formula, None)]:
    substitute_axioms(condition, state, universe) # TODO - I could also just add an instantiated action to the plan
    if not condition.holds(state, type_to_objects):
      return False
    if instance is not None:
      state = instance.apply(state, type_to_objects)
  return True

def solve(universe, search, max_time):
  t0 = time()
  plan = search(universe, max_time-(time()-universe.start_time), INF)
  universe.search_time += time() - t0
  return plan

def solve_real(universe, search, max_time):
  temp_initial_atoms = universe.initial_atoms.copy() # TODO - do this differently?
  temp_type_to_objects = universe.type_to_objects.copy()
  universe.initial_atoms = universe.real_initial_atoms
  universe.type_to_objects = defaultdict(set)
  for obj in universe.real_objects:
    universe.type_to_objects[obj.type].add(obj)

  plan = solve(universe, search, max_time)
  universe.initial_atoms = temp_initial_atoms # Reset these
  universe.type_to_objects = temp_type_to_objects
  return plan

def solve_abstract(universe, abstract_atoms, search, max_time):
  temp_initial_atoms = universe.initial_atoms.copy() # TODO - do this differently?
  temp_type_to_objects = universe.type_to_objects.copy()

  universe.initial_atoms = {atom for atom in abstract_atoms if atom.predicate is not Concrete}
  universe.type_to_objects = defaultdict(set)
  for atom in abstract_atoms:
    if is_concrete(atom):
      arg = constant_from_concrete(atom)
      universe.type_to_objects[arg.type].add(arg)

  plan = solve(universe, search, max_time)
  goals = None
  if plan is not None:
    static_atoms = universe.initial_atoms - universe.real_initial_atoms
    goals = set_union(get_target_atoms(universe, plan, static_atoms)) # NOTE - discount any achieved along
  universe.initial_atoms = temp_initial_atoms # Reset these
  universe.type_to_objects = temp_type_to_objects
  return plan, goals

##################################################

# TODO - include actions for "grounding" samples for high cost actions
# TODO - achieve goals by the outputs and add the test streams. And add these to the search
# TODO - do the planning for actions version
# NOTE - if I add the action, then I won't be able to use successors? Well I could just produce all successors from the action I guess?

def initialize_universe(universe):
  universe.real_initial_atoms = universe.initial_atoms.copy() # TODO - handle this differently
  universe.real_objects = {obj for ty in universe.type_to_objects for obj in universe.type_to_objects[ty]}
  for cs in universe.problem.cond_streams: # TODO - I can either make one per type or one per stream
    outputs = tuple(make_abstract_constant(out.type, shared=True) for out in cs.outputs)
    level = INF
    for out in outputs:
      out.level = level
      out.stream = None
    cs.level = level
    cs.abs_outputs = outputs # TODO - add outputs
  universe.queue = deque()
  universe.queue_index = 0

def produce_bindings(universe, order, plan, greedy, shared, dfs):
  if dfs:
    bindings = dfs_bindings(order, universe, greedy=greedy, shared=shared)
  else:
    #bindings = first_bindings(order, universe, shared=shared)
    #bindings = first_level_bindings(order, universe, greedy=greedy, shared=shared)
    bindings = single_bindings(order, universe, greedy=greedy, shared=shared)
  if bindings is None:
    return None
  bound_plan = []
  for action, args in plan:
    bound_args = replace_abstract_constants(args, bindings)
    assert bound_args is not None
    bound_plan.append((action, bound_args))
  return bound_plan

def simple_focused(problem, search=DEFAULT_SEARCH, max_time=INF, max_iterations=INF,
                   optimal=False, stream_cost=10, check_feasible=False, max_level=0,
                   greedy=True, shared=True, dfs=True,
                   verbose=False, debug=False):

  universe = Universe(problem, use_ground=False, make_stream_instances=True, make_action_instances=True) # Need to assign a cost to all
  #universe = Universe(problem, use_ground=False, make_stream_instances=True, make_action_instances=not optimal)
  initialize_universe(universe)
  universe.action_to_function = get_stream_functions(universe) if stream_cost is not None else {}

  ####################

  for _ in irange(0, max_iterations):
    if verbose: print
    print_status(universe)
    if (time() - universe.start_time) >= max_time:
      break
    universe.iterations += 1
    if debug:
      focused_debug(universe)

    if check_feasible: # Checks if the problem is feasible with no parameters
      plan = solve_real(universe, search, max_time)
      if plan is not None:
        assert is_real_solution(universe, plan)
        return plan, universe

    make_streams(universe, max_level) # TODO - can also do these in iterations/waves
    if debug:
      abstract_focused_debug(universe)

    ####################

    atom_nodes = determine_reachable(universe)
    for instance in universe.action_instances: # TODO - do I need to reset this ever?
      if isinstance(instance, ActionInstance):
        add_stream_cost(universe, atom_nodes, instance, stream_cost) # NOTE - need to add these before the state resets
    plan, goals = solve_abstract(universe, atom_nodes, search, max_time)
    if verbose:
      print 'Cost:', plan_cost(universe, plan), 'Plan:', convert_plan(plan) # TODO - use an upper bound and continue?
      raw_input('Continue?')
    if plan is None:
      if not universe.temp_blocked:
        break
      universe.temp_blocked = set()
      universe.resets += 1
      continue

    ####################

    constants = set_union(set(args) for _, args in plan)
    abstract_constants = filter(lambda c: isinstance(c, AbstractConstant), constants)
    new_goals = goals.copy()
    new_goals.update(map(Concrete, abstract_constants)) # NOTE - could do this for all supporting goals without distinction
    order = extract_streams(atom_nodes, new_goals)

    #visualize_streams(goals, abstract_constants)
    if verbose:
      print 'Goals:', len(goals)
      print 'Abstract constants:', len(abstract_constants)
      print 'Order:', order

    ####################

    bound_plan = produce_bindings(universe, order, plan, greedy, shared, dfs)
    if verbose: print 'Bound plan:', bound_plan
    if bound_plan is not None:
      assert is_real_solution(universe, bound_plan)
      return bound_plan, universe
  return None, universe
