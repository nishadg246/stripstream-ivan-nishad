from time import time
from stripstream.pddl.logic.connectives import And
from stripstream.pddl.logic.predicates import NamedPredicate
from stripstream.pddl.operators import STRIPSAction
from stripstream.utils import irange, INF
from stripstream.algorithms.universe import Universe
from stripstream.algorithms.focused.utils import Concrete, print_status
from stripstream.algorithms.utils import DEFAULT_SEARCH
from stripstream.pddl.utils import convert_plan
from stripstream.algorithms.plan import plan_cost
from stripstream.algorithms.focused.simple_focused import focused_debug, solve_real, \
  is_blocked, make_streams, initialize_universe, solve, is_real_solution, abstract_focused_debug, produce_bindings

# NOTE - this is advantageous over simple_focused when requiring bias costs to be used correctly

# TODO - merge plan_focused and simple_focused to automatically apply cheap streams
COST_THRESHOLD = INF # TODO - use this to decide whether to explicitly plan or call upfront

# TODO - python implementation of lazy_planner that applies all cond_streams at once
# TODO - experiment with no intersection to enforce that an abstract object is used only once (but support several abstract objects)
# TODO - allow intersection of generators

# TODO - revisit the claiming abstract objects version
# Although that one appears to be more compact, it likely creates more operator instances

##################################################

def add_new_action(universe, action):
  #universe.add_action(action) # TODO - remove the static checks here
  assert action.name not in universe.name_to_action
  universe.name_to_action[action.name] = action
  for atom in (action.condition.get_atoms() | action.effect.get_atoms()): # Should only have to add cost and static predicate
    if atom.is_function():
      universe.add_func_atom(atom)
    else:
      universe.add_atom(atom)

# NOTE - can make automatic streams axioms
#def make_stream_axiom(cs):
#  return STRIPSAxiom(cs.conditions + map(Concrete, cs.inputs),
#                    cs.outputs + map(Concrete, cs.outputs)) # NOTE - mixes use of predicates between static and dynamic

def make_stream_action(cs, stream_cost):
  name = '_stream_%s'%cs.n
  parameters = list(cs.inputs) + list(cs.outputs)
  StreamPredicate = NamedPredicate(name, map(lambda p: p.type, parameters)) # NOTE - only do this for actions with effects?
  conditions = cs.conditions + [StreamPredicate(*parameters)] + map(Concrete, cs.inputs)
  effects = cs.effects + map(Concrete, cs.outputs)
  cost = stream_cost*cs.cost
  action = STRIPSAction(name, parameters, conditions, effects, cost) # NOTE - there will be a predicate instance for every stream action
  action.internal = True
  action.predicate = StreamPredicate
  action.cond_stream = cs
  cs.action = action
  return action

##################################################

def solve_plan(universe, search, max_time):
  temp_initial_atoms = universe.initial_atoms.copy() # TODO - do this differently?
  universe.initial_atoms = set(universe.real_initial_atoms)
  for stream in universe.new_streams:
    if stream.include and not is_blocked(stream, universe): # NOTE - alternatively I could fix this and just add negative versions for blocked actions
      parameters = list(stream.inputs) + list(stream.abs_outputs)
      universe.initial_atoms.add(stream.cond_stream.action.predicate(*parameters))
  universe.initial_atoms.update(map(Concrete, universe.real_objects)) # TODO - adjust costs based on inputs
  plan = solve(universe, search, max_time)
  universe.initial_atoms = temp_initial_atoms # Reset these
  return plan

def extract_streams(universe, stream_plan):
  order = []
  for op, args in stream_plan:
    if op.internal:
      cs = op.cond_stream
      inputs = args[:len(cs.inputs)]
      outputs = args[len(cs.inputs):]
      stream = op.cond_stream(inputs) # TODO - selectively identify targets
      stream.targets = stream.instantiate_effects(outputs) + map(Concrete, outputs)
      order.append(stream)
  if universe.verbose:
    print 'Order:', order
  return order

##################################################

def plan_focused(problem, search=DEFAULT_SEARCH, max_time=INF, max_iterations=INF,
                   optimal=False, stream_cost=10, check_feasible=False, max_level=0,
                   greedy=True, shared=True, dfs=True,
                   verbose=False, debug=False):

  universe = Universe(problem, use_ground=False, make_stream_instances=True, make_action_instances=False)
  universe.add_predicate(Concrete)
  for action in universe.actions:
    action.internal = False
    action.condition = And(action.condition, *[Concrete(param) for param in action.parameters])
  for cs in universe.problem.cond_streams: # TODO - I can either make one per type or one per stream
    if cs.plannable:
      add_new_action(universe, make_stream_action(cs, stream_cost))
  initialize_universe(universe)

  ####################

  for _ in irange(0, max_iterations):
    if verbose: print
    print_status(universe)
    if time() - universe.start_time >= max_time: break
    universe.iterations += 1
    if debug: focused_debug(universe)

    if check_feasible: # Checks if the problem is feasible with no parameters
      plan = solve_real(universe, search, max_time)
      if plan is not None:
        universe.real_initial_atoms.update(map(Concrete, universe.real_objects)) # TODO - avoid this
        assert is_real_solution(universe, plan)
        return plan, universe

    make_streams(universe, max_level)
    if debug: abstract_focused_debug(universe)

    ####################

    stream_plan = solve_plan(universe, search, max_time)
    if verbose:
      print 'Cost:', plan_cost(universe, stream_plan), 'Plan:', convert_plan(stream_plan) # TODO - use an upper bound and continue?
    if stream_plan is None:
      if not universe.temp_blocked:
        break
      universe.temp_blocked = set()
      universe.resets += 1
      continue

    ####################

    order = extract_streams(universe, stream_plan)
    plan = filter(lambda (op, args): not op.internal, stream_plan)
    bound_plan = produce_bindings(universe, order, plan, greedy, shared, dfs)
    if bound_plan is not None:
      universe.real_initial_atoms.update(map(Concrete, universe.real_objects)) # TODO - avoid this
      assert is_real_solution(universe, bound_plan)
      return bound_plan, universe
  return None, universe