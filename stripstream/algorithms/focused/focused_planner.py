from time import time

from stripstream.pddl.logic.connectives import And
from stripstream.utils import irange, INF
from stripstream.algorithms.search.pyplanners import strips_planner
from stripstream.algorithms.instantiation import smart_instantiate_operators
from stripstream.algorithms.focused.stream_search import StreamAction, extract_plan
from stripstream.algorithms.focused.stream_instances import get_stream_instances, dynamic_compute_costs, process_eager_streams
from stripstream.algorithms.focused.bind_abstract import dfs_groundings, single_groundings
from stripstream.algorithms.universe import Universe
from stripstream.algorithms.focused.utils import AbstractConstant, Concrete, get_target_atoms, get_stream_functions, \
    add_stream_cost, print_status
from stripstream.algorithms.incremental.incremental_planner import DEFAULT_SEARCH
from stripstream.pddl.utils import get_value, convert_plan
from stripstream.algorithms.plan import plan_cost, feasible_subplan, is_solution, print_plan_stats

DYNAMIC_INSTANCES = True

# TODO
# - Planner that does a search with one abstract object for each type
# - Then it separates when an object is different within each
# - Attempts to find an assignment by then planning for generators
#   - Minimize the number of dual parents or something
#   - This can be done with a CSP or my own planner I believe
#   - This is totally PSPACE-hard because I might need an arbitrarily long sequence of generations to fill it in

# NOTE
# - This is somewhat similar to optimistic planning as it optimistically assumes the same object can be used
# - Would require substantially more replanning for sure
# - Problem of blocking some generators in order to try everything
#   - Can just block things during the matching
#   - If a constraint is no longer achievable, then we block that constraint in the resulting plan

# TODO
# - Planner that applies all the streams on the concrete and abstract variables
# - Solve the problem using the static predicates created
# - Later backtracks to find an assignment that actually works
# - Hill climbing on the plan in order to make progress
# - Just instantiates one thing at a time

# NOTE
# - The set of objects is fixed when you do relaxed planning
# - I could save the relaxed planning tree and dynamically update it iteration to iteration
# - I could either make all instances upfront or ground them lazily

# Non-abstract always on the first level of the RPG
# TODO - relaxed planning assumes independent goals, but we want a conjunctive assignment...
# Well if we are just using the conditions along the plan, it truly is just relaxed planning
# Trying to fill in a concrete value for an abstract value is the other problem
# Maybe I could try removing static predicates if there exists concrete values that satisfy them
# - What if there exists some values that satisfy some but not all the values yet (but they can be checked)

# Do the version which doesn't try to reassign variables first
# In some cases, it might make sense to actually replace abstract with concrete
# Should I add ground variables to things?

##################################################

def print_solved(universe, plan):
  universe.print_statistics()
  print_plan_stats(plan, universe)

# TODO - currently only handling costs for ones without a variable cost

#def get_cost_functions(universe):
#  action_to_function = {}
#  for action in universe.name_to_action.values():
#    for atom in action.effect.get_atoms():
#      if isinstance(atom.predicate, Function):
#        action_to_function[action] = atom
#        #action_to_function[action] = atom.predicate
#  return action_to_function

def add_stream_costs(universe, l_costs, stream_cost):
  t0 = time()
  #instances = list(smart_instantiate_operators(action_to_function, universe))
  instances = list(smart_instantiate_operators(universe.actions, universe))
  for action in instances:
    add_stream_cost(universe, l_costs, action, stream_cost)
  if universe.verbose:
    print 'stream_costs | instances: %s, time: %s'%(len(instances), time() - t0)

##################################################

# TODO - do relaxed planning using smart instantiation to save time
# TODO - version which tries aggressively to find samples for just one plan forever
# NOTE - this strategy can be viewed as the replanning version of planning independently
# TODO - could do a version that fills in certain parameters before others
# TODO - once feasible, smooth the plan using a better planner

# TODO - flag to delay addition to stream pool for certain number of steps

# TODO - use this to be called by the hierarchical algorithm
def focused_subroutine(universe, search=DEFAULT_SEARCH,
                    max_time=INF, max_iterations=INF, stream_cost=10,
                    check_feasible=False, greedy=True, dfs=True, verbose=False, debug=False,
                    stream_limit=INF, sort_order=True, replace_initial=False, optimal=False):
  assert not optimal or stream_cost is None or stream_cost == 0
  # TODO - should really add these in focused_planner
  for operator in universe.name_to_action.values() + universe.axioms:
    bound_parameters = set(p for a in operator.condition.get_atoms() for p in a.args
                           if a.predicate in universe.stream_predicates | set(universe.derived_predicates))
    operator.condition = And(operator.condition, *[Concrete(param) for param in operator.parameters
                                                   if param not in bound_parameters])
  # NOTE - objects are grounded. This is harmless but annoying

  stream_actions = [StreamAction(cs) for cs in universe.problem.cond_streams]
  for stream_action in stream_actions:
    for obj in stream_action.out_consts:
      universe.add_object(obj)
  plannable_streams = filter(lambda a: a.cond_stream.plannable, stream_actions)

  #universe.cost_to_function = get_cost_functions(universe)
  universe.action_to_function = get_stream_functions(universe) if stream_cost is not None else {}
  solution = []
  for _ in irange(0, max_iterations):
    if verbose: print
    print_status(universe)
    if time() - universe.start_time >= max_time:
      break
    universe.iterations += 1
    if debug:
      for type in universe.type_to_objects:
        print type, len(universe.type_to_objects[type]), \
          map(get_value, filter(lambda o: not isinstance(o, AbstractConstant), universe.type_to_objects[type]))[:10]
      raw_input('Continue?\n')

    if check_feasible: # Checks if the problem is feasible with no parameters
      universe.temporary_atoms = set()
      t0 = time()
      plan = search(universe, max_time-(time()-universe.start_time), INF)
      universe.search_time += time() - t0
      if plan is not None:
        #if verbose: print_solved(universe, plan)
        print_status(universe)
        return solution if replace_initial else plan, universe

    t0 = time()
    process_eager_streams(universe)
    if DYNAMIC_INSTANCES:
      l_costs, s_costs, static_atoms = dynamic_compute_costs(plannable_streams, universe)
    else:
      l_costs, s_costs, static_atoms = get_stream_instances(plannable_streams, universe)
    if verbose:
      print 'compute_costs | predicates: %s, streams: %s, time: %s'%(len(l_costs), len(s_costs), time() - t0)

    if stream_cost is not None and search != strips_planner: # TODO - make this specific to FastDownward
      add_stream_costs(universe, l_costs, stream_cost)

    t0 = time()
    plan = search(universe, max_time-(time()-universe.start_time), INF)
    universe.search_time += time() - t0
    if verbose:
      print 'Cost:', plan_cost(universe, plan), 'Plan:', convert_plan(plan) # TODO - use an upper bound and continue?
    if plan is None:
      if len(universe.temp_blocked) == 0:
        break
      universe.temp_blocked = set()
      universe.resets += 1
      continue

    plan_goals = get_target_atoms(universe, plan, static_atoms)
    goals = {goal for level in plan_goals for goal in level}
    if len(goals) != 0:
      relaxed_plan, achievers = extract_plan(goals, l_costs, s_costs)
      t0 = time()
      if dfs:
        groundings = dfs_groundings(plan_goals, relaxed_plan, achievers, sort_order, stream_limit, universe, greedy=greedy)
        #groundings = queue_bindings(plan_goals, relaxed_plan, achievers, sort_order, problem)
      else:
        groundings = single_groundings(plan_goals, relaxed_plan, achievers, sort_order, stream_limit, universe, greedy=greedy)
      if verbose: print 'groundings | time:', time() - t0
      #for cs in problem.cond_streams:
      #  print cs, cs.calls, cs.call_time
      #raw_input('Continue?')
    else:
      groundings = {}

    #print groundings
    #print universe.perm_blocked
    #print universe.temp_blocked
    #raw_input('awefawfe')
    if groundings is not None and (not optimal or len(goals) == 0): # Prove optimal once have final costs
      plan = [(action, tuple(groundings.get(arg, arg) for arg in args)) for action, args in plan]

      if replace_initial:
        subplan, success = feasible_subplan(universe, plan)
        state = universe.initial_fluents()
        for action, args in subplan:
          state = action.instantiate(args).apply(state, universe.type_to_objects)
        fluent_atoms = filter(lambda a: a.predicate in universe.fluent_predicates, state) # NOTE - might be unnecessary
        static_atoms = filter(lambda a: a.predicate not in universe.fluent_predicates, universe.initial_atoms)
        universe.initial_atoms = set(fluent_atoms) | set(static_atoms)
        solution += subplan

      if not is_solution(universe, plan):
        if verbose:
          print plan
          #raw_input('Failed plan')
        continue
      #if verbose: print_solved(universe, plan)
      print_status(universe)
      return solution if replace_initial else plan, universe
    #if verbose: raw_input('Continue?')
  #if verbose: universe.print_statistics()
  print_status(universe)
  return None, universe

# TODO - version which instantly, permanently blocks and adds to queue

def focused_planner(problem, search=DEFAULT_SEARCH,
                    max_time=INF, max_iterations=INF, stream_cost=10,
                    check_feasible=False, greedy=True, dfs=True, verbose=False, debug=False,
                    stream_limit=INF, sort_order=True, replace_initial=False, optimal=False):
  """
  Focused algorithm.

  :param problem: :class:`.STRIPStreamProblem`
  :param search: python function of the search subroutine
  :param max_time: numeric maximum planning time
  :param max_iterations: int maximum subroutine calls
  :param stream_cost: int or ``None`` that scales the cost of calling a stream
  :param check_feasible: boolean flag to check whether the non-abstract problem is feasible at the start of each iteration
  :param greedy: boolean flag to greedily return first binding
  :param dfs: boolean flag to perform a dfs search over bindings
  :param verbose: boolean flag which toggles the print output
  :param debug: boolean flag which prints the objects on each iteration
  :param stream_limit: int for the maximum number of streams processed per iteration
  :param sort_order: boolean flag to sort streams by order
  :param replace_initial: boolean flag to update the initial state using the plan prefix
  :return: a sequence of :class:`.Action` and tuple of :class:`.Constant` pairs as well as :class:`.Universe`
  """
  #print SEPARATOR
  universe = Universe(problem, use_ground=True, make_stream_instances=True) # Previously removed for hierarchy
  return focused_subroutine(universe, search,
                    max_time, max_iterations, stream_cost,
                    check_feasible, greedy, dfs, verbose, debug,
                    stream_limit, sort_order, replace_initial, optimal)
