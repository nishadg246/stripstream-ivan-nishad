from time import time

from stripstream.utils import INF, set_union
from stripstream.algorithms.universe import Universe
from stripstream.algorithms.focused.utils import AbstractConstant, Concrete, print_status
from stripstream.algorithms.incremental.incremental_planner import DEFAULT_SEARCH
from stripstream.algorithms.focused.simple_focused import initialize_universe, make_streams, \
  determine_reachable, solve_abstract, extract_streams, is_real_solution, produce_bindings, solve_real, \
  call_real_stream
from stripstream.pddl.cond_streams import EasyTestStream
from stripstream.pddl.logic.atoms import Atom

# TODO: can convert test streams to have the appropriate sign for the problem by inverting them
# Almost any stream that finishes quickly should just be evaluated immediately in the incremental algorithm

# TODO: slowest part now is get_target_atoms
# 3    0.065    0.022    3.063    1.021 stripstream/algorithms/focused/utils.py:58(get_target_atoms)

def signed_focused(problem, search=DEFAULT_SEARCH, max_time=INF,
                   optimal=False, check_feasible=False, verbose=False):

  # To start, can just eagerly evaluate these guys
  negative_tests = {}
  for stream in problem.cond_streams:
    if isinstance(stream, EasyTestStream) and (not stream.sign) and (not stream.conditions) and (len(stream.effects) == 1):
      if stream.eager:
        # TODO: don't make instances for lazy samples
        stream.plannable = False
      else:
        stream.make_instances = False
      [effect] = stream.effects
      negative_tests[effect.predicate] = stream
  # TODO: keep track of called negative streams?

  universe = Universe(problem, use_ground=False)
  initialize_universe(universe)

  best_plan = None
  while (time() - universe.start_time) < max_time:
    if verbose: print
    print_status(universe)
    universe.iterations += 1

    if check_feasible: # Checks if the problem is feasible with no parameters
      real_plan = solve_real(universe, search, max_time)
      if real_plan is not None:
        best_plan = real_plan
        if not optimal:
          break
    make_streams(universe, max_level=INF)
    atom_nodes = determine_reachable(universe)
    abstract_plan, goals = solve_abstract(universe, atom_nodes, search, max_time)
    if abstract_plan is None:
      if not universe.temp_blocked:
        break
      universe.temp_blocked = set()
      universe.resets += 1
      continue

    ####################

    # TODO: move this into solve_abstract?
    positive_goals = set(filter(lambda g: isinstance(g, Atom), goals))
    negative_goals = map(lambda g: g.formula, goals - positive_goals)
    print negative_goals
    constants = set_union(set(args) for _, args in abstract_plan)
    abstract_constants = filter(lambda c: isinstance(c, AbstractConstant), constants)
    if verbose:
      print 'Positive goals:', len(positive_goals)
      print 'Negative goals:', len(negative_goals)
      print 'Abstract constants:', len(abstract_constants)
    order = extract_streams(atom_nodes, positive_goals | set(map(Concrete, abstract_constants)))
    if verbose: print 'Order:', order
    # TODO: we want the negative stream to fail (this becomes good for us). This affects binding

    bound_plan = produce_bindings(universe, order, abstract_plan, greedy=False, shared=False, dfs=False)
    # TODO: replan using only concrete here rather than finding bindings if successful
    for goal in negative_goals:
      stream = negative_tests[goal.predicate]
      if any(isinstance(a, AbstractConstant) for a in goal.args):
        # TODO: later use bound values
        bound_plan = None
        continue
      [effect] = stream.effects
      param_map = dict(zip(effect.args, goal.args))
      stream_instance = stream(param_map[arg] for arg in stream.inputs)
      if not stream.eager and call_real_stream(stream_instance, universe):
          bound_plan = None
      if not stream_instance.enumerated:
        bound_plan = None

    if bound_plan is not None:
      best_plan = bound_plan
      if not optimal:
        break
  assert (best_plan is None) or is_real_solution(universe, best_plan)
  return best_plan, universe
