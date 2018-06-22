from collections import deque
from time import time

from stripstream.algorithms.plan import plan_cost, print_plan_stats
from stripstream.algorithms.universe import Universe
from stripstream.algorithms.utils import DEFAULT_SEARCH
from stripstream.pddl.cond_streams import TestCondStream
from stripstream.pddl.logic.atoms import Atom
from stripstream.pddl.logic.operations import Initialize
from stripstream.pddl.objects import Object
from stripstream.pddl.utils import get_value, convert_plan
from stripstream.utils import irange, INF


# TODO - future ideas
# - priority queue that prioritizes quick to evaluate
# - could also experiment with other ways of planning in batches like FFRob
# - detect optimal if the shortest path ignoring static preconditions
# - either process eager immediately once or always make a new sample on each iteration

def call_stream(universe, stream, **kwargs):
  if universe.verbose:
    print 'Called', stream
  if isinstance(stream.cond_stream, TestCondStream):
    universe.test_calls += 1
  universe.calls += 1
  values = stream.call(universe=universe, **kwargs)
  for value in values:
    if isinstance(value, Object):
      universe.add_object(value)
    elif isinstance(value, Atom):
      universe.add_initial_atom(value)
    elif isinstance(value, Initialize):
      universe.add_perm_initialize(value)
    else:
      raise ValueError(value)
  return values

def add_streams(universe, queue):
  while universe.new_streams:
    stream = universe.new_streams.pop(0)
    if stream.cond_stream.eager and (not stream.called):
      call_stream(universe, stream)
    else:
      queue.append(stream)

def call_queue(queue, universe, frequency, max_time):
  local_calls = 0
  universe.new_problem = False
  while queue and (not universe.new_problem or (local_calls < frequency)):
    if (time() - universe.start_time) > max_time:
      return False
    stream = queue.popleft()
    assert all(con in universe.initial_atoms for con in stream.instantiate_conditions())
    #if len(stream.cond_stream.outputs) == 0 and all(con in pddl_problem.initial_atoms for con in stream.instantiate_effects([])):
    #  continue
    call_stream(universe, stream)
    add_streams(universe, queue) # TODO - I could also call once all promised
    if not stream.cond_stream.eager or not stream.called: # NOTE - used to have bug where "and" instead of "or"
      local_calls += 1
    if not stream.enumerated:
      queue.append(stream)
    else:
      universe.enumerated += 1
  return universe.new_problem

# NOTE - cannot immediately call new instances because they may keep producing new values
# All streams in the queue are satisfied

def call_queue_wave(queue, universe, max_values, max_time):
  wave = 0
  universe.new_problem = False
  while queue and (not universe.new_problem or (wave < max_values)):
    called_queue = deque()
    while queue:
      if (time() - universe.start_time) > max_time:
        return False
      stream = queue.popleft()
      if not all(con in universe.initial_atoms for con in stream.instantiate_conditions()):
        print 'Warning: not all conditions achieved' # TODO - bug which happens when a constant is in Stream conditions (something in universe isn't right)
        print stream.instantiate_conditions()
        print stream
        called_queue.append(stream)
        continue
      call_stream(universe, stream)
      if not stream.enumerated:
        called_queue.append(stream)
      else:
        universe.enumerated += 1
    add_streams(universe, queue)
    queue.extend(called_queue)
    wave += 1
  # NOTE - could call all the eager tests here
  return universe.new_problem

##################################################

# TODO - make a max_size rather than max_calls

def print_status(universe, best_plan, best_cost):
    print 'Iteration: %d | Time: %s | Calls: %s | Search Time: %s | Solved: %s | Cost: %s'%(universe.iterations,
      round(time() - universe.start_time, 3), universe.calls, round(universe.search_time, 3),
      best_plan is not None, best_cost)

def debug_planner(universe, max_objects=10):
  for type in universe.type_to_objects:
    print type, len(universe.type_to_objects[type]), map(get_value, universe.type_to_objects[type])[:max_objects]
  raw_input('Continue?\n')

def update_best_plan(universe, search, best_plan, best_cost, max_time):
  if max_time is None:
    return best_plan, best_cost
  t0 = time()
  plan = search(universe, max_time, best_cost)
  universe.search_time += time() - t0
  if plan is not None:
    cost = plan_cost(universe, plan) # plan_cost | plan_length
    if cost < best_cost:
      best_plan, best_cost = plan, cost
      if universe.verbose:
        print best_cost, convert_plan(best_plan)
  return best_plan, best_cost

def incremental_planner(problem, search=DEFAULT_SEARCH, max_time=INF,
                        min_cost=INF,
                        waves=True, frequency=1, optimal=False, # TODO: remove optimal
                        postprocess_time=None, verbose=False, debug=False):
  """
  Incremental algorithm.

  :param problem: :class:`.STRIPStreamProblem`
  :param search: python function of the search subroutine
  :param max_time: numeric maximum planning time
  :param max_iterations: int maximum subroutine calls
  :param waves: boolean flag when ``True`` considers each stream evaluation iteration to be the current queue rather than a single stream
  :param frequency: numeric number of evaluation iterations given by ``waves`` (``float('inf')`` implies to evaluate all streams per iteration)
  :param optimal: boolean flag which saves the best current solution
  :param verbose: boolean flag which toggles the print output
  :param debug: boolean flag which prints the objects on each iteration
  :return: a sequence of :class:`.Action` and tuple of :class:`.Constant` pairs as well as :class:`.Universe`
  """
  for stream in problem.cond_streams:
    if not stream.sign and not stream.eager:
      print 'Warning! Setting stream', stream, 'to eager because used negatively'
      stream.eager = True

  #print SEPARATOR
  universe = Universe(problem, use_ground=False, verbose=verbose)
  #useful_atoms, useful_types = useful_universe(universe)
  #stream_pairs = get_streams(useful_atoms, useful_types, universe) # TODO - prune any unnecessary cond_streams
  queue = deque()
  add_streams(universe, queue)
  best_plan, best_cost = None, INF
  while True:
    print_status(universe, best_plan, best_cost)
    if debug:
      debug_planner(universe)
    universe.iterations += 1
    if (not optimal) or (postprocess_time is None):
      best_plan, best_cost = update_best_plan(universe, search, best_plan, best_cost, 
        max_time-(time()-universe.start_time))
    if (best_plan is not None) and ((not optimal) or (best_cost <= 0)):
      break
    if waves and not call_queue_wave(queue, universe, frequency, max_time):
      break
    if not waves and not call_queue(queue, universe, frequency, max_time):
      break
  add_streams(universe, queue) # TODO: hack for postprocess_time != None to not give "Could not find instantiation for PNE!"
  best_plan, best_cost = update_best_plan(universe, search, best_plan, best_cost, postprocess_time)
  print_status(universe, best_plan, best_cost)
  if verbose:
    universe.print_statistics()
    if best_plan is not None:
      print_plan_stats(best_plan, universe)
      print 'Cost: {}\nLength: {}'.format(best_cost, len(best_plan))
  return best_plan, universe
