#!/usr/bin/env python

from time import time
import argparse
import sys
import os

from openravepy import RaveSetDebugLevel, DebugLevel, Environment, RaveDestroy

from manipulation.oracle import ManipulationOracle
from manipulation.primitives.display import is_viewer_active
from manipulation.visualizations import visualize_plan, execute_viewer, Plan
from manipulation.problems import *

# TODO - clean these up to switch between different formulations
#from stripstream.pddl.examples.openrave.tamp import compile_problem, convert_state, executable_plan
#from stripstream.pddl.examples.openrave.tamp_no_conf import compile_problem
#from stripstream.pddl.examples.openrave.relative_tamp import compile_problem, convert_state, Pick, Place, Move
#from stripstream.pddl.examples.openrave.stack_tamp import compile_problem, convert_state, Pick, Place, Move
from stripstream.pddl.examples.openrave.easy_tamp import compile_problem, convert_state, executable_plan
#from stripstream.pddl.examples.openrave.easy_tamp_icaps import compile_problem, convert_state, executable_plan
#from stripstream.pddl.examples.openrave.easy_tamp_advanced import compile_problem, convert_state, executable_plan
#from stripstream.pddl.examples.openrave.clean_tamp import compile_problem, convert_state

from stripstream.algorithms.incremental.incremental_planner import incremental_planner
from stripstream.algorithms.focused.focused_planner import focused_planner
from stripstream.algorithms.hierarchy.replan import replan_hierarchy, first_selector
from stripstream.algorithms.hierarchy.aggressive import goal_serialization
from stripstream.algorithms.search.fast_downward import get_fast_downward
from stripstream.pddl.utils import convert_plan
from stripstream.algorithms.utils import FAST_DOWNWARD, INCREMENTAL, FOCUSED
from stripstream.utils import SEPARATOR

import datetime

DIRECTORY = './logs/manipulation/'

def get_directory_path(problem_name, planner_name): # TODO - unify this with run_constarint
  dt = datetime.datetime.now()
  return DIRECTORY + '{}/{}/{}/{}/'.format(problem_name, planner_name, dt.strftime('%Y-%m-%d'), dt.strftime('%H-%M-%S'))

def solve_tamp(env, mproblem, display, incremental, focused, movie):
  RaveSetDebugLevel(DebugLevel.Fatal) #RaveSetDebugLevel(DebugLevel.Debug)
  mproblem.set_viewer(env)
  mproblem.load_env(env)

  oracle = ManipulationOracle(mproblem, env)
  stream_problem = compile_problem(oracle)
  print stream_problem
  oracle.draw_goals() # NOTE - this must be after compile_problem to ensure the goal_poses convert
  if is_viewer_active(oracle.env):
    raw_input('Start?')

  search_fn = get_fast_downward('eager', verbose=False) # dijkstra | astar | wastar1 | wastar2 | wastar3 | eager | lazy
  verbose = False
  debug = False
  profile = False

  assert incremental != focused
  planner = INCREMENTAL if incremental else FOCUSED
  #planner = 'hierarchy' # 'hierarchy' | 'serial'

  t0 = time()
  def solve():
    if planner == INCREMENTAL:
      return incremental_planner(stream_problem, search=search_fn, frequency=1, waves=True, verbose=verbose, debug=debug) # 1 | 20 | 100 | INF
    elif planner == FOCUSED:
      return focused_planner(stream_problem, search=search_fn, greedy=False, stream_cost=10, verbose=verbose, debug=debug)
    elif planner == 'serial':
      return goal_serialization(stream_problem, verbose=verbose, debug=debug)
    elif planner == 'hierarchy':
      selector = first_selector # first_selector | all_selector
      return replan_hierarchy(stream_problem, selector=selector, first_only=False, execute=True, verbose=verbose)
    else:
      raise ValueError(planner)

  if profile:
    from misc.profiling import run_profile
    (plan, universe), prof = run_profile(solve)
  else:
    (plan, universe), prof = solve(), None
  if prof is not None:
    from misc.profiling import str_profile
    print SEPARATOR
    print str_profile(prof)
  universe.print_statistics()
  #print_plan_stats(plan, universe)

  print SEPARATOR
  print 'Planner:', planner
  print 'Search:', FAST_DOWNWARD
  print 'Solved:', plan is not None
  print 'Length:', len(plan) if plan is not None else None
  print 'Time:', time() - t0
  if plan is not None:
    print 'Plan:' #, convert_plan(plan)
    for i, (action, args) in enumerate(convert_plan(plan)):
      print '%s) %s%s'%(i, action, tuple(args))

  if plan is None:
    return

  #if is_viewer_active(oracle.env):
  #  raw_input('Done!')
  #return

  #trace_plan(stream_problem, plan)
  #print get_raw_states(stream_problem, plan)

  images_directory = os.path.join(get_directory_path(mproblem.name, os.path.basename(__file__)), 'images') if movie else None
  if display:
    #display_solution(oracle, plan, directory_path, 'plan', movie)
    #states = map(lambda s: convert_state(oracle, s), get_states(pddl_problem, plan))
    #visualize_states(states, oracle, display=True, save=images_directory)

    start = convert_state(oracle, stream_problem.initial_atoms)
    executable = executable_plan(oracle, plan)
    visualize_plan(Plan(start, executable), oracle, display=True, save=images_directory)

  #sample_time = sum(get_sample_time(operator) for operator in plan.operators) # TODO - need access parent pap
  #print 'Sample time: %.5f | Sample ratio %.3f%%'%(sample_time, 100*sample_time/total_time)

##################################################

def main(argv):
  parser = argparse.ArgumentParser() # Automatically includes help
  parser.add_argument('--problem', help='problem name.', default='simple')
  parser.add_argument('-fd', action='store_true', help='FastDownward (a dummy flag).')
  parser.add_argument('-viewer', action='store_true', help='enable viewer.')
  parser.add_argument('-display', action='store_true', help='display solution.')
  parser.add_argument('-focus', action='store_true', help='focused.')
  parser.add_argument('-movie', action='store_true', help='record a movie.')
  args = parser.parse_args()

  if args.problem not in list_problems():
    print args.problem, 'is not a valid problem'
    return

  env = Environment() # NOTE - need this for load_problem
  problem = load_problem(args.problem)
  try:
    execute = lambda: solve_tamp(env, problem, args.display, incremental=not args.focus, focused=args.focus, movie=args.movie)
    if args.viewer: execute_viewer(env, execute)
    else: execute()
  finally:
    if env.GetViewer() is not None:
      env.GetViewer().quitmainloop()
    RaveDestroy()

if __name__ == '__main__':
  main(sys.argv[1:])