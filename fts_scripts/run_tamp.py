#!/usr/bin/env python2

from openravepy import RaveSetDebugLevel, DebugLevel, Environment, RaveDestroy
from manipulation.oracle import ManipulationOracle
from manipulation.primitives.display import is_viewer_active
from manipulation.visualizations import visualize_plan, execute_viewer, execute_plan
from manipulation.problems import *

# TODO - clean this up
from stripstream.fts.examples.tamp import tamp_problem, convert_state, convert_plan
#from stripstream.fts.examples.tamp_base import tamp_problem, convert_state, convert_plan
#from stripstream.fts.examples.unfactored_tamp import tamp_problem

from stripstream.fts.stripstream_conversion import get_single_state, constraint_to_stripstream
from stripstream.algorithms.search.fast_downward import get_fast_downward
from stripstream.algorithms.incremental.incremental_planner import incremental_planner
from stripstream.algorithms.focused.focused_planner import focused_planner
from stripstream.utils import SEPARATOR
from stripstream.algorithms.utils import FAST_DOWNWARD, INCREMENTAL, FOCUSED

from time import time
from collections import namedtuple
import argparse
import datetime
import os

DIRECTORY = './logs/manipulation/'

def get_directory_path(problem_name, planner_name): # TODO - unify this with run_tamp
  dt = datetime.datetime.now()
  return DIRECTORY + '{}/{}/{}/{}/'.format(problem_name, planner_name,
                                           dt.strftime('%Y-%m-%d'), dt.strftime('%H-%M-%S'))

def solve_constraint_tamp(env, manip_problem, display, focused, movie, execute = True, verbose=False):
  #RaveSetDebugLevel(DebugLevel.Fatal) #RaveSetDebugLevel(DebugLevel.Debug)
  manip_problem.set_viewer(env)
  manip_problem.load_env(env)
  oracle = ManipulationOracle(manip_problem, env)

  fts_problem = tamp_problem(oracle)
  print
  print fts_problem

  stream_problem = constraint_to_stripstream(fts_problem)
  print
  print stream_problem

  oracle.draw_goals() # NOTE - this must be after compile_problem to ensure the goal_poses convert
  if is_viewer_active(oracle.env):
    raw_input('Start?')

  search = get_fast_downward('eager') # 'dijkstra | astar | wastar1 | wastar2 | wastar3 | eager | lazy
  planner = FOCUSED if focused else INCREMENTAL

  print SEPARATOR
  t0 = time()
  if planner == FOCUSED:
    # TODO - why do I need make_stream_instances=True here?
    plan, _ = focused_planner(stream_problem, search=search, check_feasible=False, greedy=False, dfs=True, verbose=verbose)
  elif planner == INCREMENTAL:
    frequency = 1 # 1 | 20 | 100 | INF
    # NOTE - the versions I submitted to RSS had implicitly waves=False
    plan, _ = incremental_planner(stream_problem, search=search, frequency=frequency, waves=True, verbose=verbose)
  else:
    raise ValueError('Must specify a planner')

  print SEPARATOR
  print 'Planner:', planner
  print 'Search:', FAST_DOWNWARD
  print 'Solved:', plan is not None
  print 'Length:', len(plan) if plan is not None else None
  print 'Time:', time() - t0

  if plan is None:
    return

  # NOTE - need to set constants
  #PRE_SMOOTH_TRAJECTORIES = True
  #REPLAN_TRAJECTORIES = True
  #SMOOTH_TRAJECTORIES = identity_trajectories # identity_trajectories | smooth_trajectories

  #print oracle.robot.GetAffineTranslationMaxVels(), oracle.robot.GetAffineRotationAxisMaxVels()
  #oracle.robot.SetAffineTranslationMaxVels(5*oracle.robot.GetAffineTranslationMaxVels())

  #SetAffineRotationAxisMaxVels(oracle.robot) # SetAffineRotationAxisMaxVels | SetAffineRotation3DMaxVels
  #SetAffineTranslationMaxVels(oracle.robot)
  # SetActiveDOFVelocities, SetDOFVelocities
  # SetDOFAccelerationLimits
  # GetActiveDOFMaxAccel but not SetActiveDOFMaxAccel

  #oracle.robot.SetActiveDOFVelocities(5*oracle.robot.GetActiveDOFVelocities())
  #action.base_trajs[0].cspace.set_active()
  #indices = oracle.robot.GetActiveDOFIndices()
  #print indices
  #print oracle.robot.GetActiveDOFMaxAccel(indices)
  #oracle.robot.SetActiveDOFMaxAccel(5*oracle.robot.GetActiveDOFMaxAccel(indices), indices)

  # NOTE - not sure why the focused algorithm isn't working that well here anymore

  images_directory = None
  if movie:
    directory_path = get_directory_path(manip_problem.name, 'fts')
    images_directory = os.path.join(directory_path, 'images/')
    print 'Save:', images_directory
  if display or movie:
    Plan = namedtuple('Plan', ['start', 'operators'])
    start = convert_state(oracle, get_single_state(fts_problem.initial_state))
    executables = convert_plan(oracle, plan)
    if execute:
      if movie:
        from misc.utils import launch_quicktime
        launch_quicktime() # TODO - produces missing value if quicktime is already enabled
      execute_plan(Plan(start, executables), oracle, pause=False)
    else:
      visualize_plan(Plan(start, executables), oracle, display=display, save=images_directory)


##################################################

def main():
  parser = argparse.ArgumentParser() # Automatically includes help
  parser.add_argument('--problem', help='problem name.', default='simple')
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
    execute = lambda: solve_constraint_tamp(env, problem, args.display,
        focused=args.focus, movie=args.movie)
    if args.viewer: execute_viewer(env, execute)
    else: execute()
  finally:
    if env.GetViewer() is not None:
      env.GetViewer().quitmainloop()
    RaveDestroy()

if __name__ == '__main__':
  main()