#!/usr/bin/env python

from time import time
from stripstream.algorithms.incremental.incremental_planner import incremental_planner
from stripstream.pddl.examples.openrave.motion_planning import compile_problem
from manipulation.oracle import ManipulationOracle
from manipulation.primitives.display import is_viewer_active

import argparse

from openravepy import RaveSetDebugLevel, DebugLevel, Environment, RaveDestroy
from manipulation.visualizations import execute_viewer
from manipulation.problems import *

# The focused algorithm is really like a lazy PRM
def solve_motion(env, mproblem):
  RaveSetDebugLevel(DebugLevel.Fatal) #RaveSetDebugLevel(DebugLevel.Debug)
  mproblem.set_viewer(env)
  mproblem.load_env(env)
  oracle = ManipulationOracle(mproblem, env)
  stream_problem = compile_problem(oracle)
  print stream_problem
  oracle.draw_goals() # NOTE - this must be after compile_problem to ensure the goal_poses convert
  if is_viewer_active(oracle.env):
    raw_input('Start?')

  planner = 'eager' # 'dijkstra | astar | wastar1 | wastar2 | wastar3 | eager | lazy
  frequency = 1 # 1 | 20 | 100 | INF
  optimal = True
  max_time = 120

  t0 = time()
  plan, _ = incremental_planner(stream_problem, planner=planner, frequency=frequency,
                                optimal=optimal, max_time=max_time, verbose=True)
  print
  print 'Lazy'
  print 'Plan:', plan
  print 'Solved:', plan is not None
  print 'Time:', time() - t0

##################################################

def main():
  parser = argparse.ArgumentParser() # Automatically includes help
  parser.add_argument('-v', action='store_true', help='enable viewer.')
  args = parser.parse_args()

  env = Environment() # NOTE - need this for load_problem
  problem = load_problem('trivial_namo')
  try:
    execute = lambda: solve_motion(env, problem)
    if args.v: execute_viewer(env, execute)
    else: execute()
  finally:
    if env.GetViewer() is not None:
      env.GetViewer().quitmainloop()
    RaveDestroy()

if __name__ == '__main__':
  main()