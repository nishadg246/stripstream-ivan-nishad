#!/usr/bin/env python2

import argparse
import sys

from openravepy import Environment, RaveDestroy

from stripstream.algorithms.focused.simple_focused import simple_focused
from stripstream.algorithms.search.fast_downward import get_fast_downward

#from manipulation.motion.single_query import vector_traj_helper
from robotics.openrave.utils import open_gripper, \
  Conf, initialize_openrave, execute_viewer

from stripstream.fts.constraint import Eq, ConType
from stripstream.fts.variable import VarType, Par, Var, X, U, nX
from stripstream.fts.clause import Clause
from stripstream.fts.sampler import Sampler
from stripstream.fts.problem import FTSProblem
from stripstream.fts.stripstream_conversion import constraint_to_stripstream
from stripstream.fts.utils import convert_plan
from stripstream.utils import SEPARATOR, INF

from robotics.openrave.problems import dantam_distract
from robotics.openrave.tamp_fixed_base import cfree_traj_fn, \
  sample_grasp_traj_fn, sample_free_motion_fn, sample_holding_motion_fn, visualize_solution

# TODO: not sure why I created this one...

####################

PROBLEM = lambda env: dantam_distract(env, 8)
ARM = 'leftarm'
MAX_GRASPS = 1

####################

def get_problem(env, problem, robot, manipulator, base_manip, bodies, initial_conf):
  all_bodies = bodies.values()
  assert len({body.GetKinematicsGeometryHash() for body in all_bodies}) == 1 # NOTE - assuming all objects has the same geometry

  body1 = all_bodies[-1] # Generic object 1
  body2 = all_bodies[-2] if len(bodies) >= 2 else body1 # Generic object 2
  # TODO - bug that when n=1, it automatically passes holding collisions because only one body used
  grasps = problem.known_grasps[:MAX_GRASPS] if problem.known_grasps else []
  poses = problem.known_poses if problem.known_poses else []

  # Limits the number of poses
  #needed_poses = list(set(problem.initial_poses.values() + problem.goal_poses.values()))
  #poses = needed_poses + list(set(poses) - set(needed_poses))[:9]

  cfree_traj = cfree_traj_fn(env, manipulator, body1, body2, all_bodies)

  sample_grasp_traj = sample_grasp_traj_fn(env, manipulator, body1, all_bodies)
  sample_free_motion = sample_free_motion_fn(manipulator, base_manip, all_bodies)
  sample_holding_motion = sample_holding_motion_fn(manipulator, base_manip, body1, all_bodies)

  # OBJ, BOOL = VarType(domain=list(bodies)), VarType(domain=[True, False])
  # POSE, CONF, TRAJ = VarType(domain=poses+grasps), VarType(), VarType()
  #
  # Stable = ConType([POSE], satisfying=[(p,) for p in poses])
  # Grasp = ConType([POSE], satisfying=[(g,) for g in grasps])
  # Motion = ConType([CONF, CONF, TRAJ])
  # MotionH = ConType([CONF, CONF, POSE, TRAJ])
  # GraspMotion = ConType([POSE, POSE, CONF, TRAJ])
  # CFree = ConType([TRAJ, POSE], test=cfree_traj)
  #
  # O, P, G = Par(OBJ), Par(POSE), Par(POSE)
  # Q, Q2, T = Par(CONF), Par(CONF), Par(TRAJ)
  #
  # state_vars = [Var('R_Q', CONF), Var('O_P', POSE, args=[OBJ]), Var('O_H', BOOL, args=[OBJ])]
  # control_vars = [Var('R_T', TRAJ)]
  #
  # transition = [
  #   Clause([Stable(X['O_P', O]), Grasp(nX['O_P', O]),
  #           Eq(X['O_H', O], False), Eq(nX['O_H', O], True),
  #           GraspMotion(X['O_P', O], nX['O_P', O], X['R_Q'], U['R_T'])] +
  #          [Eq(X['O_H', obj], False) for obj in bodies] +
  #          [CFree(U['R_T'], X['O_P', obj]) for obj in bodies], name='pick'),
  #   Clause([Grasp(X['O_P', O]), Stable(nX['O_P', O]),
  #           Eq(X['O_H', O], True), Eq(nX['O_H', O], False),
  #           GraspMotion(nX['O_P', O], X['O_P', O], X['R_Q'], U['R_T'])] +
  #          [CFree(U['R_T'], X['O_P', obj]) for obj in bodies], name='place'),
  #   Clause([Motion(X['R_Q'], nX['R_Q'], U['R_T'])] +
  #          [Eq(X['O_H', obj], False) for obj in bodies] +
  #          [CFree(U['R_T'], X['O_P', obj]) for obj in bodies], name='move'),
  #   Clause([MotionH(X['R_Q'], nX['R_Q'], X['O_P', O], U['R_T']), Eq(X['O_H', O], True)] +
  #          [CFree(U['R_T'], X['O_P', obj]) for obj in bodies], name='move_holding')]
  #
  # samplers = [
  #   Sampler([GraspMotion(P, G, Q, T)], gen=sample_grasp_traj, inputs=[P, G], domain=[Stable(P), Grasp(G)]),
  #   Sampler([Motion(Q, Q2, T)], gen=sample_free_motion, inputs=[Q, Q2]),
  #   Sampler([MotionH(Q, Q2, G, T)], gen=sample_holding_motion, inputs=[Q, Q2, G], domain=[Grasp(G)])]
  #
  # initial_state = [Eq(X['R_Q'], initial_conf)] + [
  #   Eq(X['O_P', obj], pose) for obj, pose in problem.initial_poses.iteritems()
  # ] + [Eq(X['O_H', obj], False) for obj in problem.initial_poses]
  #
  # goal_constraints = [Eq(X['R_Q'], initial_conf)] + [
  #   Eq(X['O_P', obj], pose) for obj, pose in problem.goal_poses.iteritems()]
  #
  # return FTSProblem(state_vars, control_vars, transition, samplers, initial_state, goal_constraints)

  OBJ, BOOL = VarType(domain=list(bodies)), VarType(domain=[True, False])
  POSE, CONF, TRAJ = VarType(domain=poses+grasps), VarType(), VarType()

  Stable = ConType([POSE], satisfying=[(p,) for p in poses])
  Grasp = ConType([POSE], satisfying=[(g,) for g in grasps])
  Motion = ConType([CONF, CONF, TRAJ])
  MotionH = ConType([CONF, CONF, POSE, TRAJ])
  CFree = ConType([TRAJ, POSE], test=cfree_traj)

  state_vars = [Var('R_Q', CONF), Var('O_P', POSE, args=[OBJ]), Var('O_H', BOOL, args=[OBJ])]
  control_vars = [Var('R_T', TRAJ)]

  O, G, Q, Q2, T = Par(OBJ), Par(POSE), Par(CONF), Par(CONF), Par(TRAJ)
  transition = [
    Clause([Stable(X['O_P', O]), Grasp(nX['O_P', O]),
            Eq(X['O_H', O], False), Eq(nX['O_H', O], True)] +
           [Eq(X['O_H', obj], False) for obj in bodies] +
           [CFree(U['R_T'], X['O_P', obj]) for obj in bodies], name='pick'),
    Clause([Grasp(X['O_P', O]), Stable(nX['O_P', O]),
            Eq(X['O_H', O], True), Eq(nX['O_H', O], False)] +
           [CFree(U['R_T'], X['O_P', obj]) for obj in bodies], name='place'),
    Clause([Motion(X['R_Q'], nX['R_Q'], U['R_T'])] +
           [Eq(X['O_H', obj], False) for obj in bodies] +
           [CFree(U['R_T'], X['O_P', obj]) for obj in bodies], name='move'),
    Clause([MotionH(X['R_Q'], nX['R_Q'], X['O_P', O], U['R_T']), Eq(X['O_H', O], True)] +
           [CFree(U['R_T'], X['O_P', obj]) for obj in bodies], name='move_holding')]

  samplers = [
    Sampler([Motion(Q, Q2, T)], gen=sample_free_motion, inputs=[Q, Q2]),
    Sampler([MotionH(Q, Q2, G, T)], gen=sample_holding_motion, inputs=[Q, Q2, G], domain=[Grasp(G)])]

  ####

  initial_state = [Eq(X['R_Q'], initial_conf)] + [
    Eq(X['O_P', obj], pose) for obj, pose in problem.initial_poses.iteritems()
  ] + [Eq(X['O_H', obj], False) for obj in problem.initial_poses]

  goal_constraints = [Eq(X['R_Q'], initial_conf)] + [
    Eq(X['O_P', obj], pose) for obj, pose in problem.goal_poses.iteritems()]

  return FTSProblem(state_vars, control_vars, transition, samplers, initial_state, goal_constraints)

##################################################

def solve_tamp(env):
  viewer = env.GetViewer() is not None
  problem = PROBLEM(env)

  robot, manipulator, base_manip, _ = initialize_openrave(env, ARM, min_delta=.01)
  bodies = {obj: env.GetKinBody(obj) for obj in problem.object_names}

  open_gripper(manipulator)
  initial_conf = Conf(robot.GetConfigurationValues()[manipulator.GetArmIndices()])

  ####################

  fts_problem = get_problem(env, problem, robot, manipulator, base_manip, bodies, initial_conf)
  print
  print fts_problem

  stream_problem = constraint_to_stripstream(fts_problem)
  print
  print stream_problem

  for stream in stream_problem.cond_streams:
    print stream, stream.max_level

  # TODO - why is this slower/less reliable than the other one (extra axioms, eager evaluation?)
  if viewer: raw_input('Start?')
  search_fn = get_fast_downward('eager', max_time=10, verbose=False)
  #solve = lambda: incremental_planner(stream_problem, search=search_fn, frequency=1, waves=True, debug=False)
  solve = lambda: simple_focused(stream_problem, search=search_fn, max_level=INF, shared=False, debug=False, verbose=False)
  env.Lock()
  plan, universe = solve()
  env.Unlock()

  print SEPARATOR

  plan = convert_plan(plan)
  if plan is not None:
    print 'Success'
    for i, (action, args) in enumerate(plan):
      print i+1, action, args
  else:
    print 'Failure'

  ####################

  if viewer and plan is not None:
    print SEPARATOR
    visualize_solution(env, problem, initial_conf, robot, manipulator, bodies, plan)
  raw_input('Finish?')

##################################################

def main(argv):
  parser = argparse.ArgumentParser() # Automatically includes help
  parser.add_argument('-viewer', action='store_true', help='enable viewer.')
  args = parser.parse_args()

  env = Environment()
  try:
    execute = lambda: solve_tamp(env)
    if args.viewer: execute_viewer(env, execute)
    else: execute()
  finally:
    if env.GetViewer() is not None:
      env.GetViewer().quitmainloop()
    RaveDestroy()
  print 'Done!'
  # TODO - it's segfaulting here...

if __name__ == '__main__':
  main(sys.argv[1:])