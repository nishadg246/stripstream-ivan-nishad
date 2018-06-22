#!/usr/bin/env python

import argparse
import sys

from lis_scripts.hpn_utils import *

from stripstream.algorithms.incremental.incremental_planner import incremental_planner
from stripstream.algorithms.focused.simple_focused import simple_focused
from stripstream.algorithms.search.fast_downward import get_fast_downward
from stripstream.pddl.utils import convert_plan

from manipulation.constants import GRASP_APPROACHES, GRASP_TYPES

from stripstream.pddl.objects import EasyType, EasyParameter
from stripstream.pddl.logic.predicates import EasyPredicate
from stripstream.pddl.operators import Action, Axiom
from stripstream.pddl.logic.quantifiers import ForAll, Exists
from stripstream.pddl.logic.connectives import Not, And, Or
from stripstream.pddl.logic.atoms import Equal
from stripstream.pddl.utils import rename_easy
from stripstream.pddl.problem import STRIPStreamProblem
from stripstream.pddl.cond_streams import EasyListGenStream, EasyTestStream

from stripstream.utils import SEPARATOR, INF


import math
from itertools import product
from random import sample
import inspect
import numpy as np

import redux_tests.planGlobals as glob
import redux_tests.sss_sim as sim
import util.windowManager3D as wm
from geometry.objects2 import World
from util.mmUtil import objectGraspFrameAux, GDesc
from util.miscUtil import Hashable
from geometry.transformations import euler_from_quaternion
from robot.rrt import runRRT, interpolate
from stripstream.utils import flatten

import geometry.hu as hu
import geometry.shapes as shapes

# These are PR2 specific, could be replaced with a different robot
from redux_tests.pr2_test.pr2Robot import makeRobot




Type, Pred, Param = EasyType, EasyPredicate, EasyParameter

####################

PROBLEM = dantam2 # dantam | dantam2
ARM = 'leftarm'
MAX_GRASPS = 1
TOP_GRASPS = True
SIDE_GRASPS = False
DISABLE_TRAJECTORIES = True
DISABLE_TRAJ_COLLISIONS = True

assert not DISABLE_TRAJECTORIES or DISABLE_TRAJ_COLLISIONS
if DISABLE_TRAJECTORIES:
  print 'Warning: trajectories are disabled'
if DISABLE_TRAJ_COLLISIONS:
  print 'Warning: trajectory collisions are disabled'

####################

# Types
OBJ, POSE, GRASP = Type(), Type(), Type()
CONF, TRAJ = Type(), Type()

####################

# Fluents
ConfEq = Pred(CONF)
PoseEq = Pred(OBJ, POSE)
GraspEq = Pred(OBJ, GRASP)
HandEmpty = Pred()

####################

# Derived
SafePose = Pred(OBJ, POSE)
SafeTraj = Pred(OBJ, TRAJ)

####################

# Static trajectory
FreeMotion = Pred(CONF, CONF, TRAJ)
HoldingMotion = Pred(CONF, CONF, GRASP, TRAJ)
GraspMotion = Pred(POSE, GRASP, CONF, TRAJ)

# Static collision
CFreePose = Pred(POSE, POSE)
CFreeTraj = Pred(TRAJ, POSE)

####################

O, P, G = Param(OBJ), Param(POSE), Param(GRASP)
O2, P2 = Param(OBJ), Param(POSE)
Q, Q2 = Param(CONF), Param(CONF)
T = Param(TRAJ)

rename_easy(locals())

####################

actions = [
  Action(name='pick', parameters=[O, P, G, Q, T],
    condition=And(PoseEq(O, P), HandEmpty(), ConfEq(Q), GraspMotion(P, G, Q, T),
                  ForAll([O2], Or(Equal(O, O2), SafeTraj(O2, T)))),
                  #ForAll([O2], Or(Equal(O, O2), And(SafePose(O2, P), SafeGTraj(O2, GT))))),
    effect=And(GraspEq(O, G), Not(HandEmpty()), Not(PoseEq(O, P)))),

  Action(name='place', parameters=[O, P, G, Q, T],
    condition=And(GraspEq(O, G), ConfEq(Q), GraspMotion(P, G, Q, T),
                  ForAll([O2], Or(Equal(O, O2), And(SafePose(O2, P), SafeTraj(O2, T))))),
    effect=And(PoseEq(O, P), HandEmpty(), Not(GraspEq(O, G)))),

  Action(name='move', parameters=[Q, Q2, T],
    condition=And(ConfEq(Q), HandEmpty(), FreeMotion(Q, Q2, T),
                  ForAll([O2], SafeTraj(O2, T))),
    effect=And(ConfEq(Q2), Not(ConfEq(Q)))),

  Action(name='move_holding', parameters=[Q, Q2, T, O, G],
    condition=And(ConfEq(Q), GraspEq(O, G), HoldingMotion(Q, Q2, G, T),
                  ForAll([O2], Or(Equal(O, O2), SafeTraj(O2, T)))),
    effect=And(ConfEq(Q2), Not(ConfEq(Q)))),
]

axioms = [
  Axiom(effect=SafePose(O2, P), condition=Exists([P2], And(PoseEq(O2, P2), CFreePose(P, P2)))),
  Axiom(effect=SafeTraj(O2, T), condition=Exists([P2], And(PoseEq(O2, P2), CFreeTraj(T, P2)))),
]

##################################################

def solve_tamp(viewer):
  world = World()
  robot = makeRobot(np.array(glob.workspace))
  realWorld = sim.RealWorld(world, robot)
  problem = PROBLEM(realWorld)

  #initialize_openrave(env, ARM, min_delta=.01)
  #manipulator = robot.GetActiveManipulator()
  #cspace = CSpace.robot_arm(manipulator)
  #base_manip = interfaces.BaseManipulation(robot, plannername=None, maxvelmult=None)

  #bodies = world.objectShapes
  bodies = world.objects
  print bodies

  all_bodies = bodies.values()
  # NOTE - assuming all objects has the same geometry
  body1 = all_bodies[-1] # Generic object 1
  body2 = all_bodies[-2] if len(bodies) >= 2 else body1 # Generic object 2
  grasps = get_grasps(realWorld, body1, TOP_GRASPS, SIDE_GRASPS)[:MAX_GRASPS]
  poses = problem.known_poses if problem.known_poses else []

  #open_gripper(manipulator)
  initial_conf = Conf(robot.GetConfigurationValues()[manipulator.GetArmIndices()])

  def _enable_all(enable): # Enables or disables all bodies for collision checking
    for body in all_bodies:
      body.Enable(enable)

  ####################

  def cfree_pose(pose1, pose2): # Collision free test between an object at pose1 and an object at pose2
    body1.Enable(True)
    set_pose(body1, pose1.value)
    body2.Enable(True)
    set_pose(body2, pose2.value)
    return not env.CheckCollision(body1, body2)

  def _cfree_traj_pose(traj, pose): # Collision free test between a robot executing traj and an object at pose
    _enable_all(False)
    body2.Enable(True)
    set_pose(body2, pose.value)
    for conf in traj.path():
      set_manipulator_conf(manipulator, conf)
      if env.CheckCollision(robot, body2):
        return False
    return True

  def _cfree_traj_grasp_pose(traj, grasp, pose): # Collision free test between an object held at grasp while executing traj and an object at pose
    _enable_all(False)
    body1.Enable(True)
    body2.Enable(True)
    set_pose(body2, pose.value)
    for conf in traj.path():
      set_manipulator_conf(manipulator, conf)
      manip_trans = manipulator.GetTransform()
      set_pose(body1, object_trans_from_manip_trans(manip_trans, grasp.grasp_trans))
      if env.CheckCollision(body1, body2):
        return False
    return True

  def cfree_traj(traj, pose): # Collision free test between a robot executing traj (which may or may not involve a grasp) and an object at pose
    if DISABLE_TRAJ_COLLISIONS:
      return True
    return _cfree_traj_pose(traj, pose) and (traj.grasp is None or _cfree_traj_grasp_pose(traj, traj.grasp, pose))

  ####################

  def sample_grasp_traj(pose, grasp): # Sample pregrasp config and motion plan that performs a grasp
    _enable_all(False)
    body1.Enable(True)
    set_pose(body1, pose.value)
    manip_trans, approach_vector = manip_from_pose_grasp(pose, grasp)
    grasp_conf = solve_inverse_kinematics(env, manipulator, manip_trans) # Grasp configuration
    if grasp_conf is None: return
    if DISABLE_TRAJECTORIES:
      yield [(Conf(grasp_conf), object())]
      return

    set_manipulator_conf(manipulator, grasp_conf)
    robot.Grab(body1)
    grasp_traj = vector_traj_helper(env, robot, approach_vector) # Trajectory from grasp configuration to pregrasp
    #grasp_traj = workspace_traj_helper(base_manip, approach_vector)
    robot.Release(body1)
    if grasp_traj is None: return
    grasp_traj.grasp = grasp
    pregrasp_conf = Conf(grasp_traj.end()) # Pregrasp configuration
    yield [(pregrasp_conf, grasp_traj)]

  def sample_free_motion(conf1, conf2): # Sample motion while not holding
    if DISABLE_TRAJECTORIES:
      yield [(object(),)] # [(True,)]
      return
    _enable_all(False)
    set_manipulator_conf(manipulator, conf1.value)
    #traj = motion_plan(env, cspace, conf2.value, self_collisions=True)
    traj = cspace_traj_helper(base_manip, cspace, conf2.value, max_iterations=10)
    if not traj: return
    traj.grasp = None
    yield [(traj,)]

  def sample_holding_motion(conf1, conf2, grasp): # Sample motion while holding
    if DISABLE_TRAJECTORIES:
      yield [(object(),)] # [(True,)]
      return
    _enable_all(False)
    body1.Enable(True)
    set_manipulator_conf(manipulator, conf1.value)
    manip_trans = manipulator.GetTransform()
    set_pose(body1, object_trans_from_manip_trans(manip_trans, grasp.grasp_trans))
    robot.Grab(body1)
    #traj = motion_plan(env, cspace, conf2.value, self_collisions=True)
    traj = cspace_traj_helper(base_manip, cspace, conf2.value, max_iterations=10)
    robot.Release(body1)
    if not traj: return
    traj.grasp = grasp
    yield [(traj,)]

  ####################

  cond_streams = [
    # Pick/place trajectory
    EasyListGenStream(inputs=[P, G], outputs=[Q, T], conditions=[],
                      effects=[GraspMotion(P, G, Q, T)], generator=sample_grasp_traj),

    # Move trajectory
    EasyListGenStream(inputs=[Q, Q2], outputs=[T], conditions=[],
                      effects=[FreeMotion(Q, Q2, T)], generator=sample_free_motion, order=1, max_level=0),
    EasyListGenStream(inputs=[Q, Q2, G], outputs=[T], conditions=[],
                      effects=[HoldingMotion(Q, Q2, G, T)], generator=sample_holding_motion, order=1, max_level=0),

    # Collisions
    EasyTestStream(inputs=[P, P2], conditions=[], effects=[CFreePose(P, P2)],
                test=cfree_pose, eager=True),
    EasyTestStream(inputs=[T, P], conditions=[], effects=[CFreeTraj(T, P)],
                test=cfree_traj),
  ]

  ####################

  constants = map(GRASP, grasps) + map(POSE, poses)
  initial_atoms = [
    ConfEq(initial_conf),
    HandEmpty(),
  ] + [
    PoseEq(obj, pose) for obj, pose in problem.initial_poses.iteritems()
  ]
  goal_formula = And(ConfEq(initial_conf), *(PoseEq(obj, pose) for obj, pose in problem.goal_poses.iteritems()))
  stream_problem = STRIPStreamProblem(initial_atoms, goal_formula, actions + axioms, cond_streams, constants)

  if viewer:
    env.UpdatePublishedBodies()
    raw_input('Start?')
  search_fn = get_fast_downward('eager', max_time=10)
  solve = lambda: incremental_planner(stream_problem, search=search_fn, frequency=1, waves=True, debug=False)
  #solve = lambda: simple_focused(stream_problem, search=search_fn, max_level=INF, shared=False, debug=False, verbose=False, max_time=15)
  env.Lock()
  plan, universe = solve()
  env.Unlock()

  print SEPARATOR

  #universe.print_statistics()
  plan = convert_plan(plan)
  if plan is not None:
    print 'Success'
    for i, (action, args) in enumerate(plan):
      print i+1, action, args
  else:
    print 'Failure'

  ####################

  def _execute_traj(traj):
    #for j, conf in enumerate(traj.path()):
    #for j, conf in enumerate([traj.end()]):
    path = list(sample_manipulator_trajectory(manipulator, traj.traj()))
    for j, conf in enumerate(path):
      set_manipulator_conf(manipulator, conf)
      env.UpdatePublishedBodies()
      raw_input('%s/%s) Step?'%(j, len(path)))

  if viewer and plan is not None:
    print SEPARATOR
    # Resets the initial state
    #set_manipulator_conf(manipulator, initial_conf.value)
    set_manipulator_conf(manipulator, initial_conf)
    for obj, pose in problem.initial_poses.iteritems():
      set_pose(bodies[obj], pose.value)
    env.UpdatePublishedBodies()

    if not DISABLE_TRAJECTORIES:
      for i, (action, args) in enumerate(plan):
        raw_input('\n%s/%s) Next?'%(i, len(plan)))
        if action.name == 'move':
          _, _, traj = args
          _execute_traj(traj)
        elif action.name == 'move_holding':
          _, _, traj, _, _ = args
          _execute_traj(traj)
        elif action.name == 'pick':
          obj, _, _, _, traj = args
          _execute_traj(traj.reverse())
          robot.Grab(bodies[obj])
          _execute_traj(traj)
        elif action.name == 'place':
          obj, _, _, _, traj = args
          _execute_traj(traj.reverse())
          robot.Release(bodies[obj])
          _execute_traj(traj)
        else:
          raise ValueError(action.name)
        env.UpdatePublishedBodies()
    else:
      for i, (action, args) in enumerate(plan):
        raw_input('\n%s/%s) Next?'%(i, len(plan)))
        if action.name == 'move':
          _, q2, _ = args
          set_manipulator_conf(manipulator, q2)
        elif action.name == 'move_holding':
          _, q2, _, _, _ = args
          set_manipulator_conf(manipulator, q2)
        elif action.name == 'pick':
          obj, _, _, _, traj = args
          robot.Grab(bodies[obj])
        elif action.name == 'place':
          obj, _, _, _, traj = args
          robot.Release(bodies[obj])
        else:
          raise ValueError(action.name)
        env.UpdatePublishedBodies()

  raw_input('Finish?')

##################################################

def main(argv):
  parser = argparse.ArgumentParser() # Automatically includes help
  parser.add_argument('-viewer', action='store_true', help='enable viewer.')
  args = parser.parse_args()
  solve_tamp(args.viewer)
  print 'Done!'

if __name__ == '__main__':
  main(sys.argv[1:])
