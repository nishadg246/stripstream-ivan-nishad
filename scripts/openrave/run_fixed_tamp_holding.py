#!/usr/bin/env python

import argparse
import sys
import math

from openravepy import Environment, RaveDestroy, interfaces

from manipulation.visualizations import execute_viewer
from stripstream.algorithms.incremental.incremental_planner import incremental_planner
from stripstream.algorithms.search.fast_downward import get_fast_downward
from stripstream.pddl.utils import convert_plan
from manipulation.independent import get_grasps
from manipulation.problems.distribution import dantam_3
from manipulation.constants import GRASP_APPROACHES, GRASP_TYPES
from manipulation.motion.cspace import CSpace
from stripstream.pddl.examples.openrave.utils import open_gripper, set_base_conf, Conf, initialize_openrave
from stripstream.pddl.objects import EasyType, EasyParameter
from stripstream.pddl.logic.predicates import EasyPredicate
from stripstream.pddl.operators import Action, Axiom
from stripstream.pddl.logic.quantifiers import ForAll, Exists
from stripstream.pddl.logic.connectives import Not, And, Or
from stripstream.pddl.logic.atoms import Equal
from stripstream.pddl.utils import rename_easy
from stripstream.pddl.problem import STRIPStreamProblem
from stripstream.pddl.cond_streams import EasyListGenStream, EasyTestStream
from stripstream.utils import SEPARATOR
from robotics.openrave.fixed_tamp_holding import cfree_pose_fn, cfree_traj_fn, \
  sample_grasp_traj_fn, sample_free_motion_fn, sample_holding_motion_fn, visualize_solution

Type, Pred, Param = EasyType, EasyPredicate, EasyParameter

####################

PROBLEM = dantam_3 # dantam | dantam2
ARM = 'leftarm'
MAX_GRASPS = 1
USE_GRASP_APPROACH = GRASP_APPROACHES.TOP # TOP | SIDE
USE_GRASP_TYPE = GRASP_TYPES.GRASP # GRASP | TOUCH

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

def solve_tamp(env):
  viewer = env.GetViewer() is not None
  problem = PROBLEM(env)

  robot = env.GetRobots()[0]
  set_base_conf(robot, (-.75, .2, -math.pi/2))
  initialize_openrave(env, ARM, min_delta=.01)
  manipulator = robot.GetActiveManipulator()
  cspace = CSpace.robot_arm(manipulator)
  base_manip = interfaces.BaseManipulation(robot, plannername=None, maxvelmult=None)

  bodies = {obj: env.GetKinBody(obj) for obj in problem.object_names}
  all_bodies = bodies.values()
  assert len({body.GetKinematicsGeometryHash() for body in all_bodies}) == 1 # NOTE - assuming all objects has the same geometry
  body1 = all_bodies[-1] # Generic object 1
  body2 = all_bodies[-2] if len(bodies) >= 2 else body1 # Generic object 2
  grasps = get_grasps(env, robot, body1, USE_GRASP_APPROACH, USE_GRASP_TYPE)[:MAX_GRASPS]
  poses = problem.known_poses if problem.known_poses else []

  open_gripper(manipulator)
  initial_conf = Conf(robot.GetConfigurationValues()[manipulator.GetArmIndices()])

  ####################

  cond_streams = [
    # Pick/place trajectory
    EasyListGenStream(inputs=[P, G], outputs=[Q, T], conditions=[],
                      effects=[GraspMotion(P, G, Q, T)],
                      generator=sample_grasp_traj_fn(env, robot, manipulator, body1, all_bodies)),

    # Move trajectory
    EasyListGenStream(inputs=[Q, Q2], outputs=[T], conditions=[],
                      effects=[FreeMotion(Q, Q2, T)],
                      generator=sample_free_motion_fn(manipulator, base_manip, cspace, all_bodies), order=1, max_level=0),
    EasyListGenStream(inputs=[Q, Q2, G], outputs=[T], conditions=[],
                      effects=[HoldingMotion(Q, Q2, G, T)],
                      generator=sample_holding_motion_fn(robot, manipulator, base_manip, cspace, body1, all_bodies), order=1, max_level=0),

    # Collisions
    EasyTestStream(inputs=[P, P2], conditions=[], effects=[CFreePose(P, P2)],
                test=cfree_pose_fn(env, body1, body2), eager=True),
    EasyTestStream(inputs=[T, P], conditions=[], effects=[CFreeTraj(T, P)],
                test=cfree_traj_fn(env, robot, manipulator, body1, body2, all_bodies)),
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

  if viewer: raw_input('Start?')
  search_fn = get_fast_downward('eager', max_time=10, verbose=False)
  solve = lambda: incremental_planner(stream_problem, search=search_fn, frequency=1, waves=True, debug=False)
  #solve = lambda: simple_focused(stream_problem, search=search_fn, max_level=INF, shared=False, debug=False, verbose=False, max_time=15)
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