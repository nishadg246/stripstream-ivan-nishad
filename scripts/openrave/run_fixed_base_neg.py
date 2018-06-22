#!/usr/bin/env python2

import argparse
import sys

from openravepy import Environment, RaveDestroy

#from stripstream.algorithms.incremental.incremental_planner import incremental_planner
from stripstream.algorithms.focused.signed_focused import signed_focused
from stripstream.algorithms.search.fast_downward import get_fast_downward

from stripstream.pddl.utils import convert_plan
from robotics.openrave.problems import dantam_distract
from robotics.openrave.utils import open_gripper, \
  Conf, initialize_openrave, execute_viewer
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

from robotics.openrave.tamp_fixed_base import cfree_pose_fn, cfree_traj_fn, \
  sample_grasp_traj_fn, sample_free_motion_fn, sample_holding_motion_fn, visualize_solution

Type, Pred, Param = EasyType, EasyPredicate, EasyParameter

####################

PROBLEM = lambda env: dantam_distract(env, 8)
ARM = 'leftarm'
MAX_GRASPS = 1

####################

# Types
OBJ, POSE, GRASP = Type(), Type(), Type()
CONF, TRAJ = Type(), Type()

# Fluents
ConfEq = Pred(CONF)
PoseEq = Pred(OBJ, POSE)
GraspEq = Pred(OBJ, GRASP)
HandEmpty = Pred()

# Derived
UnsafePose = Pred(OBJ, POSE)
UnsafeTraj = Pred(OBJ, TRAJ)

# Static trajectory
FreeMotion = Pred(CONF, CONF, TRAJ)
HoldingMotion = Pred(CONF, CONF, GRASP, TRAJ)
GraspMotion = Pred(POSE, GRASP, CONF, TRAJ)

# Static collision
CPose = Pred(POSE, POSE)
CTraj = Pred(TRAJ, POSE)

O, P, G = Param(OBJ), Param(POSE), Param(GRASP)
O2, P2 = Param(OBJ), Param(POSE)
Q, Q2 = Param(CONF), Param(CONF)
T = Param(TRAJ)

rename_easy(locals())

####################

# 8 * (9*9 + 2*9)
actions = [
  Action(name='pick', parameters=[O, P, G, Q, T],
         condition=And(PoseEq(O, P), HandEmpty(), ConfEq(Q), GraspMotion(P, G, Q, T),
                       ForAll([O2], Or(Equal(O, O2), Not(UnsafeTraj(O2, T))))),
    effect=And(GraspEq(O, G), Not(HandEmpty()), Not(PoseEq(O, P)))),

  # TODO: maybe the problem results from how FastDownward makes axioms for universal quantifiers
  Action(name='place', parameters=[O, P, G, Q, T],
         condition=And(GraspEq(O, G), ConfEq(Q), GraspMotion(P, G, Q, T),
                       #ForAll([O2], Not(Or(UnsafePose(O2, P), UnsafeTraj(O2, T))))), # Already faster
                       ForAll([O2], And(Not(UnsafePose(O2, P)), Not(UnsafeTraj(O2, T))))), # Already faster
                       #ForAll([O2], Or(Equal(O, O2), And(Not(UnsafePose(O2, P)), Not(UnsafeTraj(O2, T)))))),  # Bad! Slow!
                       #ForAll([O2], Not(UnsafePose(O2, P)))),
                       #*[And(Not(UnsafePose(o, P)), Not(UnsafeTraj(o, T))) for ... ]
                       #ForAll([O2], Or(Equal(O, O2), Not(Or(UnsafePose(O2, P), UnsafeTraj(O2, T)))))), # Bad! Slow!
    effect=And(PoseEq(O, P), HandEmpty(), Not(GraspEq(O, G)))),

  #Action(name='move', parameters=[Q, Q2],
  #       condition=ConfEq(Q),
  #       effect=And(ConfEq(Q2), Not(ConfEq(Q)))),

  Action(name='move', parameters=[Q, Q2, T],
         condition=And(ConfEq(Q), HandEmpty(), FreeMotion(Q, Q2, T),
                       ForAll([O2], Not(UnsafeTraj(O2, T)))),
         effect=And(ConfEq(Q2), Not(ConfEq(Q)))),

  Action(name='move_holding', parameters=[Q, Q2, T, O, G],
         condition=And(ConfEq(Q), GraspEq(O, G), HoldingMotion(Q, Q2, G, T),
                       ForAll([O2], Or(Equal(O, O2), Not(UnsafeTraj(O2, T))))),
         effect=And(ConfEq(Q2), Not(ConfEq(Q)))),
]

# 9 * 9 * (2*9*9 + 2*9) = 14580
axioms = [
  Axiom(effect=UnsafePose(O2, P), condition=Exists([P2], And(PoseEq(O2, P2), CPose(P, P2)))),
  Axiom(effect=UnsafeTraj(O2, T), condition=Exists([P2], And(PoseEq(O2, P2), CTraj(T, P2)))),
]

##################################################

def solve_tamp(env):
  viewer = env.GetViewer() is not None
  problem = PROBLEM(env)

  robot, manipulator, base_manip, _ = initialize_openrave(env, ARM, min_delta=.01)
  bodies = {obj: env.GetKinBody(obj) for obj in problem.object_names}
  all_bodies = bodies.values()
  assert len({body.GetKinematicsGeometryHash() for body in all_bodies}) == 1 # Assuming all objects has the same geometry
  body1 = all_bodies[-1] # Generic object 1
  body2 = all_bodies[-2] if len(bodies) >= 2 else body1 # Generic object 2
  grasps = problem.known_grasps[:MAX_GRASPS] if problem.known_grasps else []
  poses = problem.known_poses if problem.known_poses else []

  open_gripper(manipulator)
  initial_conf = Conf(robot.GetConfigurationValues()[manipulator.GetArmIndices()])

  ####################

  cfree_pose = cfree_pose_fn(env, body1, body2)
  cfree_traj = cfree_traj_fn(env, manipulator, body1, body2, all_bodies)

  cond_streams = [
    # Pick/place trajectory
    EasyListGenStream(inputs=[P, G], outputs=[Q, T], conditions=[],
                      effects=[GraspMotion(P, G, Q, T)],
                      generator=sample_grasp_traj_fn(env, manipulator, body1, all_bodies)),

    # TODO: these slow down as soon as I enable them for some reason... Is this because not shared?
    # Could also because share type with other things?
    # Move trajectory
    EasyListGenStream(inputs=[Q, Q2], outputs=[T], conditions=[],
                      effects=[FreeMotion(Q, Q2, T)],
                      generator=sample_free_motion_fn(manipulator, base_manip, all_bodies)),
    EasyListGenStream(inputs=[Q, Q2, G], outputs=[T], conditions=[],
                      effects=[HoldingMotion(Q, Q2, G, T)],
                      generator=sample_holding_motion_fn(manipulator, base_manip, body1, all_bodies)),

    # TODO: as soon as collisions or trajectories are enabled, it is slow
    # Collisions
    EasyTestStream(inputs=[P, P2], conditions=[], effects=[CPose(P, P2)],
                   test=lambda *args: not cfree_pose(*args), eager=True, sign=False),
    EasyTestStream(inputs=[T, P], conditions=[], effects=[CTraj(T, P)],
                   test=lambda *args: not cfree_traj(*args), sign=False),
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
  search_fn = get_fast_downward('eager', max_time=10, verbose=True)
  #solve = lambda: incremental_planner(stream_problem, search=search_fn, frequency=1, waves=True, debug=False)
  solve = lambda: signed_focused(stream_problem, search=search_fn, verbose=True)
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

if __name__ == '__main__':
  main(sys.argv[1:])