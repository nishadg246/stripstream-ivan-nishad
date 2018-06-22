#!/usr/bin/env python2

import argparse
import sys

from openravepy import Environment, RaveDestroy

from robotics.openrave.problems import simple, TOP_HOLDING_LEFT_ARM, \
  SIDE_HOLDING_LEFT_ARM
from robotics.openrave.utils import open_gripper, \
  initialize_openrave, execute_viewer, EnvironmentStateSaver, set_manipulator_conf, draw_affine_limits
from robotics.openrave.transforms import get_pose
from robotics.openrave.movable_base_utils import Conf, Pose, cfree_pose_fn, sample_free_base_motion_fn, \
  visualize_solution, grasp_generator_fn, pose_generator_fn, \
  get_stable_test, sample_grasp_traj_fn, process_plan

from stripstream.algorithms.incremental.incremental_planner import incremental_planner
from stripstream.algorithms.focused.simple_focused import simple_focused

from stripstream.algorithms.search.fast_downward import get_fast_downward
from stripstream.pddl.utils import convert_plan
from stripstream.pddl.objects import EasyType, EasyParameter
from stripstream.pddl.logic.predicates import EasyPredicate
from stripstream.pddl.operators import Action, Axiom
from stripstream.pddl.logic.quantifiers import ForAll, Exists
from stripstream.pddl.logic.connectives import Not, And, Or
from stripstream.pddl.logic.atoms import Equal
from stripstream.pddl.utils import rename_easy
from stripstream.pddl.problem import STRIPStreamProblem
from stripstream.pddl.cond_streams import EasyListGenStream, EasyTestStream, EasyGenStream
from stripstream.utils import SEPARATOR, INF

Type, Pred, Param = EasyType, EasyPredicate, EasyParameter

####################

#PROBLEM = lambda env: simple(env, 1)
PROBLEM = simple
ARM = 'leftarm'
TOP_GRASPS = False
SIDE_GRASPS = True
MIN_DELTA = 0.01
#MIN_DELTA = 0.02

assert TOP_GRASPS != SIDE_GRASPS
if TOP_GRASPS:
  MAX_GRASPS = 1
  CARRY_CONFIG = TOP_HOLDING_LEFT_ARM
if SIDE_GRASPS:
  MAX_GRASPS = 4
  CARRY_CONFIG = SIDE_HOLDING_LEFT_ARM

####################

# Types
OBJ, POSE, GRASP = Type(), Type(), Type()
CONF, TRAJ, SURFACE = Type(), Type(), Type()
#TRANS = Type()


# Fluents
ConfEq = Pred(CONF)
PoseEq = Pred(OBJ, POSE)
GraspEq = Pred(OBJ, GRASP)
HandEmpty = Pred()

# Derived
SafePose = Pred(OBJ, POSE)
SafeTraj = Pred(OBJ, TRAJ)
OnSurface = Pred(OBJ, SURFACE)
Holding = Pred(OBJ)

# Static trajectory
FreeMotion = Pred(CONF, CONF, TRAJ)
HoldingMotion = Pred(CONF, CONF, GRASP, TRAJ)
GraspMotion = Pred(OBJ, POSE, GRASP, CONF, TRAJ)
#GraspMotion = Pred(OBJ, POSE, GRASP, CONF, TRAJ, TRAJ)

# Static collision
CFreePose = Pred(POSE, POSE)
CFreeTraj = Pred(TRAJ, POSE)

IsPushable = Pred(OBJ)
IsPose = Pred(OBJ, POSE)
IsGrasp = Pred(OBJ, GRASP)
Stable = Pred(POSE, SURFACE)

####################

O, P, G = Param(OBJ), Param(POSE), Param(GRASP)
O2, P2 = Param(OBJ), Param(POSE)
Q, Q2 = Param(CONF), Param(CONF)
T = Param(TRAJ)
T2 = Param(TRAJ)
S = Param(SURFACE)

rename_easy(locals())

####################

# NOTE - I suppose I could include O in one trajectory's collision and not the other. However, no point because can't move that object to manipulate

actions = [
  Action(name='pick', parameters=[O, P, G, Q, T],
    condition=And(PoseEq(O, P), HandEmpty(), ConfEq(Q), GraspMotion(O, P, G, Q, T)),
                  #ForAll([O2], Or(Equal(O, O2), SafeTraj(O2, T)))),
    effect=And(GraspEq(O, G), Not(HandEmpty()), Not(PoseEq(O, P)))),
    #condition=And(PoseEq(O, P), HandEmpty(), ConfEq(Q), GraspMotion(O, P, G, Q, T, T2),
    #              ForAll([O2], Or(Equal(O, O2), And(SafeTraj(O2, T), SafeTraj(O2, T2))))),
    #effect=And(GraspEq(O, G), Not(HandEmpty()), Not(PoseEq(O, P)))),

  Action(name='place', parameters=[O, P, G, Q, T],
    condition=And(GraspEq(O, G), ConfEq(Q), GraspMotion(O, P, G, Q, T)),
                  #ForAll([O2], Or(Equal(O, O2), And(SafePose(O2, P), SafeTraj(O2, T))))),
                  #ForAll([O2], Or(Equal(O, O2), SafeTraj(O2, T)))),
  effect=And(PoseEq(O, P), HandEmpty(), Not(GraspEq(O, G)))),

  Action(name='move', parameters=[Q, Q2, T],
         condition=And(ConfEq(Q), FreeMotion(Q, Q2, T)),
         effect=And(ConfEq(Q2), Not(ConfEq(Q)))),

  # TODO: cost split into linear and total pieces such that can evaluate the eager stuff first

  #Action(name='move', parameters=[Q, Q2, T],
  #  condition=And(ConfEq(Q), HandEmpty(), FreeMotion(Q, Q2, T),
  #                ForAll([O2], SafeTraj(O2, T))),
  #  effect=And(ConfEq(Q2), Not(ConfEq(Q)))),

  #Action(name='move_holding', parameters=[Q, Q2, T, O, G],
  #  condition=And(ConfEq(Q), GraspEq(O, G), HoldingMotion(Q, Q2, G, T),
  #                ForAll([O2], Or(Equal(O, O2), SafeTraj(O2, T)))),
  #  effect=And(ConfEq(Q2), Not(ConfEq(Q)))),
]

axioms = [
  #Axiom(effect=SafePose(O2, P), condition=Exists([P2], And(PoseEq(O2, P2), CFreePose(P, P2)))),
  #Axiom(effect=SafeTraj(O2, T), condition=Exists([P2], And(PoseEq(O2, P2), CFreeTraj(T, P2)))),
  Axiom(effect=OnSurface(O, S), condition=Exists([P], And(PoseEq(O, P), Stable(P, S), IsPose(O, P)))),
  Axiom(effect=Holding(O), condition=Exists([G], And(GraspEq(O, G), IsGrasp(O, G)))),
]

##################################################

def body_initial_atoms(name, initial_poses, bodies, surfaces):
  stable_test = get_stable_test(bodies, surfaces)
  pose = initial_poses[name]
  return [PoseEq(name, pose), IsPose(name, pose)] + [
           Stable(pose, surface) for surface in surfaces if stable_test(name, pose, surface)]

def solve_tamp(env, use_focused):
  use_viewer = env.GetViewer() is not None
  problem = PROBLEM(env)
  # TODO: most of my examples have literally had straight-line motions plans

  robot, manipulator, base_manip, ir_model = initialize_openrave(env, ARM, min_delta=MIN_DELTA)
  set_manipulator_conf(ir_model.manip, CARRY_CONFIG)
  bodies = {obj: env.GetKinBody(obj) for obj in problem.movable_names}
  surfaces = {surface.name: surface for surface in problem.surfaces}
  open_gripper(manipulator)
  initial_q = Conf(robot.GetConfigurationValues())
  initial_poses = {name: Pose(name, get_pose(body)) for name, body in bodies.iteritems()}
  # TODO: just keep track of the movable degrees of freedom
  # GetActiveDOFIndices, GetActiveJointIndices, GetActiveDOFValues
  saver = EnvironmentStateSaver(env)

  #cfree_pose = cfree_pose_fn(env, bodies)
  #cfree_traj = cfree_traj_fn(env, robot, manipulator, body1, body2, all_bodies)

  #base_generator = base_generator_fn(ir_model)

  #base_values = base_values_from_full_config(initial_q.value)
  #goal_values = full_config_from_base_values(base_values + np.array([1, 0, 0]), initial_q.value)
  #goal_conf = Conf(goal_values)
  #return

  ####################

  # TODO - should objects contain their geometry

  cond_streams = [
    EasyListGenStream(inputs=[O, P, G], outputs=[Q, T], conditions=[IsPose(O, P), IsGrasp(O, G)],
                      effects=[GraspMotion(O, P, G, Q, T)],
                      generator=sample_grasp_traj_fn(base_manip, ir_model, bodies, CARRY_CONFIG)),

    EasyGenStream(inputs=[Q, Q2], outputs=[T], conditions=[], effects=[FreeMotion(Q, Q2, T)],
                  generator=sample_free_base_motion_fn(base_manip, bodies), order=1, max_level=0),

    EasyTestStream(inputs=[O, P, O2, P2], conditions=[IsPose(O, P), IsPose(O2, P2)],
                   effects=[CFreePose(P, P2)],
                   test=lambda *args: True, #cfree_pose,
                   eager=True),
    EasyTestStream(inputs=[T, P], conditions=[], effects=[CFreeTraj(T, P)],
                   test=lambda *args: True),
    #test=cfree_traj),

    EasyListGenStream(inputs=[O], outputs=[G], conditions=[],
                      effects=[IsGrasp(O, G)],
                      generator=grasp_generator_fn(bodies, TOP_GRASPS, SIDE_GRASPS, MAX_GRASPS)),
    EasyListGenStream(inputs=[O, S], outputs=[P], conditions=[],
                effects=[IsPose(O, P), Stable(P, S)],
                generator=pose_generator_fn(bodies, surfaces)),

    #EasyGenStream(inputs=[O, P, G], outputs=[Q], conditions=[IsPose(O, P), IsGrasp(O, G)],
    #            effects=[], generator=base_generator),
  ]

  ####################

  constants = []
  initial_atoms = [
    ConfEq(initial_q),
    HandEmpty()]
  for name in initial_poses:
    initial_atoms += body_initial_atoms(name, initial_poses, bodies, surfaces)

  goal_formula = And(ConfEq(initial_q), *[
    OnSurface(obj, surface) for obj, surface in problem.goal_surfaces.iteritems()
  ])
  #goal_formula = ConfEq(goal_conf)
  #obj, _ = problem.goal_surfaces.items()[0]
  #goal_formula = And(Holding(obj))
  #goal_formula = Holding(obj) # TODO: this cause a bug

  stream_problem = STRIPStreamProblem(initial_atoms, goal_formula, actions + axioms, cond_streams, constants)

  print stream_problem
  handles = draw_affine_limits(robot)
  if use_viewer:
    for surface in problem.surfaces:
      surface.draw(env)
    raw_input('Start?')

  max_time = INF
  search_fn = get_fast_downward('eager', max_time=10, verbose=False)
  if use_focused:
    solve = lambda: simple_focused(stream_problem, search=search_fn, max_level=INF,
                                   shared=False, debug=False, verbose=False, max_time=max_time)
  else:
    solve = lambda: incremental_planner(stream_problem, search=search_fn,
     frequency=10, waves=False, debug=False, max_time=max_time)
  with env:
    plan, universe = solve()

  print SEPARATOR

  plan = convert_plan(plan)
  if plan is not None:
    print 'Success'
    for i, (action, args) in enumerate(plan):
      print i+1, action, args
  else:
    print 'Failure'

  ####################

  if plan is not None:
    commands = process_plan(robot, bodies, plan)
    if use_viewer :
      print SEPARATOR
      saver.Restore()
      visualize_solution(commands, step=False)
  raw_input('Finish?')

##################################################

def main(_):
  parser = argparse.ArgumentParser()
  parser.add_argument('-focus', action='store_true', help='use focused algorithm.')
  parser.add_argument('-viewer', action='store_true', help='enable viewer.')
  args = parser.parse_args()

  env = Environment()
  try:
    execute = lambda: solve_tamp(env, args.focus)
    if args.viewer: execute_viewer(env, execute)
    else: execute()
  finally:
    if env.GetViewer() is not None:
      env.GetViewer().quitmainloop()
    RaveDestroy()
  print 'Done!'

if __name__ == '__main__':
  main(sys.argv[1:])