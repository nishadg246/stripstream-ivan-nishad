#!/usr/bin/env python2

import argparse
import sys

from openravepy import Environment, RaveDestroy

from robotics.openrave.movable_base_utils import pose_generator_fn, visualize_solution, \
    get_stable_test, sample_grasp_traj_fn
from robotics.openrave.problems import cylinder, soup, TOP_HOLDING_LEFT_ARM, \
    SIDE_HOLDING_LEFT_ARM
from robotics.openrave.push_utils import *
from robotics.openrave.transforms import get_pose
from robotics.openrave.utils import open_gripper, \
    initialize_openrave, execute_viewer, draw_affine_limits, EnvironmentStateSaver
from stripstream.algorithms.incremental.incremental_planner import incremental_planner
from stripstream.algorithms.search.fast_downward import get_fast_downward
from stripstream.pddl.cond_streams import EasyListGenStream, \
  EasyListFnStream, EasyTestStream, RawListGenStream
from stripstream.pddl.logic.atoms import Equal
from stripstream.pddl.logic.connectives import Not, And, Or
from stripstream.pddl.logic.quantifiers import ForAll, Exists
from stripstream.pddl.operators import Action, Axiom
from stripstream.pddl.problem import STRIPStreamProblem
from stripstream.pddl.utils import convert_plan
from stripstream.utils import SEPARATOR

# To start with, allow all combos of poses
# Discard ones that are not on the same table

####################

#PROBLEM = lambda env: simple(env, 1)
PROBLEM = soup # soup | cylinder
ARM = 'leftarm'
TOP_GRASPS = False
SIDE_GRASPS = True
MIN_DELTA = 0.01

assert TOP_GRASPS != SIDE_GRASPS
if TOP_GRASPS:
  CARRY_CONFIG = TOP_HOLDING_LEFT_ARM
if SIDE_GRASPS:
  CARRY_CONFIG = SIDE_HOLDING_LEFT_ARM

####################



# TODO: plate and item types
# Plate type is rotationally invariant (to start with)
# Can push bottoms of objects and will conform to the correct direction (if heavy enough)
# Plate type can only be picked on edges
# Can also implicitly realize this using the lack of grasps
# Suppose you could only place on edges perpendicular to them
# Would need to sample poses and grasps together


####################

# NOTE - I suppose I could include O in one trajectory's collision and not the other. However, no point because can't move that object to manipulate

actions = [
  Action(name='pick', parameters=[O, P, G, Q, T],
         condition=And(PointEq(O, P), HandEmpty(), ConfEq(Q), GraspMotion(O, P, G, Q, T)),
                       #ForAll([O2], Or(Equal(O, O2), SafeTraj(O2, T)))),
         effect=And(Holding(O), Not(HandEmpty()), Not(PointEq(O, P)))),
    #condition=And(PoseEq(O, P), HandEmpty(), ConfEq(Q), GraspMotion(O, P, G, Q, T, T2),
    #              ForAll([O2], Or(Equal(O, O2), And(SafeTraj(O2, T), SafeTraj(O2, T2))))),
    #effect=And(GraspEq(O, G), Not(HandEmpty()), Not(PoseEq(O, P)))),

  Action(name='place', parameters=[O, P, G, Q, T],
         condition=And(Holding(O), ConfEq(Q), GraspMotion(O, P, G, Q, T)),
                  #ForAll([O2], Or(Equal(O, O2), And(SafePose(O2, P), SafeTraj(O2, T))))),
                  #ForAll([O2], Or(Equal(O, O2), SafeTraj(O2, T)))),
         effect=And(PointEq(O, P), HandEmpty(), Not(Holding(O)))),

  Action(name='move', parameters=[Q, Q2],
         condition=And(ConfEq(Q)),
         effect=And(ConfEq(Q2), Not(ConfEq(Q)))),

  Action(name='push', parameters=[O, P, P2, Q, T],
         condition=And(PointEq(O, P), ConfEq(Q), PushMotion(O, P, P2, Q, T)),
         effect=And(PointEq(O, P2), Not(PointEq(O, P)))),
]

axioms = [
  #Axiom(effect=SafePose(O2, P), condition=Exists([P2], And(PointEq(O2, P2), CFreePose(P, P2)))),
  #Axiom(effect=SafeTraj(O2, T), condition=Exists([P2], And(PointEq(O2, P2), CFreeTraj(T, P2)))),
  Axiom(effect=OnSurface(O, S), condition=Exists([P], And(PointEq(O, P), Stable(P, S), IsPose(O, P)))),
]

##################################################

def solve_tamp(env):
  use_viewer = env.GetViewer() is not None
  problem = PROBLEM(env)

  robot, manipulator, base_manip, ir_model = initialize_openrave(env, ARM, min_delta=MIN_DELTA)
  set_manipulator_conf(ir_model.manip, CARRY_CONFIG)
  bodies = {obj: env.GetKinBody(obj) for obj in problem.movable_names}
  surfaces = {surface.name: surface for surface in problem.surfaces}
  open_gripper(manipulator)
  initial_conf = Conf(robot.GetConfigurationValues())
  initial_poses = {name: Pose(get_pose(body)) for name, body in bodies.iteritems()}
  saver = EnvironmentStateSaver(env)

  stable_test = get_stable_test(bodies, surfaces)

  ####################

  cond_streams = [
    # # Pick/place trajectory
    EasyListGenStream(inputs=[O, P, G], outputs=[Q, T], conditions=[IsPose(O, P), IsGrasp(O, G)],
                      effects=[GraspMotion(O, P, G, Q, T)],
                      generator=sample_grasp_traj_fn(base_manip, ir_model, bodies, CARRY_CONFIG, max_failures=200)),
    #
    # # Move trajectory
    #MultiEasyGenStream(inputs=[Q, Q2], outputs=[T], conditions=[],
    #             effects=[FreeMotion(Q, Q2, T)], generator=sample_free_motion, order=1, max_level=0),
    #MultiEasyGenStream(inputs=[Q, Q2, O, G], outputs=[T], conditions=[IsGrasp(O, G)],
    #             effects=[HoldingMotion(Q, Q2, G, T)], generator=sample_holding_motion, order=1, max_level=0),
    #
    # # Collisions
    EasyTestStream(inputs=[O, P, O2, P2], conditions=[IsPose(O, P), IsPose(O2, P2)],
                   effects=[CFreePose(P, P2)],
                   test=lambda *args: True,
                   eager=True),
    EasyTestStream(inputs=[T, P], conditions=[], effects=[CFreeTraj(T, P)],
                   test=lambda *args: True),

    EasyListFnStream(inputs=[O], outputs=[G], conditions=[],
                      effects=[IsGrasp(O, G)],
                      function=grasp_generator_fn(bodies)),

    #EasyListGenStream(inputs=[O, S], outputs=[P], conditions=[],
    #                  effects=[IsPose(O, P), Stable(P, S)],
    #                  generator=pose_generator_fn(bodies, surfaces)),

    #EasyListGenStream(inputs=[O, S], outputs=[P], conditions=[],
    #                  effects=[IsPose(O, P), Stable(P, S), IsEdge(P)],
    #                  generator=edge_generator_fn(bodies, surfaces)),

    # TODO: remove O from here
    RawListGenStream(inputs=[O, P, P2], outputs=[Q, T],
                      #conditions=[IsPushable(O), IsPose(O, P), IsPose(O, P2), Stable(P, S), Stable(P2, S)],
                      conditions=[IsPose(O, P), IsPose(O, P2)],
                      effects=[PushMotion(O, P, P2, Q, T)],
                      generator=sample_push_traj_fn(ir_model, bodies, surfaces, CARRY_CONFIG)),

    # TODO: could have fixed obstacle blocking a push
    #MultiEasyGenStream(inputs=[O, S, P], outputs=[P2],
    #                 conditions=[IsPose(O, P), Stable(P, S)],
    #                 effects=[PushMotion(O, P, P2, Q, T)],
    #                 generator=edge_generator_fn(bodies, surfaces)),

    #EasyGenStream(inputs=[O, P, G], outputs=[Q], conditions=[IsPose(O, P), IsGrasp(O, G)],
    #            effects=[], generator=base_generator),
  ]

  ####################

  constants = []
  initial_atoms = [
    ConfEq(initial_conf),
    HandEmpty(),
  ] + [
    PointEq(name, pose) for name, pose in initial_poses.iteritems()
  ] + [
    IsPose(name, pose) for name, pose in initial_poses.iteritems()
  ] + [
    Stable(pose, surface) for obj, pose in initial_poses.iteritems() for surface in surfaces if stable_test(obj, pose, surface)
  ]

  #goal_formula = And(ConfEq(initial_conf), *[
  #  OnSurface(obj, surface) for obj, surface in problem.goal_surfaces.iteritems()
  #])
  #goal_formula = ConfEq(goal_conf)
  obj, _ = problem.goal_surfaces.items()[0]
  goal_formula = And(Holding(obj))

  stream_problem = STRIPStreamProblem(initial_atoms, goal_formula, actions + axioms, cond_streams, constants)

  print stream_problem
  handles = draw_affine_limits(robot)
  if use_viewer:
    for surface in problem.surfaces:
      surface.draw(env)
    raw_input('Start?')

  search_fn = get_fast_downward('eager', max_time=10, verbose=False)
  solve = lambda: incremental_planner(stream_problem, search=search_fn, frequency=1, waves=True, debug=False)
  #solve = lambda: simple_focused(stream_problem, search=search_fn, max_level=INF, shared=False, debug=False, verbose=False, max_time=15)
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
      visualize_solution(commands)
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