#!/usr/bin/env python

import argparse
import sys
import math

from openravepy import RaveSetDebugLevel, DebugLevel, Environment, RaveDestroy, databases, interfaces
from manipulation.visualizations import execute_viewer
from stripstream.algorithms.incremental.incremental_planner import incremental_planner
from stripstream.algorithms.focused.focused_planner import focused_planner
from stripstream.algorithms.focused.simple_focused import simple_focused
from stripstream.algorithms.focused.plan_focused import plan_focused
from stripstream.algorithms.search.fast_downward import get_fast_downward
from stripstream.pddl.utils import convert_plan

from manipulation.independent import get_grasps, initialize_openrave, get_base_conf, get_arm_conf
from manipulation.problems.distribution import dantam, dantam2
from manipulation.primitives.transforms import set_base_values, get_config, set_trans, get_trans, unit_trans, \
  object_trans_from_manip_trans, quat_from_trans, get_full_config, set_manipulator_values
from manipulation.primitives.utils import Config, ManipVector
from manipulation.constants import GRASP_APPROACHES, GRASP_TYPES
from manipulation.motion.single_query import motion_plan, workspace_traj_helper
from manipulation.motion.cspace import CSpace
from manipulation.bodies.robot import open_gripper2

from stripstream.pddl.objects import EasyType, EasyParameter
from stripstream.pddl.logic.predicates import EasyPredicate
from stripstream.pddl.operators import Action, Axiom
from stripstream.pddl.logic.quantifiers import ForAll, Exists
from stripstream.pddl.logic.connectives import Not, And, Or
from stripstream.pddl.logic.atoms import Equal
from stripstream.pddl.utils import rename_easy
from stripstream.pddl.problem import STRIPStreamProblem
from stripstream.pddl.cond_streams import EasyListGenStream, EasyTestStream

from manipulation.primitives.inverse_kinematics import inverse_kinematics_helper
from manipulation.bodies.robot import manip_from_pose_grasp
from manipulation.primitives.transforms import set_pose
from stripstream.pddl.streams import Stream
from stripstream.pddl.cond_streams import ClassStream
from stripstream.pddl.utils import get_value
from stripstream.utils import SEPARATOR, INF

Type, Pred, Param = EasyType, EasyPredicate, EasyParameter

####################

PROBLEM = dantam2 # dantam | dantam2
ARM = 'leftarm'
USE_GRASP_APPROACH = GRASP_APPROACHES.TOP # TOP | SIDE
USE_GRASP_TYPE = GRASP_TYPES.GRASP # GRASP | TOUCH

####################

# Types
OBJ, POSE, GRASP = Type(), Type(), Type()
MCONF, GTRAJ, MTRAJ = Type(), Type(), Type()

####################

# Fluents
ManipEq = Pred(MCONF)
PoseEq = Pred(OBJ, POSE)
GraspEq = Pred(OBJ, GRASP)
HandEmpty = Pred()

####################

# Derived
SafePoseMove = Pred(OBJ, MTRAJ)
SafePoseMoveHolding = Pred(OBJ, MTRAJ, GRASP)
SafeGraspMove = Pred(OBJ, MTRAJ)
SafeMove = Pred(OBJ, MTRAJ)
SafePose = Pred(OBJ, POSE)
SafeGraspTraj = Pred(OBJ, GTRAJ)

####################

# Static trajectory
ManipMotion = Pred(MCONF, MCONF, MTRAJ)
GraspMotion = Pred(POSE, GRASP, MCONF, GTRAJ)

# Pick/Place static collision
CFreePosePose = Pred(POSE, POSE)
CFreeGTrajPose = Pred(GTRAJ, POSE)

# Move static collision
CFreeMTrajPose = Pred(MTRAJ, POSE)
CFreeMTrajGrasp = Pred(MTRAJ, GRASP)
CFreeMTrajGraspPose = Pred(MTRAJ, GRASP, POSE)

####################

O, P, G = Param(OBJ), Param(POSE), Param(GRASP)
O2, O3 = Param(OBJ), Param(OBJ)
P2 = Param(POSE)

MQ, MQ2 = Param(MCONF), Param(MCONF)
MT = Param(MTRAJ)
GT = Param(GTRAJ)

rename_easy(locals())

####################

actions = [
  Action(name='pick', parameters=[O, P, G, MQ, GT],
    condition=And(PoseEq(O, P), HandEmpty(), ManipEq(MQ), GraspMotion(P, G, MQ, GT),
                  ForAll([O2], Or(Equal(O, O2), SafePose(O2, P)))),
    effect=And(GraspEq(O, G), Not(HandEmpty()), Not(PoseEq(O, P)))),

  Action(name='place', parameters=[O, P, G, MQ, GT],
    condition=And(GraspEq(O, G), ManipEq(MQ), GraspMotion(P, G, MQ, GT),
                  ForAll([O2], Or(Equal(O, O2), SafePose(O2, P)))),
    effect=And(PoseEq(O, P), HandEmpty(), Not(GraspEq(O, G)))),

  Action(name='move', parameters=[MQ, MQ2, MT],
    condition=And(ManipEq(MQ), ManipMotion(MQ, MQ2, MT), ForAll([O2], SafeMove(O2, MT))),
    effect=And(ManipEq(MQ2), Not(ManipEq(MQ)))),
]

axioms = [
  # Pick/Place collisions
  Axiom(effect=SafePose(O2, P), condition=Exists([P2], And(PoseEq(O2, P2), CFreePosePose(P, P2)))),
  Axiom(effect=SafeGraspTraj(O2, GT), condition=Exists([P2], And(PoseEq(O2, P2), CFreeGTrajPose(GT, P2)))),

  # Move
  Axiom(effect=SafeMove(O2, MT), condition=Or(SafePoseMove(O2, MT), SafeGraspMove(O2, MT))),
  Axiom(effect=SafePoseMove(O2, MT), condition=Exists([P2], And(PoseEq(O2, P2), CFreeMTrajPose(MT, P2)))),
  Axiom(effect=SafeGraspMove(O2, MT), condition=Exists([G], And(GraspEq(O2, G), CFreeMTrajGrasp(MT, G),
                                        ForAll([O3], Or(Equal(O2, O3), SafePoseMoveHolding(O3, MT, G)))))),
  Axiom(effect=SafePoseMoveHolding(O3, MT, G), condition=Exists([P2], And(PoseEq(O3, P2), CFreeMTrajGraspPose(MT, G, P2)))),
]

##################################################

def solve_tamp(env):
  viewer = env.GetViewer() is not None
  problem = PROBLEM(env)

  robot = env.GetRobots()[0]
  set_base_values(robot, (-.75, .2, -math.pi/2))
  initialize_openrave(env, ARM)
  manipulator = robot.GetActiveManipulator()
  cspace = CSpace.robot_arm(manipulator)
  base_manip = interfaces.BaseManipulation(robot, plannername=None, maxvelmult=None)

  bodies = {obj: env.GetKinBody(obj) for obj in problem.object_names}
  geom_hashes = {body.GetKinematicsGeometryHash() for body in bodies.values()}
  assert len(geom_hashes) == 1 # NOTE - assuming all objects has the same geometry

  all_bodies = bodies.values()
  body1 = all_bodies[-1]
  body2 = all_bodies[-2] if len(bodies) >= 2 else body1
  grasps = get_grasps(env, robot, body1, USE_GRASP_APPROACH, USE_GRASP_TYPE)[:1]
  poses = problem.known_poses if problem.known_poses else []

  open_gripper2(manipulator)
  initial_manip = get_arm_conf(manipulator, Config(get_full_config(robot)))

  def enable_all(enable):
    for body in all_bodies:
      body.Enable(enable)

  ####################

  def cfree_pose_pose(pose1, pose2):
    body1.Enable(True)
    set_pose(body1, pose1.value)
    body2.Enable(True)
    set_pose(body2, pose2.value)
    return not env.CheckCollision(body1, body2)

  def cfree_gtraj_pose(gt, p):
    return cfree_mtraj_pose(gt, p) and cfree_mtraj_grasp_pose(gt, gt.grasp, p)

  ####################

  def cfree_mtraj_grasp(mt, g):
    enable_all(False)
    body1.Enable(True)
    for conf in mt.path():
      set_manipulator_values(manipulator, conf) # NOTE - can also grab
      set_pose(body1, object_trans_from_manip_trans(get_trans(manipulator), g.grasp_trans))
      if env.CheckCollision(body1):
        return False
    return True

  def cfree_mtraj_pose(mt, p):
    enable_all(False)
    body2.Enable(True)
    set_pose(body2, p.value)
    for conf in mt.path():
      set_manipulator_values(manipulator, conf)
      if env.CheckCollision(body2):
        return False
    return True

  def cfree_mtraj_grasp_pose(mt, g, p):
    enable_all(False)
    body1.Enable(True)
    body2.Enable(True)
    set_pose(body2, p.value)
    for conf in mt.path():
      set_manipulator_values(manipulator, conf)
      set_pose(body1, object_trans_from_manip_trans(get_trans(manipulator), g.grasp_trans))
      if env.CheckCollision(body1, body2):
        return False
    return True

  ####################

  def sample_grasp_traj(pose, grasp):
    enable_all(False)
    body1.Enable(True)
    set_pose(body1, pose.value)
    manip_trans, approach_vector = manip_from_pose_grasp(pose, grasp)
    grasp_config = inverse_kinematics_helper(env, robot, manip_trans)
    if grasp_config is None: return

    set_manipulator_values(manipulator, grasp_config)
    robot.Grab(body1)
    grasp_traj = workspace_traj_helper(base_manip, approach_vector)
    robot.Release(body1)
    if grasp_traj is None: return
    grasp_traj.grasp = grasp
    yield [(Config(grasp_traj.end()), grasp_traj)]

  class MotionStream(Stream):
    def get_values(self, universe, dependent_atoms=set(), **kwargs):
      mq1, mq2 = map(get_value, self.inputs)

      collision_atoms = filter(lambda atom: atom.predicate in [CFreeMTrajGrasp, CFreeMTrajPose], dependent_atoms)
      collision_params = {atom: atom.args[0] for atom in collision_atoms}
      grasp = None
      for atom in collision_atoms:
        if atom.predicate is CFreeMTrajGrasp:
          assert grasp is None # Can't have two grasps
          _, grasp = map(get_value, atom.args)
      placed = []
      for atom in collision_atoms:
        if atom.predicate is CFreeMTrajPose:
          _, pose = map(get_value, atom.args)
          placed.append(pose)
      #placed, grasp = [], None
      #print grasp, placed

      if placed or grasp:
        assert len(placed) <= len(all_bodies)
        enable_all(False)
        for b, p in zip(all_bodies, placed):
          b.Enable(True)
          set_pose(b, p.value)
        if grasp:
          assert grasp is None or len(placed) <= len(all_bodies)-1
          set_pose(body1, object_trans_from_manip_trans(get_trans(manipulator), grasp.grasp_trans))
          robot.Grab(body1)

        set_manipulator_values(manipulator, mq1.value)
        mt = motion_plan(env, cspace, mq2.value, self_collisions=True)
        if grasp: robot.Release(body1)
        if mt:
          self.enumerated = True # NOTE - if satisfies all constraints then won't need another. Not true. What if called with different grasps...
          # TODO - could always hash this trajectory for the current set of constraints
          bound_collision_atoms = [atom.instantiate({collision_params[atom]: MTRAJ(mt)}) for atom in collision_atoms]
          #bound_collision_atoms = []
          return [ManipMotion(mq1, mq2, mt)] + bound_collision_atoms
        raise ValueError()

      enable_all(False)
      set_manipulator_values(manipulator, mq1.value)
      mt = motion_plan(env, cspace, mq2.value, self_collisions=True)
      if mt:
        return [ManipMotion(mq1, mq2, mt)]
      return []

  ####################

  cond_streams = [
    # Pick/place trajectory
    EasyListGenStream(inputs=[P, G], outputs=[MQ, GT], conditions=[],
                      effects=[GraspMotion(P, G, MQ, GT)], generator=sample_grasp_traj),

    # Move trajectory
    ClassStream(inputs=[MQ, MQ2], outputs=[MT], conditions=[],
                effects=[ManipMotion(MQ, MQ2, MT)], StreamClass=MotionStream, order=1, max_level=0),

    # Pick/place collisions
    EasyTestStream(inputs=[P, P2], conditions=[], effects=[CFreePosePose(P, P2)],
                test=cfree_pose_pose, eager=True),
    EasyTestStream(inputs=[GT, P], conditions=[], effects=[CFreeGTrajPose(GT, P)],
                test=cfree_gtraj_pose),

    # Move collisions
    EasyTestStream(inputs=[MT, P], conditions=[], effects=[CFreeMTrajPose(MT, P)],
                test=cfree_mtraj_pose),
    EasyTestStream(inputs=[MT, G], conditions=[], effects=[CFreeMTrajGrasp(MT, G)],
                test=cfree_mtraj_grasp),
    EasyTestStream(inputs=[MT, G, P], conditions=[], effects=[CFreeMTrajGraspPose(MT, G, P)],
                test=cfree_mtraj_grasp_pose),
  ]

  ####################

  constants = map(GRASP, grasps) + map(POSE, poses)
  initial_atoms = [
    ManipEq(initial_manip),
    HandEmpty(),
  ] + [
    PoseEq(obj, pose) for obj, pose in problem.initial_poses.iteritems()
  ]
  goal_formula = And(ManipEq(initial_manip), *(PoseEq(obj, pose) for obj, pose in problem.goal_poses.iteritems()))
  stream_problem = STRIPStreamProblem(initial_atoms, goal_formula, actions + axioms, cond_streams, constants)

  if viewer: raw_input('Start?')
  search_fn = get_fast_downward('eager')
  #plan, universe = incremental_planner(stream_problem, search=search_fn, frequency=INF, waves=True, debug=False) # 1 | 20 | 100 | INF
  #plan, _ = focused_planner(stream_problem, search=search_fn, frequency=1, waves=True, debug=False) # 1 | 20 | 100 | INF
  #plan, universe = simple_focused(stream_problem, search=search_fn, max_level=INF, debug=False, verbose=False) # 1 | 20 | 100 | INF
  #plan, _ = plan_focused(stream_problem, search=search_fn, debug=False) # 1 | 20 | 100 | INF

  from misc.profiling import run_profile, str_profile
  #solve = lambda: incremental_planner(stream_problem, search=search_fn, frequency=INF, waves=True, debug=False)
  solve = lambda: simple_focused(stream_problem, search=search_fn, max_level=INF, shared=False, debug=False, verbose=False)
  #with env:
  env.Lock()
  (plan, universe), prof = run_profile(solve)
  env.Unlock()

  print SEPARATOR
  universe.print_domain_statistics()
  universe.print_statistics()
  print SEPARATOR
  print str_profile(prof)
  print SEPARATOR

  plan = convert_plan(plan)
  if plan is not None:
    print 'Success'
    for i, (action, args) in enumerate(plan):
      print i+1, action, args
  else:
    print 'Failure'

  ####################

  def step_path(traj):
    #for j, conf in enumerate(traj.path()):
    for j, conf in enumerate([traj.end()]):
      set_manipulator_values(manipulator, conf)
      raw_input('%s/%s) Step?'%(j, len(traj.path())))

  if viewer and plan is not None:
    print SEPARATOR
    # Resets the initial state
    set_manipulator_values(manipulator, initial_manip.value)
    for obj, pose in problem.initial_poses.iteritems():
      set_pose(bodies[obj], pose.value)

    for i, (action, args) in enumerate(plan):
      raw_input('\n%s/%s) Next?'%(i, len(plan)))
      if action.name == 'move':
        mq1, mq2, mt = args
        step_path(mt)
      elif action.name == 'pick':
        o, p, g, mq, gt = args
        step_path(gt.reverse())
        robot.Grab(bodies[o])
        step_path(gt)
      elif action.name == 'place':
        o, p, g, mq, gt = args
        step_path(gt.reverse())
        robot.Release(bodies[o])
        step_path(gt)
      else:
        raise ValueError(action.name)
      env.UpdatePublishedBodies()
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