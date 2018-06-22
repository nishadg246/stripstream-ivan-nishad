#!/usr/bin/env python

from time import time
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
from manipulation.problems.distribution import dantam, dantam2, move_several_4
from manipulation.primitives.transforms import set_base_values, get_config, set_trans, get_trans, unit_trans, \
  object_trans_from_manip_trans, quat_from_trans, get_full_config, set_manipulator_values
from manipulation.primitives.utils import Config, ManipVector
from manipulation.constants import GRASP_APPROACHES, GRASP_TYPES
from manipulation.motion.single_query import motion_plan, workspace_traj_helper
from manipulation.motion.cspace import CSpace
from manipulation.bodies.robot import open_gripper2, close_gripper2, grasp_gripper2

from stripstream.pddl.objects import EasyType, EasyParameter
from stripstream.pddl.logic.predicates import EasyPredicate
from stripstream.pddl.operators import Action, Axiom
from stripstream.pddl.logic.quantifiers import ForAll, Exists
from stripstream.pddl.logic.connectives import Not, And, Or
from stripstream.pddl.logic.atoms import Equal
from stripstream.pddl.utils import rename_easy
from stripstream.pddl.problem import STRIPStreamProblem
from stripstream.pddl.cond_streams import EasyListGenStream, EasyTestStream, EasyGenStream

from manipulation.primitives.inverse_kinematics import inverse_kinematics_helper
from manipulation.bodies.robot import manip_from_pose_grasp
from manipulation.primitives.transforms import set_pose, get_pose
from stripstream.pddl.streams import Stream
from stripstream.pddl.cond_streams import ClassStream
from stripstream.pddl.utils import get_value
from stripstream.utils import SEPARATOR, INF

Type, Pred, Param = EasyType, EasyPredicate, EasyParameter

####################

OBJ, POSE, GRASP = Type(), Type(), Type()
BCONF, MCONF = Type(), Type()
BTRAJ, MTRAJ = Type(), Type()
MVEC, GTRAJ = Type(), Type()

####################

BaseEq = Pred(BCONF)
ManipEq = Pred(MCONF)

HandEmpty = Pred()
PoseEq = Pred(OBJ, POSE)
GraspEq = Pred(OBJ, GRASP)

Kin = Pred(POSE, GRASP, BCONF, MCONF) # NOTE - could remove OBJ from these?
ManipMotion = Pred(MCONF, MCONF, BCONF, MTRAJ) # Need BASE_CONF to avoid static collisions
GraspTraj = Pred(POSE, GRASP, BCONF, MCONF, GTRAJ)
#GraspTraj = Pred(MVEC, BCONF, MCONF, GTRAJ)

PoseCFree = Pred(POSE, POSE)
MTrajPoseCFree = Pred(MTRAJ, POSE)
MTrajGraspCFree = Pred(MTRAJ, GRASP)
MTrajGraspPoseCFree = Pred(MTRAJ, GRASP, POSE)

SafePoseMove = Pred(OBJ, MTRAJ)
SafePoseMoveHolding = Pred(OBJ, MTRAJ, GRASP)
SafeGraspMove = Pred(OBJ, MTRAJ)
SafeMove = Pred(OBJ, MTRAJ)

SafePose = Pred(OBJ, POSE)
#SafeGraspTraj = Pred(OBJ, GRASP, MANIP_TRAJ) # With fixed obstacles
#SafeGraspTrajPose = Pred(OBJ, GRASP, MANIP_TRAJ, POSE) # With respect to moveable objects

#SafeArm = Pred(OBJ, MANIP_CONF, TRAJ) # TODO - make a "object" for the fixed environment which isn't movable
SafeManip = Pred(BTRAJ)

Movable = Pred(OBJ)
EnvCollision = Pred(POSE)

Holding = Pred(OBJ)

####################

O, P, G = Param(OBJ), Param(POSE), Param(GRASP)
O2, O3 = Param(OBJ), Param(OBJ)
P2 = Param(POSE)

BQ, MQ = Param(BCONF), Param(MCONF)
BQ2, MQ2 = Param(BCONF), Param(MCONF)
BT, MT = Param(BTRAJ), Param(MTRAJ)
MV, GT = Param(MVEC), Param(GTRAJ)

rename_easy(locals())

####################

# NOTE - can add the movable objects here

operators = [
  # Without grasp trajectory
  #Action(name='pick', parameters=[O, P, G, BQ, MQ],
  #  condition=And(PoseEq(O, P), HandEmpty(), BaseEq(BQ), ManipEq(MQ), Kin(P, G, BQ, MQ)), #ForAll([OB], Or(Equal(O, OB), Safe(OB, T))))),
  #  effect=And(GraspEq(O, G), Not(HandEmpty()), Not(PoseEq(O, P)))),
  #
  #Action(name='place', parameters=[O, P, G, BQ, MQ],
  #  condition=And(GraspEq(O, G), BaseEq(BQ), ManipEq(MQ), Kin(P, G, BQ, MQ),
  #                ForAll([O2], Or(Equal(O, O2), SafePose(O2, P)))),
  #  effect=And(PoseEq(O, P), HandEmpty(), Not(GraspEq(O, G)))),

  # With grasp trajectory
  Action(name='pick', parameters=[O, P, G, BQ, MQ, GT],
    condition=And(PoseEq(O, P), HandEmpty(), BaseEq(BQ), ManipEq(MQ), GraspTraj(P, G, BQ, MQ, GT)),
    effect=And(GraspEq(O, G), Not(HandEmpty()), Not(PoseEq(O, P)))),

  Action(name='place', parameters=[O, P, G, BQ, MQ, GT],
    condition=And(GraspEq(O, G), BaseEq(BQ), ManipEq(MQ), GraspTraj(P, G, BQ, MQ, GT),
                  ForAll([O2], Or(Equal(O, O2), SafePose(O2, P)))),
    effect=And(PoseEq(O, P), HandEmpty(), Not(GraspEq(O, G)))),
  # NOTE - I could make actions that backup or move forward instead

  Action(name='move_arm', parameters=[MQ, MQ2, BQ, MT],
    #condition=And(ManipEq(MQ), ManipMotion(MQ, MQ2, BQ, MT), BaseEq(BQ), ForAll([O2], SafePoseMove(O2, MT))),
    condition=And(ManipEq(MQ), ManipMotion(MQ, MQ2, BQ, MT), BaseEq(BQ), ForAll([O2], SafeMove(O2, MT))),
    #condition=And(ManipEq(MQ), ManipMotion(MQ, MQ2, BQ, MT), BaseEq(BQ),
    #              ForAll([O2], Or(SafeGraspMove(O2, MT), SafePoseMove(O2, MT)))), # NOTE - bad idea, creates one per each combo
    effect=And(ManipEq(MQ2), Not(ManipEq(MQ)))),

  #Action(name='move_arm', parameters=[MQ, MQ2, BQ, MT],
  #  condition=And(HandEmpty(), ManipEq(MQ), ManipMotion(MQ, MQ2, BQ, MT), BaseEq(BQ), ForAll([OB], SafePoseMove(OB, MT))),
  #  effect=And(ManipEq(MQ2), Not(ManipEq(MQ)))),
  #
  #Action(name='move_arm_holding', parameters=[MQ, MQ2, BQ, O, G, MT],
  #  condition=And(GraspEq(O, G), ManipEq(MQ), ManipMotion(MQ, MQ2, BQ, MT), BaseEq(BQ), ForAll([OB], SafePoseMove(OB, MT))),
  #  effect=And(ManipEq(MQ2), Not(ManipEq(MQ)))),

  #Action(name='move_base', parameters=[BQ, BQ2, BT],
  #  condition=And(BaseEq(BQ), SafeManip(BT), BaseMotion(BQ, BQ2, BT)),
  #  effect=And(BaseEq(BQ2), Not(BaseEq(BQ)))),

  Axiom(effect=SafePose(O2, P), condition=Exists([P2], And(PoseEq(O2, P2), PoseCFree(P, P2)))),

  #Axiom(effect=Holding(O), condition=Exists([G], GraspEq(O, G))),

  #Axiom(effect=SafePoseMove(OB, MT), condition=Or(Holding(OB), Exists([P2], And(PoseEq(OB, P2), MTrajPoseCFree(MT, P2))))),
  #Axiom(effect=SafePoseMove(OB, MT), condition=Or(Exists([G], And(GraspEq(OB, G), MTrajGraspCFree(MT, G))),
  #                                                Exists([P2], And(PoseEq(OB, P2), MTrajPoseCFree(MT, P2))))),

  Axiom(effect=SafeMove(O2, MT), condition=Or(SafePoseMove(O2, MT), SafeGraspMove(O2, MT))),
  Axiom(effect=SafePoseMove(O2, MT), condition=Exists([P2], And(PoseEq(O2, P2), MTrajPoseCFree(MT, P2)))),
  Axiom(effect=SafeGraspMove(O2, MT),
        #condition=Exists([G], And(GraspEq(O2, G), MTrajGraspCFree(MT, G)))),
        condition=Exists([G], And(GraspEq(O2, G), MTrajGraspCFree(MT, G),
                                  ForAll([O3], Or(Equal(O2, O3), SafePoseMoveHolding(O3, MT, G)))))),
  #Axiom(effect=SafeMove(O2, MT),
  #      condition=Or(Exists([P2], And(PoseEq(O2, P2), MTrajPoseCFree(MT, P2))),
  #                   Exists([G], And(GraspEq(O2, G), MTrajGraspCFree(MT, G),
  #                     ForAll([O3], Or(Equal(O2, O3), SafePoseMoveHolding(O3, MT, G))))))),
  Axiom(effect=SafePoseMoveHolding(O3, MT, G), condition=Exists([P2], And(PoseEq(O3, P2), MTrajGraspPoseCFree(MT, G, P2)))),

  #Axiom(effect=SafeHolding(OB, P),
  #      condition=Exists([P2], And(PoseEq(OB, P2), PoseCFree(P, P2)))),
]

# TODO - easier way of specifying collisions?

# TODO - make this entirely self-contained

##################################################

def solve_tamp(env):
  viewer = env.GetViewer() is not None
  #problem = dantam(env)
  problem = dantam2(env)
  #problem = move_several_4(env)

  robot = env.GetRobots()[0]
  set_base_values(robot, (-.75, .2, -math.pi/2))
  initialize_openrave(env, 'leftarm')
  manipulator = robot.GetActiveManipulator()
  cspace = CSpace.robot_arm(manipulator)
  base_manip = interfaces.BaseManipulation(robot, plannername=None, maxvelmult=None)

  #USE_GRASP_APPROACH = GRASP_APPROACHES.SIDE
  USE_GRASP_APPROACH = GRASP_APPROACHES.TOP
  #USE_GRASP_TYPE = GRASP_TYPES.TOUCH
  USE_GRASP_TYPE = GRASP_TYPES.GRASP

  bodies = {obj: env.GetKinBody(obj) for obj in problem.object_names}
  geom_hashes = {body.GetKinematicsGeometryHash() for body in bodies.values()}
  assert len(geom_hashes) == 1 # NOTE - assuming all objects has the same geometry

  all_bodies = bodies.values()
  body1 = all_bodies[-1]
  body2 = all_bodies[-2] if len(bodies) >= 2 else body1
  grasps = get_grasps(env, robot, body1, USE_GRASP_APPROACH, USE_GRASP_TYPE)[:1]
  poses = problem.known_poses if problem.known_poses else []

  ##################################################

  def enable_all(enable):
    for body in all_bodies:
      body.Enable(enable)

  def collision_free(pose1, pose2):
    body1.Enable(True)
    set_pose(body1, pose1.value)
    body2.Enable(True)
    set_pose(body2, pose2.value)
    return not env.CheckCollision(body1, body2)

  def grasp_env_cfree(mt, g):
    enable_all(False) # TODO - base config?
    body1.Enable(True)
    for conf in mt.path():
      set_manipulator_values(manipulator, conf) # NOTE - can also grab
      set_pose(body1, object_trans_from_manip_trans(get_trans(manipulator), g.grasp_trans))
      if env.CheckCollision(body1):
        return False
    return True

  def grasp_pose_cfree(mt, g, p):
    enable_all(False) # TODO - base config?
    body1.Enable(True)
    body2.Enable(True)
    set_pose(body2, p.value)
    for conf in mt.path():
      set_manipulator_values(manipulator, conf)
      set_pose(body1, object_trans_from_manip_trans(get_trans(manipulator), g.grasp_trans))
      if env.CheckCollision(body1, body2):
        return False
    return True

  ##################################################

  """
  class CollisionStream(Stream): # TODO - could make an initial state version of this that doesn't need pose2
    def get_values(self, **kwargs):
      self.enumerated = True
      pose1, pose2 = map(get_value, self.inputs)
      if collision_free(pose1, pose2):
        return [PoseCFree(pose1, pose2)]
      return []
      #return [Movable()] # NOTE - only make movable when it fails a collision check

  CheckedInitial = Pred(POSE) # How should this be handled? The planner will need to revisit on the next state anyways
  class EnvCollisionStream(Stream): # NOTE - I could also make an environment OBJECT which I mutate. I'm kind of doing that now
    movable = []
    def get_values(self, **kwargs):
      self.enumerated = True
      pose, = map(get_value, self.inputs)
      results = [CheckedInitial(pose)]
      for obj, pose2 in problem.initial_poses.iteritems():
        if obj not in self.movable and collision_free(pose, pose2):
          pass
          #results.append(PoseCFree(pose, pose2))
        else:
          self.movable.append(obj)
          results.append(PoseEq(obj, pose2))
          #results.append(Movable(obj))
      if results:
        pass # NOTE - I could make this fail if there is a collision
        # I could prevent the binding by directly adding CheckedInitial to the universe
        # In general, I probably can just mutate the problem however I see fit here
      return results
  """

  ##################################################

  # NOTE - can do pose, approach manip, true approach traj, motion plan

  # NOTE - can make something that produces approach trajectories

  def get_manip_vector(pose, grasp):
    manip_trans, approach_vector = manip_from_pose_grasp(pose, grasp)
    #enable_all(False)
    #if manipulator.CheckEndEffectorCollision(manip_trans):
    #  return None
    return ManipVector(manip_trans, approach_vector)

  def sample_ik(pose, grasp, base_conf): # TODO - make this return the grasp
    enable_all(False)
    set_base_values(robot, base_conf.value)
    body1.Enable(True)
    set_pose(body1, pose.value)
    manip_vector = get_manip_vector(pose, grasp)
    grasp_config = inverse_kinematics_helper(env, robot, manip_vector.manip_trans) # NOTE - maybe need to find all IK solutions
    #print manipulator.CheckEndEffectorCollision(manip_trans)
    if grasp_config is not None:
      yield [Config(grasp_config)]
    #traj = workspace_traj_helper(base_manip, approach_vector)

  def sample_grasp_traj(pose, grasp, base_conf):
    enable_all(False)
    set_base_values(robot, base_conf.value)
    body1.Enable(True)
    set_pose(body1, pose.value)
    manip_vector = get_manip_vector(pose, grasp)
    grasp_config = inverse_kinematics_helper(env, robot, manip_vector.manip_trans) # NOTE - maybe need to find all IK solutions
    if grasp_config is None: return

    set_manipulator_values(manipulator, grasp_config)
    grasp_traj = workspace_traj_helper(base_manip, manip_vector.approach_vector)
    if grasp_config is None: return
    yield [(Config(grasp_traj.end()), grasp_traj)]

  ##################################################

  # NOTE - can either include the held object in the traj or have a special condition that not colliding

  def sample_arm_traj(mq1, mq2, bq): # TODO - need to add holding back in
    yield None
    #enable_all(False)
    #with robot:
    #  set_base_values(robot, bq.value)
    #  pass


  # TODO - does it make sense to make a new stream for the biasing or to continuously just pass things
  # I suppose I could cache the state or full plan as context

  class MotionStream(Stream): # TODO - maybe make this produce the correct values
    num = 0
    #def get_values(self, **kwargs):
    #  self.enumerated = True
    #  mq1, mq2, bq = map(get_value, self.inputs)
    #  #mt = None
    #  mt = MotionStream.num
    #  MotionStream.num += 1 # Ensures all are unique
    #  return [ManipMotion(mq1, mq2, bq, mt)]
    def sample_motion_plan(self, mq1, mq2, bq):
      set_manipulator_values(manipulator, mq1.value)
      set_base_values(robot, bq.value)
      return motion_plan(env, cspace, mq2.value, self_collisions=True)
    def get_values(self, universe, dependent_atoms=set(), **kwargs):
      mq1, mq2, bq = map(get_value, self.inputs)

      collision_atoms = filter(lambda atom: atom.predicate in [MTrajGraspCFree, MTrajPoseCFree], dependent_atoms)
      collision_params = {atom: atom.args[0] for atom in collision_atoms}
      grasp = None
      for atom in collision_atoms:
        if atom.predicate is MTrajGraspCFree:
          assert grasp is None # Can't have two grasps
          _, grasp = map(get_value, atom.args)
      placed = []
      for atom in collision_atoms:
        if atom.predicate is MTrajPoseCFree:
          _, pose = map(get_value, atom.args)
          placed.append(pose)
      #placed, grasp = [], None
      print grasp, placed

      if placed or grasp:
        assert len(placed) <= len(all_bodies) # How would I handle many constraints on the same traj?
        enable_all(False)
        for b, p in zip(all_bodies, placed):
          b.Enable(True)
          set_pose(b, p.value)
        if grasp:
          assert grasp is None or len(placed) <= len(all_bodies)-1
          set_pose(body1, object_trans_from_manip_trans(get_trans(manipulator), grasp.grasp_trans))
          robot.Grab(body1)

        mt = self.sample_motion_plan(mq1, mq2, bq)
        if grasp: robot.Release(body1)
        if mt:
          self.enumerated = True # NOTE - if satisfies all constraints then won't need another. Not true. What if called with different grasps...
          # TODO - could always hash this trajectory for the current set of constraints
          bound_collision_atoms = [atom.instantiate({collision_params[atom]: MTRAJ(mt)}) for atom in collision_atoms]
          #bound_collision_atoms = []
          return [ManipMotion(mq1, mq2, bq, mt)] + bound_collision_atoms
        raise ValueError()

      enable_all(False)
      mt = self.sample_motion_plan(mq1, mq2, bq)
      if mt:
        return [ManipMotion(mq1, mq2, bq, mt)]
      return []

  ##################################################

  cond_streams = [
    #MultiEasyGenStream(inputs=[O], outputs=[P], conditions=[], effects=[LegalPose(O, P)], generator=sample_poses),

    #EasyGenStream(inputs=[MQ, MQ2, BQ], outputs=[MT], conditions=[],
    #              effects=[ManipMotion(MQ, MQ2, BQ, MT)], generator=sample_arm_traj),
    ClassStream(inputs=[MQ, MQ2, BQ], outputs=[MT], conditions=[],
                effects=[ManipMotion(MQ, MQ2, BQ, MT)], StreamClass=MotionStream, order=1, max_level=0),

    #MultiEasyGenStream(inputs=[P, G, BQ], outputs=[MQ], conditions=[],
    #                   effects=[Kin(P, G, BQ, MQ)], generator=sample_ik),
    EasyListGenStream(inputs=[P, G, BQ], outputs=[MQ, GT], conditions=[],
                      effects=[GraspTraj(P, G, BQ, MQ, GT)], generator=sample_grasp_traj),

    #EasyTestStream(inputs=[O, P, T], conditions=[], effects=[CFree(O, P, T)],
    #            test=collision_free, eager=True),

    EasyTestStream(inputs=[P, P2], conditions=[], effects=[PoseCFree(P, P2)],
                test=collision_free, eager=True),
    #ClassStream(inputs=[P, P2], conditions=[], outputs=[],
    #            effects=[PoseCFree(P, P2)], StreamClass=CollisionStream, eager=True),


    EasyTestStream(inputs=[MT, P], conditions=[], effects=[MTrajPoseCFree(MT, P)],
                test=lambda mt, p: True, eager=True),
    EasyTestStream(inputs=[MT, G], conditions=[], effects=[MTrajGraspCFree(MT, G)],
                test=lambda mt, g: True, eager=True),
    EasyTestStream(inputs=[MT, G, P], conditions=[], effects=[MTrajGraspPoseCFree(MT, G, P)],
                test=lambda mt, g, p: True, eager=True),
    #ClassStream(inputs=[P], conditions=[], outputs=[],
    #            effects=[CheckedInitial(P)], StreamClass=EnvCollisionStream),
  ]

  ##################################################

  constants = map(GRASP, grasps) + map(POSE, poses)
  initial_full = Config(get_full_config(robot))
  initial_base = get_base_conf(initial_full)
  initial_manip = get_arm_conf(manipulator, initial_full)
  initial_atoms = [
    BaseEq(initial_base),
    ManipEq(initial_manip),
    HandEmpty(),
  ] + [
    PoseEq(obj, pose) for obj, pose in problem.initial_poses.iteritems()
  ]
  goal_formula = And(ManipEq(initial_manip), *(PoseEq(obj, pose) for obj, pose in problem.goal_poses.iteritems()))
  stream_problem = STRIPStreamProblem(initial_atoms, goal_formula, operators, cond_streams, constants)

  if viewer: raw_input('Start?')
  search_fn = get_fast_downward('eager')
  #plan, universe = incremental_planner(stream_problem, search=search_fn, frequency=INF, waves=True, debug=False) # 1 | 20 | 100 | INF
  #plan, _ = focused_planner(stream_problem, search=search_fn, frequency=1, waves=True, debug=False) # 1 | 20 | 100 | INF
  #plan, universe = simple_focused(stream_problem, search=search_fn, max_level=INF, debug=False, verbose=False) # 1 | 20 | 100 | INF
  #plan, _ = plan_focused(stream_problem, search=search_fn, debug=False) # 1 | 20 | 100 | INF

  from misc.profiling import run_profile, str_profile
  #solve = lambda: incremental_planner(stream_problem, search=search_fn, frequency=INF, waves=True, debug=False)
  solve = lambda: simple_focused(stream_problem, search=search_fn, max_level=INF, shared=False, debug=False, verbose=True)
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

  ##################################################

  def step_path(traj):
    #for j, conf in enumerate(traj.path()):
    for j, conf in enumerate([traj.end()]):
      set_manipulator_values(manipulator, conf)
      raw_input('%s/%s) Step?'%(j, len(traj.path())))

  if viewer and plan is not None:
    print SEPARATOR
    # Resets the initial state
    open_gripper2(manipulator)
    set_base_values(robot, initial_base.value)
    set_manipulator_values(manipulator, initial_manip.value)
    for obj, pose in problem.initial_poses.iteritems():
      set_pose(bodies[obj], pose.value)

    for i, (action, args) in enumerate(plan):
      raw_input('\n%s/%s) Next?'%(i, len(plan)))
      if action.name == 'move_arm':
        mq1, mq2, bq, mt = args
        #set_manipulator_values(manipulator, mq2.value)
        step_path(mt)
      elif action.name == 'pick':
        #o, p, q, mq, bq = args
        o, p, g, mq, bq, gt = args
        step_path(gt.reverse())
        #grasp_gripper2(manipulator, g) # NOTE - g currently isn't a real grasp
        robot.Grab(bodies[o])
        step_path(gt)
      elif action.name == 'place':
        #o, p, q, mq, bq = args
        o, p, g, mq, bq, gt = args
        step_path(gt.reverse())
        robot.Release(bodies[o])
        #open_gripper2(manipulator)
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