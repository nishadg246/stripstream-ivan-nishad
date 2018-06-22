from redux_tests.testRig import emptyHand, holding, inRegion, placedAndHolding, placed
from hpn.traceFile import debug
from util.planUtil import ObjGraspB, PoseD, ObjPlaceB
from mm_operators.mmGenGrasp import potentialGraspConfGen, graspConfHypGen
from redux_tests.testRig import typicalErrProbs
from redux_tests.testObjects import makeLegTable, makeIkeaTable
from redux_tests.testObjects import ikZ
from hpn.flatMLSReplanner import Experiment, PlanTest, makeMLS
from mm_operators.mmBeliefState import typeOfObjName
from util.dist import MultivariateGaussianDistribution
import redux_tests.planGlobals as glob
import geometry.hu as hu

from stripstream.pddl.examples.hpn.or_hpn import set_or_state, or_from_hpn_conf, or_manipulation_problem, \
  hpn_from_or_conf, get_object_frame, SIDE_APPROACH_TRANS, get_approach_frame, get_hpn_grasp_tform, get_or_grasps, \
  or_from_hpn_approach_vector, real_manipulation_problem
from stripstream.utils import SEPARATOR
from stripstream.pddl.examples.openrave.easy_tamp import convert_state

from itertools import combinations
from manipulation.primitives.transforms import set_full_config, xyzt_from_trans
from manipulation.visualizations import execute_viewer
from openravepy import Environment, RaveDestroy
from itertools import islice
import numpy as np
import copy
import math

from redux_tests.pr2_test.testPr2 import tZ, table1Pose, table2Pose, tinyVar, smallVar, medVar, bigVar

# TODO - maybe just make directly
def make_move(q1, q2):
  return ('Move', ([q1, q2], None))

def make_move_no_base(q1, q2):
  return ('MoveNB', ([q1, q2], None))

#def make_pick(hand, obj, grasp_desc, grasp, mu, grasp_conf, approach_conf, var=TINY_VAR):
def make_pick(hand, obj, grasp_index, mu_grasp, grasp_conf, approach_conf, var=tinyVar):
  # grasp_index is just for updating discrete distribution over grasps
  #grasp_desc = glob.objectDataBase['graspDesc'][obj]
  #gb = ObjGraspB(obj, grasp_desc, grasp, None, PoseD(hu.Pose(*mu), var))
  grasp_descs = None
  gb = ObjGraspB(obj, grasp_descs, grasp_index, None, PoseD(hu.Pose(*mu_grasp), var))
  return ('Pick', (approach_conf, grasp_conf, hand, gb))

def make_place(hand, obj, rest_face, mu_pose, grasp_conf, approach_conf, var=tinyVar):
  # rest_face is just for updating discrete distribution over resting face
  #face_frames = world.getFaceFrames(obj)
  face_frames = None # Not actually used in the operator
  pb = ObjPlaceB(obj, face_frames, rest_face, PoseD(hu.Pose(*mu_pose), var))
  return ('Place', (approach_conf, grasp_conf, hand, pb))

def make_look(obj, look_conf): # NOTE - includes moving the head
  return ('LookAt', (look_conf, obj))

#################################################################

def satisfies(belief, partial_state):
  # s.fluentValue(f1) where s is fbch.State
  return all(fluent.valueInDetails(belief) is True for fluent in partial_state.fluents)

from random import choice
from geometry import ranges
from redux_tests.pr2_test import pr2IkPoses

HORIZONTAL, VERTICAL = pr2IkPoses.ikTrans()

def sample_base_pose_from_hand_pose(robot, hand_pose, attempts=20): #Hand pose represents the tip of the hand
  base_limits = map(lambda pair: ranges.realRange(*pair), list(robot.limits(['pr2Base'])))
  for i in range(attempts):
    base_pose = hand_pose.compose(choice(HORIZONTAL).inversePose()).pose()
    if not (base_limits[0].inside(base_pose.x) and base_limits[1].inside(base_pose.y) and base_limits[2].inside(base_pose.theta)):
      continue
    base_pose = hu.Pose(base_pose.x, base_pose.y, 0.0, base_pose.theta)
    #if not object_collides(base.applyTrans(base_pose), obstacles = self.obstacles):
    return base_pose
  return None

def distance_fn(robot, q1, q2):
  return robot.distConf(q1, q2)

def config_interpolation(robot, start, end, step_distance=glob.rrtStep):
  configs = []
  config = start
  while True:
    configs.append(config)
    if config == end:
      return configs
    config = robot.stepAlongLine(end, config, step_distance, forward=True, moveChains=start.conf.keys())

# geometry.objects2.World # Seems to only have the known state elements (i.e. not the current positions)
# print world.__dict__.keys()
# ['robot', 'objectShapes', 'regions', 'objects', 'workspace', 'typePointClouds', 'world']
# world.world = world?
# world.objects is a dict of geometry.objects2.MultiChain
# world.objectShapes is a cached dict of shape
# getObjectShapeAtOrigin

def collision(world, conf, obstacles=tuple()):
  obstacles = world.objects.keys()
  if not obstacles:
    return False
  placed_robot = conf.placement()
  #placed_robot, frames = conf.robot.placement(conf, self.world)
  #return any(placed_robot.collides(world.objectPlaces[obst]) for obst in obstacles)
  for obst in obstacles:
    obst_shape = world.getObjectShapeAtOrigin(obst)
    placed_obst = obst_shape.applyTrans()
    if placed_robot.collides(obst_shape):
      return True
  return False

def world_check_collisions(real_world): # RealWorld in sim.py
  robot_conf = real_world.robotConf
  #for obj in real_world.objectConfs:
  #  if real_world.checkRobotCollision(robot_conf, real_world.objectShapes[obj]):
  #    return True
  placed_robot = robot_conf.placement()
  for placed_obj in real_world.objectShapes.values():
    if placed_robot.collides(placed_obj):
      return True
  return False

# NOTE - could also use sigma points
def with_high_probability(mvg, probs):
  # Treats dimensions independently
  assert mvg.shape[1] == len(probs)
  return mvg.pnm(np.array(probs))

from scipy.integrate import nquad
from scipy.stats.mvn import mvnun
def sample_high_probability(mvg, test, prob, samples=100):
  within = 0.
  for i in range(samples):
    within += test(mvg.draw())
    if within/samples >= prob:
      return True
    if (samples - i + within)/samples < prob:
      return False
  return within/samples >= prob

def pose_high_probability(pose_belief):
  mvg = get_pose_mvg(pose_belief)
  def test(sample):
    dx, dy, dz, dtheta = np.abs(sample - np.array(mvg.mu).reshape(-1))
    return math.sqrt(dx**2 + dy**2) < 1e-2 and abs(dz) < 1e-2 and dtheta < 1e-2
  return sample_high_probability(mvg, test, .95)

def get_pose_mvg(pose_belief):
  return MultivariateGaussianDistribution(pose_belief.poseD.muTuple, np.diag(pose_belief.poseD.var), pose4=False)

def sample_pose(pose_belief, dist):
  face_frame = pose_belief.faceFrames[pose_belief.restFace]
  return hu.Pose(*dist.draw()).compose(face_frame.inverse())

# TODO - weight samples?
def sample_robot_obj_collisions(world, conf, pose_belief, prob, samples=100):
  # pose_belief.shadow(world)
  # pose_belief.makeShadow(pbs, prob)
  #import util.windowManager3D as wm
  placed_robot = conf.placement()
  dist = get_pose_mvg(pose_belief)
  collisions = 0.
  #pose_belief.draw(win, ...)
  for i in range(samples):
    pose = sample_pose(pose_belief, dist)
    placed_obj = world.getObjectShapeAtOrigin(pose_belief.obj).applyTrans(pose)
    #placed_obj.draw('W', color='blue')
    collisions += placed_robot.collides(placed_obj)
    if collisions/samples >= prob:
      return True
    if (samples - i + collisions)/samples < prob:
      return False
  return collisions/samples >= prob

def sample_obj_obj_collisions(world, pose_belief1, pose_belief2, prob, samples=100):
  dist1 = get_pose_mvg(pose_belief1)
  dist2 = get_pose_mvg(pose_belief2)
  shape1 = world.getObjectShapeAtOrigin(pose_belief1.obj)
  shape2 = world.getObjectShapeAtOrigin(pose_belief2.obj)
  collisions = 0.
  for i in range(samples):
    placed_obj1 = shape1.applyTrans(sample_pose(pose_belief1, dist1))
    placed_obj2 = shape2.applyTrans(sample_pose(pose_belief2, dist2))
    collisions += placed_obj1.collides(placed_obj2)
    if collisions/samples >= prob:
      return True
    if (samples - i + collisions)/samples < prob:
      return False
  return collisions/samples >= prob

from operator import itemgetter

def check_belief_collisions(belief, prob): # TODO - individual or product probability?
  # TODO - pairwise object collisions
  world = belief.pbs.getWorld() # belief.beliefContext.world
  conf = belief.pbs.getConf() # belief.conf
  # belief.pbs.getPlaceB, belief.pbs.getPlacedObjBs

  pose_beliefs = map(itemgetter(1), belief.pbs.objectBs.values())
  #return any(sample_robot_obj_collisions(world, conf, pose_belief, prob) for fixed, pose_belief in belief.pbs.objectBs.values())
  if any(sample_robot_obj_collisions(world, conf, pose_belief, prob) for pose_belief in pose_beliefs):
    return True
  if any(sample_obj_obj_collisions(world, pb1, pb2, prob) for pb1, pb2 in combinations(pose_beliefs, 2)):
    return True
  return False

# NOTE - BHPN uses the UKF for updates

#from mm_operators.mmOps import singleTargetUpdate

#################################################################

from manipulation.oracle import ManipulationOracle
from manipulation.primitives.display import is_viewer_active
from stripstream.algorithms.search.fast_downward import get_fast_downward
from stripstream.algorithms.focused.focused_planner import focused_planner
from stripstream.algorithms.incremental.incremental_planner import incremental_planner
from manipulation.primitives.utils import get_env
from manipulation.primitives.transforms import trans_from_pose
from stripstream.pddl.utils import convert_plan
from manipulation.primitives.look import look_at_ik

from manipulation.constants import ACTIVE_LEFT, ACTIVE_RIGHT

from manipulation.primitives.transforms import object_trans_from_manip_trans
from manipulation.bodies.robot import manip_from_pose_grasp

from manipulation.visualizations import visualize_plan, execute_viewer, Plan, set_state
from stripstream.pddl.examples.openrave.easy_tamp import compile_problem, executable_plan
from stripstream.algorithms.plan import get_states

# robot
# ['confCache', 'color', 'headChainName', 'armStowAngles', 'OSa', 'scanner', 'gripMax', 'baseChainName',
# 'chainNames', 'chainDependRev', 'nominalConf', 'bodyChains', 'chainDependencies', 'verticalTrans',
# 'gripperFaceFrame', 'selfCollideChainNames', 'chains', 'armChainNames', 'moveChainNames',
# 'gripperChainNames', 'horizontalTrans', 'name', 'compiledChainsOS', 'selfCollidePairs', 'wristFrameNames',
# 'toolOffsetZ', 'confCacheKeys', 'toolOffsetX']

#def objFrame(self): # planUtil.pyx
#  faceFrame = self.faceFrames[self.restFace]
#  return self.poseD.mode().compose(faceFrame.inverse())

#def objectGraspFrame(robot, gdesc, obj_frame, hand): # mmUtil -> objectGraspFrame
#  centerFrame = gdesc.frame.compose(hu.Pose(0,0,gdesc.dz,0))
#  graspFrame = obj_frame.compose(centerFrame)
#  wristFrame = graspFrame.compose(robot.gripperFaceFrame[hand].inverse())
#  return wristFrame

# TODO - try a manual pick and place policy using their generators. Do I want the prim or function stuff?
# Prim seems to be called using op.evalPrim(s.details)
# But executePrim doesn't seem to be called. This is because fbch is effectively just HPN
# env.executePrim is different than executePrim as a method
# Neither the prim or the fn seem to be called with just the simulator

# NOTE - Leslie and Tomas reason about faces to have x, y, z, theta distributions?

#print belief.objects[obj].attrs.keys() # ['fixed', 'restFace', 'z', 'poseModeProb', 'shape', 'faceFrames', 'type', 'graspDesc']
#print belief.objects[obj].__dict__.keys() # ['name', 'attrs']
#print belief.pbs.objectBs[obj].items() # [(name, (fix, pb)), ...]
#print belief.pbs.__dict__.keys() # ['exceptions', 'shadowWorld', 'pbs', 'targetConfs', 'beliefContext', 'held', 'base', 'objectBs', 'conf', 'shadowProb', 'avoidShadow', 'conditions', 'graspB']

#type_name = typeOfObjName(obj)
#world.getObjType(name)
#shape, _ = glob.constructor[type_name](name=obj)
#print 'Origin', shape.parts()[0].origin() # Centered around half dz
#print shape.parts()[0] # ((-0.0445, -0.027, 0.0), (0.0445, 0.027, 0.1175))
#shape = belief.objects[obj].attrs['shape'] # NOTE - this is different than the previous line
#print 'Origin', shape.parts()[0].origin() # Unit transform
#print shape.parts()[0] # ((-0.0445, -0.027, -0.05875), (0.0445, 0.027, 0.05875))


from redux_tests.pr2_test.pr2InvKin import gripperTip, gripperToolOffset
from redux_tests.pr2_test.pr2Robot import right_gripperToolOffsetX, right_gripperToolOffsetZ, gFaceFrame, gripperFaceFrame
# print right_gripperToolOffsetZ # NOTE - very close to hu.Transform(gripperToolOffset) (only difference is the offset)
# gripperFaceFrame[hand] # pr2HeadFrame, pr2LeftArmFrame, pr2RightArmFrame

# NOTE - the right gripper has the extra sensor on it

# Face = GDesc*Pose
# Grasp = ObjFrame
# Wrist = Grasp*Tool.T
def get_old_obj_trans(conf, or_grasp):
  cart_conf = conf.getCart()
  manip_trans = cart_conf.get('pr2RightArmFrame')
  grasp_trans = hu.Transform(np.linalg.inv(or_grasp.grasp_trans)) # NOTE - this works
  tool_trans = hu.Transform(gripperToolOffset) # NOTE - this works
  obj_trans = manip_trans.compose(tool_trans).compose(grasp_trans.inverse())
  return obj_trans

class ORAgent(object):
  def __init__(self, replan=True):
    self.plan = None
    self.replan = replan
    #self.hand = 'left' if ACTIVE_LEFT else 'right'
    self.hand = 'left'
    print 'Hand', self.hand
    # TODO - move the planning here?
  def convert_action(self, belief, action, args):
    default_conf = belief.conf
    #hpn_robot = default_conf.robot
    or_robot = self.oracle.robot

    # TODO - would be good to load the grasps
    if action == 'pick':
      obj, _, grasp, _, pap = args
      grasp_conf, approach_conf = pap.grasp_config, pap.vector_config # TODO - vector config makes a big difference here?
      hpn_grasp_conf = hpn_from_or_conf(default_conf, or_robot, grasp_conf)
      hpn_approach_conf = hpn_from_or_conf(default_conf, or_robot, approach_conf)

      mu_grasp = (0, 0, 0, 0) # This is the error with respect to the grasp
      grasp_index = grasp.grasp_index

      return make_pick(self.hand, obj, grasp_index, mu_grasp, hpn_grasp_conf, hpn_approach_conf)
    elif action == 'move':
      # TODO - if the robot collides, it won't move
      # NOTE - the goal config moving to the end config is in collision?
      # NOTE - maybe the moving is all relative commands?

      start_conf, end_conf = args
      hpn_start_conf = hpn_from_or_conf(default_conf, or_robot, start_conf)
      hpn_end_conf = hpn_from_or_conf(default_conf, or_robot, end_conf)

      #assert not collision( belief.pbs.getWorld(), hpn_end_conf)

      #return make_move(hpn_start_conf, hpn_end_conf) # NOTE - more likely to complain that original conf collides - bad path?
      return make_move(default_conf, hpn_end_conf) # Default config is the mode of a belief state? Closer to the actual value
    elif action == 'move_no_base':
      start_conf, end_conf = args
      hpn_start_conf = hpn_from_or_conf(default_conf, or_robot, start_conf)
      hpn_end_conf = hpn_from_or_conf(default_conf, or_robot, end_conf)
      return make_move_no_base(hpn_start_conf, hpn_end_conf)
    elif action == 'place':
      obj, pose, _, _, pap = args
      grasp_conf, approach_conf = pap.grasp_config, pap.vector_config # TODO - vector config makes a big difference here?
      hpn_grasp_conf = hpn_from_or_conf(default_conf, or_robot, grasp_conf)
      hpn_approach_conf = hpn_from_or_conf(default_conf, or_robot, approach_conf)
      face_frames = belief.objects[obj].attrs['faceFrames'] # shape.faceFrames()

      rest_face = 4 # TODO - expand to other faces
      mu_pose = hu.Transform(trans_from_pose(pose.value)).compose(face_frames[rest_face]).pose().xyztTuple()
      #mu_pose = xyzt_from_trans(pose.value).tolist()

      return make_place(self.hand, obj, rest_face, mu_pose, hpn_grasp_conf, hpn_approach_conf)
    elif action == 'look':
      obj, look_config = args
      hpn_look_conf = hpn_from_or_conf(default_conf, or_robot, look_config)

      return make_look(obj, hpn_look_conf)
    else:
      raise NotImplementedError(action)

  def policy(self, belief, goal):

    # The positions of objects don't change as the robot moves (i.e they are in a global frame)
    # The variances grow though as the robot moves
    # The robot config has no uncertainty
    # Thus, things are sort of relative to the robot

    # Sample poses that are reachable given the uncertainty and can see the table
    # Can ignore transition uncertainty for action effects
    # Only need to keep track of uncertainty along hte first move action as a precondition
    # Can either sample robot trajectory rollouts or object poses as the robot moves with increased uncertainty

    # Just do large objects for base collisions

    from manipulation.primitives.transforms import vector_trans
    from manipulation.bodies.robot import approach_vector_from_object_trans

    DISPLAY_GRASPS_HPN = False
    if DISPLAY_GRASPS_HPN:
      obj = 'objA'
      conf = belief.pbs.getConf() # belief.conf # Default conf (i.e. base is wrong)
      grasp_desc = belief.objects[obj].attrs['graspDesc']
      shape = belief.objects[obj].attrs['shape']
      print shape.parts()[0].vertices()
      import util.windowManager3D as wm
      win = 'W'
      for i in range(len(grasp_desc)):
        wm.getWindow(win).clear()
        obj_trans = get_object_frame(conf, self.hand, grasp_desc[i])
        #approach_trans = get_approach_frame(conf, self.hand)
        #print np.round(approach_trans.matrix, 2)

        approach_vector = or_from_hpn_approach_vector(conf.robot, self.hand, grasp_desc[i])
        transformed_vector = approach_vector_from_object_trans(obj_trans.matrix, approach_vector)
        obj_trans = hu.Transform(vector_trans(obj_trans.matrix, transformed_vector))

        #temp_matrix = obj_trans.matrix.copy() # NOTE - this works as well?
        #temp_matrix[:3, 3] = approach_trans.matrix[:3, 3]
        #obj_trans = hu.Transform(temp_matrix)

        #print shape.origin() # Origin
        #print np.round(obj_trans.matrix, 2)
        shape.applyTrans(obj_trans).draw(win, color='blue')
        conf.draw(win, color='red')
        raw_input('Continue?')
      return None

    assert not check_belief_collisions(belief, .9)

    if satisfies(belief, goal):
      return []
    if self.plan is None:
      env = get_env()
      manip_problem = or_manipulation_problem(env, belief, goal)
      #manip_problem.set_viewer(env)

      self.oracle = ManipulationOracle(manip_problem, env, active_arms=[self.hand], reset_robot=False) # TODO - need to avoid resetting the original configuration
      # TODO - try just replacing the current problem

      DISPLAY_GRASPS_OR = False
      if DISPLAY_GRASPS_OR: # NOTE - OR Grasps are from Manip to Pose (not Gripper)
        from manipulation.bodies.robot import get_manip_trans, approach_vector_from_object_trans
        from manipulation.primitives.transforms import object_trans_from_manip_trans, set_trans, point_from_trans
        from manipulation.grasps.grasps import get_grasps
        from manipulation.primitives.display import draw_arrow

        obj = 'objA'
        #set_trans(self.oracle.bodies[obj], np.eye(4))
        #print self.oracle.bodies[obj].ComputeAABB()

        #for grasp in get_grasps(self.oracle, obj): # NOTE - this randomizes the grasps
        for grasp in get_or_grasps(self.oracle, obj):
          manip_trans = get_manip_trans(self.oracle)
          #grasp_trans = or_from_hpn_grasp(belief.conf.robot, self.hand, grasp_desc[i]).matrix
          obj_trans = object_trans_from_manip_trans(manip_trans, grasp.grasp_trans)
          set_trans(self.oracle.bodies[obj], obj_trans)

          approach_vector = approach_vector_from_object_trans(obj_trans, grasp.approach_vector)
          approach_trans = vector_trans(obj_trans, approach_vector)
          _ = draw_arrow(env, point_from_trans(approach_trans), point_from_trans(obj_trans))

          print
          print grasp.approach_vector
          print grasp.grasp_index
          print np.round(obj_trans, 2)
          raw_input('Continue?')
        return None

      # TODO - check that the plan is still good by converting the state

      stream_problem = compile_problem(self.oracle)

      self.oracle.draw_goals() # NOTE - this must be after compile_problem to ensure the goal_poses convert
      if is_viewer_active(self.oracle.env):
        raw_input('Start?')

      search_fn = get_fast_downward('eager', verbose=False) # dijkstra | astar | wastar1 | wastar2 | wastar3 | eager | lazy
      plan, universe = focused_planner(stream_problem, search=search_fn, greedy=False, stream_cost=10, verbose=False, debug=False)
      #plan, universe = incremental_planner(stream_problem, search=search_fn, verbose=True, debug=False)
      if plan is None:
        plan = []

      #visualize_plan(Plan(convert_state(self.oracle, stream_problem.initial_atoms),
      #                    executable_plan(self.oracle, plan)), self.oracle, display=True, save=None)
      #for state in get_states(universe, plan):
      #  or_state = convert_state(self.oracle, state)
      #  set_state(or_state, self.oracle)
      #  raw_input('Continue?')

      look_iterations = 3
      self.plan = []
      state_sequence = get_states(universe, plan)
      for i, (action, args) in enumerate(convert_plan(plan)):
        or_state = convert_state(self.oracle, state_sequence[i])
        set_state(or_state, self.oracle)
        if action.name in ['move']:
          last_config = self.oracle.get_robot_config()
          for _ in range(look_iterations):
            for obj in belief.objects:
              if or_state.get(obj, True) is not None:
                result = look_at_ik(self.oracle, obj)
                if result is not None:
                  look_config = self.oracle.get_robot_config()
                  self.plan += [
                    ('move_no_base', (last_config, look_config)),
                    ('look', (obj, look_config)),
                  ]
                  last_config = look_config # TODO - does this mean I have to change the next action config?
                  #print 'Looking at', obj
                  #raw_input('Pause')
          self.plan.append((action.name, args))
        elif action.name in ['pick', 'place']:
          obj, pose, grasp, approach_config, pap = args
          self.plan += [
            ('move_no_base', (approach_config, pap.vector_config)),
            ('move_no_base', (pap.vector_config, pap.grasp_config)),
            (action.name, args),
            ('move_no_base', (pap.grasp_config, pap.vector_config)),
            ('move_no_base', (pap.vector_config, approach_config)),
          ]
          if self.replan:
            break
        else:
          self.plan.append((action.name, args))
        #raw_input('Continue?')
      print self.plan

    if not self.plan:
      return None
    print SEPARATOR
    action, args = self.plan.pop(0)
    print action, args
    if not self.plan and self.replan:
      self.plan = None
    return [self.convert_action(belief, action, args)] # NOTE - need to return a list for now

#################################################################

class TestAgent(object):
  def __init__(self, exp, goal): # TODO - allow something else to be here?
    self.exp = exp
    self.goal = goal
    self.iterations = 0
    self.plan = None

  def policy(self, belief, goal):
    if satisfies(belief, goal):
      return []
    #if self.iterations != 0:
    #  return []
    self.iterations += 1
    if self.plan is not None:
      if len(self.plan) == 0:
        return []
      return [self.plan.pop(0)]

    print
    print belief.pbs.__class__ # PBS (Planning Belief State?)
    print belief.pbs.getWorld().__class__ # World
    # TODO - realWorld is RealWorld or RobotEnv within PlanTest. Has executePrim within it

    print 'Regions', self.exp.regions

    USE_HAND = 'right'

    print belief.makeObjGraspB(USE_HAND) # Gets the current grasp?

    conf, (leftHeld, leftObjGraspType, leftObjGrasp), \
      (rightHeld, rightObjGraspType, rightObjGrasp), objects = makeMLS(belief) # Extract MLS from b
    objects_dict = {props[0]: props[1:] for props in objects}

    robot = conf.robot
    #print conf.cartConf()
    print


    name = 'objA'
    type_name, support_face, pose = objects_dict[name]
    wrapped_pose = hu.Pose(*pose)
    identity_pose = hu.Pose(0, 0, 0, 0)

    print wrapped_pose.matrix

    print type_name, support_face, pose
    geom, _ = glob.constructor[type_name](name=name)


    grasp_desc = belief.objects[name].attrs['graspDesc']
    grasp_index = 0


    grasp = grasp_desc[grasp_index]

    # G * M  = P
    print grasp.frame # Transform
    print grasp.dx, grasp.dy, grasp.dz # Don't know what these do...

    grasp_trans = grasp.frame.inverse().compose(wrapped_pose)

    print robot.potentialBasePosesGen
    print robot.inverseKinWristGen # Allows choosing base pose
    print graspConfHypGen

    #grasp_cart = RobotCartConf(grasp_trans, robot) # First argument must be a conf?
    grasp_cart = conf.getCart()
    grasp_cart = grasp_cart.set('pr2RightArm', grasp_trans)

    print
    print 'Kinematics'
    print conf.getCart()
    #print conf
    #print robot.inverseKin(get_manip(conf), conf)
    print robot.inverseKin(grasp_cart, conf)
    print

    # TODO - need to sample the base?

    # Return plan with one move operation
    currConf = copy.copy(conf)
    goalConf = currConf.setBaseConf((0.2, 0.2, 0.2))
    self.plan = [
      make_move(currConf, goalConf),
      make_pick(USE_HAND, name, grasp_desc, grasp_index, pose, tinyVar, currConf, currConf),
    ]

    print
    print conf
    print leftHeld, leftObjGraspType, leftObjGrasp
    print rightHeld, rightObjGraspType, rightObjGrasp
    print objects

    #print goal.satisfies(belief) # Nope
    #print belief.satisfies(goal) # Nope
    print goal
    print self.plan
    raw_input('Found plan! Continue?')
    print

    return None

    #return None # Fail
    #return [] # Sucess
    return self.plan[:1] # NOTE - only the first step really matters

#################################################################

def test0():
  var = tinyVar # bigVar | tinyVar
  exp = Experiment({'tableIkea1' : (hu.Pose(1.3, 0.0, 0.0, math.pi/2.0), var)},
                   {'objA' : (hu.Pose(1.1, 0.0, ikZ, 0.0), var)},
                   ['tableIkea1Top', 'tableIkea1Left'],
                   easy=False) # easy replaces the variance

  #goal = emptyHand(hand)
  #goal = holding('objA', hand=hand, graspType=grasp_type)
  goal = inRegion(['objA'], 'tableIkea1Left')

  return exp, goal

def testBusy(hardSwap=True, **args):
    glob.rebindPenalty = 40
    # Put this back to make the problem harder
    #back = hu.Pose(1.1, 0.0, tZ, 0.0)
    back = hu.Pose(1.45, 0.0, tZ, 0.0)
    #parking1 = hu.Pose(1.15, 0.3, tZ, 0.0)
    #parking2 = hu.Pose(1.15, -0.3, tZ, 0.0)
    exp = Experiment({'table1' : (table1Pose, smallVar),
                   'table2' : (table2Pose, smallVar)},
                  {'objA' : (back, medVar),
                   'objB': (hu.Pose(1.15, -0.4, tZ, 0.0), medVar),
                   'objC': (hu.Pose(0.65, -1.2, tZ, 0.0), medVar),
                   'objD': (hu.Pose(1.15, -0.2, tZ, 0.0), medVar),
                   'objE': (hu.Pose(1.15, 0.0, tZ, 0.0), medVar),
                   'objF': (hu.Pose(1.15, 0.2, tZ, 0.0), medVar),
                   'objG': (hu.Pose(1.15, 0.4, tZ, 0.0), medVar)},
                  ['table1Top', 'table2Top', 'table1MidFront',
                   'table1MidRear'],
                  easy=args.get('easy', False))
    goal = inRegion(['objA', 'objB'], ['table1MidFront', 'table1MidRear']) # A on other table
    goal1 = inRegion('objA', 'table2Top') # A and B on other table
    goal2 = inRegion(['objA', 'objB'], 'table2Top') # B in back
    goal3 = inRegion('objB', 'table1MidRear')
    actualGoal = goal if hardSwap else goal3

    return exp, actualGoal

#################################################################

def solve_belief(env, hand='right', grasp_type=0, no_window=True):
  exp, goal = test0()

  print SEPARATOR

  #print glob.graspableNames # ['obj', 'soda', 'ts', 'handle', 'soup', 'oilBottle', 'rfunnel', 'bfunnel', 'sb']
  #print glob.pushableNames # ['obj', 'soda', 'big', 'learn', 'tall', 'bar', 'soup', 'downy', 'cascade', 'cvxcascade', 'oilBottle', 'rfunnel', 'bfunnel', 'sb']
  #print glob.crashableNames # ['table']
  for key in glob.objectDataBase: # ['useReactivePick', 'targetPourFrames', 'symmetries', 'sourcePourFrame', 'detection', 'supportRegions', 'graspDesc']
    print key, glob.objectDataBase[key], '\n'
  #print glob.objectTypes # Identify map (for now at least)?
  #print glob.constructor # Map from type to function

  print SEPARATOR

  #agent = TestAgent(exp, goal)
  agent = ORAgent()

  test = PlanTest(exp, noWindow=no_window)
  test.run(goal, planner=agent.policy)

#################################################################

def solve_deterministic(env, hand='left', no_window=True):
  test = test0 # test0 | testBusy
  exp, goal = test()

  print SEPARATOR

  test = PlanTest(exp, noWindow=no_window)
  test.load() # TODO - do I need to pass regions here?

  #test_obj = test.realWorld.objectConfs.keys()[0]
  #test.realWorld.getObjectPose()
  #print 'Initial collision:', test.realWorld.checkRobotCollision(test.realWorld.robotConf, test.realWorld.objectShapes[test_obj])
  print 'Initial collisions:', world_check_collisions(test.realWorld)

  manip_problem = real_manipulation_problem(env, test.realWorld, goal, hand)
  #manip_problem.set_viewer(env)
  oracle = ManipulationOracle(manip_problem, env, active_arms=[hand], reset_robot=False)
  stream_problem = compile_problem(oracle)
  oracle.draw_goals() # NOTE - this must be after compile_problem to ensure the goal_poses convert
  if is_viewer_active(oracle.env):
    raw_input('Start?')

  search_fn = get_fast_downward('eager', verbose=False) # dijkstra | astar | wastar1 | wastar2 | wastar3 | eager | lazy
  plan, universe = focused_planner(stream_problem, search=search_fn, greedy=False, stream_cost=10, verbose=False, debug=False)
  #plan, universe = incremental_planner(stream_problem, search=search_fn, verbose=True, debug=False)
  if plan is None:
    return
  print plan

  if no_window:
    visualize_plan(Plan(convert_state(oracle, stream_problem.initial_atoms),
                        executable_plan(oracle, plan)), oracle, display=True, save=None)

#################################################################

IDENTITY_POSE = hu.Pose(0, 0, 0, 0)

from mm_operators.mmOps import PickGen, PlaceGen, probForGenerators, PoseInRegionGen
from mm_operators.mmGen import pickGenTop, potentialGraspConfGen, placeInRegionGenGen, placeInGenTop, potentialRegionPoseGen

# NOTE - Leslie and Tomas's primitives try placing in the belief of something


from mm_operators.mmOps import moveNBPrim, movePrim # These produce motions

# NOTE - potentialGraspConfGen is used for both pick and place (duh)

pick_fn = PickGen(['Pose', 'PoseFace', 'PickConf', 'PreConf', 'ConfDelta',
               'SupportObj', 'SupportObjPoseFace'],
           ['Obj', 'GraspFace', 'GraspMu',
            'PoseVar', 'RealGraspVar', 'PoseDelta',
            'GraspDelta', 'Hand', probForGenerators])

def grasp_test(env, no_window=True, name='objA', hand='right', window_name='W'):
  exp, goal = test0()

  test = PlanTest(exp, noWindow=no_window)
  test.load() # TODO - do I need to pass regions here?
  belief = test.bs

  print SEPARATOR

  # NOTE - should cascade functions to get values
  # NOTE - I need to be able to generate from the current placement
  #domainProbs.maxGraspVar
  #graspV = pbs.beliefContext.domainProbs.maxGraspVar

  hand = 'left'
  grasp_index = 2
  inputs = [
    'objA', grasp_index, (0, 0, 0, 0),
    typicalErrProbs.placeVar, typicalErrProbs.pickVar, typicalErrProbs.placeDelta,
    typicalErrProbs.graspDelta, hand, probForGenerators
  ]
  for outputs in pick_fn.fun(inputs, goal.fluents, belief):
    print outputs
    raw_input('Continue?')
  return

  print SEPARATOR

  conf, _, _, objects = makeMLS(belief) # Extract MLS from b
  objects_dict = {props[0]: props[1:] for props in objects}

  type_name, support_face, pose = objects_dict[name]
  wrapped_pose = hu.Pose(*pose)
  grasp_desc = belief.objects[name].attrs['graspDesc'] # 1 is side grasp and 3 is top grasp

  set_or_state(env, belief.pbs.getWorld(), conf, {name: hu.Pose(*pose) for name, _, _, pose in objects})
  robot = env.GetRobots()[0]

  placeB = belief.pbs.makePlaceB(name, support=support_face, pose=wrapped_pose,
                          var=tinyVar, delta=typicalErrProbs.placeDelta) # ObjPlaceB
  for grasp_index in range(len(grasp_desc)):
    print '\nGrasp index:', grasp_index
    graspB = belief.pbs.makeGraspB(name, grasp=grasp_index, support=None, pose=IDENTITY_POSE,
                            var=tinyVar, delta=typicalErrProbs.graspDelta) # ObjGraspB
    for conf1, conf2, _ in islice(potentialGraspConfGen(belief.pbs, placeB, graspB, None, hand,
                                                        None, prob=0.001, nMax=100, findApproach=True), 10):
      set_full_config(robot, or_from_hpn_conf(robot, conf1))
      robot.SetDOFValues(np.array([0.54800022]), robot.GetManipulator(hand+'arm').GetGripperIndices())
      conf1.draw(window_name)
      raw_input('One?')
      set_full_config(robot, or_from_hpn_conf(robot, conf2))
      conf2.draw(window_name)
      raw_input('Two?')

#################################################################

def openrave_test(env): # NOTE - cannot do Leslie and Tomas's viewers with openrave
  test = testBusy # test0 | testBusy
  exp, _ = test()

  test = PlanTest(exp, noWindow=True)
  test.load()
  conf, (leftHeld, leftObjGraspType, leftObjGrasp), \
    (rightHeld, rightObjGraspType, rightObjGrasp), objects = makeMLS(test.bs)

  set_or_state(env, test.world, conf, {name: hu.Pose(*pose) for name, _, _, pose in objects})
  raw_input('Finish?')

#################################################################

def main(or_viewer=False, hpn_viewer=False):
  #exp = Experiment({'tableIkea1' : (hu.Pose(1.3, 0.0, 0.0, math.pi/2.0), TINY_VAR)},
  #                 {'objA' : (hu.Pose(1.1, 0.0, ikZ, 0.0), TINY_VAR)},
  #                 ['tableIkea1Top', 'tableIkea1Left'])
  #PlanTest(exp)
  #raw_input('Here') # NOTE - cannot run both viewers

  assert not or_viewer or not hpn_viewer
  print 'Window:', not debug('noW')

  env = Environment() # NOTE - need this for load_problem
  #execute = lambda: solve_belief(env, no_window=not hpn_viewer)
  execute = lambda: solve_deterministic(env, no_window=not hpn_viewer)
  #execute = lambda: openrave_test(env)
  #execute = lambda: grasp_test(env, no_window=not hpn_viewer)

  try:
    if or_viewer: execute_viewer(env, execute)
    else: execute()
  finally:
    if env.GetViewer() is not None:
      env.GetViewer().quitmainloop()
    RaveDestroy()

import argparse

if __name__ == '__main__':
  parser = argparse.ArgumentParser() # Automatically includes help
  parser.add_argument('-rave', action='store_true', help='enable OpenRAVE viewer.')
  parser.add_argument('-hpn', action='store_true', help='enable HPN viewer.')
  args = parser.parse_args()

  main(args.rave, args.hpn)