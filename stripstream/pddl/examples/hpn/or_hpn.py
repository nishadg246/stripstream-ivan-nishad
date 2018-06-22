import numpy as np
from mm_operators.mmBeliefState import typeOfObjName
from openravepy import RaveCreateKinBody

from manipulation.bodies.bodies import set_name, set_color, set_transparency
from manipulation.primitives.transforms import set_full_config, set_trans, pose_from_base_values, base_values_from_pose


#{'pr2LeftGripper': [0.07], 'pr2RightArm': [-0.9064216613769531, 0.15107053518295288, -1.399999976158142, -2.0179331302642822, -1.6354209184646606, -1.7891108989715576, -2.0501229763031006],
# 'pr2Base': [0.0, 0.0, 0.0], 'pr2Torso': [0.3], 'pr2RightGripper': [0.07], 'pr2Head': [0.0, 0.0], 'pr2LeftArm': [2.1, 1.29, 0.0, -0.15, 0.0, -0.1, 0.0]}

# print [str(joint.GetName()) for joint in robot.GetJoints()]
# ['bl_caster_rotation_joint', 'bl_caster_l_wheel_joint', 'bl_caster_r_wheel_joint', 'br_caster_rotation_joint',
# 'br_caster_l_wheel_joint', 'br_caster_r_wheel_joint', 'fl_caster_rotation_joint', 'fl_caster_l_wheel_joint',
# 'fl_caster_r_wheel_joint', 'fr_caster_rotation_joint', 'fr_caster_l_wheel_joint', 'fr_caster_r_wheel_joint',
# 'torso_lift_joint', 'head_pan_joint', 'head_tilt_joint', 'l_shoulder_pan_joint', 'l_shoulder_lift_joint',
# 'l_upper_arm_roll_joint', 'l_elbow_flex_joint', 'l_forearm_roll_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint',
# 'l_gripper_l_finger_joint', 'l_gripper_motor_slider_joint', 'l_gripper_motor_screw_joint', 'l_gripper_joint',
# 'laser_tilt_mount_joint', 'r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint',
# 'r_elbow_flex_joint', 'r_forearm_roll_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint', 'r_gripper_l_finger_joint',
# 'r_gripper_motor_slider_joint', 'r_gripper_motor_screw_joint', 'r_gripper_joint', 'torso_lift_motor_screw_joint']
from redux_tests import planGlobals as glob
from util.colorNames import colors
from geometry.conf import RobotJointConf

def get_color(shape):
  return shape.properties['color']

def convert_color(name):
  return np.array(colors[name])/255.

#################################################################

def box_body(env, shape, name=None, transparency=None):
  boxes = []
  for part in shape.parts():
    lower, upper = part.bbox()
    boxes.append(np.concatenate([(lower+upper)/2., (upper-lower)/2.]))
    #print part.vertices(), part.origin()
  body = RaveCreateKinBody(env, '')
  body.InitFromBoxes(np.array(boxes), draw=True) # TODO - array of boxes
  if name is not None: set_name(body, name)
  set_color(body, convert_color(get_color(shape)))
  if transparency is not None:
    set_transparency(body, transparency)
  return body

def add_body(env, world, name, pose):
#def add_body(env, name, pose):
  #type_name = world.getObjType(name) # NOTE - could also get this from the world
  type_name = typeOfObjName(name)
  shape, _ = glob.constructor[type_name](name=name) # NOTE - this makes the center be at +dz/2
  #shape = belief.objects[obj].attrs['shape']
  shape = world.getObjectShapeAtOrigin(name)
  body = box_body(env, shape, name=name)
  if pose is not None:
    set_trans(body, pose.matrix)
  env.Add(body) # Add the default pose if no pose
  return body

def set_or_state(env, world, conf, objects):
#def convert_state(env, conf, objects):
  env.Reset()
  robot = env.ReadRobotXMLFile('robots/pr2-beta-static.zae') # TODO - load if not already loaded
  env.Add(robot)
  set_full_config(robot, or_from_hpn_conf(robot, conf))
  bodies = []
  for name, pose in objects.iteritems():
    bodies.append(add_body(env, world, name, pose))
  return robot, bodies

#################################################################

BASE_NAME = 'pr2Base'
GRIPPER_NAMES = ['pr2LeftGripper', 'pr2RightGripper']

def or_joints_from_hpn_name(robot, name):
  if name == 'pr2LeftGripper':
    return robot.GetManipulator('leftarm').GetGripperIndices()
  if name == 'pr2RightGripper':
    return robot.GetManipulator('rightarm').GetGripperIndices()
  if name == 'pr2LeftArm':
    return robot.GetManipulator('leftarm').GetArmIndices()
  if name == 'pr2RightArm':
    return robot.GetManipulator('rightarm').GetArmIndices()
  if name == 'pr2Torso':
    return [robot.GetJointIndex('torso_lift_joint')]
  if name == 'pr2Head':
    return [robot.GetJointIndex(joint) for joint in ('head_pan_joint', 'head_tilt_joint')]
  if name == BASE_NAME:
    #return range(len(robot.GetJoints()), len(robot.GetJoints())+7)
    return range(-4, 0) + range(-7, -4)
  raise ValueError(name)

def or_from_hpn_conf(robot, conf):
  hpn_conf = conf.conf.copy()
  hpn_conf[BASE_NAME] = pose_from_base_values(hpn_conf[BASE_NAME])
  or_conf = robot.GetConfigurationValues()
  for name, value in hpn_conf.iteritems():
    indices = or_joints_from_hpn_name(robot, name)
    or_conf[indices] = value
  return or_conf

#def hpn_from_or_conf(hpn_robot, or_robot, or_conf):
def hpn_from_or_conf(default_conf, or_robot, or_conf):
  #hpn_conf = {}
  hpn_robot = default_conf.robot
  hpn_conf = default_conf.conf.copy()
  for name in hpn_robot.chainNames:
    if name not in GRIPPER_NAMES: # TODO - OR invalid gripper value
      indices = or_joints_from_hpn_name(or_robot, name)
      hpn_conf[name] = or_conf.value[indices].tolist() # NOTE - tolist() is import for hashing
  hpn_conf[BASE_NAME] = base_values_from_pose(hpn_conf[BASE_NAME]).tolist()
  return RobotJointConf(hpn_conf, hpn_robot)

#################################################################

from manipulation.constants import APPROACH_DISTANCE


SIDE_APPROACH_TRANS = np.eye(4)
SIDE_APPROACH_TRANS[:3, 3] = np.array([.5, 1, 0])
SIDE_APPROACH_TRANS[:3, 3] *= APPROACH_DISTANCE
TOP_APPROACH_TRANS = np.eye(4)
TOP_APPROACH_TRANS[:3, 3] = np.array([0, 0, 0])
#TOP_APPROACH_TRANS[:3, 3] = np.array([0, 1, 0]) # OpenRAVE applies the APPROACH_DISTANCE later
#TOP_APPROACH_TRANS[:3, 3] = np.array([1, 0, 0])
#TOP_APPROACH_TRANS[:3, 3] = np.array([0, 0, 1])
TOP_APPROACH_TRANS[:3, 3] *= APPROACH_DISTANCE

def get_hpn_grasp_tform(hpn_robot, hand, grasp_desc):
  grasp_tform = grasp_desc.frame.compose(hu.Pose(0,0,grasp_desc.dz,0))
  #grasp_tform = grasp_desc.frame
  return hpn_robot.gripperFaceFrame[hand].compose(grasp_tform.inverse())

  #return hpn_robot.gripperFaceFrame[hand].compose(get_grasp_trans(grasp_desc).inverse()).inverse()
  #return hpn_robot.gripperFaceFrame[hand].compose(get_grasp_trans(grasp_desc))
  #return get_grasp_trans(grasp_desc)
  #return get_grasp_trans(grasp_desc).inverse()
  #gripper_matrix = np.eye(4)
  #gripper_matrix[:3, :3] = hpn_robot.gripperFaceFrame[hand].matrix[:3, :3]
  #gripper_trans = hu.Transform(gripper_matrix)
  #return gripper_trans.compose(get_grasp_trans(grasp_desc).inverse())

def or_from_hpn_grasp(hpn_robot, hand, grasp_desc):
  return hpn_robot.toolOffsetZ[hand].inverse().compose(get_hpn_grasp_tform(hpn_robot, hand, grasp_desc)).matrix

# OpenRAVE defines the approach vector relative to object

def get_hpn_approach_vector(hpn_robot, hand):
  # AV = A * W or W * A
  # G = FF * (GF * DZ).T
  # P = W * G
  approach_trans = hu.Transform(TOP_APPROACH_TRANS)
  return hpn_robot.gripperFaceFrame[hand].compose(approach_trans)
  #return approach_trans

def make_hpn_point(x, y, z):
  return hu.Point(np.array([[x], [y], [z], [1.0]]))

def or_from_hpn_approach_vector(hpn_robot, hand, grasp_desc):
  approach_vector = -np.array([0., 1., 0.])
  grasp_tform = get_hpn_grasp_tform(hpn_robot, hand, grasp_desc)
  relation = (hpn_robot.gripperFaceFrame[hand].inverse().compose(grasp_tform)).inverse()
  point = make_hpn_point(*approach_vector)
  return relation.applyToPoint(point).matrix[:3,0]

# TODO - vary based on side or top grasp
#def or_from_hpn_approach_vector(hpn_robot, hand):
def get_approach_frame(conf, hand):
  #approach_trans = hu.Transform(TOP_APPROACH_TRANS)
  cart = conf.cartConf()
  wrist_frame = cart[conf.robot.armChainNames[hand]]
  # NOTE - maybe I don't need the face frame?
  #return approach_trans.compose(wrist_frame.compose(conf.robot.gripperFaceFrame[hand])) # Point is +z, perpendicular is +x, side is +y
  #return wrist_frame.compose(conf.robot.gripperFaceFrame[hand]).compose(approach_trans) # Point is +y, perpendicular is +x, side is +z
  return  wrist_frame.compose(get_hpn_approach_vector(conf.robot, hand))

def get_object_frame(conf, hand, gdesc): # mmUtil -> objectGraspFrame
  # W = P * G * ?? * Tool.T
  #wristFrame = obj_frame.compose(gdesc.frame.compose(hu.Pose(0,0,gdesc.dz,0))).compose(robot.gripperFaceFrame[hand].inverse())
  cart = conf.cartConf()
  wrist_frame = cart[conf.robot.armChainNames[hand]]
  #return wrist_frame.compose(gdesc.frame.compose(hu.Pose(0,0,gdesc.dz,0)).compose(conf.robot.gripperFaceFrame[hand].inverse()).inverse())
  return wrist_frame.compose(get_hpn_grasp_tform(conf.robot, hand, gdesc))

#################################################################

def get_real_world(): # NOTE - seems to still be None
  return glob.realWorld

def get_grasp_descs(obj_name):
  obj_type = typeOfObjName(obj_name)
  return glob.objectDataBase['graspDesc'].get(obj_type, [])
  #return glob.objectDataBase['graspDesc'].get(obj_name, [])

#def get_object_names():
#  # NOTE - glob.objectTypes maps to itself
#  #return glob.objectTypes.keys()

from manipulation.constants import REGION_Z_OFFSET
from manipulation.regions import AARegion
from manipulation.problems.problem import ManipulationProblem

from mm_operators.mmFluents import In, Holding
from hpn.flatMLSReplanner import makeMLS, makeRealState
import geometry.hu as hu
from bhpn.belief import Bd
from manipulation.grasps.grasps import Grasp

# 3 Options
# - Convert to OR and solve
# - Solve within HPN
# - Use a mix of methods (ex HPN grasps by OR motion)

# TODO - avoid recomputing the problem when making the oracle

from manipulation.primitives.transforms import get_trans

def get_gripper_tform(manipulator):
  # NOTE - HPN uses gripper tform instead of manipulator tform
  manip_tform = get_trans(manipulator)
  tool_tform = manipulator.GetLocalToolTransform()
  #print tool_tform
  #[[ 0.    0.    1.    0.18]
  # [ 0.    1.    0.    0.  ]
  # [-1.    0.    0.    0.  ]
  # [ 0.    0.    0.    1.  ]]
  #print gFaceFrame.matrix
  # [[ 0.    1.    0.    0.18]
  #  [ 0.    0.    1.    0.  ]
  #  [ 1.    0.    0.    0.  ]
  #  [ 0.    0.    0.    1.  ]]
  #print left_gripperToolOffsetX.matrix
  # [[ 1.   -0.    0.    0.18]
  #  [ 0.    1.    0.    0.  ]
  #  [ 0.    0.    1.    0.  ]
  #  [ 0.    0.    0.    1.  ]]
  #print left_gripperToolOffsetZ.matrix # = tool_tform
  # [[ 0.    0.    1.    0.18]
  #  [ 0.    1.    0.    0.  ]
  #  [-1.    0.    0.    0.  ]
  #  [ 0.    0.    0.    1.  ]]
  return np.dot(manip_tform, np.linalg.inv(tool_tform))
  # = conf.cartConf()[conf.robot.armChainNames[hand]].matrix # Equal to the following

from manipulation.constants import USE_GRASP_APPROACH, USE_GRASP_TYPE
def get_or_grasps(oracle, obj_name):
  grasp_key = (oracle.get_geom_hash(obj_name), USE_GRASP_APPROACH, USE_GRASP_TYPE)
  return oracle.grasp_database[grasp_key]

# Old HPN Grasps
# Hand = Pose*Grasp
# Manip*Tool = Hand
# Manip*Tool*Grasp.T = Pose
# self.gripper = world.gripperAtOrigin()
# pr2LeftArmFrame = hand_pose.compose(self.world.gripperTip.inversePose())
# hand_pose = placement.compose(grasp)

# OR Grasps
# Manip*Grasp = Pose

# New HPN Grasps
#robot = robot
#cart = self.robotConf.cartConf()
#tool = robot.toolOffsetX[hand]
#handPose = cart[robot.armChainNames[hand]].compose(robot.toolOffsetX[hand])
#grasp = handPose.inverse().compose(opose)
# Grasp = Hand.T * Pose
# Hand*Grasp = Pose
# executePick from redux_tests.sim

def is_under_grasp(grasp):
  return grasp.grasp_trans[0,3] > 0

# TODO - identify top grasps
def make_or_grasps(hpn_robot, hand, grasp_descs, top=True, side=False):
  grasps = []
  if not grasp_descs:
    return grasps
  side_indices = range(0, 2)
  top_indices = range(2, 4)
  use_indices = []
  if side: use_indices += side_indices
  if top: use_indices += top_indices

  for grasp_index in use_indices:
    assert grasp_index < len(grasp_descs)
    grasp_desc = grasp_descs[grasp_index]
    grasp_trans = or_from_hpn_grasp(hpn_robot, hand, grasp_desc)
    #approach_trans = get_hpn_approach_vector(hpn_robot, hand).matrix
    #relative_trans = np.linalg.solve(grasp_trans, approach_trans)[:3, 3]
    approach_vector = or_from_hpn_approach_vector(hpn_robot, hand, grasp_desc)

    grasp = Grasp(grasp_trans, np.array([.1]), None, approach_vector)
    grasp.grasp_index = grasp_index
    grasp.grasp_desc = grasp_descs
    grasps.append(grasp)
  return grasps

def make_or_regions(obst_pose, reg_list):
  regions = []
  for reg_name, box, reg_pose in reg_list:
    box = box.applyTrans(obst_pose.compose(reg_pose.inverse()))
    #box = box.applyTrans(reg_pose.inverse().compose(obst_pose)) # NOTE - both are equivalent here
    color = np.random.uniform(0, 1, 4)
    color[3] = .5
    #color = convert_color(get_color(box))
    lower, upper = box.bbox()
    regions.append(AARegion(reg_name, zip(lower[:2], upper[:2]), upper[2] + REGION_Z_OFFSET, color=color))
  return regions

# belief # ['grasps', 'planningExceptions', 'conf', 'pbs', 'beliefContext', 'held', 'objects', 'domain', 'graspTypes', 'relPoseCache', 'ukf']
# context = belief.beliefContext # ['genCacheStats', 'domainProbs', 'roadMap', 'regions', 'genCaches', 'world']

def or_manipulation_problem(env, belief, goal, hand='right'):
  world = belief.pbs.getWorld() # belief.beliefContext.world
  state = makeMLS(belief) # Extract MLS from b
  return get_manipulation_problem(env, world, state, goal, hand=hand)

def real_manipulation_problem(env, real_world, goal, hand='right'):
  world = real_world.world
  state = makeRealState(real_world)
  return get_manipulation_problem(env, world, state, goal, hand=hand)

def get_manipulation_problem(env, world, (conf, left_info, right_info, objects), goal, hand='right'):
  #grasps = {obj: make_or_grasps(belief.pbs.getRobot(), hand, belief.objects[obj].attrs['graspDesc']) for obj in belief.objects}
  grasps = {obj: make_or_grasps(conf.robot, hand, get_grasp_descs(obj), top=True, side=True) for obj in world.objects}

  object_dict = {name: hu.Pose(*pose) for name, _, _, pose in objects}
  # print glob.graspableNames # ['obj', 'soda', 'ts', 'handle', 'soup', 'oilBottle', 'rfunnel', 'bfunnel', 'sb']

  start_holding = False
  left_held, left_grasp_index, left_grasp_delta = left_info
  if left_held is not None:
    start_holding = left_held, grasps[left_held][left_grasp_index] # TODO - this might not be a grasp allowed in grasps
    object_dict[left_held] = None # TODO - make this the held transform?
  right_held, right_grasp_index, right_grasp_delta = right_info
  if right_held is not None:
    start_holding = right_held, grasps[right_held][right_grasp_index]
    object_dict[right_held] = None
  assert left_held is None or right_held is None

  set_or_state(env, world, conf, object_dict)

  obstacle_names = filter(lambda name: not grasps[name], world.objects)
  #obstacle_names = filter(lambda name: belief.objects[name].attr('fixed'), belief.objects)
  object_names = filter(lambda name: name not in obstacle_names, world.objects)
  #object_names = filter(lambda name: name not in obstacle_names, belief.objects)
  table_names = filter(lambda name: 'table' in name, obstacle_names) # TODO - can infer this from shape type of obstacle_names
  #table_names = glob.objectDataBase['supportRegions'].keys()

  regions = []
  for obst_name, reg_list in world.regions.iteritems():
    regions += make_or_regions(object_dict[obst_name], reg_list)

  goal_holding = None
  goal_regions = {}
  for fluent in goal.fluents:
    if isinstance(fluent, Bd):
      literal, arg, prob = fluent.args
      if isinstance(literal, In):
        obj, region = literal.args
        goal_regions[obj] = region
      elif isinstance(literal, Holding):
        #hand, = literal.args
        obj = literal.value # This equals arg?
        goal_holding = False if obj == 'none' else obj
      else:
        raise NotImplementedError(literal)
    else:
      raise NotImplementedError(fluent)

  return ManipulationProblem('bhpn', start_holding=start_holding,
      object_names=object_names, table_names=table_names, regions=regions,
      goal_regions=goal_regions, goal_holding=goal_holding, grasps=grasps)
