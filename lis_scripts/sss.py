import math
import string
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

import geometry.hu as hu
import geometry.shapes as shapes

# These are PR2 specific, could be replaced with a different robot
from redux_tests.pr2_test.pr2Robot import makeRobot
glob.usePR2(big_workspace=True)

##################################################
# Interface to BHPN
##################################################

DEBUG = False

class Environment:
    def __init__(self):
        self.world = World()
        robot = makeRobot(np.array(glob.workspace))
        self.world.setRobot(robot)
        self.robot = self.world.robot
        self.realWorld = sim.RealWorld(self.world, self.robot)
        self.grasps = {}
        self.arm = 'right'
        self.lock = True
        self.windows = {}
    def Add(self, body):                # Body instance
        self.world.addObjectShape(body.shape)
    def GetRobots(self):
        return [Robot(self)]
    def GetViewer(self):
        for window, size in self.windows.iteritems():
          wm.makeWindow(window, glob.viewPort, glob.windowSizes.get(window, size))
        return self.windows if self.windows else None
    def GetKinBody(self, objName):            # objName is string
        return Body(self, self.bodies()[objName])
    #def Load(self, dir):
    #    print 'env.Load called with', dir
    def Lock(self):
        self.lock = True
    def Unlock(self):
        self.lock = False
    def _draw_robot(self):
        for window in self.windows:
          self.conf().value.draw(window, attached=self.realWorld.attached, color='gold')
    def _draw_shape(self, shape):
        for window in self.windows:
            shape.draw(window, color=get_color(shape))
    def _clear(self):
        for window in self.windows:
            wm.getWindow(window).clear()
    def UpdatePublishedBodies(self):
        self._clear()
        self._draw_robot()
        for body in self.bodies().values():
            self._draw_shape(body)
    def set_arm(self, arm):
        self.arm = 'left' if arm[0] == 'l' else 'right'
    def set_default_robot_config(self):
        baseConf = (self.conf().value or self.robot.nominalConf).baseConf()
        conf = self.realWorld.robot.makeConf(*baseConf, ignoreStandard=True)
        self.realWorld.setRobotConf(conf) # instances of Manipulator and Conf
        #if not self.lock: self._draw_robot()
    def set_manipulator_conf(self, manipulator, conf):
        if DEBUG: print conf
        if isinstance(conf, list): # TODO - this needs to be fixed...
            chain = self.robot.armChainNames[self.arm]
            confVal = self.conf().value.set(chain, conf)
        elif isinstance(conf, Conf):
          if isinstance(conf.value, list):
              chain = self.robot.armChainNames[self.arm]
              confVal = self.conf().value.set(chain, conf.value)
          else: # ConfJointConf
              confVal = conf.value
        else:
            confVal = conf
        if DEBUG: print 'set_manipulator_conf', confVal
        self.realWorld.setRobotConf(confVal)
        #if not self.lock: self._draw_robot()
    def set_base_values(self, robot, baseConf):
        self.realWorld.setRobotConf(self.realWorld.robotConf.setBaseConf(baseConf))
    def set_pose(self, body, poseValue):
        if DEBUG: print 'set_pose', body.shape.name(), poseValue.pose()
        self.realWorld.setObjectPose(body.shape.name(), poseValue)
        #if not self.lock: self._draw_shape(self.bodies()[body.shape.name()])
    def bodies(self):
        return self.realWorld.objectShapes
    def conf(self):
        return Conf(self.realWorld.robotConf)
    def CheckCollision(self, body1, body2):
        body2Name = body2.shape.name()
        if isinstance(body1, Robot):
            robotShape = self.realWorld.robotConf.placement()
            return robotShape.collides(self.bodies()[body2Name])
        else:
            body1Name = body1.shape.name()
            return self.bodies()[body1Name].collides(self.bodies()[body2Name])

##################################################
# Types used in StripStream TAMP
##################################################

class Pose(Hashable):
    def __init__(self, pose):
        self.value = pose
        Hashable.__init__(self)
    def desc(self):
        return self.value.pose().xyztTuple()

class Grasp(Hashable):
    def __init__(self, objName, grasp, trans, approach=None):
        self.value = grasp
        self.objName = objName
        self.grasp_trans = trans
        self.approach_vector = approach
        Hashable.__init__(self)
    def desc(self):
        return (self.objName, self.value)

class Conf(Hashable):
    def __init__(self, conf):
        self.value = conf
        Hashable.__init__(self)
    def __getitem__(self, name):
        return self.value[name]
    def desc(self):
        return tuple(self.value) if isinstance(self.value, list) else self.value
    def __repr__(self):
      return self.__class__.__name__

class Traj(Hashable):
    def __init__(self, path, grasp=None):
        self.pathConfs = tuple(path)
        self.grasp = grasp
        Hashable.__init__(self)
    def path(self):
        return self.pathConfs
    def traj(self):
        return self.pathConfs
    def desc(self):
        return (self.pathConfs, self.grasp)

class Body:
    def __init__(self, env, shape):
        self.shape = shape              # shape at origin
        self.enabled = False
        #self.point = None
        self.env = env
    def Enable(self, value):
        self.enabled = value
    def GetKinematicsGeometryHash(self):
        return None                     # TODO

class Robot:
    def __init__(self, env):
        self.env = env
    def GetActiveManipulator(self):
        return Manipulator(self, self.env.arm)
    def GetConfigurationValues(self):
        return self.env.conf()
    def Grab(self, body):
        self.env.realWorld.attach(body.shape.name(), self.env.arm)
        self.env.realWorld.delObjectState(body.shape.name())
    def Release(self, body):
        detached = self.env.realWorld.detach(self.env.arm)
        self.env.realWorld.setObjectPose(detached.name(), detached.origin().pose())

class Manipulator:
    def __init__(self, robot, arm):
        self.robot = robot
        self.robot.env.set_arm(arm)
    def GetArmIndices(self):
        return self.robot.env.robot.armChainNames[self.robot.env.arm]
    def GetTransform(self):
        return self.robot.env.conf().value.cartConf()[self.GetArmIndices()]

class CSpace:                   # dummy
    @staticmethod
    def robot_arm(manipulator):
        return manipulator

class interfaces:               # dummy
    @staticmethod
    def BaseManipulation(robot, plannername=None, maxvelmult=None):
        return robot

##################################################
# Functions used in StripStream TAMP
##################################################

def initialize_openrave(env, arm, min_delta=.01):
    env.set_arm(arm)
    env.Unlock()                # make sure we draw
    if env.arm == 'left': 
        glob.useRight = False; glob.useLeft = True
    else:
        glob.useRight = True; glob.useLeft = False
    env.set_default_robot_config()
    #env.min_delta = min_delta # For OpenRAVE collision checking

def get_grasps(env, robot, body1, use_grasp_approach, use_grasp_type):
    if DEBUG: print body1.shape.name(), env.grasps[body1.shape.name()], env.grasps
    return env.grasps[body1.shape.name()]

def open_gripper(manipulator):
    env = manipulator.robot.env
    nconf = env.conf().value.set(env.robot.gripperChainNames[env.arm],
                                 [env.robot.gripMax])
    set_manipulator_conf(manipulator, Conf(nconf))

def set_default_robot_config(robot): # instance of Robot
    robot.env.set_default_robot_config()

#def set_point(body, point):
#    body.point = point
#
#def get_point(body):
#    return body.point

def set_pose(body, pose):
    poseValue = pose.value if isinstance(pose, Pose) else pose
    # Might need to center in z (and x,y?)
    body.env.set_pose(body, poseValue)

def set_base_values(robot, base):
    robot.env.set_base_values(robot, base)

set_base_conf = set_base_values

def set_manipulator_conf(manipulator, conf):
    manipulator.robot.env.set_manipulator_conf(manipulator, conf)

def object_trans_from_manip_trans(t1, t2):
    return Pose((t1.value).compose(t2.value))

def get_name(body):
    return body.shape.name()

def get_color(shape):
  return shape.properties['color']

#def _enable_all(val):
#    return

def manip_from_pose_grasp(pose, grasp):
    # This is the robot wrist
    manip_trans = pose.value.compose(grasp.grasp_trans)
    # Ad-hoc backoff strategy
    if abs(manip_trans.matrix[2,0]) < 0.1: # horizontal
        offset = hu.Pose(-glob.approachBackoff,0.,glob.approachPerpBackoff,0.)
    else:                               # vertical
        offset = hu.Pose(-glob.approachBackoff,0.,0.,0.)
    manip_trans_approach = manip_trans.compose(offset)
    # The grasp and the approach
    return manip_trans, manip_trans_approach

def solve_inverse_kinematics(env, manipulator, manip_trans):
    robot = env.robot
    hand = env.arm
    conf = env.conf().value
    if DEBUG: print 'solve_ik',
    for conf in robot.inverseKinWristGen(manip_trans, hand, conf,
                                         basePose=conf.basePose()):
        if DEBUG: print conf
        #return conf[robot.armChainNames[env.arm]]
        return conf
    print None

def vector_traj_helper(env, robot, approach_trans): # Trajectory from grasp configuration to pregrasp
    approach_conf = solve_inverse_kinematics(env, robot, approach_trans)
    path = interpolate(approach_conf, env.conf().value)
    return Traj([Conf(q) for q in path])

def cspace_traj_helper(base_manip, cspace, conf2Value, max_iterations=100):
    # cspace is a manipulator...
    env = cspace.robot.env
    initConf = env.conf().value
    destConf = conf2Value
    allowedViol = None
    moveChains = [cspace.GetArmIndices()]
    maxIter = glob.maxRRTIter
    failIter = glob.failRRTIter
    optimize = True
    path, viol = runRRT(env.realWorld, initConf, destConf, allowedViol,
                        moveChains, maxIter, failIter, optimize, False)
    #path = [initConf[cspace.GetArmIndices()], destConf]
    if DEBUG: print 'cspace_traj', path
    return Traj([Conf(q) for q in path]) if path else None

def sample_manipulator_trajectory(manipulator, traj):
    return traj.path()

##################################################
# Making box bodies
##################################################

def Ba(bb, **prop): return shapes.BoxAligned(np.array(bb), None, **prop)
def Sh(args, **prop): return shapes.Shape(list(args), None, **prop)

# Grasps
# from the side
# the z offset raises or lowers the grasp relative to midpoint of object
gMat0 = hu.Transform(np.array([(0.,1.,0.,0.),
                               (0.,0.,1.,-0.025),
                               (1.,0.,0.,0.02),
                               (0.,0.,0.,1.)]))
gMat0h = hu.Transform(np.array([(0.,1.,0.,0.0),
                               (0.,0.,1.,-0.025),
                               (1.,0.,0.,0.05),
                               (0.,0.,0.,1.)]))
gMat1 = hu.Transform(np.array([(0.,-1.,0.,0.),
                               (0.,0.,-1.,0.025),
                               (1.,0.,0.,0.02),
                               (0.,0.,0.,1.)]))
gMat1h = hu.Transform(np.array([(0.,-1.,0.,0.),
                               (0.,0.,-1.,0.025),
                               (1.,0.,0.,0.05),
                               (0.,0.,0.,1.)]))
gMat4 = hu.Pose(0,0,0,math.pi/2).compose(gMat0)
gMat5 = hu.Pose(0,0,0,-math.pi/2).compose(gMat0)
# from the top
gMat2= hu.Transform(np.array([(-1.,0.,0.,0.),
                              (0.,0.,-1.,0.025),
                              (0.,-1.,0.,0.),
                              (0.,0.,0.,1.)]))
gMat3= hu.Transform(np.array([(1.,0.,0.,0.),
                              (0.,0.,1.,-0.025),
                              (0.,-1.,0.,0.),
                              (0.,0.,0.,1.)]))

gdesc0 = lambda obj: GDesc(obj, gMat0, 0.05, 0.05, 0.025)
gdesc0h = lambda obj: GDesc(obj, gMat0h, 0.05, 0.05, 0.025)
gdesc1 = lambda obj: GDesc(obj, gMat1, 0.05, 0.05, 0.025)
gdesc1h = lambda obj: GDesc(obj, gMat1h, 0.05, 0.05, 0.025)
gdesc2 = lambda obj: GDesc(obj, gMat2, 0.05, 0.05, 0.025)
gdesc3 = lambda obj: GDesc(obj, gMat3, 0.05, 0.05, 0.025)
gdesc4 = lambda obj: GDesc(obj, gMat4, 0.05, 0.05, 0.025)
gdesc5 = lambda obj: GDesc(obj, gMat5, 0.05, 0.05, 0.025)

BLUE = 'blue'
RED = 'red'
COLORS = ['red', 'green', 'blue', 'cyan', 'purple', 'pink', 'orange']

#color_count = 0

def pickColor(name):
    #global color_count
    #color = COLORS[color_count%len(COLORS)]
    #color_count += 1
    #return color
    if name[-1] in string.uppercase:
        cn = len(COLORS)
        return COLORS[string.uppercase.index(name[-1])%cn]
    else:
        return 'black'

shape_name = 'box'

def makeBox(dx=0.025, dy=0.025, dz=0.1, name='boxA', color=None):
    glob.objectDataBase['graspDesc'][shape_name] = []
    if glob.useHorizontal:
        glob.objectDataBase['graspDesc'][shape_name] += [
          gdesc0(name), gdesc1(name), gdesc4(name), gdesc5(name)]
    if glob.useVertical:
        glob.objectDataBase['graspDesc'][shape_name] += [gdesc2(name), gdesc3(name)]
    color = color or pickColor(name)
    return Sh([Ba([(-dx, -dy, 0.), (dx, dy, dz)])], name=name, color=color), []

glob.objectTypes[shape_name] = 'box'
glob.constructor[shape_name] = makeBox
glob.useHorizontal = False
glob.useVertical = True

def box_body(env, length, width, height, name='box', color='black'):
    def trans(graspIndex):
        # We're using the origin of object, so don't need to specify restFace
        wrist = objectGraspFrameAux(gd, graspIndex, graspMu,
                                    None, None, Ident,
                                    robot, hand, origin=True)
        return wrist
    shape, _ = glob.constructor[shape_name](dx=length/2., dy=width/2., dz=height, name=name, color=color)
    if length <= 0.7 and width <= 0.7:  # graspable
        gd = env.world.getObjData(shape_name, 'graspDesc', [])
        hand = env.arm
        robot = env.robot
        graspMu = hu.Pose(0.0, -0.025, 0.0, 0.0) # Note !! constant
        Ident = hu.Pose(0,0,0,0)
        env.grasps[name] = [Grasp(name, i, trans(i)) for i,_ in enumerate(gd)]
    else:
        env.grasps[name] = []
    return Body(env, shape)

##################################################
# Creating problems
##################################################

class ManipulationProblem:
    def __init__(self, name,
                 object_names=[], table_names=[], initial_poses={},
                 goal_poses={}, known_poses=[]):
        self.name = name
        self.object_names = object_names
        self.table_names = table_names
        self.initial_poses = initial_poses
        self.goal_poses = goal_poses
        self.known_poses = known_poses

BODY_PLACEMENT_Z_OFFSET = 1e-3
#REARRANGEMENT = True
#ENVIRONMENTS_DIR = './'

def function_name(stack): # NOTE - stack = inspect.stack()
  return stack[0][3]

def flatten(x):
    xflat = []
    for xi in x: xflat.extend(xi)
    return xflat

def unit_quat():
  return np.array([0,0,0,1])

def pose_from_quat_point(quat, xyz):
    angle = euler_from_quaternion(quat)[2] # z rotation
    return hu.Pose(xyz[0], xyz[1], xyz[2], angle)

##################################################
# Problem definitions
##################################################

# This is taken verbatim from manipulation.problems.distribution
def dantam2(env): # (Incremental Task and Motion Planning: A Constraint-Based Approach)
  #assert REARRANGEMENT
  #env.Load(ENVIRONMENTS_DIR + 'empty.xml')

  m, n = 3, 3
  #m, n = 5, 5
  n_obj = 2
  side_dim = .07 # .05 | .07
  height_dim = .1
  box_dims = (side_dim, side_dim, height_dim)
  separation = (side_dim, side_dim)
  #separation = (side_dim/2, side_dim/2)

  coordinates = list(product(range(m), range(n)))
  assert n_obj <= len(coordinates)
  obj_coordinates = sample(coordinates, n_obj)

  length = m*(box_dims[0] + separation[0])
  width = n*(box_dims[1] + separation[1])
  height = .7
  table = box_body(env, length, width, height, name='table', color='brown')
  #set_point(table, (0, 0, 0))
  env.Add(table)
  set_pose(table, Pose(pose_from_quat_point(unit_quat(), np.array([0, 0, height/2]))))

  robot = env.GetRobots()[0]
  set_default_robot_config(robot)
  set_base_conf(robot, (-.75, .2, -math.pi/2))

  poses = []
  #z = get_point(table)[2] + height + BODY_PLACEMENT_Z_OFFSET
  z = height + height_dim/2 + BODY_PLACEMENT_Z_OFFSET
  for r in range(m):
    row = []
    #x = get_point(table)[0] - length/2 +  (r+.5)*(box_dims[0] + separation[0])
    x = - length/2 + (r+.5)*(box_dims[0] + separation[0])
    for c in range(n):
      #y = get_point(table)[1] - width/2 + (c+.5)*(box_dims[1] + separation[1])
      y = - width/2 + (c+.5)*(box_dims[1] + separation[1])
      row.append(Pose(pose_from_quat_point(unit_quat(), np.array([x, y, z]))))
    poses.append(row)

  initial_poses = {}
  goal_poses = {}
  for i, (r, c) in enumerate(obj_coordinates):
    row_color = np.zeros(4)
    row_color[2-r] = 1.
    if i == 0:
      name = 'goal%d-%d'%(r, c)
      color = BLUE
      goal_poses[name] = poses[m/2][n/2]
    else:
      name = 'block%d-%d'%(r, c)
      color = RED
    initial_poses[name] = poses[r][c]
    obj = box_body(env, *box_dims, name=name, color=color)
    env.Add(obj)                # was not here - TLP
    set_pose(obj, poses[r][c].value)

  known_poses = list(flatten(poses))

  return ManipulationProblem(
      function_name(inspect.stack()),
      object_names=initial_poses.keys(), table_names=[get_name(table)],
      goal_poses=goal_poses,
      initial_poses=initial_poses, known_poses=known_poses)
