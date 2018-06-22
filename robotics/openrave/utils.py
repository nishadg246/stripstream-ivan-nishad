import math
import random
import sys
from itertools import count
from random import uniform

import numpy as np
from openravepy import rotationMatrixFromAxisAngle, Sensor, RaveCreateCollisionChecker, databases, interfaces, \
  IkParameterization, GeometryType, RaveCreateKinBody, planning_error, CollisionOptionsStateSaver, DOFAffine, \
  RaveGetEnvironment, RaveGetAffineDOFValuesFromTransform, openravepy_int
from openravepy.misc import SetViewerUserThread

from robotics.openrave.transforms import quat_from_axis_angle, trans_from_pose, manip_trans_from_object_trans, \
  length, normalize, trans_from_point, get_point, \
  trans_from_quat, trans_from_axis_angle, set_quat, \
  pose_from_quat_point, unit_quat, quat_from_angle_vector
from stripstream.utils import irange, INF

MIN_DELTA = 0.01
SURFACE_Z_OFFSET = 1e-3
GRIPPER_WIDTH = 0.07

AFFINE_MASK = DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis
ROTATION_AXIS = (0, 0, 1)

# TODO - method that creates these classes
class Wrapper(object):
  def __init__(self, value):
    self.id = next(self._ids)
    self.value = value
  def __repr__(self):
    return self.__class__.__name__ + '(%d)'%self.id #str_object(self.value[4:7])

class Conf(Wrapper): _ids = count(0)
class Pose(Wrapper): _ids = count(0)
class Grasp(Wrapper): _ids = count(0)
class Traj(Wrapper): _ids = count(0)

#################################################################

def get_env(body):
  return RaveGetEnvironment(body.GetEnvironmentId())

def get_name(body):
  return str(body.GetName())

def get_geometries(body):
  return (geometry for link in body.GetLinks() for geometry in link.GetGeometries())

def in_env(body):
  return body.GetEnvironmentId != 0

def set_color(body, color):
  for geometry in get_geometries(body):
    geometry.SetDiffuseColor(color)
    #geometry.SetAmbientColor(color) # Color when in shadow
    #geometry.SetTransparency(0)

def set_transparency(body, transparency):
  for geometry in get_geometries(body):
    geometry.SetTransparency(transparency)

def get_box_dimensions(box):
  [link] = box.GetLinks()
  [geometry] = link.GetGeometries()
  assert geometry.GetType() == GeometryType.Box
  return 2*geometry.GetBoxExtents()

def box_body(env, name, dx, dy, dz, color=None, transparency=None):
  body = RaveCreateKinBody(env, '')
  body.InitFromBoxes(np.array([[0, 0, .5*dz, .5*dx, .5*dy, .5*dz]]), draw=True)
  body.SetName(name)
  if color is not None:
    set_color(body, color)
  if transparency is not None:
    set_transparency(body, transparency)
  return body

def mirror_arm_config(robot, config):
  return config*np.array([1 if left_min == right_min else -1 for left_min, right_min in
                    zip(robot.GetDOFLimits(robot.GetManipulator('leftarm').GetArmIndices())[0],
                        robot.GetDOFLimits(robot.GetManipulator('rightarm').GetArmIndices())[0])])

#################################################################

def set_manipulator_conf(manipulator, values):
  manipulator.GetRobot().SetDOFValues(values, manipulator.GetArmIndices())

def set_base_conf(body, base_values):
  trans = body.GetTransform()
  trans[:3, :3] = rotationMatrixFromAxisAngle(np.array([0, 0, base_values[-1]])) # matrixFromAxisAngle
  trans[:2, 3] = base_values[:2]
  body.SetTransform(trans)

#################################################################

def set_gripper(manipulator, values):
  manipulator.GetRobot().SetDOFValues(values, manipulator.GetGripperIndices())

def get_open_conf(manipulator):
  _, upper = manipulator.GetRobot().GetDOFLimits(manipulator.GetGripperIndices())
  return upper

def get_close_conf(manipulator):
  lower, _ = manipulator.GetRobot().GetDOFLimits(manipulator.GetGripperIndices())
  return lower

def get_gripper(manipulator):
  return manipulator.GetRobot().GetDOFValues(manipulator.GetGripperIndices())

def is_gripper_open(manipulator):
  return np.allclose(get_gripper(manipulator), get_open_conf(manipulator), atol=1e-1, rtol=0)

def is_gripper_closed(manipulator):
  return np.allclose(get_gripper(manipulator), get_close_conf(manipulator), atol=1e-1, rtol=0)

def open_gripper(manipulator):
  set_gripper(manipulator, get_open_conf(manipulator))

def close_gripper(manipulator):
  set_gripper(manipulator, get_close_conf(manipulator))

def simulate_grasp(task_manip):
  return task_manip.CloseFingers(offset=None, movingdir=None, execute=False, outputtraj=False, outputfinal=True,
                coarsestep=None, translationstepmult=None, finestep=None, outputtrajobj=True)

#################################################################

# TODO - base sampler that just samples uniformly at random
# TODO - should these include collision checking or not?

def random_inverse_reachability(ir_model, manip_trans):
  index_manip_iterator = [(manip_trans, manip_trans)]
  try:
    for base_trans, _, _ in ir_model.randomBaseDistributionIterator(index_manip_iterator):
      #set_trans(ir_model.robot, base_trans)
      yield base_trans
  except (ValueError, planning_error): # ValueError: low >= high because of an empty sequence
    raise StopIteration

def openrave_inverse_reachability(ir_model, manip_trans):
  index_manip_iterator = [(manip_trans, manip_trans)]
  try:
    for base_trans, _, _ in ir_model.sampleBaseDistributionIterator(index_manip_iterator, logllthresh=-INF, Nprematuresamples=1):
      #set_trans(ir_model.robot, base_trans)
      #if env.CheckCollision(robot) or robot.CheckSelfCollision():
      yield base_trans
  except (ValueError, planning_error): # ValueError: low >= high because of an empty sequence
    raise StopIteration

def solve_inverse_kinematics(manipulator, manip_trans, collisions=True):
  robot = manipulator.GetRobot()
  env = robot.GetEnv()
  with robot:
    robot.SetActiveDOFs(manipulator.GetArmIndices())
    #with collision_saver(env, openravepy_int.CollisionOptions.ActiveDOFs):
    config = manipulator.FindIKSolution(manip_trans, 0) # Finds a solution near the robot's current joint values
    if config is None:
      return None
    robot.SetDOFValues(config, manipulator.GetArmIndices())
    if collisions and (env.CheckCollision(robot) or robot.CheckSelfCollision()):
      return None
    return config

#################################################################

def manip_from_pose_grasp(pose, grasp):
  # base_from_gripper * gripper_from_obj = base_from_obj
  # base_from_obj * (gripper_from_obj)^-1 = gripper_from_obj
  # arm.GetLocalToolDirection() == [ 0.  0.  1.] (z is the approach direction)
  return manip_trans_from_object_trans(trans_from_pose(pose), grasp)

# TODO: check limits for boxes
def top_grasps(box): # Rotate around z axis
  # returns gripper_from_obj
  (w, l, h) = get_box_dimensions(box)
  origin = trans_from_point(0, 0, -h)
  reflect = trans_from_quat(quat_from_axis_angle(0, -math.pi, 0))
  for i in range(4):
    rotate_z = trans_from_axis_angle(0, 0, i*math.pi/2)
    yield reflect.dot(origin).dot(rotate_z)

def side_grasps(box, under=True): # Convert to z then rotate around it?
  (w, l, h) = get_box_dimensions(box)
  origin = trans_from_point(0, 0, -h/2)
  for j in range(1 + under):
    swap_xz = trans_from_axis_angle(0, -math.pi/2 + j*math.pi, 0)
    for i in range(4):
      rotate_z = trans_from_axis_angle(0, 0, i*math.pi/2)
      yield swap_xz.dot(rotate_z).dot(origin)

#################################################################

PREGRASP_DISTANCE = 0.1

class Attachment(object):
  def sample(self):
    raise NotImplementedError()

class SingleGrasp(Attachment):
  def __init__(self, origin_grasp):
    self.origin_grasp = origin_grasp
    pregrasp_vector = PREGRASP_DISTANCE * normalize(np.array([0, 0, -1]))
    self.gripper_from_pregrasp = trans_from_point(*pregrasp_vector)
  def sample(self):
    return self.origin_grasp, self.gripper_from_pregrasp

class InvariantGrasp(Attachment):
  def sample(self):
    rotation = quat_from_angle_vector(random.uniform(0, 2*np.pi), [0, 0, 1])
    gripper_from_obj = self.origin_grasp.dot(trans_from_quat(rotation))
    return gripper_from_obj, self.gripper_from_pregrasp
  # TODO: halton sequence to ensure we cover nicely
  def compute(body):
    raise NotImplementedError()

class CylinderSideXGrasp(InvariantGrasp):
  # TODO: could instead just sample poses for rotationally invariant things
  def __init__(self, radius):
    swap_xz = trans_from_quat(quat_from_angle_vector(math.pi / 2, [0, 1, 0]))
    translate = trans_from_point(0, 0, radius)
    self.origin_grasp = translate.dot(swap_xz)
    pregrasp_vector = PREGRASP_DISTANCE * normalize(np.array([0.5, 0, -1]))
    self.gripper_from_pregrasp = trans_from_point(*pregrasp_vector)
  @staticmethod
  def compute(body):
    radius, height = get_mesh_radius(body), get_mesh_height(body)
    if GRIPPER_WIDTH < 2 * radius:
      return None
    return CylinderSideXGrasp(radius)

class CylinderSideYGrasp(InvariantGrasp):
  # TODO: could instead just sample poses for rotationally invariant things
  def __init__(self, radius):
    swap_xz = trans_from_quat(quat_from_angle_vector(math.pi / 2, [0, 1, 0]))
    swap_xy = trans_from_quat(quat_from_angle_vector(math.pi / 2, [0, 0, 1]))
    translate = trans_from_point(0, 0, radius)
    self.origin_grasp = translate.dot(swap_xy).dot(swap_xz)
    pregrasp_vector = PREGRASP_DISTANCE * normalize(np.array([0, 0, -1]))
    self.gripper_from_pregrasp = trans_from_point(*pregrasp_vector)
  @staticmethod
  def compute(body):
    radius, height = get_mesh_radius(body), get_mesh_height(body)
    if GRIPPER_WIDTH < height:
      return None
    return CylinderSideYGrasp(radius)

class CylinderTopGrasp(InvariantGrasp):
  # TODO: could instead just sample poses for rotationally invariant things
  def __init__(self, height):
    bottom = trans_from_point(0, 0, -height / 2)
    reflect = trans_from_quat(quat_from_axis_angle(0, -math.pi, 0))
    self.origin_grasp = reflect.dot(bottom)
    pregrasp_vector = PREGRASP_DISTANCE * normalize(np.array([0, 0, -1]))
    self.gripper_from_pregrasp = trans_from_point(*pregrasp_vector)
  @staticmethod
  def compute(body):
    radius, height = get_mesh_radius(body), get_mesh_height(body)
    if GRIPPER_WIDTH < 2 * radius:
      return None
    return CylinderTopGrasp(height)

def cylinder_contact(radius, height, base_offset=0.01): # base_offset=0.01
  swap_xz = trans_from_quat(quat_from_angle_vector(-math.pi, [0, 1, 0]))
  translate = trans_from_point(-radius, 0, -height/2+base_offset)
  return translate.dot(swap_xz)

#################################################################

# DynamicsCollisionConstraint::Check
# https://github.com/rdiankov/openrave/blob/ff43549fb6db281c7bc9a85794b88c29d6522ab4/src/libopenrave/planningutils.cpp

#def sample_trajectory(body, traj):
#  spec = traj.GetConfigurationSpecification()
#  #print spec.GetDOF()
#  yield traj.GetWaypoint(0)
#  for i in range(traj.GetNumWaypoints()-1):
#    for conf in linear_interpolation(body, traj.GetWaypoint(i), traj.GetWaypoint(i+1)):
#      yield conf

def extract_base_values(robot, spec, data):
  return RaveGetAffineDOFValuesFromTransform(spec.ExtractTransform(np.identity(4), data, robot), AFFINE_MASK, ROTATION_AXIS)

def active_within_limits(body):
    lower, upper = body.GetActiveDOFLimits()
    q = body.GetActiveDOFValues()
    return np.all(lower <= q) and np.all(q <= upper)

def base_within_limits(robot, q):
    # TODO: rotation
    lower, upper = robot.GetAffineTranslationLimits()
    return np.all(lower[:2] <= q[:2]) and np.all(q[:2] <= upper[:2])


#################################################################

def execute_viewer(env, execute):
  if sys.platform == 'darwin': # NOTE - problem with OpenRAVE on OS X
    SetViewerUserThread(env, 'qtcoin', execute)
  else:
    env.SetViewer('qtcoin')
    execute()

def initialize_openrave(env, manipulator_name, min_delta=MIN_DELTA, collision_checker='ode'):
  sys.setrecursionlimit(2500) # For the motion planners
  env.StopSimulation()
  for sensor in env.GetSensors():
    #print sensor
    #print sensor.GetSensorData()
    #print sensor.GetSensorGeometry()
    #print sensor.GetTransform()
    sensor.Configure(Sensor.ConfigureCommand.PowerOff)
    sensor.Configure(Sensor.ConfigureCommand.RenderDataOff)
    sensor.Configure(Sensor.ConfigureCommand.RenderGeometryOff)
    env.Remove(sensor)
  env.SetCollisionChecker(RaveCreateCollisionChecker(env, collision_checker))
  env.GetCollisionChecker().SetCollisionOptions(0)

  assert len(env.GetRobots()) == 1
  robot = env.GetRobots()[0]

  cd_model = databases.convexdecomposition.ConvexDecompositionModel(robot)
  if not cd_model.load():
    print 'Generating convex decomposition model'
    cd_model.autogenerate()

  l_model = databases.linkstatistics.LinkStatisticsModel(robot)
  if not l_model.load():
    print 'Generating link statistics model'
    l_model.autogenerate()
  l_model.setRobotWeights()
  l_model.setRobotResolutions(xyzdelta=min_delta) # xyzdelta is the minimum Cartesian distance of an object

  robot.SetActiveManipulator(manipulator_name) # NOTE - Need this or the manipulator computations are off
  manipulator = robot.GetManipulator(manipulator_name)
  robot.SetActiveDOFs(manipulator.GetArmIndices())
  ikmodel = databases.inversekinematics.InverseKinematicsModel(robot=robot, iktype=IkParameterization.Type.Transform6D,
      forceikfast=True, freeindices=None, freejoints=None, manip=None)
  if not ikmodel.load():
    print 'Generating inverse kinematics model'
    ikmodel.autogenerate()
  print ikmodel.getikname()
  print ikmodel.getfilename()

  base_manip = interfaces.BaseManipulation(robot, plannername=None, maxvelmult=None)
  ir_model = databases.inversereachability.InverseReachabilityModel(robot)
  #if not ir_model.load():
  #  print 'Generating inverse reachability model'
  #  ir_model.autogenerate()

  return robot, manipulator, base_manip, ir_model

#################################################################

def collision_saver(env, options):
  return CollisionOptionsStateSaver(env.GetCollisionChecker(), options)

def active_collision_saver(env):
  # https://github.com/personalrobotics/or_ompl
  # http://openrave.org/docs/0.8.2/architecture/speed_up_planning/
  return collision_saver(env, openravepy_int.CollisionOptions.ActiveDOFs)

def default_collision_saver(env):
  return collision_saver(env, 0)

class EnvironmentStateSaver(object): # TODO - will fail if any objects have been deleted
  def __init__(self, env):
    # TODO: ensure enabled is correct
    self.env = env
    self.robots = env.GetRobots()
    self.bodies = env.GetBodies() # Includes robots
    self.grabbed_bodies = [robot.GetGrabbed() for robot in self.robots]
    self.robot_savers = [robot.CreateRobotStateSaver() for robot in self.robots]
    self.body_savers = [body.CreateKinBodyStateSaver() for body in self.bodies]
  def Restore(self):
    #self.env.Reset() # TODO: causes a robot error
    for robot in self.robots:
      # Ensures released before removed
      robot.ReleaseAllGrabbed()
    for b in (set(self.env.GetBodies()) - set(self.bodies)):
      self.env.Remove(b)
    for b in (set(self.bodies) - set(self.env.GetBodies())):
      self.env.Add(b)
    for rs in self.robot_savers:
      rs.Restore()
    for bs in self.body_savers:
      bs.Restore()
    for robot, bodies in zip(self.robots, self.grabbed_bodies):
      for body in bodies:
        robot.Grab(body) # TODO: ensure grapping correct link
  def __enter__(self):
    pass
  def __exit__(self, type, value, traceback):
    self.Restore()

#################################################################

def set_active(robot, indices=(), use_base=False, axis=ROTATION_AXIS):
  if use_base is False:
    robot.SetActiveDOFs(indices)
  else:
    robot.SetActiveDOFs(indices, AFFINE_MASK, axis)

def set_active_manipulator(robot, manipulator_name):
  if manipulator_name == 'base':
    set_active(robot, use_base=True)
  else:
    robot.SetActiveManipulator(manipulator_name)
    set_active(robot, indices=robot.GetManipulator(manipulator_name).GetArmIndices())

# TODO - VerifyTrajectory where I disable obstacles

# NOTE - add active DOF here
# def vector_motion_plan(manipulator, vector):
#   robot = manipulator.GetRobot()
#   with manipulator.GetRobot():
#     robot.SetActiveManipulator(manipulator)
#     robot.SetActiveDOFs(get_active_arm_indices(robot))
#     with collision_saver(base_manip.robot.GetEnv(), openravepy_int.CollisionOptions.ActiveDOFs):
#       end_config = inverse(env, robot, vector_trans(get_trans(robot.GetActiveManipulator()), vector))
#       if end_config is None:
#         return None
#       return linear_arm_traj_helper(env, robot, end_config, manip_name=manip_name)
#

# TODO - should I set the active manipulator here or elsewhere?

#################################################################

def aabb_min(aabb):
  return aabb.pos() - aabb.extents()

def aabb_max(aabb):
  return aabb.pos() + aabb.extents()

# TODO - Halton sequence
class AASurface(object):
  def __init__(self, name, xy_min, xy_max, z, color=(0,0,0,.25)):
    self.xy_min = xy_min
    self.xy_max = xy_max
    self.z = z
    self.name = name
    self.color = color
  def supports(self, body):
    aabb = body.ComputeAABB()
    return np.all(self.xy_min <= aabb_min(aabb)[:2]) and \
           np.all(aabb_max(aabb)[:2] <= self.xy_max) and \
           (abs(aabb_min(aabb)[2] - self.z) < 1e-6)
  def sample_placement(self, body, max_attempts=10):
    with body.CreateKinBodyStateSaver():
      for _ in irange(0, max_attempts):
        quat = quat_from_axis_angle(0, 0, uniform(0, 2*math.pi))
        set_quat(body, quat)
        aabb = body.ComputeAABB()
        low = self.xy_min + aabb.extents()[:2]
        high = self.xy_max - aabb.extents()[:2]
        if np.any(low >= high):
          continue
        xy = (high-low)*np.random.rand(*low.shape) + low
        z = self.z + aabb.extents()[2]
        point = np.concatenate([xy, [z]]) + (get_point(body) - aabb.pos())
        pose = pose_from_quat_point(quat, point)
        #set_pose(body, pose)
        return pose
        # TODO - collision here?
    return None
  def get_surface(self):
    min_corner = np.append(self.xy_min, self.z)
    max_corner = np.append(self.xy_max, self.z)
    center = (min_corner + max_corner) / 2
    extents = (max_corner - min_corner) / 2
    unit_corners = [(-1, -1), (+1, -1), (+1, +1), (-1, +1)]
    convex_hull = [np.append(c, 0)*extents for c in unit_corners]
    return pose_from_quat_point(unit_quat(), center), convex_hull

  def draw(self, env):
    self.draw_handle = env.drawtrimesh(points=np.array((
        (self.xy_min[0],self.xy_min[1],self.z), (self.xy_min[0],self.xy_max[1],self.z),
        (self.xy_max[0],self.xy_max[1],self.z), (self.xy_max[0],self.xy_min[1],self.z))),
      indices=np.array(((0,1,2), (0,3,2)),np.int64), colors=np.array((self.color)))
  def __repr__(self):
    return '%s(%s)'%(self.__class__.__name__, self.name)

def compute_surface(body):
  aabb = body.ComputeAABB()
  return AASurface(get_name(body),
                   aabb_min(aabb)[:2],
                   aabb_max(aabb)[:2],
                   aabb_max(aabb)[2] + SURFACE_Z_OFFSET)

#################################################################

def sample_edge_point(polygon, radius):
  from misc.numerical import sample_categorical
  edges = zip(polygon, polygon[-1:] + polygon[:-1])
  edge_weights = {i: max(length(v2 - v1) - 2*radius, 0) for i, (v1, v2) in enumerate(edges)}
  # TODO: fail if no options
  while True:
    index = sample_categorical(edge_weights)
    v1, v2 = edges[index]
    t = random.uniform(radius, length(v2 - v1) - 2*radius)
    yield t*normalize(v2 - v1) + v1

# TODO: always pick perpendicular to the edge

def get_closest_edge_point(polygon, point):
  edges = zip(polygon, polygon[-1:] + polygon[:-1])
  best = None
  for v1, v2 in edges:
    proj = (v2 - v1)[:2].dot((point - v1)[:2])
    if proj <= 0:
      closest = v1
    elif length((v2 - v1)[:2]) <= proj:
      closest = v2
    else:
      closest = proj*normalize((v2 - v1))
    if (best is None) or (length((point - closest)[:2]) < length((point - best)[:2])):
      best = closest
  return best

  #dx, dy, _ = v2 - v1
  #normal = np.array([dy, -dx])
  #distance = normal.dot((point - v1)[:2])


def draw_affine_limits(robot):
  lower, upper = robot.GetAffineTranslationLimits()
  corners = [
    (lower[0], lower[1]),
    (lower[0], upper[1]),
    (upper[0], upper[1]),
    (upper[0], lower[1]),
    (lower[0], lower[1])]
  return robot.GetEnv().drawlinestrip(np.array([np.append(c, 0) for c in corners]),
                              2, colors=np.array([0, 0, 0, 1]))


def draw_circle_2d(env, center, radius, n=24, color=np.array([0, 0, 0, 1])):
  vertices = []
  for i in xrange(n):
      theta = i*2*math.pi/n
      unit = np.array([math.cos(theta), math.sin(theta), 0])
      vertices.append(center+radius*unit)
  vertices.append(vertices[0])
  return env.drawlinestrip(np.array(vertices), 2, colors=color)

def get_bodies(env):
  return {body.GetName(): body for body in env.GetBodies() if not body.IsRobot()}


def enable_all(bodies, enable): # Enables or disables all bodies for collision checking
  for body in bodies.values():
    body.Enable(enable)


def get_mesh_radius(body):
  [geometry] = get_geometries(body)
  mesh = geometry.GetCollisionMesh()
  return max(length(v[:2]) for v in mesh.vertices)


def get_mesh_height(body):
  [geometry] = get_geometries(body)
  mesh = geometry.GetCollisionMesh()
  return 2*max(abs(v[2]) for v in mesh.vertices)