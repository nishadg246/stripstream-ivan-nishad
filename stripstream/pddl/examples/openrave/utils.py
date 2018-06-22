from openravepy import rotationMatrixFromAxisAngle, Sensor, RaveCreateCollisionChecker, databases, interfaces, \
  IkParameterization, GeometryType
from itertools import count

import numpy as np

class Conf(object): # TODO - method that creates these classes
  _ids = count(0)
  def __init__(self, value):
    self.id = self._ids.next()
    self.value = value
  def __repr__(self):
    return self.__class__.__name__ + '(%d)'%self.id

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

def open_gripper(manipulator):
  _, upper = manipulator.GetRobot().GetDOFLimits(manipulator.GetGripperIndices())
  set_gripper(manipulator, upper)

def close_gripper(manipulator):
  lower, _ = manipulator.GetRobot().GetDOFLimits(manipulator.GetGripperIndices())
  set_gripper(manipulator, lower)

# M * G = P
def object_trans_from_manip_trans(manip_trans, grasp_trans):
  return np.dot(manip_trans, grasp_trans)

# G.T * M.T = P.T
def manip_trans_from_object_trans(object_trans, grasp_trans):
  #return np.dot(object_trans, np.linalg.inv(grasp_trans))
  return np.linalg.solve(grasp_trans.T, object_trans.T).T

#################################################################

def solve_inverse_kinematics(env, manipulator, manip_trans):
  robot = manipulator.GetRobot()
  with robot:
    robot.SetActiveDOFs(manipulator.GetArmIndices())
    #with collision_saver(env, openravepy_int.CollisionOptions.ActiveDOFs):
    config = manipulator.FindIKSolution(manip_trans, 0) # Finds a solution near the robot's current joint values
    if config is None:
      return None
    robot.SetDOFValues(config, manipulator.GetArmIndices())
    if env.CheckCollision(robot) or robot.CheckSelfCollision():
      return None
    return config

#################################################################

def get_box_dimensions(box):
  assert box.GetLinks() == 1
  [link] = box.GetLinks()
  assert link.GetGeometries() == 1
  [geometry] = link.GetGeometries()
  assert geometry.GetType() == GeometryType.Box
  return geometry.GetBoxExtents() # TODO - return times 2?

def top_grasps(box): # Rotate around z axis
  (w, l, h) = get_box_dimensions(box)
  uq, up = unit_quat(), unit_point()
  origin = trans_from_quat_point(uq, np.array([0, 0, -h]))
  bottom = trans_from_quat_point(uq, np.array([0, 0, -h]))
  reflect = trans_from_rot_point(rot_reflection(2), up)
  for i in range(4):
    rotate_z = trans_from_quat_point(quat_from_axis_angle(0, 0, i*math.pi/2), up)
    yield reflect.dot(origin).dot(bottom).dot(rotate_z)

def side_grasps(box, under=True): # Convert to z then rotate around it?
  (w, l, h) = get_box_dimensions(box)
  uq, up = unit_quat(), unit_point()
  origin = trans_from_quat_point(uq, np.array([0, 0, -2*h]))
  #origin = trans_from_quat_point(uq, np.array([0, 0, -h]))
  #origin = unit_trans()
  for j in range(1 + under):
    swap_xz = trans_from_quat_point(quat_from_axis_angle(0, -math.pi/2 + j*math.pi, 0), up)
    for i in range(4):
      rotate_z = trans_from_quat_point(quat_from_axis_angle(0, 0, i*math.pi/2), up)
      yield swap_xz.dot(rotate_z).dot(origin)

#################################################################

# DynamicsCollisionConstraint::Check
# https://github.com/rdiankov/openrave/blob/ff43549fb6db281c7bc9a85794b88c29d6522ab4/src/libopenrave/planningutils.cpp
def linear_interpolation(body, q1, q2): # Sequence doesn't include q1
  dq = body.SubtractActiveDOFValues(q2, q1)
  steps = np.abs(np.divide(dq, body.GetActiveDOFResolutions())) + 1
  n = int(np.max(steps))
  for i in range(n):
    yield q1 + (1.+i)/n*dq

def extract_config(manipulator, spec, data):
  return spec.ExtractJointValues(data, manipulator.GetRobot(), manipulator.GetArmIndices())

#def sample_trajectory(body, traj):
#  spec = traj.GetConfigurationSpecification()
#  #print spec.GetDOF()
#  yield traj.GetWaypoint(0)
#  for i in range(traj.GetNumWaypoints()-1):
#    for conf in linear_interpolation(body, traj.GetWaypoint(i), traj.GetWaypoint(i+1)):
#      yield conf

def sample_manipulator_trajectory(manipulator, traj): # TODO - set active joints?
  spec = traj.GetConfigurationSpecification()
  waypoints = [extract_config(manipulator, spec, traj.GetWaypoint(i)) for i in range(traj.GetNumWaypoints())]
  yield waypoints[0]
  for start, end in zip(waypoints, waypoints[1:]):
    for conf in linear_interpolation(manipulator.GetRobot(), start, end):
      yield conf

#################################################################

def initialize_openrave(env, manipulator_name, min_delta=.01, collision_checker='ode'):
  env.StopSimulation()
  for sensor in env.GetSensors():
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
    cd_model.autogenerate()
  l_model = databases.linkstatistics.LinkStatisticsModel(robot)
  if not l_model.load():
    l_model.autogenerate()
  l_model.setRobotWeights()
  l_model.setRobotResolutions(xyzdelta=min_delta) # xyzdelta is the minimum Cartesian distance of an object

  robot.SetActiveManipulator(manipulator_name) # NOTE - Need this or the manipulator computations are off
  ikmodel = databases.inversekinematics.InverseKinematicsModel(robot=robot, iktype=IkParameterization.Type.Transform6D,
      forceikfast=True, freeindices=None, freejoints=None, manip=None)
  if not ikmodel.load():
    ikmodel.autogenerate()
  return robot, robot.GetManipulator(manipulator_name)
