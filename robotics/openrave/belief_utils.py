import math
import os
import random
from collections import namedtuple, defaultdict, Counter

import numpy as np
from openravepy import TriMesh

from manipulation.bodies.geometry import read_pcd, convex_hull
from manipulation.motion.cspace import CSpace
from manipulation.motion.trajectories import PathTrajectory
from motion_planners.rrt_connect import birrt
from robotics.openrave.motion import mp_birrt, get_collision_fn, get_sample_fn, get_extend_fn, get_distance_fn
from robotics.openrave.problems import TAN, GREY, BLUE, GREEN, RED
from robotics.openrave.transforms import pose_from_quat_point, trans_from_quat_point, quat_from_z_rot, \
  trans_from_pose, pose_from_trans, trans_from_quat, quat_from_angle_vector, trans_from_point, \
  base_values_from_full_config, set_base_values, length
from robotics.openrave.utils import Conf, get_close_conf, get_open_conf, sample_edge_point

ODE = 'ode'
FCL = 'fcl_'

COLORS = {
  'table': TAN,
  'dinner': GREY,
  'oil': BLUE,
  'block': GREEN,
  'soup': RED,
}
CLOUDS = {
  'oil': 'oilBottle.pcd',
  'soup': 'soup16.pcd',
  'block': 'soda.pcd',
}

MESHES = { # Cap is literally the bottle cap
    'oil': 'oil_bottle_simple_new.off', # oil_bottle_simple_cap, oil_bottle_simple_new, oilBottleWithCapNew, oilBottleWithCap, ...
    'soup': 'soup16.off', #
    'block': 'soda.off', # soda.off, soda_original.off, soda_points.off
}

##################################################

World = namedtuple('World', ['holding', 'surfaces', 'items'])
#Surface = namedtuple('Surface', ['type', 'pose', 'convex_hull'])
Object = namedtuple('Object', ['type', 'pose', 'registered'])
Occupancy = namedtuple('Occupancy', ['voxels', 'pose'])

class Surface(object):
  def __init__(self, ty, pose, convex_hull):
    self.type = ty
    self.pose = pose
    self.convex_hull = convex_hull
    self.observations = 0
    self.registrations = 0 # When close to it
  def __repr__(self):
    return self.__class__.__name__ + repr((self.type,
                                           self.pose if self.pose is None else len(self.pose),
                                           self.observations))

class Belief(object):
  def __init__(self, holding, surfaces, items, world_from_reference=np.eye(4)):
    self.holding = holding
    self.surfaces = surfaces
    self.items = items
    self.world_from_reference = world_from_reference
  def __repr__(self):
    holding_type = self.holding if (self.holding is None) else self.holding.type
    item_types = [(i.type, i.pose if type(i.pose) != np.ndarray else len(i.pose), i.registered) for i in self.items]
    return self.__class__.__name__ + repr((holding_type, self.surfaces, item_types))

class Task(object):
  def __init__(self, holding=False, object_surfaces=tuple(),
               localized_items=tuple(), registered_items=tuple(),
               clustered_items=tuple(), left_items=tuple(),
               right_items=tuple()):
    self.holding = holding
    self.object_surfaces = tuple(object_surfaces)
    self.localized_items = tuple(localized_items)
    self.registered_items = tuple(registered_items)
    self.clustered_items = tuple(clustered_items)
    self.left_items = tuple(left_items)
    self.right_items = tuple(right_items)
  def __repr__(self):
    return self.__class__.__name__ + repr((self.holding, self.object_surfaces))

##################################################

def get_rectangle(width, length):
  extents = np.array([width, length, 0])/2.
  unit_corners = [(-1, -1), (+1, -1), (+1, +1), (-1, +1)]
  #return [np.array(c)*extents for c in unit_corners]
  return [np.append(c, 0)*extents for c in unit_corners]

def get_table_pose(x, y, z, theta=0):
  #return pose_from_quat_point(unit_quat(), np.array([x, y, z]))
  return pose_from_quat_point(quat_from_z_rot(theta), np.array([x, y, z]))

def unit_from_theta(theta):
  return np.array([math.cos(theta), math.sin(theta)])


def apply_affine(affine, points):
  return affine.dot(np.hstack([points, np.ones((len(points), 1))]).T).T[:, :3]


def is_point_in_polygon(point, polygon):
  sign = None
  for i in xrange(len(polygon)):
    v1, v2 = polygon[i-1][:2], polygon[i][:2]
    delta = v2 - v1
    normal = np.array([-delta[1], delta[0]])
    dist = normal.dot(point[:2] - v1)
    if i == 0: # TODO: equality?
      sign = np.sign(dist)
    elif np.sign(dist) != sign:
      return False
  return True


def sample_polygon_tform(polygon, points):
  min_z = np.min(points[:, 2])
  aabb_min = np.min(polygon, axis=0)
  aabb_max = np.max(polygon, axis=0)
  while True:
    x = random.uniform(aabb_min[0], aabb_max[0])
    y = random.uniform(aabb_min[1], aabb_max[1])
    theta = random.uniform(0, 2*np.pi)
    surface_from_origin = trans_from_quat_point(quat_from_z_rot(theta), np.array([x, y, -min_z]))
    yield surface_from_origin
    #if all(is_point_in_polygon(p, polygon) for p in apply_affine(surface_from_origin, points)):
    #  yield surface_from_origin


def is_surface_pose(surface, mesh, pose):
  world_from_surface = trans_from_pose(surface.pose)
  world_from_mesh = trans_from_pose(pose)
  surface_from_mesh = np.linalg.inv(world_from_surface).dot(world_from_mesh)
  points_surface = apply_affine(surface_from_mesh, mesh.vertices)
  min_z = np.min(points_surface[:, 2])
  return (abs(min_z) < 0.01) and all(is_point_in_polygon(p, surface.convex_hull) for p in points_surface)

def is_point_on_surface(surface, point_world):
  surface_from_world = np.linalg.inv(trans_from_pose(surface.pose))
  point_surface = surface_from_world.dot(np.append(point_world, 1))[:3]
  return is_point_in_polygon(point_surface, surface.convex_hull[::-1])

def sample_surface_pose(surface, mesh):
  world_from_surface = trans_from_pose(surface.pose)
  for surface_from_origin in sample_polygon_tform(surface.convex_hull, mesh.vertices):
    pose = pose_from_trans(world_from_surface.dot(surface_from_origin))
    if is_surface_pose(surface, mesh, pose):
      yield pose


def sample_edge_pose(surface, mesh):
  world_from_surface = trans_from_pose(surface.pose)
  radius = max(length(v[:2]) for v in mesh.vertices)
  origin_from_base = trans_from_point(0, 0, np.min(mesh.vertices[:, 2]))
  for point in sample_edge_point(surface.convex_hull, radius):
    theta = random.uniform(0, 2 * np.pi)
    surface_from_origin = trans_from_quat_point(quat_from_z_rot(theta), point)
    yield pose_from_trans(world_from_surface.dot(surface_from_origin).dot(origin_from_base))


def get_object_on_surface(ty, object_meshes, surface):
  return Object(ty, next(sample_surface_pose(surface, object_meshes[ty])))


def get_object_meshes(cloud_dir):
  object_meshes = {}
  for ty, f in CLOUDS.iteritems():
    path = os.path.join(cloud_dir, f)
    print 'Loading {} cloud: {}'.format(ty, path)
    points = read_pcd(path)
    print 'Found {} points'.format(len(points))
    mesh = mesh_from_points(points)
    print 'Vertices: {} | Faces: {}'.format(len(mesh.vertices), len(mesh.indices))
    object_meshes[ty] = mesh
  return object_meshes


def mesh_from_points(points):
  vertices, indices = convex_hull(points)
  new_indices = []
  for triplet in indices:
    centroid = np.average(vertices[triplet], axis=0)
    v1, v2, v3 = vertices[triplet]
    normal = np.cross(v3 - v1, v2 - v1)
    if normal.dot(centroid) > 0:
      new_indices.append(triplet[::-1])
    else:
      new_indices.append(triplet)
  return TriMesh(vertices, new_indices)

##################################################

MAX_GRASP_WIDTH = 0.07
#GRASP_LENGTH = 0.06 # Slightly less than 2.5 inches
GRASP_LENGTH = 0.04
APPROACH_LENGTH = 0.1 + GRASP_LENGTH

#MAX_GRASP_WIDTH = 0.08255
#GRASP_LENGTH = 0.08
#GRASP_LENGTH = 0.02

# handles = draw_axes(env, arm.GetEndEffectorTransform(), length=0.1)
# arm.GetEndEffectorTransform() == arm.GetTransform()
# arm.GetGraspTransform() = arm.GetLocalToolTransform()
# arm.GetLocalToolDirection() == [ 0.  0.  1.]
# arm.GetEndEffectorTransform()
# The manipulator frame is defined by the end effector link position * GetLocalToolTransform().
# All inverse kinematics and jacobian queries are specifying this frame.

# base_from_gripper * gripper_from_obj = base_from_obj
# base_from_obj * (gripper_from_obj)^-1 = gripper_from_obj
# returns gripper_from_obj
# Intuitively, is the pose of object with respect to gripper
# The axis is still aligned here more or less (just reflected over z)
# Requires swapping z and x

def get_top_grasps(mesh, under=False):
  w, l, h = np.max(mesh.vertices, axis=0) - \
            np.min(mesh.vertices, axis=0)
  reflect_z = trans_from_quat(quat_from_angle_vector(math.pi, [0, 1, 0]))
  translate = trans_from_point(0, 0, h/2 - GRASP_LENGTH)
  if w < MAX_GRASP_WIDTH:
    for i in range(1 + under):
      rotate_z = trans_from_quat(quat_from_angle_vector(math.pi / 2 + i*math.pi, [0, 0, 1]))
      yield translate.dot(rotate_z).dot(reflect_z), np.array([w])
  if l < MAX_GRASP_WIDTH:
    for i in range(1 + under):
      rotate_z = trans_from_quat(quat_from_angle_vector(i*math.pi, [0, 0, 1]))
      yield translate.dot(rotate_z).dot(reflect_z), np.array([l])

def get_prepush_setting(mesh, under=False):
  ######## write
  w, l, h = np.max(mesh.vertices, axis=0) - \
            np.min(mesh.vertices, axis=0)
  for j in range(1 + under):
    swap_xz = trans_from_quat(quat_from_angle_vector(math.pi/2 + j*math.pi, [0, 1, 0]))
    if w < MAX_GRASP_WIDTH:
      translate = trans_from_point(0, 0, -2*l)
      for i in range(2):
        rotate_z = trans_from_quat(quat_from_angle_vector(math.pi / 2 + i * math.pi, [1, 0, 0]))
        yield translate.dot(rotate_z).dot(swap_xz), np.array([w])
    if l < MAX_GRASP_WIDTH:
      translate = trans_from_point(0, 0, -2*l)
      for i in range(2):
        rotate_z = trans_from_quat(quat_from_angle_vector(i * math.pi, [1, 0, 0]))
        yield translate.dot(rotate_z).dot(swap_xz), np.array([l])



def get_side_grasps(mesh, under=False):
  w, l, h = np.max(mesh.vertices, axis=0) - \
            np.min(mesh.vertices, axis=0)
  for j in range(1 + under):
    swap_xz = trans_from_quat(quat_from_angle_vector(math.pi/2 + j*math.pi, [0, 1, 0]))
    if w < MAX_GRASP_WIDTH:
      translate = trans_from_point(0, 0, l / 2 - GRASP_LENGTH)
      for i in range(2):
        rotate_z = trans_from_quat(quat_from_angle_vector(math.pi / 2 + i * math.pi, [1, 0, 0]))
        yield translate.dot(rotate_z).dot(swap_xz), np.array([w])
    if l < MAX_GRASP_WIDTH:
      translate = trans_from_point(0, 0, w / 2 - GRASP_LENGTH)
      for i in range(2):
        rotate_z = trans_from_quat(quat_from_angle_vector(i * math.pi, [1, 0, 0]))
        yield translate.dot(rotate_z).dot(swap_xz), np.array([l])

# def test_grasps():
#   world_from_gripper = arm.GetTransform()
#   for grasp, _ in get_top_grasps(object_meshes['block']):
#  #for grasp, _ in get_side_grasps(object_meshes['block']):
#    world_from_obj = world_from_gripper.dot(grasp)
#    set_pose(env.GetKinBody('block0'), pose_from_trans(world_from_obj))
#    raw_input('Continue?')
#   return

##################################################

def get_conf(robot, manip_name):  # TODO: treat torso as base
  full_conf = robot.GetConfigurationValues()
  if manip_name == 'base':
    base_conf = base_values_from_full_config(full_conf)
    #with robot: # TODO: fix this
    #  set_active(robot, use_base=True)
    #  base_conf = robot.GetActiveDOFValues()
    return Conf(base_conf)
  manipulator = robot.GetManipulator(manip_name)
  return Conf(full_conf[manipulator.GetArmIndices()])


def set_conf(robot, manip_name, conf):  # TODO: require that conf be a Conf?
  if manip_name == 'base':
    set_base_values(robot, conf)
    #set_active(robot, use_base=True) # TODO: fix this
    #robot.SetActiveDOFValues(conf)
  else:
    manipulator = robot.GetManipulator(manip_name)
    robot.SetDOFValues(conf, manipulator.GetArmIndices())


def sample_trajectory(robot, q1, q2, name, **kwargs):
  cspace = CSpace.robot_manipulator(robot, name)
  cspace.name = name
  with robot:
    cspace.set_active()
    base_path = mp_birrt(robot, q1, q2, **kwargs)
    if base_path is None:
      return None
  return PathTrajectory(cspace, base_path)

def sample_base_trajectory(robot, q1, q2, **kwargs):
  # TODO: ActiveDOFs doesn't work here for FCL?
  env = robot.GetEnv()
  cspace = CSpace.robot_manipulator(robot, 'base')
  cspace.name = 'base'
  with robot:
    cspace.set_active()
    collision_fn = get_collision_fn(env, robot, self_collisions=False, limits=True)
    base_path = birrt(q1, q2, get_distance_fn(robot), get_sample_fn(env, robot),
             get_extend_fn(robot), collision_fn, **kwargs)
    if base_path is None:
      return None
  return PathTrajectory(cspace, base_path)

##################################################

def close_gripper_trajectory(arm):
  # TODO: use task manip
  cspace = CSpace.robot_gripper(arm)
  cspace.name = '{}_gripper'.format(arm.GetName()[0])
  return PathTrajectory(cspace, [get_open_conf(arm), get_close_conf(arm)])

def open_gripper_trajectory(arm):
  cspace = CSpace.robot_gripper(arm)
  cspace.name = '{}_gripper'.format(arm.GetName()[0])
  return PathTrajectory(cspace, [get_close_conf(arm), get_open_conf(arm)])

def get_input(message, options):
    full_message = '{} [{}]: '.format(message, ','.join(options))
    response = raw_input(full_message)
    while response not in options:
        response = raw_input(full_message)
    return response

def remove_except(robot):
  env = robot.GetEnv()
  robot.ReleaseAllGrabbed()
  for body in env.GetBodies():
    if body != robot:
      env.Remove(body)

##################################################

def ray_collision(body, start, end):
  # [fclcollision.h:579 FCLCollisionChecker::CheckCollision] fcl doesn't support Ray collisions
  rays = np.array([
    np.append(start, (end - start)),
  ])
  env = body.GetEnv()
  return env.CheckCollisionRays(rays, body) # front_facing_only?


def entities_from_task(task):
  task_surfaces = defaultdict(int)
  task_objects = defaultdict(int)
  if task.holding not in (None, False):
    task_objects[task.holding] += 1
  for obj, surface in task.object_surfaces:
    task_objects[obj] += 1  # We have prior that cannot be in two places at once
    task_surfaces[surface] = 1  # Doesn't prevent use in other goals
  for obj in task.localized_items + task.registered_items:
    if not task_objects[obj]:
      task_objects[obj] += 1
  for cluster in task.clustered_items:
      for item, n in Counter(cluster).items():
          task_objects[item] = max(task_objects[item], n)
  for item, n in Counter(task.left_items + task.right_items).items():
      task_objects[item] = max(task_objects[item], n)
      # TODO: left and right tables

  #print dict(task_objects), dict(task_surfaces)
  return task_objects, task_surfaces


def belief_from_task(belief, task):
    type_counts = defaultdict(int)
    for o in (belief.surfaces + belief.items):
        type_counts[o.type] += 1
    if belief.holding is not None:
        type_counts[belief.holding.type] += 1

    task_objects, task_surfaces = entities_from_task(task)
    new_items, new_surfaces = [], []
    task_surfaces['table'] = 1
    for ty in task_surfaces:
        for i in xrange(type_counts[ty], task_surfaces[ty]):
            new_surfaces.append(Surface(ty, None, None))
    # for cl in item_types:
    for ty in task_objects:
        for i in xrange(type_counts[ty], task_objects[ty]):
            new_items.append(Object(ty, None, False))
    return Belief(belief.holding, (belief.surfaces + new_surfaces), (belief.items + new_items))