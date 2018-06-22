from inspect import currentframe, getfile
from itertools import product
from random import sample
import os
import math

import numpy as np

from stripstream.utils import flatten
from utils import set_base_conf, Pose, box_body, get_name, \
  set_manipulator_conf, mirror_arm_config, open_gripper, close_gripper, top_grasps, Grasp, \
  SURFACE_Z_OFFSET, compute_surface
from robotics.openrave.transforms import pose_from_quat_point, set_pose, unit_quat

TOP_HOLDING_LEFT_ARM = [0.67717021, -0.34313199, 1.2, -1.46688405, 1.24223229, -1.95442826, 2.22254125]
SIDE_HOLDING_LEFT_ARM = [0.39277395, 0.33330058, 0., -1.52238431, 2.72170996, -1.21946936, -2.98914779]
REST_LEFT_ARM = [2.13539289, 1.29629967, 3.74999698, -0.15000005, 10000., -0.10000004, 10000.]
WIDE_LEFT_ARM = [1.5806603449288885, -0.14239066980481405, 1.4484623937179126, -1.4851759349218694, 1.3911839347271555, -1.6531320011389408, -2.978586584568441]
CENTER_LEFT_ARM = [-0.07133691252641006, -0.052973836083405494, 1.5741805775919033, -1.4481146328076862, 1.571782540186805, -1.4891468812835686, -9.413338322697955]
WIDE_RIGHT_ARM = [-1.3175723551150083, -0.09536552225976803, -1.396727055561703, -1.4433371993320296, -1.5334243909312468, -1.7298129320065025, 6.230244924007009]

PROBLEMS_DIR = os.path.dirname(os.path.abspath(getfile(currentframe())))
ENVIRONMENTS_DIR = os.path.join(PROBLEMS_DIR, 'environments')

BLACK = np.array((0, 0, 0, 0))
GREY = np.array((.5, .5, .5, 0))
WHITE = np.array((1, 1, 1, 0))

RED = np.array((1, 0, 0, 0))
GREEN = np.array((0, 1, 0, 0))
BLUE = np.array((0, 0, 1, 0))

YELLOW = np.array((1, 1, 0, 0))
PINK = np.array((1, 0, 1, 0))
CYAN = np.array((0, 1, 1, 0))

TAN = np.array((255, 165, 79, 0))/255.0

##################################################

# TODO - make tiny region that essentially is a point

class RealProblem(object):
  def __init__(self, movable_names=[], surfaces=[], goal_surfaces={}):
    self.movable_names = movable_names
    self.surfaces = surfaces
    self.goal_surfaces = goal_surfaces
  def __str__(self):
    s = self.__class__.__name__ + '\n'
    for name, value in self.__dict__.iteritems():
      s += name + ': %s\n'%str(value)
    return s
  __repr__ = __str__

def initialize_two_tables(env):
  env.Load(os.path.join(ENVIRONMENTS_DIR, '2tables.xml'))

  tables = filter(lambda body: 'table' in get_name(body), env.GetBodies())
  surfaces = map(compute_surface, tables)

  robot = env.GetRobots()[0]
  set_manipulator_conf(robot.GetManipulator('leftarm'), REST_LEFT_ARM)
  open_gripper(robot.GetManipulator('leftarm'))
  set_manipulator_conf(robot.GetManipulator('rightarm'), mirror_arm_config(robot, REST_LEFT_ARM))
  close_gripper(robot.GetManipulator('rightarm'))
  robot.SetDOFValues([.15], [robot.GetJointIndex('torso_lift_joint')])
  # set_base_conf(robot, (0, 0, 0))

  base_limit = 3.5 # 2.5
  robot.SetAffineTranslationLimits(*(base_limit * np.array([[-1, -1, 0], [1, 1, 0]])))
  # TODO - be careful about these joint limits with the OpenRAVE planners
  return surfaces

def simple(env):
  surfaces = initialize_two_tables(env)
  fixed_names = map(get_name, env.GetBodies())
  surface_map = {s.name: s for s in surfaces}

  objA = box_body(env, 'objA', .07, .05, .2, color=BLUE)
  env.Add(objA)
  while True:
    pose = surface_map['table1'].sample_placement(objA)
    if pose is not None:
      set_pose(objA, pose)
      break

  goal_surfaces = {get_name(objA): 'table2'}

  movable_names = filter(lambda name: name not in fixed_names, map(get_name, env.GetBodies()))

  return RealProblem(movable_names=movable_names,
                     surfaces=surfaces, goal_surfaces=goal_surfaces)

def cylinder(env):
  from manipulation.bodies.bodies import mesh_cylinder_body

  surfaces = initialize_two_tables(env)
  fixed_names = map(get_name, env.GetBodies())
  surface_map = {s.name: s for s in surfaces}

  #objA = mesh_cylinder_body(env, .03, .1, name='objA', color=BLUE)
  #objA = mesh_cylinder_body(env, .05, .05, name='objA', color=BLUE)
  #objA = mesh_cylinder_body(env, .265/2, .025, name='objA', color=BLUE) # Plate
  objA = mesh_cylinder_body(env, .2/2, .025, name='objA', color=BLUE)

  env.Add(objA)
  while True:
    pose = surface_map['table1'].sample_placement(objA)
    if pose is not None:
      set_pose(objA, pose)
      break

  goal_surfaces = {get_name(objA): 'table2'}

  movable_names = filter(lambda name: name not in fixed_names, map(get_name, env.GetBodies()))

  return RealProblem(movable_names=movable_names,
                     surfaces=surfaces, goal_surfaces=goal_surfaces)

def soup(env):
  from manipulation.bodies.bodies import mesh_cylinder_body

  surfaces = initialize_two_tables(env)
  fixed_names = map(get_name, env.GetBodies())
  surface_map = {s.name: s for s in surfaces}

  body = mesh_cylinder_body(env, .03, .1, name='soup', color=BLUE)
  env.Add(body)
  while True:
    pose = surface_map['table1'].sample_placement(body)
    if pose is not None:
      set_pose(body, pose)
      break

  goal_surfaces = {get_name(body): 'table2'}

  movable_names = filter(lambda name: name not in fixed_names, map(get_name, env.GetBodies()))

  return RealProblem(movable_names=movable_names,
                     surfaces=surfaces, goal_surfaces=goal_surfaces)


##################################################

class ManipulationProblem(object):
  def __init__(self, #name, camera_trans=CAMERA_TRANS,
      object_names=[], table_names=[], regions=[],
      start_holding=False, goal_config=None, goal_holding=None, goal_poses={}, goal_regions={},
      known_poses=None, initial_poses=None, known_grasps=None):
    self.object_names = object_names
    self.table_names = table_names
    self.regions = regions
    self.start_holding = start_holding
    self.goal_config = goal_config
    self.goal_holding = goal_holding
    self.goal_poses = goal_poses
    self.goal_regions = goal_regions
    self.known_poses = known_poses
    self.initial_poses = initial_poses
    self.known_grasps = known_grasps
  #def set_viewer(self, env):
  #  #set_viewer_options(env, name=VIEWER_NAME+': %s'%self.name, camera_trans=CAMERA_TRANS)
  #  set_viewer_options(env, name=VIEWER_NAME+': %s'%self.name, camera_trans=self.camera_trans)
  def __str__(self):
    s = self.__class__.__name__ + '\n'
    for name, value in self.__dict__.iteritems():
      s += name + ': %s\n'%str(value)
    return s
  __repr__ = __str__

##################################################

def dantam_distract(env, n_obj): # (Incremental Task and Motion Planning: A Constraint-Based Approach)
  env.Load(os.path.join(ENVIRONMENTS_DIR, 'empty.xml'))

  m, n = 3, 3
  side_dim = .07
  height_dim = .1
  box_dims = (side_dim, side_dim, height_dim)
  separation = (side_dim, side_dim)

  coordinates = list(product(range(m), range(n)))
  assert n_obj <= len(coordinates)
  obj_coordinates = sample(coordinates, n_obj)

  length = m*(box_dims[0] + separation[0])
  width = n*(box_dims[1] + separation[1])
  height = .7
  table = box_body(env, 'table', length, width, height, color=TAN)
  set_pose(table, pose_from_quat_point(unit_quat(), np.array([0, 0, 0])))
  env.Add(table)

  robot = env.GetRobots()[0]
  set_manipulator_conf(robot.GetManipulator('leftarm'), TOP_HOLDING_LEFT_ARM)
  open_gripper(robot.GetManipulator('leftarm'))
  set_manipulator_conf(robot.GetManipulator('rightarm'), mirror_arm_config(robot, REST_LEFT_ARM))
  close_gripper(robot.GetManipulator('rightarm'))
  robot.SetDOFValues([.15], [robot.GetJointIndex('torso_lift_joint')])
  set_base_conf(robot, (-.75, .2, -math.pi/2))

  poses = []
  z =  height + SURFACE_Z_OFFSET
  for r in range(m):
    row = []
    x = -length/2 + (r+.5)*(box_dims[0] + separation[0])
    for c in range(n):
      y = -width/2 + (c+.5)*(box_dims[1] + separation[1])
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
    obj = box_body(env, name, *box_dims, color=color)
    set_pose(obj, poses[r][c].value)
    env.Add(obj)

  known_poses = list(flatten(poses))
  object_names = initial_poses.keys()
  known_grasps = list(map(Grasp, top_grasps(env.GetKinBody(object_names[0]))))

  return ManipulationProblem(object_names=object_names, table_names=[table.GetName()],
    goal_poses=goal_poses, initial_poses=initial_poses, known_poses=known_poses, known_grasps=known_grasps)