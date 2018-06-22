import math
import random
from itertools import islice, count

import numpy as np

from movable_base_utils import full_from_active, Prehensile, \
  Move, Grab, Release, check_collision, Grasp
from robotics.openrave.belief_utils import apply_affine, is_point_in_polygon
from robotics.openrave.motion import plan_straight_path
from robotics.openrave.transforms import set_pose, \
  trans_from_point, set_trans, trans_from_pose, pose_from_trans, get_point, trans_from_quat_point, \
  quat_from_z_rot, length, point_from_pose, normalize, angle_from_vector, quat_from_angle_vector, trans_from_quat, \
  pose_from_quat_point
from robotics.openrave.utils import solve_inverse_kinematics, \
  set_manipulator_conf, Conf, Pose, manip_from_pose_grasp, \
  random_inverse_reachability, \
  sample_edge_point, cylinder_contact, enable_all, \
  get_mesh_radius, get_mesh_height, CylinderSideXGrasp, CylinderTopGrasp, CylinderSideYGrasp

# TODO - unify with fixed_tamp_holding

APPROACH_VECTOR = 0.15*np.array([0, 0, -1])
PUSH_MAX_DISTANCE = 0.3

# NOTE - can either associate poses with specific obstacles or not

####################

class Push(object):
  _ids = count(0)
  def __init__(self, fa_traj, p_traj, ra_traj,
               obj=None, pose1=None, pose2=None, contact=None):
    self.id = next(self._ids)
    self.fa_traj = fa_traj
    self.p_traj = p_traj
    self.ra_traj = ra_traj
    self.obj = obj
    self.pose1 = pose1
    self.pose2 = pose2
    self.contact = contact
  def __repr__(self):
    return self.__class__.__name__ + '(%d)'%self.id

# TODO - distinguish between movable. Does the object have grasps? Mass?

def is_edge_pose(surface, pose):
  # Pose is assumed to be the center of mass
  world_from_surface = trans_from_pose(surface.pose)
  world_from_mesh = trans_from_pose(pose)
  surface_from_mesh = np.linalg.inv(world_from_surface).dot(world_from_mesh)
  points_surface = apply_affine(surface_from_mesh, [point_from_pose(pose)])
  min_z = np.min(points_surface[:, 2])
  return (abs(min_z) < 0.01) and all(is_point_in_polygon(p, surface.convex_hull) for p in points_surface)

####################

from stripstream.pddl.objects import EasyType as Type, EasyParameter as Param
from stripstream.pddl.logic.predicates import EasyPredicate as Pred
from stripstream.pddl.utils import rename_easy

# Types
CYL, POINT, GRASP = Type(), Type(), Type()
CONF, TRAJ, SURFACE = Type(), Type(), Type()

# Fluents
ConfEq = Pred(CONF)
PointEq = Pred(CYL, POINT)
GraspEq = Pred(CYL, GRASP)
Holding = Pred(CYL)
HandEmpty = Pred()

# Derived
SafePose = Pred(CYL, POINT)
SafeTraj = Pred(CYL, TRAJ)
OnSurface = Pred(CYL, SURFACE)

# Static trajectory
FreeMotion = Pred(CONF, CONF, TRAJ)
HoldingMotion = Pred(CONF, CONF, TRAJ)
GraspMotion = Pred(CYL, POINT, GRASP, CONF, TRAJ)
#GraspMotion = Pred(OBJ, POSE, GRASP, CONF, TRAJ, TRAJ)
PushMotion = Pred(CYL, POINT, POINT, CONF, TRAJ)

# Static collision
CFreePose = Pred(POINT, POINT)
CFreeTraj = Pred(TRAJ, POINT)

IsPushable = Pred(CYL)
IsPose = Pred(CYL, POINT)
IsGrasp = Pred(CYL, GRASP)
Stable = Pred(POINT, SURFACE)
IsEdge = Pred(POINT)


O, P, G = Param(CYL), Param(POINT), Param(GRASP)
O2, P2 = Param(CYL), Param(POINT)
Q, Q2 = Param(CONF), Param(CONF)
T = Param(TRAJ)
T2 = Param(TRAJ)
S = Param(SURFACE)

rename_easy(locals())

####################

def edge_generator_fn(bodies, surfaces):
  def pose_generator(obj, surface):
    surface_pose, convex_hull = surfaces[surface].get_surface()
    world_from_surface = trans_from_pose(surface_pose)
    body = bodies[obj]
    aabb = body.ComputeAABB()
    radius = 0 # TODO: compute from aabb
    translation = np.array([0, 0, aabb.extents()[2]]) + (get_point(body) - aabb.pos())
    for point in sample_edge_point(convex_hull, radius):
      theta = random.uniform(0, 2 * np.pi)
      surface_from_origin = trans_from_quat_point(quat_from_z_rot(theta), translation + point)
      #body.Enable(True)
      pose = pose_from_trans(world_from_surface.dot(surface_from_origin))
      if pose is None:
        break
      yield [(Pose(pose),)]
  return pose_generator

def grasp_generator_fn(bodies):
  #candidates = [CylinderSideXGrasp, CylinderTopGrasp, CylinderSideYGrasp]
  candidates = [CylinderSideXGrasp, CylinderSideYGrasp]
  def grasp_generator(obj):
    grasps = []
    for c in candidates:
      grasp = c.compute(bodies[obj])
      if grasp is not None:
        grasps.append(grasp)
    return [(Grasp(obj, grasp),) for grasp in grasps]
  return grasp_generator

####################

# TODO: could make the grasps be orthogonal
# TODO: sample closest to edge
# TODO: sample closest to robot

def sample_push_traj_fn(ir_model, bodies, surfaces, carry_arm_conf, max_failures=200):
  arm = ir_model.manip
  robot = ir_model.robot

  pregrasp_vector = 0.1 * normalize(np.array([1, 0, -1]))
  gripper_from_pregrasp = trans_from_point(*pregrasp_vector)
  # TODO: include pregrasp here

  def sample_grasp_traj(obj, pose, pose2):
    enable_all(bodies, False)
    body = bodies[obj]
    #body.Enable(True) # TODO: fix this
    set_pose(body, pose.value)

    # TODO: check if on the same surface

    point = point_from_pose(pose.value)
    point2 = point_from_pose(pose2.value)
    distance = length(point2 - point)
    if (distance <= 0.0) or (0.6 <= distance):
      return
    direction = normalize(point2 - point)
    orientation = quat_from_angle_vector(angle_from_vector(direction), [0, 0, 1])

    #rotate_direction = trans_from_quat_point(orientation, unit_point())

    steps = int(math.ceil(distance / PUSH_MAX_DISTANCE) + 1)
    distances = np.linspace(0., distance, steps)
    points = [point + d * direction for d in distances]
    poses = [pose_from_quat_point(orientation, point) for point in points]
    pose_objects = [pose] + map(Pose, poses[1:-1]) + [pose2]
    print distance, steps, distances, len(poses)

    radius, height = get_mesh_radius(body), get_mesh_height(body)
    contact = cylinder_contact(radius, height)

    pushes = []
    # TODO: I could do all trajectories after all pushes planned
    for i in xrange(len(poses)-1):
      p1, p2 = poses[i:i+2]
      # TODO: choose midpoint for base
      ir_world_from_gripper = manip_from_pose_grasp(p1, contact)
      set_manipulator_conf(arm, carry_arm_conf)
      for world_from_base in islice(random_inverse_reachability(ir_model, ir_world_from_gripper), max_failures):
        set_trans(robot, world_from_base)
        set_manipulator_conf(arm, carry_arm_conf)
        if check_collision(robot):
          continue
        q = Conf(robot.GetConfigurationValues())
        push_arm_confs = []
        approach_paths = []
        for p in (p1, p2):
          # TODO: make sure I have the +x push conf
          world_from_gripper = manip_from_pose_grasp(p, contact)
          set_manipulator_conf(arm, carry_arm_conf)
          grasp_arm_conf = solve_inverse_kinematics(arm, world_from_gripper, collisions=False)
          if grasp_arm_conf is None:
            break
          push_arm_confs.append(grasp_arm_conf)
          set_manipulator_conf(arm, grasp_arm_conf)
          pregrasp_arm_conf = solve_inverse_kinematics(arm, world_from_gripper.dot(gripper_from_pregrasp), collisions=False)
          if pregrasp_arm_conf is None:
            break
          #if DISABLE_MOTIONS:
          if True:
            approach_paths.append([carry_arm_conf, pregrasp_arm_conf, grasp_arm_conf])
            continue
          """
          set_manipulator_conf(arm, pregrasp_arm_conf)
          grasp_path = plan_straight_path(robot, grasp_arm_conf, pregrasp_arm_conf)
          # robot.Release(body)
          if grasp_path is None:
            continue
          pregrasp_path = plan_path(base_manip, pregrasp_arm_conf, carry_arm_conf)
          if pregrasp_path is None:
            continue
          t = Prehensile(full_from_active(robot, grasp_path + pregrasp_path[1:]), obj, pose, grasp)
          yield [(q, t)]
          reset_env()
          return
          """
        else:
          # Start and end may have different orientations
          pq1, pq2 = push_arm_confs
          push_path = plan_straight_path(robot, pq1, pq2)
          # TODO: make sure the straight path is actually straight
          if push_path is None:
            continue
          po1, po2 = pose_objects[i:i + 2]
          ap1, ap2 = approach_paths
          m = Push(full_from_active(robot, ap1),
                   full_from_active(robot, push_arm_confs),
                   #full_from_active(robot, push_path),
                   full_from_active(robot, ap2[::-1]),
                   obj, po1, po2, contact)
          pushes.append(PushMotion(obj, po1, po2, q, m))
          break
      else:
        print 'Failure', len(pushes)
        return
    print 'Success', len(pushes)
    yield pushes
    # TODO: I need to return individual pushes (not just noes that worked on the trajectory)

  return sample_grasp_traj

####################

def process_plan(robot, bodies, plan):
  arm = robot.GetActiveManipulator()
  commands = []
  for i, (action, args) in enumerate(plan):
    if action.name == 'move':
      q1, q2 = args
      commands.append(Move(robot, [q1.value, q2.value]))
    elif action.name == 'pick':
      obj, _, _, _, m = args
      commands += [
        Move(robot, m.fa_traj),
        Grab(arm, bodies[obj]),
        Move(robot, m.ra_traj)]
    elif action.name == 'place':
      obj, _, _, _, m = args
      commands += [
        Move(robot, m.fa_traj),
        Release(arm, bodies[obj]),
        Move(robot, m.ra_traj)]
    elif action.name == 'push':
      obj, _, _, _, m = args
      commands += [
        Move(robot, m.fa_traj),
        Grab(arm, bodies[obj]),
        Move(robot, m.p_traj),
        Release(arm, bodies[obj]),
        Move(robot, m.ra_traj)]
    else:
      raise ValueError(action.name)
  return commands
