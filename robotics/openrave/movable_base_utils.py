import time
from itertools import count
from itertools import islice
from time import sleep

import numpy as np

from robotics.openrave.motion import plan_path, plan_straight_path, get_collision_fn
from robotics.openrave.transforms import set_pose, \
  trans_from_point, set_trans, set_full_config, base_values_from_full_config
from robotics.openrave.utils import solve_inverse_kinematics, \
  set_manipulator_conf, Conf, manip_from_pose_grasp, \
  top_grasps, side_grasps, random_inverse_reachability, \
  set_active_manipulator, enable_all, SingleGrasp

# TODO - unify with fixed_tamp_holding

APPROACH_VECTOR = 0.15*np.array([0, 0, -1])
DISABLE_MOTIONS = False
DISABLE_MOTION_COLLISIONS = True

assert not DISABLE_MOTIONS or DISABLE_MOTION_COLLISIONS
if DISABLE_MOTIONS:
  print 'Warning: trajectories are disabled'
if DISABLE_MOTION_COLLISIONS:
  print 'Warning: trajectory collisions are disabled'

# NOTE - can either associate poses with specific obstacles or not
# TODO - distinguish between movable. Does the object have grasps? Mass?
# TODO: rotationally invariant grasps. These allow you to switch grasps in hand
# Represent a set of poses with one

####################

class Pose(object):
  _ids = count(0)
  def __init__(self, obj, pose, support=None):
    self.id = next(self._ids)
    self.obj = obj
    self.value = pose
    self.support = support
  def __repr__(self):
    return '{}({},{})'.format(self.__class__.__name__, self.obj, self.id)

class Grasp(object):
  _ids = count(0)
  def __init__(self, obj, grasp):
    self.id = next(self._ids)
    self.obj = obj
    self.value = grasp
  def __repr__(self):
    return '{}({},{})'.format(self.__class__.__name__, self.obj, self.id)

class Motion(object):
  _ids = count(0)
  def __init__(self, traj, obj=None, grasp=None):
    self.id = next(self._ids)
    self.traj = traj
    self.obj = obj
    self.grasp = grasp
  def __repr__(self):
    return self.__class__.__name__ + '(%d)'%self.id

class Prehensile(object):
  _ids = count(0)
  def __init__(self, ra_traj, obj=None, pose=None, grasp=None):
    self.id = next(self._ids)
    self.fa_traj = ra_traj[::-1]
    #self.fg_traj = rg_traj.reverse()
    #self.rg_traj = rg_traj
    self.ra_traj = ra_traj
    self.obj = obj
    self.pose = pose
    self.grasp = grasp
  def __repr__(self):
    return self.__class__.__name__ + '(%d)'%self.id

####################

"""
def manipulator_function(pose, grasp):
  return manip_from_pose_grasp(pose.value, grasp.value)

def base_generator_fn(ir_model, max_failures=20):
  env = ir_model.env
  robot = ir_model.robot
  #manipulator = ir_model.manipulator
  #def base_generator(manip_trans):
  def base_generator(obj, pose, grasp):
    #manip_tform = manipulator_function(pose, grasp)
    manip_tform = manip_from_pose_grasp(pose.value, grasp.value)
    generator = random_inverse_reachability(ir_model, manip_tform)
    while True:
      for _ in range(max_failures):
        base_tform = next(generator)
        if base_tform is None:
          continue
        set_trans(robot, base_tform)
        if env.CheckCollision(robot) or robot.CheckSelfCollision():
          continue
        yield base_tform
        break
      else:
        return
  return base_generator
"""

####################

def grasp_generator_fn(bodies, use_top_grasps, use_side_grasps, max_grasps):
  def grasp_generator(obj):
    grasps = []
    if use_top_grasps:
      grasps += list(top_grasps(bodies[obj]))
    if use_side_grasps:
      grasps += list(side_grasps(bodies[obj]))
    yield [(Grasp(obj, SingleGrasp(grasp)),) for grasp in grasps[:max_grasps]]
  return grasp_generator

def get_stable_test(bodies, surfaces):
  def stable_test(obj, pose, surface):
    bodies[obj].Enable(True)
    set_pose(bodies[obj], pose.value)
    return surfaces[surface].supports(bodies[obj])
  return stable_test

def pose_generator_fn(bodies, surfaces):
  def pose_generator(obj, surface):
    while True:
      bodies[obj].Enable(True)
      pose = surfaces[surface].sample_placement(bodies[obj])
      if pose is None:
        break
      yield [(Pose(obj, pose),)]
  return pose_generator

####################

def cfree_pose_fn(env, bodies):
  #bodies = get_bodies(env)
  def cfree_pose(obj1, pose1, obj2, pose2): # Collision free test between an object at pose1 and an object at pose2
    body1 = bodies[obj1]
    body1.Enable(True)
    set_pose(body1, pose1.value)

    body2 = bodies[obj2]
    body2.Enable(True)
    set_pose(body2, pose2.value)
    return not env.CheckCollision(body1, body2)
  return cfree_pose

####################

"""
def cfree_traj_fn(env, robot, manipulator, body1, body2, all_bodies):
  def _cfree_traj_pose(traj, pose): # Collision free test between a robot executing traj and an object at pose
    enable_all(all_bodies, False)
    body2.Enable(True)
    set_pose(body2, pose.value)
    for conf in traj.value:
      set_manipulator_conf(manipulator, conf)
      if env.CheckCollision(robot, body2):
        return False
    return True

  def _cfree_traj_grasp_pose(traj, grasp, pose): # Collision free test between an object held at grasp while executing traj and an object at pose
    enable_all(all_bodies, False)
    body1.Enable(True)
    body2.Enable(True)
    set_pose(body2, pose.value)
    for conf in traj.value:
      set_manipulator_conf(manipulator, conf)
      manip_trans = manipulator.GetTransform()
      set_pose(body1, object_trans_from_manip_trans(manip_trans, grasp.value))
      if env.CheckCollision(body1, body2):
        return False
    return True

  def cfree_traj(traj, pose): # Collision free test between a robot executing traj (which may or may not involve a grasp) and an object at pose
    if DISABLE_MOTION_COLLISIONS:
      return True
    if traj.pose is not None and traj.pose == pose:
      # This is the same pose of the manipulation
      return True
    return _cfree_traj_pose(traj, pose) and (traj.grasp is None or _cfree_traj_grasp_pose(traj, traj.grasp, pose))
  return cfree_traj
"""

####################

# TODO - joint limit constraints on base pose

def check_collision(robot, self_collisions=True, limits=True):
  #return robot.GetEnv().CheckCollision(robot) or robot.CheckSelfCollision()
  lower, upper = robot.GetActiveDOFLimits()
  conf = robot.GetActiveDOFValues()
  return robot.GetEnv().CheckCollision(robot) or \
         (self_collisions and robot.CheckSelfCollision()) or \
         (limits and (np.all(conf < lower) or np.all(upper < conf)))

def full_from_active(robot, active_path):
  full_path = []
  with robot:
    for active_conf in active_path:
      robot.SetActiveDOFValues(active_conf)
      full_path.append(robot.GetConfigurationValues())
  return full_path

def sample_grasp_traj_fn(base_manip, ir_model, bodies, carry_arm_conf, max_failures=100):
  # TODO - separate into two actions?
  # TODO - approach and retreat motion plans
  arm = ir_model.manip
  robot = ir_model.robot #robot = arm.GetRobot()
  #collision_fn = get_collision_fn(robot.GetEnv(), robot, self_collisions=True, limits=True)
  def sample_grasp_traj(obj, pose, grasp): # Sample pregrasp config and motion plan that performs a grasp
    body = bodies[obj]
    def reset_env():
      enable_all(bodies, False)
      body.Enable(True)
      set_pose(body, pose.value)
      set_active_manipulator(robot, arm.GetName())

    reset_env()
    for _ in xrange(max_failures):
      gripper_from_obj, gripper_from_pregrasp = grasp.value.sample()
      world_from_gripper = manip_from_pose_grasp(pose.value, gripper_from_obj)
      world_from_base = next(random_inverse_reachability(ir_model, world_from_gripper))
      set_trans(robot, world_from_base)
      set_manipulator_conf(arm, carry_arm_conf)
      if check_collision(robot):
        continue
      q = Conf(robot.GetConfigurationValues())
      grasp_arm_conf = solve_inverse_kinematics(arm, world_from_gripper)
      if grasp_arm_conf is None:
        continue
      set_manipulator_conf(arm, grasp_arm_conf)
      #robot.Grab(body)
      pregrasp_arm_conf = solve_inverse_kinematics(arm, world_from_gripper.dot(gripper_from_pregrasp))
      if pregrasp_arm_conf is None:
        #robot.Release(body)
        continue
      set_manipulator_conf(arm, pregrasp_arm_conf)
      if DISABLE_MOTIONS:
        path = [grasp_arm_conf, pregrasp_arm_conf, carry_arm_conf]
        t = Prehensile(full_from_active(robot, path), obj, pose, gripper_from_obj)
        yield [(q, t)]
        reset_env()
        return

      grasp_path = plan_straight_path(robot, grasp_arm_conf, pregrasp_arm_conf)
      #robot.Release(body)
      if grasp_path is None:
        continue
      pregrasp_path = plan_path(base_manip, pregrasp_arm_conf, carry_arm_conf)
      if pregrasp_path is None:
        continue
      t = Prehensile(full_from_active(robot, grasp_path + pregrasp_path[1:]), obj, pose, grasp)
      yield [(q, t)]
      reset_env()
      return
    else:
      pass
      #yield None # TODO: skip this turn and revisit

  return sample_grasp_traj

####################

def sample_free_base_motion_fn(base_manip, bodies):
  robot = base_manip.robot
  def sample_free_base_motion(q1, q2): # Sample motion while not holding
    if DISABLE_MOTIONS:
      yield (Motion([q1.value, q2.value]),)
      return
    enable_all(bodies, False)
    set_full_config(robot, q1.value)
    set_active_manipulator(robot, 'base')
    base_conf1 = base_values_from_full_config(q1.value)
    base_conf2 = base_values_from_full_config(q2.value)

    # TODO - assert that the start and configs are the same
    #t0 = time.time()
    base_path = plan_path(base_manip, base_conf1, base_conf2, restarts=2, iterations=20, smooth=20)
    #print base_conf1, base_conf2, base_path is not None, \
    #  time.time() - t0, (None if base_path is None else len(base_path))
    if base_path is None:
      return
    #full_path = [full_config_from_base_values(base_conf, q1.value) for base_conf in base_path]
    full_path = full_from_active(robot, base_path)
    yield (Motion(full_path),)
  return sample_free_base_motion

####################

class Command(object):
  def simulate(self):
    raise NotImplementedError()
  def step(self):
    self.simulate()
  def __repr__(self):
    return self.__class__
  __str__ = __repr__

class Move(Command):
  def __init__(self, robot, path):
    self.robot = robot
    self.path = path[1:]
  def simulate(self):
    for j, conf in enumerate(self.path):
      set_full_config(self.robot, conf)
      #if step:
      #  raw_input('%s/%s) Step?'%(j, len(confs)))
      #else:
      sleep(0.02)
  def step(self):
    #path = self.path[-1:]
    path = self.path
    for i, conf in enumerate(path):
      raw_input('%s/%s) Step?' % (i, len(path)))
      set_full_config(self.robot, conf)

class Grab(Command):
  def __init__(self, arm, body):
    self.arm = arm
    self.body = body
  def simulate(self):
    #close_gripper(self.arm)
    # Full confs include gripper values
    self.arm.GetRobot().Grab(self.body)

class Release(Command):
  def __init__(self, arm, body):
    self.arm = arm
    self.body = body
  def simulate(self):
    #open_gripper(self.arm)
    self.arm.GetRobot().Release(self.body)

def process_plan(robot, bodies, plan):
  arm = robot.GetActiveManipulator()
  commands = []
  for i, (action, args) in enumerate(plan):
    if action.name == 'move':
      _, _, m = args
      commands.append(Move(robot, m.traj))
    #elif action.name == 'move_holding':
    #  _, _, traj, _, _ = args
    #  _execute_traj(traj.value)
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
    else:
      raise ValueError(action.name)
  return commands

####################


def visualize_solution(commands, step=True):
  for i, command in enumerate(commands):
    raw_input('\n%s/%s) Next?'%(i, len(commands)))
    if step:
      command.step()
    else:
      command.simulate()
