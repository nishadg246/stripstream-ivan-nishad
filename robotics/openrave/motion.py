import math

import numpy as np

from robotics.openrave.transforms import set_active_config, get_active_config, length, normalize
from robotics.openrave.utils import extract_base_values, collision_saver, MIN_DELTA, set_active

from openravepy import planning_error, openravepy_int

def get_collision_fn(env, body, self_collisions=None, limits=True):
  lower, upper = body.GetActiveDOFLimits()
  # TODO: ensure this isn't redundant
  if self_collisions is None:
      self_collisions = (len(body.GetActiveDOFIndices()) != 0)
  def fn(q):
    body.SetActiveDOFValues(q)
    return env.CheckCollision(body) or \
           (self_collisions and body.CheckSelfCollision()) or \
           (limits and (np.any(q < lower) or np.any(upper < q)))
  return fn


def get_sample_fn(env, body, collision_fn=lambda q: False):
  limits = body.GetActiveDOFLimits()
  def fn():
    while True:
      config = np.random.uniform(*limits)
      if not collision_fn(config):
        return config
  return fn


def get_distance_fn(body):
  weights = body.GetActiveDOFWeights()
  def distance_fn(q1, q2):
    diff = body.SubtractActiveDOFValues(q2, q1)
    return math.sqrt(np.dot(weights, diff*diff))
  return distance_fn


def get_extend_fn(body):
  resolutions = body.GetActiveDOFResolutions()
  def extend_fn(q1, q2): # Sequence doesn't include q1
    n = int(np.max(np.abs(np.divide(body.SubtractActiveDOFValues(q2, q1), resolutions)))) + 1
    q = q1
    for i in range(n):
      q = (1./(n-i))*body.SubtractActiveDOFValues(q2, q) + q
      yield q
  return extend_fn


def linear_interpolation(body, q1, q2): # Sequence doesn't include q1
  dq = body.SubtractActiveDOFValues(q2, q1)
  steps = np.abs(np.divide(dq, body.GetActiveDOFResolutions())) + 1
  n = int(np.max(steps))
  for i in range(n):
    yield q1 + (1.+i)/n*dq

#################################################################

def extract_config(manipulator, spec, data):
  return spec.ExtractJointValues(data, manipulator.GetRobot(), manipulator.GetArmIndices())


def sample_manipulator_trajectory(manipulator, traj): # TODO - set active joints?
  spec = traj.GetConfigurationSpecification()
  waypoints = [extract_config(manipulator, spec, traj.GetWaypoint(i)) for i in range(traj.GetNumWaypoints())]
  yield waypoints[0]
  for start, end in zip(waypoints, waypoints[1:]):
    for conf in linear_interpolation(manipulator.GetRobot(), start, end):
      yield conf


def sample_base_trajectory(robot, traj): # TODO - set active joints instead?
  spec = traj.GetConfigurationSpecification()
  waypoints = [extract_base_values(robot, spec, traj.GetWaypoint(i)) for i in range(traj.GetNumWaypoints())]
  yield waypoints[0]
  for start, end in zip(waypoints, waypoints[1:]):
    for conf in linear_interpolation(robot, start, end):
      yield conf

#################################################################

def has_mp():
  try:
    import motion_planners
  except ImportError:
    return False
  return True


def use_mp():
  if not has_mp():
    raise RuntimeError('Requires https://github.com/caelan/motion-planners')


def mp_birrt(robot, q1, q2, **kwargs):
  use_mp()
  from motion_planners.rrt_connect import birrt
  env = robot.GetEnv()
  with collision_saver(env, openravepy_int.CollisionOptions.ActiveDOFs):
    # Excluding results in slow plans
    return birrt(q1, q2, get_distance_fn(robot), get_sample_fn(env, robot),
             get_extend_fn(robot), get_collision_fn(env, robot), **kwargs)


def mp_straight_line(robot, q1, q2):
  use_mp()
  from motion_planners.rrt_connect import direct_path
  env = robot.GetEnv()
  with collision_saver(env, openravepy_int.CollisionOptions.ActiveDOFs):
    return direct_path(q1, q2, get_extend_fn(robot),
                       get_collision_fn(env, robot))

#################################################################

def plan_path(base_manip, q1, q2, **kwargs):
  if has_mp():
    return mp_birrt(base_manip.robot, q1, q2, **kwargs)
  else:
    with base_manip.robot:
      set_active_config(base_manip.robot, q1)
      manipulator = base_manip.robot.GetActiveManipulator()
      # return cspace_traj_helper(base_manip, cspace, q2, max_iterations=10)
      return manipulator_motion_plan(base_manip, manipulator, q2, max_iterations=10)


def plan_straight_path(robot, q1, q2):
  if has_mp():
    return mp_straight_line(robot, q1, q2)
  else:
    with robot:
      set_active_config(robot, q1)
      return linear_motion_plan(robot, q2)

#################################################################

def linear_motion_plan(robot, end_config):
  env = robot.GetEnv()
  with robot:
    #robot.SetActiveDOFs(get_active_arm_indices(robot))
    with collision_saver(env, ):
      start_config = get_active_config(robot)
      path = [start_config] + list(linear_interpolation(robot, start_config, end_config))
      for conf in path:
        set_active_config(robot, conf)
        if env.CheckCollision(robot):
          return None
      return path


def cspace_motion_plan(base_manip, indices, goal, step_length=MIN_DELTA, max_iterations=10, max_tries=1):
  with base_manip.robot:
    base_manip.robot.SetActiveDOFs(indices)
    with collision_saver(base_manip.robot.GetEnv(), openravepy_int.CollisionOptions.ActiveDOFs):
      try:
        traj = base_manip.MoveActiveJoints(goal=goal, steplength=step_length, maxiter=max_iterations, maxtries=max_tries,
            execute=False, outputtraj=None, goals=None, outputtrajobj=True, jitter=None, releasegil=False, postprocessingplanner=None,
            postprocessingparameters=None)
        # TODO - extend sample_manipulator_trajectory to indices
        return traj
        #return list(sample_manipulator_trajectory(manipulator, traj))
      except planning_error:
        return None


def base_motion_plan(base_manip, goal, step_length=MIN_DELTA, max_iterations=10, max_tries=1):
#def base_motion_plan(base_manip, goal, step_length=None, max_iterations=50, max_tries=1):
  with base_manip.robot:
    set_active(base_manip.robot, use_base=True)
    with collision_saver(base_manip.robot.GetEnv(), openravepy_int.CollisionOptions.ActiveDOFs):
      try:
        traj = base_manip.MoveActiveJoints(goal=goal, steplength=step_length, maxiter=max_iterations, maxtries=max_tries,
            execute=False, outputtraj=None, goals=None, outputtrajobj=True, jitter=None, releasegil=False, postprocessingplanner=None,
            postprocessingparameters=None)
        # TODO - extend sample_manipulator_trajectory to indices
        return list(sample_base_trajectory(base_manip.robot, traj))
      except planning_error:
        return None


def manipulator_motion_plan(base_manip, manipulator, goal, step_length=MIN_DELTA, max_iterations=10, max_tries=1):
  with base_manip.robot:
    base_manip.robot.SetActiveManipulator(manipulator)
    base_manip.robot.SetActiveDOFs(manipulator.GetArmIndices())
    with collision_saver(base_manip.robot.GetEnv(), openravepy_int.CollisionOptions.ActiveDOFs):
      try:
        traj = base_manip.MoveManipulator(goal=goal,
            maxiter=max_iterations, execute=False, outputtraj=None, maxtries=max_tries,
            goals=None, steplength=step_length, outputtrajobj=True, jitter=None, releasegil=False)
        return list(sample_manipulator_trajectory(manipulator, traj))
      except planning_error:
        return None


def workspace_motion_plan(base_manip, manipulator, vector, steps=10): # TODO - use IK to check if even possibly valid
  distance, direction = length(vector), normalize(vector)
  step_length = distance/steps
  with base_manip.robot:
    base_manip.robot.SetActiveManipulator(manipulator)
    base_manip.robot.SetActiveDOFs(manipulator.GetArmIndices())
    with collision_saver(base_manip.robot.GetEnv(), openravepy_int.CollisionOptions.ActiveDOFs):
      try: # TODO - Bug in specifying minsteps. Need to specify at least 2 times the desired otherwise it stops early
        traj = base_manip.MoveHandStraight(direction, minsteps=10*steps, maxsteps=steps, steplength=step_length,
            ignorefirstcollision=None, starteematrix=None, greedysearch=True, execute=False, outputtraj=None, maxdeviationangle=None,
            planner=None, outputtrajobj=True)
        return list(sample_manipulator_trajectory(manipulator, traj))
      except planning_error:
        return None

