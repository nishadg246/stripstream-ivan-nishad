import math

from manipulation.motion.cspace import CSpace
from manipulation.motion.trajectories import PathTrajectory
from manipulation.primitives.look import get_scan_path
from robotics.openrave.belief_utils import sample_trajectory, sample_base_trajectory, close_gripper_trajectory, \
    open_gripper_trajectory, get_conf

class Command(object): # TODO: GripperCommand
    def __init__(self, arm, ty, grasp):
        self.arm = arm
        #self.body = body
        self.ty = ty
        self.grasp = grasp
    def execute(self):
        raise NotImplementedError()
    #def simulate(self):
    #    raise NotImplementedError()
    def __str__(self):
        return '{}({},{},{})'.format(self.__class__.__name__,
                                     self.arm.GetRobot().GetName(),
                                     self.arm.GetName(), self.ty)
    __repr__ = __str__

#class MotionCommand(Command):
#    def simulate(self):
#        raise NotImplementedError()
#    def execute(self):
#        raise NotImplementedError()

# TODO - motion command?

class GrabCommand(Command):
    def execute(self):
        #self.arm.GetRobot().Grab(self.body)
        robot = self.arm.GetRobot()
        env = robot.GetEnv()
        for body in env.GetBodies():
            if (robot != body) and env.CheckCollision(robot, body):
                # TODO: set active manipulator
                robot.Grab(body)


class ReleaseCommand(Command):
    def execute(self):
        #self.arm.GetRobot().Release(self.body)
        robot = self.arm.GetRobot()
        #robot.ReleaseAllGrabbed()
        # return
        # TODO: can always regrab
        # TODO: stop simulation?
        #for body in robot.GetEnv().GetBodies():
        for body in robot.GetGrabbed():
            if (robot != body) and self.arm.IsGrabbing(body):
            #if (robot != body) and (robot.IsGrabbing(body) is not None):
                robot.Release(body) # TODO: fails here when simulation is running

def process_scan_room(robot):
    # TODO: could do this whole thing symbolically using point head
    path = get_scan_path(robot, math.pi / 6)
    if path is None:
        raise RuntimeError()
    cspace = CSpace.robot_manipulator(robot, 'head')
    h_traj = PathTrajectory(cspace, path)
    h_traj.traj() # Precompute the traj
    return [h_traj]

#File "/Users/Caelan/Programs/LIS/git/lis-openrave/manipulation/motion/trajectories.py", line 133, in traj_from_path
#    maxvelmult=vel_multi, maxaccelmult=accel_multi) # NOTE - kwargs don't work unless ordered correctly
#openrave_exception: openrave (InconsistentConstraints): [virtual OpenRAVE::PlannerStatus TrajectoryRetimer::PlanPath(TrajectoryBasePtr):136] upper limit for traj point 425 dof 0 is not followed (3.003194253102363e+00 > 3.000000000000000e+00)

def process_move_base(robot, q1, q2):
    # TODO: base FCL doesn't work well with collision_saver. ODE does though?
    #with collision_saver(env, openravepy_int.CollisionOptions.ActiveDOFs):
    #b_traj = sample_trajectory(robot, q1.value, q2.value, 'base',
    #                           restarts=5, iterations=50, smooth=50)
    b_traj = sample_base_trajectory(robot, q1.value, q2.value,
                               restarts=5, iterations=50, smooth=50)
    if b_traj is None:
        raise RuntimeError()
    b_traj.traj() # TODO: confirm no joint limit errors
    return [b_traj]


def process_move_head(robot, q1, q2):
    cspace = CSpace.robot_manipulator(robot, 'head')
    h_traj = PathTrajectory(cspace, [q1.value, q2.value])
    h_traj.traj()
    return [h_traj]


def process_pick(arm, g, m):
    # TODO: compute the motion plans here

    #env = arm.GetRobot().GetEnv()
    #for body in env.GetBodies():
    #    # TODO: automatically figure out grasp using collision / task manip
    #    if np.allclose(pose_from_trans(body.GetTransform()), p.value):
    #        break
    #else:
    #    raise RuntimeError()
    #body = env.GetKinBody(i)
    #ty = belief_from_name[i].type
    return [m.fa_traj, m.fg_traj,
            close_gripper_trajectory(arm),
            GrabCommand(arm, g.type, g),
            m.rg_traj, m.ra_traj]

def process_place(arm, g, m):
    return [m.fa_traj, m.fg_traj,
            ReleaseCommand(arm, g.type, g),
            open_gripper_trajectory(arm),
            m.rg_traj, m.ra_traj]

def process_prepush(arm, g, m):
    return [close_gripper_trajectory(arm),m.fa_traj, m.fg_traj,GrabCommand(arm, g.type, g),m.fpush_traj]



def process_plan(arm, plan):
    # TODO: should this be a generator or not?
    # TODO: smooth these trajectories additionally
    robot = arm.GetRobot()
    trajectories = []
    hq = get_conf(robot, 'head')
    for step, (action, args) in enumerate(plan):
        if action.name in ('scan_table', 'register'):
            pass
        elif action.name == 'scan_room':
            trajectories += process_scan_room(robot)
        elif action.name == 'open_gripper':
            trajectories += [open_gripper_trajectory(arm)]
        elif action.name == 'close_gripper':
            trajectories += [close_gripper_trajectory(arm)]
        elif action.name == 'move_head':
            # TODO: move head is effectively free. I could just not worry about head confs...
            hq2 = args[-1]
            trajectories += process_move_head(robot, hq, hq2)
            hq = hq2
        elif action.name == 'move_base':
            trajectories += process_move_base(robot, *args)
        elif action.name == 'pick':
            i, p, g, bq, m = args
            trajectories += process_pick(arm, g, m)
        elif action.name == 'place':
            i, p, g, bq, m = args
            trajectories += process_place(arm, g, m)
        elif action.name == 'push':
            i, p, g, bq, m = args
            trajectories += process_prepush(arm, g, m)
        else:
            raise ValueError(action.name)
    return trajectories


def simulate_trajectories(env, trajectories, time_step=0.001, realtime=False):
  env.StartSimulation(time_step, realtime=realtime) # TODO - causes segmentation faults when using realtime
  for trajectory in trajectories:
    if isinstance(trajectory, Command):
      env.StopSimulation()
      trajectory.execute()
      env.StartSimulation(time_step, realtime=realtime)  # TODO - causes segmentation faults when using realtime
    else:
        trajectory.execute()
  env.StopSimulation()

#def simulate_trajectories(trajectories, blocking=True):
#    for trajectory in trajectories:
#        print 'Simulating {} trajectory'.format(trajectory.cspace.manipulator)
#        trajectory.cspace.body.GetController().SetPath(trajectory.traj())
#        if blocking:
#            trajectory.cspace.body.WaitForController(0)