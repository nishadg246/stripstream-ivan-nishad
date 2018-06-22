#!/usr/bin/env python2

import time
import argparse
import numpy as np
import sys
import os

from openravepy import Environment, RaveDestroy, RaveCreateCollisionChecker
from manipulation.primitives.look import get_scan_path

from robotics.openrave.belief_tamp import MAX_DISTANCE, get_ground_problem, DEFAULT_LEFT_ARM, REST_RIGHT_ARM
from robotics.openrave.belief_utils import Task, Belief, Surface, Object, get_table_pose, get_rectangle, ODE, FCL, \
    Occupancy, set_conf, sample_trajectory, get_object_meshes, get_input, belief_from_task
from robotics.openrave.postprocessing import process_plan, simulate_trajectories
from robotics.openrave.problems import TOP_HOLDING_LEFT_ARM, REST_LEFT_ARM, WIDE_LEFT_ARM, WIDE_RIGHT_ARM
from robotics.openrave.transforms import unit_pose, camera_look_at
from robotics.openrave.utils import set_manipulator_conf, open_gripper, mirror_arm_config, close_gripper, \
    initialize_openrave, set_gripper, EnvironmentStateSaver, execute_viewer

from stripstream.algorithms.incremental.incremental_planner import incremental_planner
from stripstream.algorithms.plan import plan_length, plan_cost
from stripstream.algorithms.search.fast_downward import get_fast_downward
from stripstream.pddl.utils import convert_plan
from stripstream.utils import SEPARATOR

MESHES_DIR = '../lis-openrave/manipulation/clouds'


TABLE_X = 2.25
TABLE_Z = 0.73

OCTREE_MIN_HEIGHT = 0.2
OCTREE_MAX_HEIGHT = 1.2
#OCTREE_MIN_HEIGHT = 0.6
#OCTREE_MAX_HEIGHT = 1.0

#OCTREE_RESOLUTION = None
#OCTREE_RESOLUTION = 0.05
#OCTREE_RESOLUTION = 0.1
OCTREE_RESOLUTION = 0.2

##################################################

# def get_problem2(object_meshes):
#  table1 = Surface('table', get_table_pose(1, 0, 0.73), get_rectangle(0.6, 1.2))
#  objects = [
#    get_object_on_surface('oil', object_meshes, table1),
#    get_object_on_surface('block', object_meshes, table1),
#    get_object_on_surface('soup', object_meshes, table1),
#  ]
#  return [table1], objects

##################################################


def get_problem_1_0(object_meshes):
    task = Task(holding='block')
    return belief_from_task(Belief(None, [], []), task), task


def get_problem_1_1(object_meshes):
    surfaces = [Surface('table', None, None)]
    items = [Object('block', None, False)]
    return Belief(None, surfaces, items), Task(holding='block')


def get_problem_1_2(object_meshes):
    surfaces = [Surface('table', get_table_pose(
        TABLE_X, 0, TABLE_Z), get_rectangle(0.6, 1.2))]
    items = [Object('block', None, False)]
    return Belief(None, surfaces, items), Task(holding='block')


def get_problem_1_3(object_meshes):
    block_z = np.min(object_meshes['block'].vertices[:, 2])
    surfaces = [Surface('table', get_table_pose(
        TABLE_X, 0, TABLE_Z), get_rectangle(0.6, 1.2))]
    items = [Object('block', get_table_pose(TABLE_X, 0, TABLE_Z - block_z), True)]
    return Belief(None, surfaces, items), Task(holding='block')


def get_problem_2_1(_):
    surfaces = [
        Surface('table', None, None),
        Surface('dinner', None, None)]
    items = [Object('block', None, False)]
    return Belief(None, surfaces, items), Task(object_surfaces=[('block', 'dinner')])


def get_problem_2_2(object_meshes):
    block_z = np.min(object_meshes['block'].vertices[:, 2])
    surfaces = [
        Surface('table', get_table_pose(
            TABLE_X, 0, TABLE_Z), get_rectangle(0.6, 1.2)),
        Surface('dinner', get_table_pose(0, TABLE_X, TABLE_Z), get_rectangle(1.2, 0.6))]
    items = [Object('block', get_table_pose(TABLE_X, 0, TABLE_Z - block_z), True)]
    return Belief(None, surfaces, items), Task(object_surfaces=[('block', 'dinner')])


def get_problem_3_2(object_meshes):
    block_z = np.min(object_meshes['block'].vertices[:, 2])
    surfaces = [Surface('table', get_table_pose(
        TABLE_X, 0, TABLE_Z), get_rectangle(0.6, 1.2))]
    items = [Object('block', get_table_pose(TABLE_X, 0, TABLE_Z - block_z), True)]
    return Belief(None, surfaces, items), Task(object_surfaces=[('block', 'table')])


def get_problem_4(object_meshes):
    table_x = 0.5
    block_z = np.min(object_meshes['block'].vertices[:, 2])
    surfaces = [Surface('table', get_table_pose(
        table_x, 0, TABLE_Z), get_rectangle(0.6, 1.2))]
    items = [Object('block', get_table_pose(table_x, 0.2, TABLE_Z - block_z))]
    return Belief(None, surfaces, items), Task(holding='block')


def get_problem_5_3(object_meshes):
    oil_z = np.min(object_meshes['oil'].vertices[:, 2])
    block_z = np.min(object_meshes['block'].vertices[:, 2])
    soup_z = np.min(object_meshes['soup'].vertices[:, 2])

    surfaces = [Surface('table', get_table_pose(
        2, 0, TABLE_Z), get_rectangle(0.6, 1.2))]
    items = [
        Object('oil', get_table_pose(TABLE_X, -0.25, TABLE_Z - oil_z), True),
        Object('block', get_table_pose(TABLE_X, 0, TABLE_Z - block_z), True),
        Object('soup', get_table_pose(TABLE_X, 0.25, TABLE_Z - soup_z), True)]
    return Belief(None, surfaces, items), Task(holding='block')

##################################################

IGNORE_LINKS = ['l_gripper_r_finger_link', 'l_gripper_r_finger_tip_link',
                'r_gripper_r_finger_link', 'r_gripper_r_finger_tip_link',
                'fr_caster_l_wheel_link', 'fl_caster_r_wheel_link',
                'fr_caster_r_wheel_link', 'fl_caster_l_wheel_link',
                'br_caster_l_wheel_link', 'bl_caster_r_wheel_link',
                'br_caster_r_wheel_link', 'bl_caster_l_wheel_link']


def load_env(env):
    viewer = env.GetViewer()
    if viewer is not None:
        viewer.SetName('Simulation')
        viewer.SetCamera(camera_look_at((0, -2.5, 5), look_point=(0, 0, 0)),
                         focalDistance=5.0)
        viewer.SetSize(640, 480)
        viewer.Move(0, 0)

    #robot = env.ReadRobotXMLFile('robots/pr2-beta-static.zae')
    #robot = env.ReadRobotXMLFile('robots/pr2-beta-sim.robot.xml')

    or_robot = env.ReadRobotXMLFile('robots/pr2-beta-static.zae')
    robot = env.ReadRobotXMLFile('robotics/openrave/environments/robot.dae')
    env.Add(robot)
    for manipulator in or_robot.GetManipulators():
        robot.AddManipulator(manipulator.GetInfo())
    for link_name in IGNORE_LINKS:
        link = robot.GetLink(link_name)
        link.SetVisible(False)
        link.Enable(False)

    set_manipulator_conf(robot.GetManipulator('leftarm'), DEFAULT_LEFT_ARM)
    open_gripper(robot.GetManipulator('leftarm'))

    set_manipulator_conf(robot.GetManipulator('rightarm'), REST_RIGHT_ARM)
    #                     mirror_arm_config(robot, REST_LEFT_ARM))
    #close_gripper(robot.GetManipulator('rightarm'))
    open_gripper(robot.GetManipulator('rightarm'))

    robot.SetDOFValues([.15], [robot.GetJointIndex('torso_lift_joint')])
    robot.SetAffineTranslationLimits(
        *(MAX_DISTANCE * np.array([[-1, -1, 0], [1, 1, 0]])))

    vel_multi = 0.25
    robot.SetDOFVelocityLimits(vel_multi * robot.GetDOFVelocityLimits())
    robot.SetAffineTranslationMaxVels(vel_multi * robot.GetAffineTranslationMaxVels())
    robot.SetAffineRotationAxisMaxVels(vel_multi * robot.GetAffineRotationAxisMaxVels())

    collision_checker = ODE if RaveCreateCollisionChecker(env, FCL) is None else FCL
    _, arm, base_manip, _ = initialize_openrave(env, 'leftarm', min_delta=.01, collision_checker=collision_checker)
    print 'Using collision checker', env.GetCollisionChecker()

    voxels = []
    if OCTREE_RESOLUTION is not None:
        # TODO: add legs of tables
        extent = 3 * (OCTREE_RESOLUTION,)
        centers = []
        for z in np.arange(OCTREE_MIN_HEIGHT, OCTREE_MAX_HEIGHT, OCTREE_RESOLUTION):
            for t in np.arange(-MAX_DISTANCE, MAX_DISTANCE, OCTREE_RESOLUTION):
                if np.random.random() < 0.9:
                    centers += [
                        (t, MAX_DISTANCE, z),
                        (t, -MAX_DISTANCE, z),
                        (MAX_DISTANCE, t, z),
                        (-MAX_DISTANCE, t, z)]
        voxels.append((extent, centers))
    return robot, Occupancy(voxels, unit_pose())

##################################################

def simulate_single(arm, plan):
    robot = arm.GetRobot()
    env = robot.GetEnv()
    arm_name = arm.GetName()

    def _execute_traj(confs, manip_name):
        for j, conf in enumerate(confs):
            set_conf(robot, manip_name, conf)
            time.sleep(0.02)
            env.UpdatePublishedBodies()
            # raw_input('%s/%s) Step?'%(j, len(confs)))

    # Resets the initial state
    # set_manipulator_conf(arm, initial_conf.value)
    # for obj, pose in problem.initial_poses.iteritems():
    #  set_pose(bodies[obj], pose.value)

    # raw_input('Start?')
    for i, (action, args) in enumerate(plan):
        raw_input('Continue?')
        if action.name == 'scan_room':
            _execute_traj(get_scan_path(robot), 'head')
        elif action.name == 'scan_table':
            pass
        elif action.name == 'move_head':
            _, q2 = args
            if type(q2) == str:
                break
            print q2
            _execute_traj([q2.value], 'head')
        elif action.name == 'move_base':
            q1, q2 = args
            if type(q2) == str:
                break
            bt = sample_trajectory(robot, q1, q2, 'base')
            assert bt is not None
            _execute_traj(bt.path()[1:], 'base')
            # _execute_traj([q2.value], 'base')
        elif action.name == 'pick':
            i, p, g, bg, lt = args
            open_gripper(arm)
            _execute_traj(lt.fa_traj.path()[1:], arm_name)  # TODO: cleanup
            _execute_traj(lt.fg_traj.path()[1:], arm_name)
            set_gripper(arm, g.gripper_q)
            robot.Grab(env.GetKinBody(i))
            _execute_traj(lt.rg_traj.path()[1:], arm_name)
            _execute_traj(lt.ra_traj.path()[1:], arm_name)
        elif action.name == 'place':
            i, p, g, bg, lt = args
            _execute_traj(lt.fa_traj.path()[1:], arm_name)
            _execute_traj(lt.fg_traj.path()[1:], arm_name)
            robot.Release(env.GetKinBody(i))
            open_gripper(arm)
            _execute_traj(lt.rg_traj.path()[1:], arm_name)
            _execute_traj(lt.ra_traj.path()[1:], arm_name)
            close_gripper(arm)
        else:
            raise ValueError(action.name)

##################################################

def solve_single(env):
    use_viewer = (env.GetViewer() is not None)
    robot, occupancy = load_env(env)
    arm = robot.GetActiveManipulator()

    #get_problem = get_problem_1_0
    #get_problem = get_problem_1_1
    #get_problem = get_problem_1_2
    get_problem = get_problem_1_3
    #get_problem = get_problem_2_1
    #get_problem = get_problem_2_2
    #get_problem = get_problem_3_2
    #get_problem = get_problem_4

    object_meshes = get_object_meshes(MESHES_DIR)
    belief, task = get_problem(object_meshes)
    print SEPARATOR
    print 'Belief:', belief
    print 'Task:', task

    avaliable_actions = None
    #avaliable_actions = ['pick']
    stream_problem, belief_from_name = get_ground_problem(arm, object_meshes, belief, task, occupancy,
                                                          available_actions=avaliable_actions)
    print stream_problem

    env.Lock()
    plan, universe = incremental_planner(stream_problem, search=get_fast_downward('astar', verbose=False),
                                         optimal=False, frequency=1, waves=True, debug=False, max_time=5.0)
    env.Unlock()
    print SEPARATOR
    length = plan_length(universe, plan)
    cost = plan_cost(universe, plan)
    plan = convert_plan(plan)
    print 'Plan: {}\nLength: {} | Cost: {}'.format(plan, length, cost)

    if use_viewer and (plan is not None):
        trajectories = process_plan(arm, belief_from_name, plan)
        print trajectories
        raw_input('Start?')
        while True:
            with EnvironmentStateSaver(env):
                simulate_trajectories(env, trajectories)
                response = get_input('Replay?', ('y', 'n'))
            if response != 'y':
                break
        # simulate_single(arm, plan)
    if use_viewer:
        raw_input('Finish?')

##################################################

def main(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument('-viewer', action='store_true', help='enable viewer.')
    args = parser.parse_args()

    env = Environment()
    try:
        execute = lambda: solve_single(env)
        if args.viewer:
            execute_viewer(env, execute)
        else:
            execute()
    finally:
        if env.GetViewer() is not None:
            env.GetViewer().quitmainloop()
        RaveDestroy()
    print 'Done!'

if __name__ == '__main__':
    main(sys.argv[1:])