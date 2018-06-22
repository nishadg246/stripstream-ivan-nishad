#!/usr/bin/env python2

import argparse
import sys

import numpy as np
from openravepy import Environment, RaveDestroy

from manipulation.primitives.display import draw_arrow
from robotics.openrave.belief_tamp import get_ground_problem, add_world, Future
from robotics.openrave.belief_utils import World, Belief, Surface, Object, get_rectangle, get_table_pose, \
    get_object_meshes, set_conf, Task, belief_from_task
from robotics.openrave.postprocessing import process_plan, simulate_trajectories
from robotics.openrave.transforms import get_point, norm, point_from_pose, pose_from_trans, point_from_trans
from robotics.openrave.utils import execute_viewer, EnvironmentStateSaver, get_name
from scripts.openrave.run_belief_state import TABLE_X, TABLE_Z, load_env, MESHES_DIR
from stripstream.algorithms.incremental.incremental_planner import incremental_planner
from stripstream.algorithms.plan import plan_length, plan_cost
from stripstream.algorithms.search.fast_downward import get_fast_downward
from stripstream.pddl.utils import convert_plan
from stripstream.utils import SEPARATOR


##################################################

def get_world_1(object_meshes):
    block_z = np.min(object_meshes['block'].vertices[:, 2])
    surfaces = [Surface('table', get_table_pose(
        TABLE_X, 0, TABLE_Z), get_rectangle(0.6, 1.2))]
    items = [Object('block', get_table_pose(TABLE_X, 0, TABLE_Z - block_z), None)]
    return World(None, surfaces, items), Task(holding='block')

def get_world_2(object_meshes):
    block_z = np.min(object_meshes['block'].vertices[:, 2])
    surfaces = [
        Surface('table', get_table_pose(TABLE_X, 0, TABLE_Z), get_rectangle(0.6, 1.2)),
        Surface('dinner', get_table_pose(0, TABLE_X, TABLE_Z), get_rectangle(1.2, 0.6))]
    items = [Object('block', get_table_pose(TABLE_X, 0, TABLE_Z - block_z), None)]
    return World(None, surfaces, items), Task(object_surfaces=[('block', 'dinner')])

def get_world_3(object_meshes):
    soup_z = np.min(object_meshes['soup'].vertices[:, 2])
    surfaces = [Surface('table', get_table_pose(
        TABLE_X, 0, TABLE_Z), get_rectangle(0.6, 1.2))]
    items = [Object('soup', get_table_pose(TABLE_X, 0, TABLE_Z - soup_z), None)]
    return World(None, surfaces, items), Task(holding='soup')

def get_world_4(object_meshes):
    block_z = np.min(object_meshes['block'].vertices[:, 2])
    surfaces = [Surface('table', get_table_pose(
        TABLE_X, 0, TABLE_Z), get_rectangle(0.6, 1.2))]
    items = [Object('block', get_table_pose(TABLE_X, 0.3, TABLE_Z - block_z), None)]
    #return World(None, surfaces, items), Task(localized_items=['block'])
    return World(None, surfaces, items), Task(registered_items=['block'])

def get_world_5(object_meshes):
    block_z = np.min(object_meshes['block'].vertices[:, 2])
    soup_z = np.min(object_meshes['soup'].vertices[:, 2])
    surfaces = [
        Surface('table', get_table_pose(TABLE_X, 0, TABLE_Z), get_rectangle(0.6, 1.2)),
        Surface('table', get_table_pose(0, TABLE_X, TABLE_Z), get_rectangle(1.2, 0.6))]
    items = [Object('block', get_table_pose(TABLE_X, 0.3, TABLE_Z - block_z), None),
             Object('soup', get_table_pose(TABLE_X, -0.3, TABLE_Z - soup_z), None)]
    return World(None, surfaces, items), Task(holding='block')

def get_world_6(object_meshes):
    block_z = np.min(object_meshes['block'].vertices[:, 2])
    soup_z = np.min(object_meshes['soup'].vertices[:, 2])
    surfaces = [
        Surface('table', get_table_pose(TABLE_X, 0, TABLE_Z), get_rectangle(0.6, 1.2)),
        Surface('table', get_table_pose(0, TABLE_X, TABLE_Z), get_rectangle(1.2, 0.6))]
    items = [Object('block', get_table_pose(TABLE_X, 0.3, TABLE_Z - block_z), None),
             Object('soup', get_table_pose(0, TABLE_X, TABLE_Z - soup_z), None)]
    return World(None, surfaces, items), Task(clustered_items=[('block', 'soup')])

def get_world_7(object_meshes):
    block_z = np.min(object_meshes['block'].vertices[:, 2])
    surfaces = [
        Surface('table', get_table_pose(0, -TABLE_X, TABLE_Z), get_rectangle(1.2, 0.6)),
        Surface('table', get_table_pose(0, TABLE_X, TABLE_Z), get_rectangle(1.2, 0.6))]
    items = [Object('block', get_table_pose(0.3, -TABLE_X, TABLE_Z - block_z), None)]
    return World(None, surfaces, items), Task(left_items=['block'])

def get_lis_world(object_meshes):
    block_z = np.min(object_meshes['block'].vertices[:, 2])
    surfaces = [
        Surface('table', get_table_pose(1.5, 0.5, TABLE_Z), get_rectangle(0.6, 1.2)),
        Surface('table', get_table_pose(0, -1.5, TABLE_Z), get_rectangle(1.2, 0.6)),
        #Surface('table', get_table_pose(-1.5, 1.5, TABLE_Z, np.pi/4), get_rectangle(1.2, 0.6)),
    ]
    items = [Object('block', get_table_pose(1.25, 0.0, TABLE_Z - block_z), None)]
    return World(None, surfaces, items), Task(right_items=['block'])

# TODO: start off holding

def fully_observable(world):
    return Belief(*world)

def unknown(world):
    return Belief(world.holding, [], [])

##################################################

def get_pose_index(items, pose):
    for index, item in enumerate(items):
        if (item.pose is not None) and (len(pose) == len(item.pose)) and np.allclose(item.pose, pose):
            return index
    return None

#def get_point_index(items, pose):
#    point1 = point_from_pose(pose) if is_oriented(pose) else pose
#    for index, item in enumerate(items):
#        if item.pose is None:
#            continue
#        point2 = point_from_pose(item.pose) if is_oriented(item.pose) else item.pose
#        if np.allclose(point1, point2):
#            return index
#    return None

def update_world(robot, world, belief, action, args):
    # Using the robot state (and octree) as ground truth
    if action.name == 'scan_room':
        #new_world = copy.copy(world)
        new_belief = Belief(belief.holding, world.surfaces[:], belief.items[:])
        return world, new_belief
    #elif action.name == 'scan_table':
    #    new_belief = Belief(belief.holding, belief.surfaces[:], world.items[:])
    #    return world, new_belief
    elif action.name == 'move_head':
        _, q2 = args
        set_conf(robot, 'head', q2.value)
        point = get_point(robot.GetManipulator('head'))
        # TODO: condition here
        # TODO: could update the measurement here
        #max_distance = MAX_REG_DISTANCE
        max_distance = 10
        items = [Object(ty, p, (norm(point_from_pose(p) - point) <= max_distance))
                 for ty,p,_ in world.items]
        return world, Belief(belief.holding, belief.surfaces[:], items)
    elif action.name == 'move_base':
        _, q2 = args
        set_conf(robot, 'base', q2.value)
        items = [Object(ty, p, False) for ty, p, _ in world.items]
        return world, Belief(belief.holding, belief.surfaces[:], items)
    elif action.name == 'pick':
        _, p, g, bg, _ = args # TODO: use g.value
        world_index = get_pose_index(world.items, p.value)
        world_items = world.items[:]
        world_item = world_items.pop(world_index)
        belief_index = get_pose_index(belief.items, p.value)
        belief_items = belief.items[:]
        belief_item = belief_items.pop(belief_index)
        #assert world_item == belief_item # Maybe it's good to not check
        return World(Object(world_item.type, g, None), world.surfaces, world_items), \
               Belief(Object(belief_item.type, g, False), world.surfaces, belief_items)
    elif action.name == 'place':
        _, p, g, bg, _ = args
        world_items = world.items[:] + [Object(world.holding.type, p.value, None)]
        belief_items = belief.items[:] + [Object(belief.holding.type, p.value, False)]
        return World(None, world.surfaces, world_items), \
               Belief(None, world.surfaces, belief_items)
    else:
        raise ValueError(action.name)

##################################################

# TODO: sample pushes
# TODO: estimate table rectangles from convex hull
# TODO: sample places near the centroid of tables
# TODO: visibility constraints on picking
# TODO: negative observations from occupancy
# TODO: initialization actions

WIDTH, HEIGHT = 640, 480
FX, FY = 772.55, 772.5

def get_camera_matrix(width, height, fx, fy):
    # cx, cy = 320.5, 240.5
    cx, cy = width / 2., height / 2.
    return np.array([
        [fx, 0, cx],
        [0, fy, cy],
        [0, 0, 1]])

#CAMERA_LINK = 'high_def_optical_frame'
CAMERA_LINK = 'head_mount_kinect_rgb_optical_frame'

def is_visible(robot, point_world, max_range=float('inf')):
    # TODO: obstructions
    link = robot.GetLink(CAMERA_LINK)
    world_from_head = link.GetTransform()
    point_head = np.linalg.inv(world_from_head).dot(np.append(point_world, 1))[:3]
    z = point_head[2]
    if (z < 0) or (max_range <= z):
        return False
    camera_matrix = get_camera_matrix(WIDTH, HEIGHT, FX, FY)
    pixel = camera_matrix.dot(point_head/z)
    x, y, _ = pixel
    return (0 <= x < WIDTH) and (0 <= y < HEIGHT) #and (0 <= z)

def draw_viewcone(robot, z=5, color=(1, 0, 0, 0.25)):
    env = robot.GetEnv()
    link = robot.GetLink(CAMERA_LINK)
    world_from_head = link.GetTransform()

    cone = [(0, 0), (WIDTH, 0), (WIDTH, HEIGHT), (0, HEIGHT)]
    camera_matrix = get_camera_matrix(WIDTH, HEIGHT, FX, FY)
    pixel_from_head = np.eye(4)
    pixel_from_head[:3, :3] = camera_matrix

    sensor_world = point_from_trans(world_from_head)
    vertices = [sensor_world]
    indices = []
    handles = []
    for i, pixel in enumerate(cone):
        ray_head = z * np.linalg.inv(camera_matrix).dot(np.append(pixel, 1))
        ray_world = world_from_head.dot(np.append(ray_head, 1))[:3]
        vertices.append(ray_world)
        indices.append((0,1+i,1+(i+1)%len(cone)))
        #handles.append(draw_arrow(env, sensor_world, ray_world))
    handles.append(env.drawtrimesh(np.array(vertices),np.array(indices),
                                   colors=np.array(color)))
    #env.GetViewer().SetCamera(world_from_head)
    return handles

def observe_env(robot, p_hit_vis=1.):
    observations = {}
    env = robot.GetEnv()
    for body in env.GetBodies():
        if (body == robot) or robot.IsGrabbing(body):
            continue
        world_from_body = body.GetTransform()
        if (np.random.random() <= p_hit_vis) and is_visible(robot, point_from_trans(world_from_body)):
            observations[get_name(body)] = pose_from_trans(world_from_body)
    return observations

def get_plan_prefix(plan):
    uncertain = False
    plan_prefix = []
    for action, args in plan:
        if any(isinstance(arg, Future) for arg in args):
            break
        if action.name in ('move_base', 'pick', 'place'):
            if uncertain:
                break
            uncertain = True
        plan_prefix.append((action, args))
    return plan_prefix

def solve_sequence(env):
    # TODO: simulate the work for real
    use_viewer = (env.GetViewer() is not None)
    robot, occupancy = load_env(env)
    arm = robot.GetActiveManipulator()
    # Two environments doesn't work because at robot can only be in one

    get_world = get_world_1
    #get_world = get_world_2
    #get_world = get_world_3
    #get_world = get_world_4

    object_meshes = get_object_meshes(MESHES_DIR)
    world, task = get_world(object_meshes)

    prior = unknown(world)
    #prior = fully_observable(world)

    print SEPARATOR
    print world
    print task

    # TODO: could try using two display windows
    while True:
        print SEPARATOR
        add_world(robot, object_meshes, world)
        #env.UpdatePublishedBodies()
        saver = EnvironmentStateSaver(env)  # TODO: won't help if removed...
        #print observe_env(robot)

        belief = belief_from_task(prior, task) # TODO: don't recreate this on each step...
        print belief

        #with env:
        env.Lock()
        stream_problem, belief_from_name = get_ground_problem(arm, object_meshes, belief, task, occupancy)
        print stream_problem
        # TODO: most of the overhead comes in translation still
        plan, universe = incremental_planner(stream_problem, search=get_fast_downward('ff-astar', verbose=False),
                                             optimal=True, frequency=1, waves=True, debug=False, max_time=5.0)
        env.Unlock()

        print SEPARATOR
        print 'Plan: {}\nLength: {} | Cost: {}'.format(convert_plan(plan), plan_length(universe, plan),
                                                       plan_cost(universe, plan))
        if plan is None:
            raise RuntimeError('Unable to find a plan')
        if not plan:
            break
        plan_prefix = get_plan_prefix(convert_plan(plan))
        print plan_prefix
        trajectories = process_plan(arm, plan_prefix)
        print trajectories
        if use_viewer:
            raw_input('Execute?')
            saver.Restore()
            simulate_trajectories(env, trajectories)

        saver.Restore()
        action, args = plan_prefix[0]
        # TODO: update the belief entirely from the env
        world, prior = update_world(robot, world, prior, action, args)
        #world, prior = update_world(robot, world, belief, action, args)
    if use_viewer:
        raw_input('Finish?')

##################################################

def main(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument('-viewer', action='store_true', help='enable viewer.')
    args = parser.parse_args()

    env = Environment()
    try:
        execute = lambda: solve_sequence(env)
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
