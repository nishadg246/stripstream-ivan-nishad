from collections import defaultdict

import numpy as np
import argparse
import sys

from openravepy import Environment, RaveDestroy

from robotics.openrave.belief_tamp import is_oriented, MAX_REG_DISTANCE, MAX_LOOK_DISTANCE, \
    add_surface, add_item, add_holding, solve_tamp, solve_hierarchical_tamp
from robotics.openrave.belief_utils import Object, Belief, Surface, get_object_meshes, belief_from_task
from robotics.openrave.postprocessing import process_plan, simulate_trajectories
from robotics.openrave.transforms import point_from_pose, length, get_point, pose_from_trans
from robotics.openrave.utils import execute_viewer, get_name, Grasp, EnvironmentStateSaver, \
    draw_circle_2d, draw_affine_limits
from scripts.openrave.run_belief_state import load_env, MESHES_DIR
from scripts.openrave.run_belief_tamp import get_pose_index, observe_env, get_plan_prefix, \
    get_world_1, get_world_2, get_world_3, get_world_4, get_world_5, get_world_6, \
    get_world_7, unknown, get_lis_world, draw_viewcone

from stripstream.pddl.utils import convert_plan
from stripstream.utils import SEPARATOR


def update_belief(robot, world, belief, plan):
    # TODO: before or after movement?
    holding, surfaces, items = belief.holding, belief.surfaces, belief.items
    surface_types = {s.type: s for s in world.surfaces}
    item_types = {i.type: i for i in world.items}
    head = robot.GetManipulator('head')
    for action, args in plan:
        if action.name == 'scan_room':
            # TODO: observe objects under some conditions
            surfaces = world.surfaces[:]
        elif action.name == 'move_head':
            #items = world.items[:]
            # TODO: should this be a sense action?
            pass
        elif action.name == 'move_base':
            items = [Object(ty, p, False) for ty, p, _ in items]
        elif action.name == 'register':
            pass
        elif action.name == 'pick':
            if holding is not None: print holding
            assert holding is None
            _, p, g, bg, _ = args # TODO: use g.value
            item = items.pop(get_pose_index(items, p.value))
            holding = Object(item.type, g, False)
        elif action.name == 'place':
            assert holding is not None
            _, p, g, bg, _ = args
            items.append(Object(holding.type, p.value, False))
            holding = None
        else:
            raise ValueError(action.name)
        # TODO: add noise to these?
        #for name, pose in observe_env(robot, p_hit_vis=0.25).items():
        visible = observe_env(robot, p_hit_vis=1)
        print 'Visible:', visible.keys()
        for name, pose in visible.items():
            ty, _ = name.split('-')
            if ty in item_types:
                # TODO: probabilities for detect and register
                for index, item in reversed(list(enumerate(items))):
                    if ty == item.type:
                        point = point_from_pose(item.pose) if is_oriented(item.pose) else item.pose
                        if (point is None) or np.allclose(point_from_pose(pose), point):
                            items.pop(index)
                #index = get_point_index(items, pose)
                distance = length(get_point(head)[:2] - point_from_pose(pose)[:2])
                pose = pose if (distance <= MAX_REG_DISTANCE) else point_from_pose(pose)
                items.append(Object(ty, pose, is_oriented(pose)))
            elif ty in surface_types:
                [matched] = [s for s in surfaces if (s.type == ty) and (s.pose is not None) and
                            np.allclose(s.pose, pose)]
                matched.observations += 1
    return Belief(holding, surfaces, items)


def observable_belief(robot, world):
    holding = None
    surfaces = []
    items = []
    surface_types = {s.type: s for s in world.surfaces}
    item_types = {i.type: i for i in world.items}
    grab_info_from_name = {info._grabbedname: info for info in robot.GetGrabbedInfo()}
    for body in robot.GetEnv().GetBodies():
        if body.IsRobot():
           continue
        name = get_name(body)
        ty, _ = name.split('-')
        if name in grab_info_from_name:
            assert (holding is None) and (ty in item_types)
            #info = grab_info_from_name[name]
            arm = robot.GetActiveManipulator()
            #link = robot.GetActiveManipulator().GetEndEffector()
            #world_from_gripper = link.GetTransform()
            world_from_gripper = arm.GetTransform()
            world_from_body = body.GetTransform()
            grasp = Grasp(np.linalg.inv(world_from_gripper).dot(world_from_body))
            #grasp = Grasp(np.linalg.inv(world_from_body).dot(world_from_gripper))
            #print link, info._robotlinkname # Equal
            #grasp = Grasp(np.linalg.inv(info._trelative))
            holding = Object(ty, grasp, True)
        else:
            pose = pose_from_trans(body.GetTransform())
            if ty in surface_types:
                [mesh] = [l.GetCollisionData() for l in body.GetLinks()]
                surfaces.append(Surface(ty, pose, mesh.vertices))
            elif ty in item_types:
                items.append(Object(ty, pose, True))
            else:
                raise ValueError(ty)
    return Belief(holding, surfaces, items)

##################################################

def initial_observable_belief(world):
    # TODO: might need to change the item registration
    return Belief(world.holding, world.surfaces, world.items)

def initial_surface_belief(world):
    return Belief(world.holding, world.surfaces, [])

def initial_visible_belief(world):
    items = [Object(item.type, point_from_pose(item), False) for item in world.items]
    return Belief(world.holding, world.surfaces, items)

def initialize_world(robot, object_meshes, world):
    env = robot.GetEnv()
    type_counts = defaultdict(int)
    def get_name(obj):
        ty = obj.type
        name = '{}-{:d}'.format(ty, type_counts[ty])
        type_counts[ty] += 1
        return name
    for surface in world.surfaces:
        add_surface(env, get_name(surface), surface)
    for item in world.items:
        add_item(env, object_meshes, get_name(item), item)
    if world.holding is not None:
        robot.Grab(add_holding(robot, object_meshes, get_name(world.holding), world.holding))

##################################################

def solve_online(env):
    # TODO: simulate the work for real
    robot, occupancy = load_env(env)
    arm = robot.GetActiveManipulator()
    available_actions = ['pick', 'place', 'move_head', 'move_base', 'scan_room', 'scan_table', 'register']

    get_world = get_lis_world # get_world_1 | get_world_2 | get_world_3 | get_world_4 | get_world_5 | get_world_6
    object_meshes = get_object_meshes(MESHES_DIR)
    world, task = get_world(object_meshes)
    #add_world(robot, object_meshes, world)
    initialize_world(robot, object_meshes, world)
    initial_tform = robot.GetTransform()
    print world
    print task

    #prior = unknown(world) # TODO: start off not facing the object
    prior = initial_surface_belief(world)

    belief = belief_from_task(prior, task)
    #belief = update_belief(robot, world, belief, []) # Updates with initial observations
    #belief = observable_belief(robot, world)
    history = []
    while True:
        #belief = belief_from_task(belief, task)
        print SEPARATOR
        print belief
        saver = EnvironmentStateSaver(env)
        handles = [
            draw_circle_2d(env, get_point(robot), MAX_LOOK_DISTANCE, color=np.array([1, 0, 0, 1])),
            draw_circle_2d(env, get_point(robot), MAX_REG_DISTANCE, color=np.array([0, 1, 0, 1])),
            draw_affine_limits(robot)] + draw_viewcone(robot, z=2.5)
        # TODO: draw view cone
        with env:
            #plan = solve_tamp(arm, object_meshes, belief, task, occupancy,
            #                                             available_actions=available_actions)
            plan = solve_hierarchical_tamp(arm, object_meshes, belief, task, occupancy,
                                           initial_tform, available_actions=available_actions)
        # TODO: some sort of shift when this happens
        if plan is None:
            raise RuntimeError('Unable to find a plan')
        if not plan:
            break
        plan_prefix = get_plan_prefix(convert_plan(plan))
        print plan_prefix
        trajectories = [process_plan(arm, [action]) for action in plan_prefix]
        print trajectories

        saver.Restore()
        raw_input('Execute?')
        for action, trajs in zip(plan_prefix, trajectories):
            simulate_trajectories(env, trajs, time_step=0.01, realtime=False)
            belief = update_belief(robot, world, belief, [action])
        #belief = observable_belief(robot, world)
        history += plan_prefix

    raw_input('Finish?')

##################################################

def main(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument('-viewer', action='store_true', help='enable viewer.')
    args = parser.parse_args()

    env = Environment()
    try:
        #execute = lambda: solve_sequence(env)
        execute = lambda: solve_online(env)
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