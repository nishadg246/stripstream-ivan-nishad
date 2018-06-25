import math
import random
import time
import cProfile
import pstats

from collections import defaultdict
from itertools import count
from itertools import product, islice

import numpy as np
from openravepy import RaveCreateCollisionChecker, Ray

from manipulation.bodies.bodies import trimesh_from_polygon, mesh_body, octree_body, cell_body
from manipulation.motion.cspace import CSpace
from manipulation.motion.trajectories import PathTrajectory
from manipulation.primitives.look import look_at_point
from robotics.openrave.belief_utils import COLORS, get_conf, set_conf, unit_from_theta, sample_surface_pose, \
    is_surface_pose, get_top_grasps, get_prepush_setting, get_side_grasps, APPROACH_LENGTH, Object, Surface, remove_except, ODE, \
    is_point_on_surface
from robotics.openrave.motion import mp_birrt, mp_straight_line
from robotics.openrave.problems import WIDE_LEFT_ARM, RED
from robotics.openrave.transforms import set_pose, point_from_pose, trans_from_pose, pose_from_trans, trans_from_point, \
    get_pose, length, point_from_trans, base_values_from_trans
from robotics.openrave.utils import Conf, Grasp, open_gripper, solve_inverse_kinematics, \
    set_manipulator_conf, normalize, is_gripper_closed, is_gripper_open, base_within_limits

from stripstream.algorithms.incremental.incremental_planner import incremental_planner, debug_planner
from stripstream.algorithms.plan import plan_length, plan_cost
from stripstream.algorithms.search.fast_downward import get_fast_downward
from stripstream.pddl.cond_streams import EasyGenStream as GeneratorStream, EasyCostStream as CostStream, \
    EasyTestStream as TestStream
from stripstream.pddl.logic.atoms import Atom
from stripstream.pddl.logic.connectives import And, Not, Or, When
from stripstream.pddl.logic.operations import Cost
from stripstream.pddl.logic.predicates import EasyPredicate as Pred, EasyFunction as Func
from stripstream.pddl.logic.quantifiers import Exists, ForAll
from stripstream.pddl.objects import EasyParameter as Param, EasyType as Type, OBJECT
from stripstream.pddl.operators import Action, Axiom
from stripstream.pddl.problem import STRIPStreamProblem
from stripstream.pddl.utils import convert_plan, rename_easy

from manipulation.constants import GRASP_APPROACHES
from manipulation.inverse_reachability.inverse_reachability import get_base_generator, load_custom_ir

# MAX_DISTANCE = 5 # 4
MAX_DISTANCE = 2.5 # TODO: get from perception

TOP_GRASPS = ['soup', 'block']
SIDE_GRASPS = []

#TOP_GRASPS = []
#SIDE_GRASPS = ['soup', 'block', 'oil']

# TODO: add some vertical component to the approach vector

LOW_LEFT_ARM = [1.9596264928676703, 1.2393828041833483, 2.062300669061837, -1.802802875697329, 9.159353547144025, -1.6300287645660583, -28.29690578864352]
LOW_RIGHT_ARM = [-2.0266726533693076, 1.2160249757054882, -2.010886040558367, -1.8520249175677286, -9.163050230101524, -1.5958489670198523, 6.102198172153903]
REST_RIGHT_ARM = [-1.2475162634483552, 1.251893223101777, -1.4696885015600012, -0.2609947994627587, -2.9971292320412792, -0.2931048813977958, 4.528480532151573]

ROBOT_RADIUS = 0.75
MAX_KIN_DISTANCE = 1.0
MAX_LOOK_DISTANCE = 2.5 # 1.5
MAX_REG_DISTANCE = 1.5
DEFAULT_LEFT_ARM = WIDE_LEFT_ARM # TOP_HOLDING_LEFT_ARM | SIDE_HOLDING_LEFT_ARM | WIDE_LEFT_ARM
PICK_PLACE_COST = 5

DRAW_OCTREE = True
FILTER_OCTREE = False

##################################################

class Future(object): # TODO: record sampler / hash arguments?
  _ids = count(0)
  def __init__(self, *args):
    self.id = next(self._ids)
    self.args = tuple(args)
  def __repr__(self):
    return self.__class__.__name__ + '(%d)'%self.id

class Manipulation(object):
  _ids = count(0)
  def __init__(self, rg_traj, ra_traj):
    self.id = next(self._ids)
    self.fa_traj = ra_traj.reverse()
    self.fg_traj = rg_traj.reverse()
    self.rg_traj = rg_traj
    self.ra_traj = ra_traj
  def __repr__(self):
    return self.__class__.__name__ + '(%d)'%self.id

class Manipulation2(object):
  _ids = count(0)
  def __init__(self, rg_traj, ra_traj, rpush_traj):
    self.id = next(self._ids)
    self.fa_traj = ra_traj.reverse()
    self.fg_traj = rg_traj.reverse()
    self.fpush_traj = rpush_traj.reverse()
    self.rg_traj = rg_traj
    self.ra_traj = ra_traj
    self.rpush_traj = rpush_traj
  def __repr__(self):
    return self.__class__.__name__ + '(%d)'%self.id

class Pose(object):
  _ids = count(0)
  def __init__(self, pose, ty):
      self.id = next(self._ids)
      self.value = pose
      self.type = ty # TODO: supported
  def __repr__(self):
      return self.__class__.__name__ + '(%d)' % self.id

##################################################

def is_oriented(pose):
    return (pose is not None) and len(pose) == 7

def extract_point(pose):
    return point_from_pose(pose) if is_oriented(pose) else pose

def add_surface(env, name, surface):
    mesh = trimesh_from_polygon(surface.convex_hull)  # dz = 0.0325
    body = mesh_body(env, mesh, name=name, color=COLORS[surface.type])
    env.Add(body)
    set_pose(body, surface.pose)
    return body

def add_item(env, object_meshes, name, item):
    body = mesh_body(env, object_meshes[item.type], name=name, color=COLORS[item.type])
    env.Add(body)
    set_pose(body, item.pose)
    return body

def add_holding(robot, object_meshes, name, holding):
    world_from_arm = robot.GetManipulator('leftarm').GetTransform()
    gripper_from_obj = trans_from_pose(holding.pose.value)
    pose = pose_from_trans(world_from_arm.dot(gripper_from_obj))
    item = Object(holding.type, pose, False)
    return add_item(robot.GetEnv(), object_meshes, name, item)

def add_world(robot, object_meshes, world):
    remove_except(robot)  # env.Reset()
    env = robot.GetEnv()
    for i, surface in enumerate(world.surfaces):
        add_surface(env, 'surface{:d}'.format(i), surface)
    for i, item in enumerate(world.items):
        add_item(env, object_meshes, 'item{:d}'.format(i), item)
    if world.holding is not None:
        # TODO: set active manipulator
        robot.Grab(add_holding(robot, object_meshes, 'holding', world.holding))
    # TODO: could try to initialize from links...

def add_octree(robot, occupancy):
    t0 = time.time()
    env = robot.GetEnv()
    lower, upper = robot.GetAffineTranslationLimits()
    filtered_voxels = []
    for extents, centers in occupancy.voxels:
        if not centers:
            continue
        box = cell_body(env, np.zeros(3), extents, name='cell')
        env.Add(box)
        base_from_voxels = trans_from_pose(occupancy.pose)
        voxel_radius = length(np.array(extents)/2)
        voxel_lower2d = lower[:2] - (ROBOT_RADIUS+ voxel_radius)*np.ones(2)
        voxel_upper2d = upper[:2] + (ROBOT_RADIUS + voxel_radius)*np.ones(2)
        for center in centers:
            base_from_center = base_from_voxels.dot(trans_from_point(*center))
            center_base2d = point_from_trans(base_from_center)[:2]
            if np.any(center_base2d < voxel_lower2d) or np.any(voxel_upper2d < center_base2d):
              continue
            if FILTER_OCTREE:
              box.SetTransform(base_from_center)
              if env.CheckCollision(box):
                continue          
            filtered_voxels.append((center, extents))
        env.Remove(box)
    print len(filtered_voxels), 'occupancy voxels in', (time.time() - t0), 'seconds'
    # TODO: do a tolerance based check of this
    if not filtered_voxels:
        return
    octree = octree_body(env, filtered_voxels, name='octree',
                         color=RED, transparency=0.5, draw=DRAW_OCTREE)
    env.Add(octree)
    set_pose(octree, occupancy.pose)
    #env.UpdatePublishedBodies()
    #raw_input('awefawef')

"""
def add_octree(robot, occupancy):
    t0 = time.time()
    env = robot.GetEnv()
    lower, upper = robot.GetAffineTranslationLimits()
    filtered_bodies = []
    for i, (extents, centers) in enumerate(occupancy.voxels):
        if not centers:
            continue
        base_from_voxels = trans_from_pose(occupancy.pose)
        voxel_radius = length(np.array(extents)/2)
        voxel_lower2d = lower[:2] - (ROBOT_RADIUS+ voxel_radius)*np.ones(2)
        voxel_upper2d = upper[:2] + (ROBOT_RADIUS + voxel_radius)*np.ones(2)
        for j, center in enumerate(centers):
            box = cell_body(env, np.zeros(3), extents, name='cell-{:d}-{:d}'.format(i, j), color=RED, transparency=0.5, draw=DRAW_OCTREE)
            base_from_center = base_from_voxels.dot(trans_from_point(*center))
            center_base2d = point_from_trans(base_from_center)[:2]
            if np.any(center_base2d < voxel_lower2d) or np.any(voxel_upper2d < center_base2d):
              continue
            box.SetTransform(base_from_center)
            if FILTER_OCTREE:
              if env.CheckCollision(box):
                continue
            env.Add(box)
            filtered_bodies.append(box)
    print len(filtered_bodies), 'occupancy voxels in', (time.time() - t0), 'seconds'
    env.UpdatePublishedBodies()
    raw_input('awefawef')
"""

def populate_env(robot, belief, object_meshes, occupancy):
    env = robot.GetEnv()
    remove_except(robot)  # env.Reset()

    names_from_type = defaultdict(list)
    def add_type(cl):
        name = '{}{}'.format(cl, len(names_from_type[cl]))
        names_from_type[cl].append(name)
        return name

    belief_from_name = {}
    if belief.holding is not None:
        ty, grasp, _ = belief.holding
        name = add_type(ty)
        belief_from_name[name] = belief.holding
        add_holding(robot, object_meshes, name, belief.holding)
    for surface in belief.surfaces:
        name = add_type(surface.type)
        belief_from_name[name] = surface
        if is_oriented(surface.pose):
            add_surface(env, name, surface)
    for item in belief.items:
        name = add_type(item.type)
        belief_from_name[name] = item
        if is_oriented(item.pose):
            add_item(env, object_meshes, name, item)
    add_octree(robot, occupancy)
    return belief_from_name

##################################################

BCONF, LCONF, HCONF = Type(), Type(), Type()
LTRAJ = Type()
SURFACE, ITEM = Type(), Type()
POSE, GRASP = Type(), Type()
CLASS, COLOR = Type(), Type()

# TODO: just name robot parts?
AtBConf = Pred(BCONF)
AtHConf = Pred(HCONF)
AtLConf = Pred(LCONF)
HandEmpty = Pred()
HasGrasp = Pred(ITEM, GRASP)
OpenLHand = Pred()

IsLeft = Pred(SURFACE)
IsRight = Pred(SURFACE)
IsFront = Pred(SURFACE)
IsBack = Pred(SURFACE)

AtPose = Pred(ITEM, POSE)
Supported = Pred(POSE, SURFACE)  # TODO: unify with IsSupported
Localized = Pred(OBJECT)
Measured = Pred(OBJECT)
Executable = Pred()
Registered = Pred(ITEM)
#Oriented = Pred(ITEM)
Oriented = Pred(POSE)
# TODO: refactor Localized/Oriented into position / orientation

IsSurface = Pred(CLASS)
IsItem = Pred(CLASS)
IsClass = Pred(OBJECT, CLASS)
# IsColor = Pred(OBJECT, CLASS)

IsKin = Pred(POSE, GRASP, BCONF, LTRAJ)
#IsPush = Pred(POSE, POSE, BCONF, LTRAJ)
IsVisible = Pred(SURFACE, HCONF, BCONF)
IsVisiblePose = Pred(POSE, HCONF, BCONF)
IsSupported = Pred(POSE, SURFACE)  # Static
IsPose = Pred(CLASS, POSE)
IsGrasp = Pred(CLASS, GRASP)
IsReachable = Pred(BCONF)

ScanRoom = Func(CLASS)
# TODO: could include more specific vantage point costs
ScanTable = Func(SURFACE, CLASS)
Distance = Func(BCONF, BCONF)

Pushed = Pred(ITEM)
Holding = Pred(ITEM)
On = Pred(ITEM, SURFACE)
RobotNear = Pred(SURFACE)
SurfaceDist = Pred(SURFACE, SURFACE)

ComputableP = Pred(POSE)
OrientableP = Pred(POSE)
ComputableBQ = Pred(BCONF)
ComputableHQ = Pred(HCONF)

# TODO: abstract values for everything that isn't immediately performable

LT = Param(LTRAJ)
C = Param(CLASS)
S, S2 = Param(SURFACE), Param(SURFACE)
I, I2 = Param(ITEM), Param(ITEM)
P, P2 = Param(POSE), Param(POSE)
G = Param(GRASP)

# Free parameters
BQ, BQ2 = Param(BCONF), Param(BCONF)
HQ, HQ2 = Param(HCONF), Param(HCONF)
LQ, LQ2 = Param(LCONF), Param(LCONF)

rename_easy(locals())  # Trick to make debugging easier

# TODO: could just do easier version of this that doesn't require localized to start
# TODO: what if the arms are in the way?
# Is it worthwhile to separate the base and head
# TODO: easier way to do this involving a large transition function?

actions = [
    Action(name='pick', parameters=[I, P, G, BQ, LT],  # TODO: Visibility constraint
           condition=And(OpenLHand(), Registered(I), AtPose(I, P), HandEmpty(), AtBConf(BQ),
                         IsKin(P, G, BQ, LT)),
           effect=And(Not(OpenLHand()), HasGrasp(I, G), Executable(),
                      Not(AtPose(I, P)), Not(HandEmpty())), cost=PICK_PLACE_COST+1),
    Action(name='push', parameters=[I, P, G, BQ, LT],  # TODO: Visibility constraint
           condition=And(OpenLHand(), Registered(I), AtPose(I, P), HandEmpty(), AtBConf(BQ),
                         IsKin(P, G, BQ, LT)),
           effect=And(Not(OpenLHand()), HasGrasp(I, G), Executable(),
                      Not(AtPose(I, P)), Not(HandEmpty())), cost=PICK_PLACE_COST),
    Action(name='place', parameters=[I, P, G, BQ, LT],
           condition=And(Not(OpenLHand()), HasGrasp(I, G), AtBConf(BQ),
                         IsKin(P, G, BQ, LT)),
           effect=And(OpenLHand(), AtPose(I, P), HandEmpty(), Executable(),
                      Not(HasGrasp(I, G))), cost=PICK_PLACE_COST),

    Action(name='open_gripper', parameters=[],
           condition=Not(OpenLHand()),
           effect=OpenLHand()),
    #Action(name='close_gripper', parameters=[],
    #       condition=OpenLHand(),
    #       effect=Not(OpenLHand())),
    #Action(name='move_left', parameters=[LQ, LQ2],
    #       condition=And(AtLConf(LQ)), #, ComputableHQ(HQ2)),
    #       effect=And(AtLConf(LQ2), Not(AtLConf(LQ)))),

    # TODO: a choose surface action that decides which surface to produce real values for (if nearby)
    # TODO: executable precondition for abstract actions

    # TODO: number of move_head actions becomes quite large
    #Action(name='move_head', parameters=[HQ, HQ2],
    #       condition=And(AtHConf(HQ), ComputableHQ(HQ2)),
    #       effect=And(AtHConf(HQ2), Executable(),
    #                  Not(AtHConf(HQ))), cost=1),
    #Action(name='move_head', parameters=[HQ, HQ2],
    #       condition=ComputableHQ(HQ2),
    #       effect=And(AtHConf(HQ2), Executable(),
    #                  When(AtHConf(HQ), Not(AtHConf(HQ)))), cost=1),
    Action(name='move_head', parameters=[HQ2],
           condition=ComputableHQ(HQ2),
           effect=And(AtHConf(HQ2), Executable(),
                      #ForAll([HQ], Not(AtHConf(HQ)))), cost=1),
                      ForAll([HQ], When(AtHConf(HQ), Not(AtHConf(HQ))))), cost=1),

    Action(name='move_base', parameters=[BQ, BQ2],  # TODO: don't need trajectory to start if I only am only using one anyways
           condition=And(AtBConf(BQ), IsReachable(BQ), IsReachable(BQ2), ComputableBQ(BQ2)),
           effect=And(AtBConf(BQ2), Executable(), Not(AtBConf(BQ)),
                      ForAll([I], Not(Registered(I))), # TODO: do I want to include this? It just makes my life harder...
                      Cost(Distance(BQ, BQ2)))),

    Action(name='register', parameters=[I, P, HQ, BQ], # TODO: preconditions that you can see
           condition=And(Localized(I), AtPose(I, P), AtHConf(HQ), AtBConf(BQ),
                         IsVisiblePose(P, HQ, BQ)), # TODO: distance requirement? and IsPose(C,P)?
           effect=And(Oriented(P), Registered(I))),

    Action(name='scan_room', parameters=[S, C],  # TODO: later add trajectory
           condition=And(Not(Localized(S)),
                         IsClass(S, C)),
           effect=And(Localized(S), Executable(),
                      Cost(ScanRoom(C)))),

    Action(name='scan_table', parameters=[S, I, C, P, HQ, BQ],
           condition=And(Localized(S), Not(Localized(I)), AtPose(I, P), AtHConf(HQ), AtBConf(BQ),
                         IsClass(I, C), IsPose(C, P), IsVisible(S, HQ, BQ)),
           effect=And(Localized(I), Measured(P), Supported(P, S), Executable(),
                      Cost(ScanTable(S, C)))),
]
# TODO: could view this as replacements for intentionally planning on producing values
# TODO: I guess I could also modify the AtPose values upon getting an observation as well
# TODO: should I restrict that these are only used for the appropriate future types?
axioms = [
    # TODO: could produce static predicates for these to avoid quantification
    Axiom(effect=ComputableBQ(BQ),
          condition=Or(Measured(BQ),
                       Exists([P, G, LT], And(IsKin(P, G, BQ, LT), OrientableP(P))),
                       Exists([S, HQ], And(IsVisible(S, HQ, BQ), Localized(S))),
                       Exists([P, HQ], And(IsVisiblePose(P, HQ, BQ), ComputableP(P))))),
    Axiom(effect=ComputableHQ(HQ),
          condition=Or(Measured(HQ),
                       Exists([S, BQ], And(IsVisible(S, HQ, BQ), Localized(S))), # Measured(BQ)?
                       Exists([P, BQ], And(IsVisiblePose(P, HQ, BQ), ComputableP(P))))),
    Axiom(effect=ComputableP(P),
          condition=Or(Measured(P),
                       Exists([S], And(IsSupported(P, S), Localized(S))))),
    Axiom(effect=OrientableP(P),
          condition=Or(Oriented(P),
                       Exists([S], And(IsSupported(P, S), Localized(S))))),
    Axiom(effect=Holding(I),
          condition=Exists([C, G], And(IsClass(I, C), IsGrasp(C, G), HasGrasp(I, G)))),
    Axiom(effect=On(I, S),
          condition=Exists([C, P], And(IsClass(I, C), IsPose(C, P), AtPose(I, P),
                                       Or(Supported(P, S), IsSupported(P, S)))))
]

# TODO: should move these inside the function so their counts reset

def get_location_atoms(belief, belief_from_name, initial_tform):
    #initial_tform = belief.world_from_reference
    # TODO: use this again?
    for name, b in belief_from_name.items():
        if isinstance(b, Surface) and (b.pose is not None):
            # TODO: belief version requires anticipating this
            reference_from_table = np.linalg.inv(initial_tform).dot(trans_from_pose(b.pose))
            x, y, _ = point_from_pose(pose_from_trans(reference_from_table))
            if x <= 0:
                yield IsBack(name)
            if 0 < x:
                yield IsFront(name)
            if y <= 0:
                yield IsRight(name)
            if 0 < y:
                yield IsLeft(name)

def get_abstract_problem(arm, goal_formula, belief, belief_from_name, initial_tform):
    # TODO: could also do hierarchy one above where we just have to pick and place objects w/o uncertainty

    robot = arm.GetRobot()
    actions = [
        Action(name='pick', parameters=[I, S],
               condition=And(On(I, S), Registered(I)), # RobotNear(S),
               effect=And(Holding(I), Not(On(I, S))),
               cost=PICK_PLACE_COST),
        Action(name='place', parameters=[I, S],
               condition=And(Holding(I)), # RobotNear(S)),
               effect=And(On(I, S), Not(Holding(I))),
               cost=PICK_PLACE_COST),
        # Action(name='push', parameters=[I, S],
        #        condition=And(On(I, S), Registered(I)), # RobotNear(S),
        #        effect=And(Holding(I), Not(On(I, S))),
        #        cost=PICK_PLACE_COST-1), 
        #Action(name='move_base', parameters=[S, S2],
        #       condition=RobotNear(S),
        #       effect=And(RobotNear(S2), Not(RobotNear(S)),
        #                  Cost(SurfaceDist(S, S2)))),
        Action(name='register', parameters=[I, S],
               condition=And(Localized(I), On(I, S)),
               effect=And(Registered(I))),
        Action(name='scan_table', parameters=[S, I, C],
           condition=And(Localized(S), Not(Localized(I)), IsClass(I, C)), # RobotNear(S),
           effect=And(Localized(I), On(I, S), Cost(ScanTable(S, C)))),
        Action(name='scan_room', parameters=[S, C],
               condition=And(Not(Localized(S)), IsClass(S, C)),
               effect=And(Localized(S), Cost(ScanRoom(C)))),
    ]
    # TODO: None surface for when starts off

    #bq0 = get_conf(robot, 'base')

    # TODO: state component for what we are acting with

    initial_atoms = []
    for name, b in belief_from_name.items():
        ENTITY = SURFACE if isinstance(b, Surface) else ITEM
        initial_atoms += [IsClass(ENTITY(name), b.type)]
        if b.pose is not None:
            initial_atoms += [Localized(ENTITY(name))]
        if isinstance(b, Surface):
            initial_atoms += [RobotNear(name), IsSurface(b.type)]
        elif isinstance(b, Object):
            initial_atoms += [IsItem(b.type)]
            if isinstance(b.pose, Grasp):
                initial_atoms += [Holding(name)]
            elif b.pose is not None:
                if b.registered:
                    initial_atoms += [Registered(name)]
                for surface, b2 in belief_from_name.items():
                    if isinstance(b2, Surface) and (b2.pose is not None) and \
                            is_point_on_surface(b2, extract_point(b.pose)):
                        initial_atoms += [On(name, surface)]
        else:
            raise ValueError(b)
    initial_atoms += list(get_location_atoms(belief, belief_from_name, initial_tform))

    def surface_dist(s1, s2):
        b1, b2 = belief_from_name[s1], belief_from_name[s2]
        if (b1.pose is None) or (b2.pose is None):
            return MAX_DISTANCE
        p1, p2 = point_from_pose(b1.pose), point_from_pose(b2.pose)
        return length((p2 - p1)[:2])

    cond_streams = [
        #CostStream(inputs=[S, S2], conditions=[], effects=[SurfaceDist(S, S2)],
        #           function=surface_dist),
        CostStream(inputs=[S, C], conditions=[IsItem(C)], effects=[ScanTable(S, C)],
                   function=lambda s, c: 10*belief_from_name[s].observations),
        CostStream(inputs=[C], conditions=[IsSurface(C)], effects=[ScanRoom(C)],
                   function=lambda c: 1),
    ]

    return STRIPStreamProblem(
        initial_atoms, goal_formula, actions, cond_streams, [])


def get_ground_problem(arm, object_meshes, belief, goal_formula,
                       belief_from_name, initial_tform, available_actions=None, abstract_plan=None):
    if available_actions is None:
      available_actions = [action.name for action in actions]

    robot = arm.GetRobot()
    env = robot.GetEnv()
    kinect = robot.GetLink('head_mount_kinect_rgb_optical_frame')
    #if kinect is None:
    #    kinect = robot.GetManipulator('head')
    movable_bodies = [body for body in env.GetBodies()
                      if isinstance(belief_from_name.get(body.GetName(), None), Object)]
    for body in movable_bodies:
        body.Enable(False)
    surface_types = {b.type for b in belief_from_name.values() if isinstance(b, Surface)}
    item_types = {b.type for b in belief_from_name.values() if isinstance(b, Object)}
    ir_database = load_custom_ir(robot, grasp_approach=GRASP_APPROACHES.TOP)

    ####################

    bq_initial = Conf(base_values_from_trans(initial_tform))
    world_from_robot0 = robot.GetTransform()
    bq0 = get_conf(robot, 'base')
    hq0 = get_conf(robot, 'head')
    lq0 = get_conf(robot, arm.GetName())
    initial_atoms = [
        AtBConf(bq0), Measured(BCONF(bq0)),
        AtHConf(hq0), Measured(HCONF(hq0)),
        AtLConf(lq0), #Measured(LCONF(lq0)),
    ]
    #assert is_gripper_open(arm) != is_gripper_closed(arm) # Holding but not closed
    if is_gripper_open(arm):
        initial_atoms.append(OpenLHand())
    if belief.holding is None:
        initial_atoms.append(HandEmpty())
    initial_atoms += [IsSurface(ty) for ty in surface_types] + \
                     [IsItem(ty) for ty in item_types]

    known_poses = {}
    for name, b in belief_from_name.items():
        ty = b.type
        ENTITY = SURFACE if ty in surface_types else ITEM
        initial_atoms.append(IsClass(ENTITY(name), ty))
        if (b.pose is not None) and not isinstance(b.pose, Grasp):
            initial_atoms.append(Localized(ENTITY(name)))
            known_poses[name] = Pose(b.pose, ty)
        if ENTITY == ITEM:
            if isinstance(b.pose, Grasp):
                initial_atoms += [HasGrasp(name, b.pose),
                                  IsGrasp(ty, b.pose), Localized(ITEM(name))]
            else:
                if name in known_poses:
                    pose = known_poses[name]
                    initial_atoms.append(Measured(POSE(pose)))
                    if is_oriented(pose.value):
                        initial_atoms.append(Oriented(pose))
                else:
                    pose = Future(name) # TODO: prevent placing at this pose?
                initial_atoms += [AtPose(name, pose), IsPose(ty, pose)]
                if b.registered:
                    initial_atoms.append(Registered(name))
    initial_atoms += list(get_location_atoms(belief, belief_from_name, initial_tform))

    ####################

    ode_checker = RaveCreateCollisionChecker(env, ODE)
    def sample_visible_head_from_pose(target_point, base_q):
        # TODO: could base this on the bounding box of the object in question as well
        with robot:
            #base_q.value[2] += 0.2
            set_conf(robot, 'base', base_q.value)
            head_q = look_at_point(robot, target_point, kinect)
            if head_q is None:
                return
            set_conf(robot, 'head', head_q)
            head_point = point_from_pose(get_pose(kinect))
            #handles = [env.drawarrow(head_point, target_point, linewidth=0.01, color=np.array([0, 0, 0, 1]))]
            #print head_q, env.CheckCollision(robot)
            #env.UpdatePublishedBodies()
            #raw_input('awefawef')
            if env.CheckCollision(robot):
                return
            ray = head_point - target_point # TODO: bodyexcluded and linkexcluded
            head_point = target_point + normalize(ray)*max(0, length(ray) - 0.05) # TODO: exclude head from collision checking?

            #results, contacts = ode_checker.CheckCollisionRays(np.array([np.append(head_point, target_point - head_point)]), robot)
            #result = np.any(results)

            world_from_head = kinect.GetTransform()
            target_head = np.linalg.inv(world_from_head).dot(np.append(target_point, 1))[:3]
            #print target_head
            targets_world = []
            discr = np.linspace(-0.1, 0.1, 3) # TODO: could randomly generate
            for dx, dy in product(discr, discr):
                new_target_head = target_head + np.array([dx, dy, 0])
                targets_world.append(world_from_head.dot(np.append(new_target_head, 1))[:3])

            rays = [Ray(head_point, t - head_point) for t in targets_world]
            #rays = [Ray(head_point, target_point - head_point)]
            if any(ode_checker.CheckCollision(r, robot) for r in rays): # TODO: reorder this check
                #handles = [env.drawarrow(head_point, t, linewidth=0.01,
                #                         color=np.array([0, 0, 0, 1])) for t in targets_world]
                #env.UpdatePublishedBodies()
                #raw_input('Collision!')
                return
        yield (Conf(head_q),)

    def sample_reg_head(pose, base_q):
        if Future in (type(pose), type(base_q)):
            # TODO: share futures for these?
            head_q = Future(pose, base_q)
            yield (head_q,)
            return
        # TODO: need to reorder these...

        #if abstract_plan:
        #    action, args = abstract_plan[0]
        #    if action.name == 'register':
        #        s, i = args
        #        if pose.type != belief_from_name[i].type: return
        #    else: return

        target_point = extract_point(pose.value)
        if MAX_REG_DISTANCE < length(base_q.value[:2] - target_point[:2]): # TODO: this is a 2D distance...
            return
        for outputs in sample_visible_head_from_pose(target_point, base_q):
            yield outputs


    def sample_visible_head(surface, base_q):
        #if abstract_plan:
        #    action, args = abstract_plan[0]
        #    if action.name == 'scan_table':
        #        s, i, c = args
        #        if surface != s: return
        #    else: return

        if (surface not in known_poses) or (type(base_q) is Future):
            return
        target_point = point_from_pose(known_poses[surface].value)
        if MAX_LOOK_DISTANCE < length(base_q.value[:2] - target_point[:2]): # TODO: this is a 2D distance...
            return
        for outputs in sample_visible_head_from_pose(target_point, base_q):
            yield outputs

    # TODO: it still finds configs outside of the environment
    # TODO: could generically do this with poses

    ####################

    def head_base_from_pose(target_point, base_from_table, distance):
        look_distance = random.uniform(distance/2, distance)
        base_xy = target_point[:2] - look_distance * base_from_table
        base_theta = math.atan2(base_from_table[1], base_from_table[0])
        base_q = Conf(np.append(base_xy, base_theta))
        if not base_within_limits(robot, base_q.value):
          return
        with robot:
            if env.CheckCollision(robot):
                return
        for head_q, in sample_visible_head_from_pose(target_point, base_q):
            yield (head_q, base_q)

    def sample_base_and_head(target_point, distance, bias_initial=False, max_attempts=100, max_bqs=15):
        num_bqs = 0
        if bias_initial:
            base_from_table = normalize(target_point[:2] - bq0.value[:2])
            for outputs in head_base_from_pose(target_point, base_from_table, distance):
                yield outputs
                num_bqs += 1
        for _ in xrange(max_attempts):
            if max_bqs <= num_bqs:
              break
            base_from_table = unit_from_theta(random.uniform(0, 2 * math.pi))
            for outputs in head_base_from_pose(target_point, base_from_table, distance):
                yield outputs
                num_bqs += 1

    def sample_visible_base(surface):
        #if abstract_plan:
        #    action, args = abstract_plan[0]
        #    if action.name == 'scan_table':
        #        s, i, c = args
        #        if surface != s:  return
        #    else: return

        if surface not in known_poses: # TODO: combine these...
            bq = Future(surface)
            hq = Future(bq)
            yield (hq, bq)
        else:
            target_point = point_from_pose(known_poses[surface].value)
            for outputs in sample_base_and_head(target_point, MAX_LOOK_DISTANCE):
                yield outputs

    def sample_reg_base(pose):
        #if abstract_plan:
        #    action, args = abstract_plan[0]
        #    if action.name == 'register':
        #        s, i = args
        #        if surface != s: return
        #    else: return

        if isinstance(pose, Future):
            bq = Future(pose)
            hq = Future(bq)
            yield (hq, bq)
        else:
            target_point = extract_point(known_poses[surface].value)
            for outputs in sample_base_and_head(target_point, MAX_REG_DISTANCE):
                yield outputs

    ####################

    def sample_table(surface, ty, max_poses=5):
        if abstract_plan:
            action, args = abstract_plan[0]
            if action.name == 'place':
                i, s = args
                if (surface != s) or (belief_from_name[i].type != ty): return
            else: return

        if surface not in known_poses:
            p = Future(surface, ty)
            yield (p,)
        else:
            for pose in islice(sample_surface_pose(belief_from_name[surface], object_meshes[ty]), max_poses):
                yield (Pose(pose, ty),)

    def test_supported(surface, pose): # TODO: instead just do this initially
        if (surface not in known_poses) or (type(pose) is Future):
            return False
        return is_surface_pose(belief_from_name[surface], object_meshes[pose.type], pose.value)

    for surface, b in belief_from_name.items():
        if not isinstance(b, Surface):
            continue
        for pose in known_poses.values():
            if (pose.type in item_types) and is_oriented(pose.value) and test_supported(surface, pose):
                initial_atoms.append(IsSupported(pose, surface))

    ####################

    def grasp_generator(ty, max_grasps=1):
        if abstract_plan:
            action, args = abstract_plan[0]
            if action.name == 'pick':
                i, s = args
                if belief_from_name[i].type != ty: return
            else: return

        mesh = object_meshes[ty]
        if ty in TOP_GRASPS:
            for i, (g, gq) in enumerate(islice(get_top_grasps(mesh), max_grasps)):
                grasp = Grasp(g)
                grasp.name = 'gtop{}'.format(i)
                grasp.type = ty
                grasp.gripper_q = gq
                yield (grasp,)
        if ty in SIDE_GRASPS:
            for i, (g, gq) in enumerate(get_side_grasps(mesh, under=False)):
                grasp = Grasp(g)
                grasp.name = 'gside{}'.format(i)
                grasp.type = ty
                grasp.gripper_q = gq
                yield (grasp,)

    def prepush_generator(ty, max_grasps=1):
      ################## write
        if abstract_plan:
            action, args = abstract_plan[0]
            if action.name == 'pick':
                i, s = args
                if belief_from_name[i].type != ty: return
            else: return

        mesh = object_meshes[ty]
        if ty in TOP_GRASPS:
            for i, (g, gq) in enumerate(islice(get_prepush_setting(mesh), max_grasps)):
                grasp = Grasp(g)
                grasp.name = 'gtop{}'.format(i)
                grasp.type = ty
                grasp.gripper_q = gq
                yield (grasp,)

    def manip_from_pose_grasp(pose, grasp):
        world_from_obj = trans_from_pose(pose.value)
        obj_from_gripper = np.linalg.inv(grasp.value)
        return world_from_obj.dot(obj_from_gripper)


    def manip_traj_from_base(_, pose, grasp, base_q=bq0):
        if Future in (type(pose), type(base_q)):
            # Don't expect to be able to take arbitrary pose and base_q strings
            return
        target_point = point_from_pose(pose.value) if is_oriented(pose.value) else pose.value
        if MAX_KIN_DISTANCE < length(base_q.value[:2] - target_point[:2]):
            return
        if not is_oriented(pose.value):
            lt = Future(pose, grasp, base_q)
            yield (lt,)
            return
        world_from_gripper = manip_from_pose_grasp(pose, grasp)
        gripper_from_approach = trans_from_point(
            *(-APPROACH_LENGTH) * arm.GetLocalToolDirection())
        world_from_approach = world_from_gripper.dot(gripper_from_approach)
        # TODO: remove poses of everything

        # TODO: ensure that are grasps are tried by streams and streams don't end prematurely
        # TODO: it seems like only one grasp is being tried...
        #raw_input('awefawef')

        cspace = CSpace.robot_manipulator(robot, arm.GetName())
        with robot:
            set_conf(robot, arm.GetName(), DEFAULT_LEFT_ARM)
            if base_q == bq0:
              # We make some ugly orientation approximations to be 2d pose
              robot.SetTransform(world_from_robot0)
            else:
              set_conf(robot, 'base', base_q.value)
            #env.UpdatePublishedBodies()
            #raw_input('Continue?')
            open_gripper(arm)
            if env.CheckCollision(robot):
                return
            grasp_manip_q = solve_inverse_kinematics(arm, world_from_gripper)
            if grasp_manip_q is None:
                return
            set_manipulator_conf(arm, grasp_manip_q)
            approach_manip_q = solve_inverse_kinematics(
                arm, world_from_approach)
            if approach_manip_q is None:
                return

            cspace.set_active()
            # TODO: workspace plan here
            grasp_path = mp_straight_line(
                robot, grasp_manip_q, approach_manip_q)
            if grasp_path is None:
                return
            approach_path = mp_birrt(
                robot, approach_manip_q, DEFAULT_LEFT_ARM)
            if approach_path is None:
                return
        lt = Manipulation(PathTrajectory(cspace, grasp_path), PathTrajectory(cspace, approach_path))
        yield (lt,)

    def manip_traj_from_base_push(_, pose, grasp, base_q=bq0):
        if Future in (type(pose), type(base_q)):
            # Don't expect to be able to take arbitrary pose and base_q strings
            return
        target_point = point_from_pose(pose.value) if is_oriented(pose.value) else pose.value
        if MAX_KIN_DISTANCE < length(base_q.value[:2] - target_point[:2]):
            return
        if not is_oriented(pose.value):
            lt = Future(pose, grasp, base_q)
            yield (lt,)
            return
        world_from_gripper = manip_from_pose_grasp(pose, grasp)
        gripper_from_approach = trans_from_point(
            *(-APPROACH_LENGTH) * arm.GetLocalToolDirection())
        world_from_approach = world_from_gripper.dot(gripper_from_approach)

        final_from_approach = trans_from_point(
            *(APPROACH_LENGTH) * arm.GetLocalToolDirection())
        world_from_final = world_from_gripper.dot(final_from_approach)


        # TODO: remove poses of everything

        # TODO: ensure that are grasps are tried by streams and streams don't end prematurely
        # TODO: it seems like only one grasp is being tried...
        #raw_input('awefawef')

        cspace = CSpace.robot_manipulator(robot, arm.GetName())
        with robot:
            set_conf(robot, arm.GetName(), DEFAULT_LEFT_ARM)
            if base_q == bq0:
              # We make some ugly orientation approximations to be 2d pose
              robot.SetTransform(world_from_robot0)
            else:
              set_conf(robot, 'base', base_q.value)
            #env.UpdatePublishedBodies()
            #raw_input('Continue?')
            open_gripper(arm)
            if env.CheckCollision(robot):
                return
            grasp_manip_q = solve_inverse_kinematics(arm, world_from_gripper)
            if grasp_manip_q is None:
                return
            set_manipulator_conf(arm, grasp_manip_q)
            approach_manip_q = solve_inverse_kinematics(
                arm, world_from_approach)
            if approach_manip_q is None:
                return
            final_manip_q = solve_inverse_kinematics(
                arm, world_from_final)
            if final_manip_q is None:
                return

            cspace.set_active()
            # TODO: workspace plan here
            grasp_path = mp_straight_line(
                robot, grasp_manip_q, approach_manip_q)
            if grasp_path is None:
                return
            approach_path = mp_birrt(
                robot, approach_manip_q, DEFAULT_LEFT_ARM)
            if approach_path is None:
                return

            push_path = mp_straight_line(
                robot, final_manip_q, approach_manip_q)
            if push_path is None:
                return


        lt = Manipulation2(PathTrajectory(cspace, grasp_path), PathTrajectory(cspace, approach_path), PathTrajectory(cspace, push_path))
        yield (lt,)

    def manip_traj_from_pose_grasp(ty, pose, grasp, max_attempts=100, max_bqs=10):
        if (type(pose) is Future) or not is_oriented(pose.value):
            bq = Future(pose, grasp)
            lt = Future(pose, grasp, bq)
            yield (bq, lt)
            return

        base_iterator = get_base_generator(robot, ir_database, [manip_from_pose_grasp(pose, grasp)])
        num_bqs = 0
        for i in xrange(max_attempts):
            if max_bqs <= num_bqs:
                break
            if False:
                # target_point = point_from_pose(manip_from_pose_grasp(pose, grasp).value)
                #base_from_item = normalize(target_point[:2] - bq0.value[:2])
                target_point = point_from_pose(pose.value)
                base_from_item = unit_from_theta(random.uniform(0, 2 * math.pi))
                base_xy = target_point[:2] - random.uniform(0, MAX_KIN_DISTANCE) * base_from_item
                # base_theta = math.atan2(base_from_item[1], base_from_item[0])
                # TODO: Gaussians around these values
                base_theta = random.uniform(0, 2 * math.pi)
                base_q = Conf(np.append(base_xy, base_theta))
            else:
                # TODO: not all of them might be visible
                try:
                  base_trans, _ = next(base_iterator)
                except StopIteration:
                  break
                base_q = Conf(base_values_from_trans(base_trans))
            if not base_within_limits(robot, base_q.value):
                continue
            #if not test_reachable(base_q):
            #    continue
            for manip_t, in manip_traj_from_base(ty, pose, grasp, base_q):
                yield (base_q, manip_t)
                num_bqs += 1

    def manip_traj_from_grasp_base(surface, ty, grasp, max_poses=50):
        if abstract_plan:
            action, args = abstract_plan[0]
            if action.name == 'place':
                i, s = args
                if (surface != s) or (belief_from_name[i].type != ty): return
            else: return

        if surface not in known_poses:
            return
        for pose in islice(sample_surface_pose(belief_from_name[surface], object_meshes[ty]), max_poses):
            pose = Pose(pose, ty)
            for manip_t, in manip_traj_from_base(ty, pose, grasp, bq0):
                yield (pose, manip_t)
                return

    def test_reachable(bq):
        #return True
        if (type(bq) == Future) or (bq == bq0):
            return True
        cspace = CSpace.robot_manipulator(robot, 'base')
        with robot:
            cspace.set_active()
            path = mp_straight_line(
                robot, bq_initial.value, bq.value)
            #if path is None:
            #    print path if path is None else len(path)
        return path is not None

    #distance_fn = get_distance_fn(robot)
    def distance_cost(bq1, bq2):
        if Future in (type(bq1), type(bq2)):
            return 2 * MAX_DISTANCE
        # return distance_fn(bq1.value, bq2.value)
        return 1 + length((bq1.value - bq2.value)[:2])

    ####################

    cond_streams = []
    # if 'pick' in available_actions:
    #   cond_streams += [
    #       GeneratorStream(inputs=[C], outputs=[G], conditions=[IsItem(C)],
    #                       effects=[IsGrasp(C, G)], generator=grasp_generator),

    #       GeneratorStream(inputs=[C, P, G], outputs=[LT],
    #                       conditions=[IsPose(C, P), IsGrasp(C, G)],
    #                       effects=[IsKin(P, G, bq0, LT)], generator=manip_traj_from_base),
    #   ]
    if 'push' in available_actions:
      cond_streams += [
          GeneratorStream(inputs=[C], outputs=[G], conditions=[IsItem(C)],
                          effects=[IsGrasp(C, G)], generator=prepush_generator),

          GeneratorStream(inputs=[C, P, G], outputs=[LT],
                          conditions=[IsPose(C, P), IsGrasp(C, G)],
                          effects=[IsKin(P, G, bq0, LT)], generator=manip_traj_from_base_push),
      ]
    if 'place' in available_actions:
      cond_streams += [
        GeneratorStream(inputs=[S, C], outputs=[P], conditions=[IsItem(C)],
                        effects=[IsSupported(P, S), IsPose(C, P)], generator=sample_table),
        GeneratorStream(inputs=[S, C, G], outputs=[P, LT],
                      conditions=[IsItem(C), IsGrasp(C, G)],
                      effects=[IsKin(P, G, bq0, LT), IsSupported(P, S), IsPose(C, P)],
                      generator=manip_traj_from_grasp_base),
      ]
    if 'scan_room' in available_actions:
      cond_streams += [
        CostStream(inputs=[C], conditions=[IsSurface(C)], effects=[ScanRoom(C)],
                   function=lambda c: 1),
      ]
    if 'scan_table' in available_actions:
      cond_streams += [
        GeneratorStream(inputs=[S], outputs=[HQ, BQ], conditions=[],
                        effects=[IsVisible(S, HQ, BQ)], generator=sample_visible_base),
        # TODO: seems to be a bug in static preconditions with objects
        GeneratorStream(inputs=[S, BQ], outputs=[HQ], conditions=[],
                        effects=[IsVisible(S, HQ, BQ)], generator=sample_visible_head),  # TODO: function stream?
        CostStream(inputs=[S, C], conditions=[IsItem(C)], effects=[ScanTable(S, C)],
                   function=lambda s, c: 10*belief_from_name[s].observations),
      ]
    if 'register' in available_actions:
      # TODO: register sometimes uses samples from things that aren't observable...
      cond_streams += [
        GeneratorStream(inputs=[P, BQ], outputs=[HQ], conditions=[], #conditions=[IsInitialBQ(BQ)],
                        effects=[IsVisiblePose(P, HQ, BQ)], generator=sample_reg_head),
        #GeneratorStream(inputs=[P], outputs=[HQ, BQ], conditions=[],
        #                effects=[IsVisiblePose(P, HQ, BQ)], generator=sample_reg_base),
        ]
    if 'move_base' in available_actions:
      cond_streams += [
        TestStream(inputs=[BQ], conditions=[],
                   effects=[IsReachable(BQ)], test=test_reachable, eager=True),
        GeneratorStream(inputs=[C, P, G], outputs=[BQ, LT], conditions=[IsPose(C, P), IsGrasp(C, G)],
                        effects=[IsKin(P, G, BQ, LT)], generator=manip_traj_from_pose_grasp),
        CostStream(inputs=[BQ, BQ2], conditions=[], effects=[Distance(BQ, BQ2), Distance(BQ2, BQ)],
                   function=distance_cost),
      ]

    operators = filter(lambda a: a.name in available_actions, actions) + axioms

    stream_problem = STRIPStreamProblem(
        initial_atoms, goal_formula, operators, cond_streams, [])

    return stream_problem

##################################################

def get_goal_formula(task):
    goal_literals = []
    if task.holding is None:
        goal_literals.append(HandEmpty())
    elif task.holding is not False:
        goal_literals.append(
            Exists([I], And(IsClass(I, task.holding), Holding(I))))
    elif task.pushed is not False:
        goal_literals.append(
            Exists([I], And(IsClass(I, task.pushed), Holding(I))))
    for item, surface in task.object_surfaces:
        goal_literals.append(
            Exists([I, S], And(IsClass(I, item), IsClass(S, surface), On(I, S))))
    for item in task.localized_items:
        goal_literals.append(
            Exists([I], And(Localized(I), IsClass(I, item))))
    for item in task.registered_items:
        goal_literals.append(
            Exists([I], And(Registered(I), IsClass(I, item))))
    for cluster in task.clustered_items:
        ty1, ty2 = cluster # TODO: later relax this
        goal_literals.append(
            Exists([S, I, I2], And(IsClass(I, ty1), IsClass(I2, ty2), On(I, S), On(I2, S))))
    for ty in task.left_items:
        goal_literals.append(
            Exists([S, I], And(IsClass(I, ty), IsLeft(S), On(I, S))))
    for ty in task.right_items:
        goal_literals.append(
            Exists([S, I], And(IsClass(I, ty), IsRight(S), On(I, S))))

    return And(*goal_literals)

##################################################

def solve_tamp(arm, object_meshes, belief, task, occupancy, available_actions=None):
    goal_formula = get_goal_formula(task)
    belief_from_name = populate_env(arm.GetRobot(), belief, object_meshes, occupancy)
    stream_problem = get_ground_problem(arm, object_meshes, belief,
                                        goal_formula, belief_from_name, available_actions)
    print stream_problem
    plan, universe = incremental_planner(stream_problem,
                                         search=get_fast_downward('ff-wastar1', remove=False, verbose=False),
                                         optimal=True, frequency=3, waves=True, debug=False, max_time=3.0,
                                         postprocess_time=5)
    # TODO: AssertionError: Could not find instantiation for PNE! when postprocess_time != None
    print 'Plan: {}\nLength: {} | Cost: {}'.format(convert_plan(plan),
                                                   plan_length(universe, plan),
                                                   plan_cost(universe, plan))
    return convert_plan(plan)

from stripstream.algorithms.hierarchy.utils import preimage_sequence

# TODO: could also just specify the goal explicitly

def solve_hierarchical_tamp(arm, object_meshes, belief, task, occupancy, initial_tform, available_actions=None):
    # TODO: partial policy skeleton
    # TODO: different levels of operator refinement
    # Only choose concrete values when close enough to the table to know what's going on
    # How do I want to specify these hierarchies?
    robot = arm.GetRobot()
    env = robot.GetEnv()
    goal_formula = get_goal_formula(task)
    belief_from_name = populate_env(robot, belief, object_meshes, occupancy)
    abstract_problem = get_abstract_problem(arm, goal_formula, belief, belief_from_name, initial_tform)
    print abstract_problem
    abstract_plan, abstract_universe = incremental_planner(abstract_problem,
                                  search=get_fast_downward('ff-astar', remove=False, verbose=False),
                                  optimal=True, frequency=float('inf'), waves=True)
    print 'Abstract plan:', convert_plan(abstract_plan)
    if not abstract_plan:
        return abstract_plan
    # TODO: translate this to task?
    subgoal = preimage_sequence(abstract_universe, abstract_plan)[0]
    #subgoal = And(*filter(lambda l: not isinstance(l, Atom) or abstract_universe.is_fluent(l), subgoal.formulas))
    subgoal = And(*filter(lambda l: not (isinstance(l, Atom) and l.holds(abstract_universe.initial_atoms,
                             abstract_universe.type_to_objects)), subgoal.formulas))

    print 'Subgoal:', subgoal
    print
    #abstract_plan = None
    pr = cProfile.Profile()
    pr.enable()
    problem = get_ground_problem(arm, object_meshes, belief, subgoal, belief_from_name,
                                 initial_tform, available_actions, convert_plan(abstract_plan))
    print problem
    with env:
      plan, universe = incremental_planner(problem,
                                           search=get_fast_downward('ff-wastar1', remove=False, verbose=False),
                                           optimal=True, frequency=2, waves=True, debug=False, max_time=5.0,
                                           postprocess_time=15)
    pr.disable()
    pstats.Stats(pr).sort_stats('tottime').print_stats(10) # cumtime
    # TODO: AssertionError: Could not find instantiation for PNE! when postprocess_time != None
    print 'Plan: {}\nLength: {} | Cost: {}'.format(convert_plan(plan),
                                                   plan_length(universe, plan),
                                                   plan_cost(universe, plan))
    #debug_planner(universe, max_objects=10)
    return convert_plan(plan)