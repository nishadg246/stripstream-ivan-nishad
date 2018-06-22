from collections import namedtuple

from stripstream.utils import random_sequence
from pygeom import collides, box, rectangle, Polygon, draw_polys, aabb, \
  sample_contained_poly_rot, transform, contains, grasps, \
  tform_matrix, pose_from_tform, np, \
  RED, BLUE, GREEN, DARK_GREY

from stripstream.pddl.logic.predicates import *
from stripstream.pddl.objects import Parameter, Constant, NamedObject
from stripstream.pddl.operators import STRIPSAction, STRIPSAxiom, Axiom
from stripstream.utils import irange
from stripstream.pddl.streams import TestCondStream, GeneratorStream, TestStream, FunctionStream
from stripstream.pddl.cond_streams import CondStream, TestCondStream
from stripstream.pddl.problem import STRIPStreamProblem

P = Parameter
C = Constant
USE_BASE = True
EAGER_TESTS = True
COLLISIONS = False

COLLISION_TIME = 0
CONTAIN_TIME = 0
REGION_TIME = 0
IK_TIME = 0

CONF = Type('conf')
BLOCK = Type('block')
POSE = Type('pose')
GRASP = Type('grasp')
REGION = Type('region')
TRAJ = Type('traj')

AtConfig = Predicate('at_config', [CONF])
HandEmpty = Predicate('hand_empty')
HasGrasp = Predicate('has_grasp', [GRASP])
AtPose = Predicate('at_pose', [BLOCK, POSE])
Holding = Predicate('holding', [BLOCK])

Safe = Predicate('safe', [BLOCK, BLOCK, POSE])
InRegion = Predicate('in_region', [BLOCK, REGION])

IsPose = Predicate('is_pose', [BLOCK, POSE])
IsGrasp = Predicate('is_grasp', [BLOCK, GRASP])
IsIK = Predicate('is_ik', [BLOCK, POSE, GRASP, CONF])
IsCollisionFree = Predicate('is_collision_free', [BLOCK, POSE, BLOCK, POSE])
IsContained = Predicate('is_contained', [BLOCK, POSE, REGION])
IsMotion = Predicate('is_motion', [CONF, CONF, TRAJ])
# TODO - holding motion

##################################################

Robot = namedtuple('Robot', ['name', 'poly', 'gripper', 'color'])
Block = namedtuple('Block', ['name', 'poly', 'color'])
Region = namedtuple('Region', ['name', 'poly', 'color'])

class BlockO(NamedObject): type = BLOCK
class RegionO(NamedObject): type = REGION
class ConfigO(HashableObject): type = CONF; prefix = 'q'
class PoseO(HashableObject): type = POSE; prefix = 'p'
class GraspO(HashableObject): type = GRASP; prefix = 'g'
class TrajO(HashableObject): type = TRAJ; prefix = 't'

##################################################

ACTION_COST = 1

class Pick(STRIPSAction):
  cost = ACTION_COST
  def __init__(self):
    params = (P('o', BLOCK), P('p', POSE), P('g', GRASP), P('q', CONF))
    b, p, g, q = params
    conditions = [
      AtPose(b, p),
      HandEmpty(),
      IsIK(b, p, g, q),
      #IsContained(block, pose, ENV_REGION),
    ]
    if USE_BASE:
      conditions.append(AtConfig(q))
    super(Pick, self).__init__(
      self.__class__.__name__, params, conditions, [
        Holding(b),
        HasGrasp(g),
        Not(HandEmpty()),
        Not(AtPose(b, p)),
    ])

class Place(STRIPSAction):
  cost = ACTION_COST
  def __init__(self):
    params = (P('o', BLOCK), P('p', POSE), P('g', GRASP), P('q', CONF))
    b, p, g, q = params
    conditions = [
      Holding(b),
      HasGrasp(g),
      IsIK(b, p, g, q),
      #IsContained(block, pose, ENV_REGION),
    ]
    if USE_BASE:
      conditions.append(AtConfig(q))
    super(Place, self).__init__(
      self.__class__.__name__, params, conditions, [
        AtPose(b, p),
        HandEmpty(),
        Not(Holding(b)),
        Not(HasGrasp(g)),
    ])

class Move(STRIPSAction):
  cost = ACTION_COST
  def __init__(self, blocks):
    params = (P('q1', CONF), P('q2', CONF), P('t', TRAJ))
    q1, q2, t = params
    conditions = [
      AtConfig(q1),
      HandEmpty(),
      IsMotion(q1, q2, t),
    ]
    if COLLISIONS:
      conditions += [Safe(other, t) for other in blocks] # TODO - always self safe using equality
    super(Move, self).__init__(
      self.__class__.__name__, params, conditions, [
        AtConfig(q2),
        Not(AtConfig(q1)),
    ])

class MoveHolding(STRIPSAction):
  cost = ACTION_COST
  def __init__(self, blocks):
    params = (P('q1', CONF), P('q2', CONF), P('b', BLOCK), P('g', GRASP), P('t', TRAJ))
    q1, q2, b, g, t = params
    conditions = [
      AtConfig(q1),
      Holding(b),
      HasGrasp(g),
      IsMotion(q1, q2, t),
    ]
    if COLLISIONS:
      conditions += [Safe(other, t) for other in blocks] # TODO - always self safe using equality
    super(MoveHolding, self).__init__(
      self.__class__.__name__, params, conditions, [
        AtConfig(q2),
        Not(AtConfig(q1)),
    ])

##################################################

class InRegionAxiom(STRIPSAxiom):
  def __init__(self):
    params = (P('b', BLOCK), P('p', POSE), P('r', REGION))
    b, p, r = params
    super(InRegionAxiom, self).__init__([
        AtPose(b, p),
        IsContained(b, p, r),
      ] , [
        InRegion(b, r),
    ])

class SafeAxiom(Axiom):
  def __init__(self):
    b1, p1, b2, p2 = P('b1', BLOCK), P('p1', POSE), P('b2', BLOCK), P('p2', POSE)
    Axiom.__init__(self, Safe(b1, b2, p2),
      Or(Holding(b1), # TODO - could also have equality here
      #Or(Equal(b1, b2), # TODO - could also have equality here
        Exists([p1], And(
          AtPose(b1, p1),
          #Not(Equal(p1, p2)), # Having the same pose is certainly a collision # NOTE - cannot use equality if I have the same object
          IsCollisionFree(b1, p1, b2, p2)))))

# TODO - alternatively, I could introduce a dummy pose that is always safe

"""
class SafeAxiom1(STRIPSAxiom):
  def __init__(self):
    params = P('b1', BLOCK), P('p1', POSE), P('b2', BLOCK), P('p2', POSE)
    b1, p1, b2, p2 = params
    super(SafeAxiom1, self).__init__([
        AtPose(b1, p1),
        IsCollisionFree(b1, p1, b2, p2),
      ] , [
        Safe(b1, b2, p2),
    ])

class SafeAxiom2(STRIPSAxiom):
  def __init__(self):
    params = P('b1', BLOCK), P('b2', BLOCK), P('p2', POSE)
    b1, b2, p2 = params
    super(SafeAxiom2, self).__init__([
        Holding(b1),
      ] , [
        Safe(b1, b2, p2),
    ])
"""

##################################################

class GraspStream(CondStream):
  def __init__(self, robot):
    self.robot = robot
    b, g = P('b', BLOCK), P('g', GRASP)
    super(GraspStream, self).__init__([b], [g], [
    ], [
      IsGrasp(b, g),
    ])
  class StreamFn(FunctionStream): # TODO - maybe find a better way of doing this?
    def function(self, (b,)):
      robot = self.cond_stream.robot
      #return [(GraspO(grasp),) for grasp in grasps(robot.poly, robot.gripper, b.value.poly)]
      g = []
      for conf in grasps(robot.poly, robot.gripper, b.value.poly):
        grasp = pose_from_tform(np.dot(tform_matrix(conf), np.linalg.inv(tform_matrix((0, 0, 0)))))
        #print conf
        #print np.dot(tform_matrix(grasp), tform_matrix((0, 0, 0)))
        #print pose_from_tform(np.dot(tform_matrix(grasp), tform_matrix((0, 0, 0))))
        g.append((GraspO(grasp),))
      #draw_polys((-5, -5, 5, 5), [(robot.poly, BLUE, None), (b.value.poly, RED, None)])
      return g

# q = p * g # TODO - I don't think something is quite right with the tform math
class IKStream(CondStream):
  def __init__(self, tamp_problem):
    self.tamp_problem = tamp_problem
    b, p, g, q = P('o', BLOCK), P('p', POSE), P('g', GRASP), P('q', CONF)
    super(IKStream, self).__init__([b, p, g], [q], [
      IsPose(b, p),
      IsGrasp(b, g),
    ], [
      IsIK(b, p, g, q),
    ])
  class StreamFn(FunctionStream):
    def function(self, (b, p, g)):
      #q = pose_from_tform(np.linalg.inv(np.dot(tform_matrix(g.value), tform_matrix(p.value)))) # TODO - why is this not right?
      q = pose_from_tform(np.dot(tform_matrix(p.value), tform_matrix(g.value)))
      poly = transform(self.cond_stream.tamp_problem.robot.poly, q)
      if not contains(self.cond_stream.tamp_problem.env_region.poly, poly) or \
          any(collides(poly, other) for other in self.cond_stream.tamp_problem.obstacles):
        return []
      return [(ConfigO(q),)]

class RegionStream(CondStream): # NOTE - can treat the interval as a region itself
  def __init__(self):
    b, p, r = P('b', BLOCK), P('p', POSE), P('r', REGION)
    super(RegionStream, self).__init__([b, r], [p], [
    ], [
      IsPose(b, p),
      IsContained(b, p, r),
    ])
  class StreamFn(GeneratorStream):
    def get_generator(self, (b, r)):
      while True:
        yield (PoseO(sample_contained_poly_rot(r.value.poly, b.value.poly)),)

class RegionTest(TestCondStream):
  def __init__(self):
    b, p, r = P('b', BLOCK), P('p', POSE), P('r', REGION)
    super(RegionTest, self).__init__([b, p, r], [], [
      IsPose(b, p),
      IsContained(b, p, r),
    ], eager=EAGER_TESTS)
  class StreamFn(TestStream):
    def test(self, (b, p, r)):
      return contains(r.value.poly, transform(b.value.poly, p.value))

class CollisionFreeTest(TestCondStream):
  def __init__(self):
    b1, p1, b2, p2 = P('b1', BLOCK), P('p1', POSE), P('b2', BLOCK), P('p2', POSE)
    super(CollisionFreeTest, self).__init__([b1, p1, b2, p2], [], [
      IsCollisionFree(b1, p1, b2, p2),
      #IsCollisionFree(b2, p2, b1, p1)
    ], eager=EAGER_TESTS)
  class StreamFn(TestStream):
    def test(self, (b1, p1, b2, p2)):
      #return not collides(transform(b1.poly, ))
      return not are_colliding(b1.value, p1.value, b2.value, p2.value)

class MotionStream(CondStream):
  def __init__(self, tamp_problem):
    self.tamp_problem = tamp_problem
    q1, q2, t = P('q1', CONF), P('q2', CONF), P('t', TRAJ)
    super(MotionStream, self).__init__([q1, q2], [t], [
    ], [
      IsMotion(q1, q2, t),
    ])
  class StreamFn(FunctionStream):
    def function(self, (q1, q2)):
      return [(TrajO((q1, q2)),)]

# TODO - motion stream which avoids current object poses
# TODO - how do I avoid calling motion plans from all other confs

##################################################

TMPProblem = namedtuple('TMPProblem', ['env_region', 'initial_config', 'initial_holding', 'initial_poses',
                                       'goal_config', 'goal_holding', 'goal_poses', 'goal_regions'])

class TMPProblem(object):
  def __init__(self, env_region, robot, obstacles, initial_config, initial_holding, initial_poses,
                     goal_config=None, goal_holding=None, goal_poses=tuple(), goal_regions=tuple()):
    self.env_region = env_region
    self.robot = robot
    self.obstacles = obstacles
    self.initial_config = initial_config
    self.initial_holding = initial_holding
    self.initial_poses = initial_poses
    self.goal_config = goal_config
    self.goal_holding = goal_holding
    self.goal_poses = goal_poses
    self.goal_regions = goal_regions
  def get_blocks(self):
    blocks, _ = zip(*self.initial_poses)
    if self.initial_holding is not None:
      blocks.append(self.initial_holding)
    return blocks
  def get_regions(self):
    _, regions = zip(*self.goal_regions)
    return regions

def sample_block_poses(blocks, env, obstacles, timeout=100): # NOTE - might need to reset the full assignment if infeasible
  block_poses = []
  placed = list(obstacles)
  for block, region in random_sequence(blocks):
    for _ in irange(0, timeout):
      p = sample_contained_poly_rot(env.poly, block.poly)
      poly = transform(block.poly, p)
      if not any(collides(poly, body) for body in placed):
        block_poses.append((block, p))
        placed.append(poly)
        break
    else:
      return sample_block_poses(blocks, env, obstacles)
  return block_poses

##################################################

def make_env():
  return Region('env', box(-10, -10, +10, +10), DARK_GREY)

def make_robot():
  gripper = (0, +2)
  return Robot('robot', Polygon([(-1, -1), (+1, -1), gripper]), gripper, BLUE)

def sample_namo_problem():
  env = make_env()
  robot = make_robot()
  obstacles = [
    box(-1, -10, +1, 0),
  ]

  initial_conf = (-5, 0, 0)
  initial_holding = None
  initial_regions = [
    #(Block('block0', rectangle(-1, +1), RED), env)
    (Block('block%s'%i, rectangle(-1, +1), RED), env) for i in range(5)
  ]
  initial_poses = sample_block_poses(initial_regions, env, obstacles+[transform(robot.poly, initial_conf)])

  return TMPProblem(env, robot, obstacles, initial_conf, initial_holding, initial_poses,
                    goal_config=(+5, 0, 0))

def sample_grasp_problem():
  env = make_env()
  robot = make_robot()
  obstacles = []

  initial_conf = (0, 0, 0)
  initial_holding = None
  initial_regions = [
    (Block('block0', rectangle(-1, +1), RED), env)
  ]
  initial_poses = sample_block_poses(initial_regions, env, obstacles+[transform(robot.poly, initial_conf)])

  return TMPProblem(env, robot, obstacles, initial_conf, initial_holding, initial_poses,
                    goal_holding=initial_regions[0][0])

##################################################

def sample_tamp_problem(block_w=1., block_h=1.):
  env = make_env()
  robot = make_robot()
  obstacles = []

  initial_conf = (0, 0, 0)
  initial_holding = None
  initial_regions = [
    (Block('block0', rectangle(-1, +1), GREEN), env)
  ]
  initial_poses = sample_block_poses(initial_regions, env, obstacles+[transform(robot.poly, initial_conf)])

  goal_regions = [
    (initial_regions[0][0], Region('goal', box(3, -7, 7, -3), GREEN)),
  ]

  return TMPProblem(env, robot, obstacles, initial_conf, initial_holding, initial_poses,
                    goal_regions=goal_regions)

##################################################

def compile_problem(tamp_problem):
  blocks = [BlockO(block) for block in tamp_problem.get_blocks()]

  initial_atoms = [
    AtConfig(ConfigO(tamp_problem.initial_config)),
  ] + [
    AtPose(BlockO(block), PoseO(pose)) for block, pose in tamp_problem.initial_poses
  ] + [
    IsPose(BlockO(block), PoseO(pose)) for block, pose in tamp_problem.initial_poses
  ]
  if tamp_problem.initial_holding is None:
    initial_atoms.append(HandEmpty())
  else:
    initial_atoms.append(Holding(BlockO(tamp_problem.initial_holding)))

  goal_literals = []
  if tamp_problem.goal_config is not None:
    goal_literals.append(AtConfig(ConfigO(tamp_problem.goal_config)))
  if tamp_problem.goal_holding is False:
    goal_literals.append(HandEmpty())
  elif tamp_problem.goal_holding is not None:
    goal_literals.append(Holding(BlockO(tamp_problem.goal_holding)))
  for block, goal in tamp_problem.goal_poses:
    goal_literals.append(AtPose(BlockO(block), PoseO(goal)))
  for block, goal in tamp_problem.goal_regions:
    goal_literals.append(InRegion(BlockO(block), RegionO(goal)))

  operators = [
    Pick(),
    Place(),
    InRegionAxiom(),
  ]

  cond_streams = [
    GraspStream(tamp_problem.robot),
    RegionStream(),
    RegionTest(),
    IKStream(tamp_problem),
  ]

  if COLLISIONS:
    operators.append(SafeAxiom())
    cond_streams.append(CollisionFreeTest())
  if USE_BASE:
    operators.append(Move(blocks))
    operators.append(MoveHolding(blocks))
    cond_streams.append(MotionStream(tamp_problem))

  objects = [ # Assorted objects
    RegionO(tamp_problem.env_region),
  ]

  return STRIPStreamProblem(initial_atoms, goal_literals, operators, cond_streams, objects)

##################################################

def get_config(atoms):
  for atom in atoms:
    if atom.predicate == AtConfig:
      config, = atom.args
      return config.value
  return None

def get_holding(atoms):
  for atom in atoms:
    if atom.predicate == Holding:
      block, = atom.args
      return block.value
  return None

def get_grasp(atoms):
  for atom in atoms:
    if atom.predicate == HasGrasp:
      grasp, = atom.args
      return grasp.value
  return None

def visualize_atoms(tamp_problem, atoms):
  polys = []
  # TODO - regions
  for atom in atoms:
    if atom.predicate == AtConfig:
      conf, = atom.args
      polys.append((transform(tamp_problem.robot.poly, conf.value), tamp_problem.robot.color, None))
    elif atom.predicate == AtPose:
      block, pose = atom.args
      polys.append((transform(block.value.poly, pose.value), block.value.color, None))
    elif atom.predicate == Holding:
      block, = atom.args
      conf = get_config(atoms)
      grasp = get_grasp(atoms)
      assert conf is not None and grasp is not None
      pose = pose_from_tform(np.dot(tform_matrix(conf), np.linalg.inv(tform_matrix(grasp))))
      polys.append((transform(block.value.poly, pose), block.value.color, None))

  for region in tamp_problem.get_regions():
    polys.append((region.poly, region.color, 2))
  for obstacle in tamp_problem.obstacles:
    polys.append((obstacle, DARK_GREY, None))

  env = tamp_problem.env_region
  #polys.append((env.poly, env.color, 10))
  draw_polys(aabb(env.poly), polys)
