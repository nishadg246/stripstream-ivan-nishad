from stripstream.pddl.examples.continuous_tamp.continuous_tamp_utils import are_colliding, in_region, \
  sample_region, sample_region_pose, inverse_kinematics, sample_inverse_kinematics, is_inverse_kinematics
from stripstream.pddl.logic.connectives import Not, Or, And
from stripstream.pddl.logic.quantifiers import Exists
from stripstream.pddl.logic.predicates import Predicate
from stripstream.pddl.objects import Parameter, Constant, NamedObject, Type, HashableObject
from stripstream.pddl.operators import STRIPSAction, STRIPSAxiom, Axiom
from stripstream.pddl.streams import GeneratorStream, TestStream, FunctionStream
from stripstream.pddl.cond_streams import CondStream, ConstCondStream, TestCondStream
from stripstream.pddl.problem import STRIPStreamProblem
from stripstream.pddl.examples.continuous_tamp.continuous_tamp_viewer import ContinuousTMPViewer

P = Parameter
C = Constant
USE_BASE = True
EAGER_TESTS = True
COLLISIONS = True

CONFIG = Type('conf')
BLOCK = Type('block')
POSE = Type('pose')
REGION = Type('region')

AtConfig = Predicate('at_config', [CONFIG])
HandEmpty = Predicate('hand_empty')
AtPose = Predicate('at_pose', [BLOCK, POSE])
Holding = Predicate('holding', [BLOCK])

Safe = Predicate('safe', [BLOCK, BLOCK, POSE])
InRegion = Predicate('in_region', [BLOCK, REGION])

#IsPose = Predicate('is_pose', [BLOCK, POSE]) # TODO - verify that the pose is within the interval
IsIK = Predicate('is_ik', [POSE, CONFIG])
IsCollisionFree = Predicate('is_collision_free', [BLOCK, POSE, BLOCK, POSE])
IsContained = Predicate('is_contained', [BLOCK, POSE, REGION])

##################################################

class BlockO(NamedObject): type = BLOCK
class RegionO(NamedObject): type = REGION
class PoseO(HashableObject): type = POSE; prefix = 'p'
class ConfigO(HashableObject): type = CONFIG; prefix = 'q'

##################################################

ACTION_COST = 1

class Pick(STRIPSAction):
  cost = ACTION_COST
  def __init__(self):
    params = (P('o', BLOCK), P('p', POSE), P('q', CONFIG))
    block, pose, config = params
    conditions = [
      AtPose(block, pose),
      HandEmpty(),
      IsIK(pose, config),
      #IsContained(block, pose, ENV_REGION),
    ]
    if USE_BASE:
      conditions.append(AtConfig(config))
    super(Pick, self).__init__(
      self.__class__.__name__, params, conditions, [
        Holding(block),
        Not(HandEmpty()),
        Not(AtPose(block, pose)),
    ])

class Place(STRIPSAction):
  cost = ACTION_COST
  def __init__(self, blocks):
    params = (P('o', BLOCK), P('p', POSE), P('q', CONFIG))
    block, pose, config = params
    conditions = [
      Holding(block),
      IsIK(pose, config),
      #IsContained(block, pose, ENV_REGION),
    ]
    if COLLISIONS:
      conditions += [Safe(other, block, pose) for other in blocks] # TODO - always self safe using equality
    if USE_BASE:
      conditions.append(AtConfig(config))
    super(Place, self).__init__(
      self.__class__.__name__, params, conditions, [
        AtPose(block, pose),
        HandEmpty(),
        Not(Holding(block)),
    ])

class Move(STRIPSAction):
  cost = ACTION_COST
  def __init__(self):
    params = (P('q1', CONFIG), P('q2', CONFIG))
    q1, q2 = params
    super(Move, self).__init__(
      self.__class__.__name__, params, [
        AtConfig(q1),
      ] , [
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

##################################################

class IKStream(CondStream):
  def __init__(self):
    pose, conf = P('p', POSE), P('q', CONFIG)
    super(IKStream, self).__init__([pose], [conf], [
    ], [
      IsIK(pose, conf),
    ])
  class StreamFn(FunctionStream):
    def function(self, (p,)):
      return [(ConfigO(inverse_kinematics(p.value)),)]

class RegionStream(CondStream): # NOTE - can treat the interval as a region itself
  def __init__(self):
    b, p, r = P('b', BLOCK), P('p', POSE), P('r', REGION)
    super(RegionStream, self).__init__([b, r], [p], [
    ], [
      IsContained(b, p, r),
    ])
  class StreamFn(GeneratorStream):
    def get_generator(self, (b, r)):
      while True:
        yield (PoseO(sample_region_pose(r.value, b.value)),)

class RegionTest(TestCondStream):
  def __init__(self):
    b, p, r = P('b', BLOCK), P('p', POSE), P('r', REGION)
    super(RegionTest, self).__init__([b, p, r], [], [
      IsContained(b, p, r),
    ], eager=EAGER_TESTS)
  class StreamFn(TestStream):
    def test(self, (b, p, r)):
      return in_region(r.value, b.value, p.value)

class CollisionFreeTest(TestCondStream):
  def __init__(self):
    b1, p1, b2, p2 = P('b1', BLOCK), P('p1', POSE), P('b2', BLOCK), P('p2', POSE)
    super(CollisionFreeTest, self).__init__([b1, p1, b2, p2], [], [
      IsCollisionFree(b1, p1, b2, p2)
    ], eager=EAGER_TESTS)
  class StreamFn(TestStream):
    def test(self, (b1, p1, b2, p2)):
      return not are_colliding(b1.value, p1.value, b2.value, p2.value)

##################################################

class ConfigStream(ConstCondStream):
  def __init__(self, env):
    self.env = env
    q = P('q', CONFIG)
    super(ConfigStream, self).__init__([q], [
    ])
  class StreamFn(GeneratorStream):
    def get_generator(self, _):
      while True:
        yield (ConfigO(sample_region(self.cond_stream.env)),)

class IKStreamBad(CondStream):
  def __init__(self, env):
    self.env = env
    b, p, q = P('b', BLOCK), P('p', POSE), P('q', CONFIG)
    super(IKStreamBad, self).__init__([b], [p, q], [], [
      #IsPose(b, p)
      IsIK(p, q),
    ])
  class StreamFn(GeneratorStream):
    def get_generator(self, (b,)):
      while True:
        p = sample_region_pose(self.cond_stream.env, b.value)
        q = sample_inverse_kinematics(p)
        yield (PoseO(p), ConfigO(q))

class IKTest(TestCondStream):
  def __init__(self):
    p, q = P('p', POSE), P('q', CONFIG)
    super(IKTest, self).__init__([p, q], [], [
      IsIK(p, q),
    ])
  class StreamFn(TestStream):
    @staticmethod
    def test((p, q)):
      return is_inverse_kinematics(p.value, q.value)

##################################################

def generative_streams(tamp_problem):
  return [
    RegionStream(),
    RegionTest(),
    IKStream(),
    CollisionFreeTest(),
  ]

def constant_streams(tamp_problem):
  return [
    IKStreamBad(tamp_problem.env_region),
    #CollisionFreeTest(),
    #RegionTest(),
  ]

def implicit_streams(tamp_problem):
  return [
    RegionStream(),
    ConfigStream(tamp_problem.env_region),
    IKTest(),
    #CollisionFreeTest(),
  ]

##################################################

def compile_problem(tamp_problem, streams_fn=generative_streams):
  blocks = [BlockO(block) for block in tamp_problem.get_blocks()]

  initial_atoms = [
    AtConfig(ConfigO(tamp_problem.initial_config)),
  ] + [
    AtPose(BlockO(block), PoseO(pose)) for block, pose in tamp_problem.initial_poses.iteritems()
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
    Place(blocks),
    InRegionAxiom(),
  ]
  if COLLISIONS:
    operators.append(SafeAxiom())
  if USE_BASE:
    operators.append(Move())

  cond_streams = streams_fn(tamp_problem)

  objects = [ # Assorted objects
    RegionO(tamp_problem.env_region),
  ]

  return STRIPStreamProblem(initial_atoms, goal_literals, operators, cond_streams, objects)

##################################################

COLORS = ['red', 'orange', 'yellow', 'green', 'blue', 'violet']

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

def visualize_atoms(viewer, atoms):
  for atom in atoms:
    if atom.predicate == AtConfig:
      config, = atom.args
      viewer.draw_robot(config.value)
    elif atom.predicate == AtPose:
      block, pose = atom.args
      viewer.draw_block(block.value, pose.value)
    elif atom.predicate == Holding:
      block, = atom.args
      config = get_config(atoms)
      assert config is not None
      viewer.draw_holding(block.value, config)

def visualize_initial(tamp_problem, planning_problem):
  viewer = ContinuousTMPViewer(tamp_problem.env_region, tamp_problem.get_regions(), title='Initial')
  visualize_atoms(viewer, planning_problem.initial_atoms)
  return viewer

def visualize_goal(tamp_problem, planning_problem):
  viewer = ContinuousTMPViewer(tamp_problem.env_region, tamp_problem.get_regions(), tl_y=300, title='Goal')
  visualize_atoms(viewer, planning_problem.goal_literals)
  return viewer
