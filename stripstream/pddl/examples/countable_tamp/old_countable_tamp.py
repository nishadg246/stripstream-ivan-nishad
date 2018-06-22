from stripstream.pddl.logic.connectives import Not, Or, And
from stripstream.pddl.logic.quantifiers import Exists
from stripstream.pddl.logic.predicates import Predicate
from stripstream.pddl.logic.atoms import Atom
from stripstream.pddl.objects import Parameter, Constant, StringObject, Type
from stripstream.pddl.operators import STRIPSAction, STRIPSAxiom, Axiom
from stripstream.utils import irange, INF
from stripstream.pddl.streams import GeneratorStream, Stream, TestStream, FunctionStream, StrictStream
from stripstream.pddl.cond_streams import CondStream, ConstCondStream, TestCondStream
from stripstream.pddl.problem import STRIPStreamProblem
from stripstream.algorithms.hierarchy.operators import InheritAction, Refinable
from stripstream.pddl.examples.countable_tamp.countable_tamp_viewer import CountableTMPViewer
from stripstream.pddl.examples.countable_tamp.countable_tamp_utils import NUM_POSES

P = Parameter
C = Constant

BASE = True
COLLISIONS = True
ACTION_COST = 1

CONFIG = Type('conf')
BLOCK = Type('block')
POSE = Type('pose')
REGION = Type('region')

AtConfig = Predicate('at_config', [CONFIG])
HandEmpty = Predicate('hand_empty', [])
AtPose = Predicate('at_pose', [BLOCK, POSE])
Holding = Predicate('holding', [BLOCK])
Unsafe = Predicate('unsafe', [POSE]) # The opposite of clear

Safe = Predicate('safe', [BLOCK, POSE])
InRegion = Predicate('in_region', [BLOCK, REGION])

IsBlock = Predicate('is_block', [BLOCK])
IsPose = Predicate('is_pose', [POSE])
IsConfig = Predicate('is_config', [CONFIG])
IsIK = Predicate('is_ik', [POSE, CONFIG])
IsCollisionFree = Predicate('is_collision_free', [POSE, POSE])
IsContained = Predicate('is_contained', [REGION, POSE])

class Pose(StringObject): type = POSE; prefix = 'p'
class Config(StringObject): type = CONFIG; prefix = 'q'

##################################################

class H1Pick(STRIPSAction, Refinable):
  cost = 2*ACTION_COST
  def __init__(self):
    block, pose, config = P('o', BLOCK), P('p', POSE), P('q', CONFIG)
    STRIPSAction.__init__(self,
      self.__class__.__name__, [block, pose, config], [
        AtPose(block, pose),
        HandEmpty(),
        IsIK(pose, config),
      ], [
        Holding(block),
        Not(HandEmpty()),
        Not(AtPose(block, pose)),
    ])
    Refinable.__init__(self, [H0Pick1(block, pose, config)])
    #Refinable.__init__(self, [H0Pick2(self)])

class H0Pick1(STRIPSAction):
  cost = ACTION_COST
  def __init__(self, block=P('o',BLOCK), pose=P('p',POSE), config=P('q',CONFIG)):
  #def __init__(self, block, pose, config):
    super(H0Pick1, self).__init__(
      self.__class__.__name__, [block, pose, config], [
        AtPose(block, pose),
        HandEmpty(),
        IsIK(pose, config),
        AtConfig(config),
      ], [
        Holding(block),
        Not(HandEmpty()),
        Not(AtPose(block, pose)),
    ])

class H0Pick2(InheritAction):
  cost = ACTION_COST
  ParentClass = H1Pick
  def __init__(self, parent):
    assert isinstance(parent, self.ParentClass) # TODO - move this to something else
    _, _, config = parent.parameters
    super(H0Pick2, self).__init__(
      self.__class__.__name__, parent, [], [
        AtConfig(config),
      ], [])

##################################################

class Pick(STRIPSAction):
  cost = ACTION_COST
  def __init__(self):
    params = (P('o', BLOCK), P('p', POSE), P('q', CONFIG))
    block, pose, config = params
    conditions = [
      AtPose(block, pose),
      HandEmpty(),
      IsIK(pose, config), # IsPose(p), IsConfig(q),
    ]
    if BASE:
      conditions.append(AtConfig(config))
    super(Pick, self).__init__(
      self.__class__.__name__, params, conditions, [
        Holding(block),
        Not(HandEmpty()),
        Not(AtPose(block, pose)),
        #Not(Unsafe(pose)),
    ])

# class Place(STRIPSAction):
#   cost = 1
#   def __init__(self, blocks):
#     params = (P('o', BLOCK), P('p', POSE), P('q', CONFIG))
#     block, pose, config = params
#     super(Place, self).__init__(
#       self.__class__.__name__, params, [
#         Holding(block),
#         AtConfig(config), # TODO - remove using BASE
#         Not(Unsafe(pose)),
#         IsIK(pose, config), # IsPose(p), IsConfig(q),
#       ] , [
#         AtPose(block, pose),
#         HandEmpty(),
#         Not(Holding(block)),
#         Unsafe(pose),
#     ])

class Place(STRIPSAction):
  cost = ACTION_COST
  def __init__(self, blocks):
    params = (P('o', BLOCK), P('p', POSE), P('q', CONFIG))
    block, pose, config = params
    conditions = [
      Holding(block),
      IsIK(pose, config), # IsPose(p), IsConfig(q),
    ]
    if COLLISIONS:
      conditions += [Safe(other, pose) for other in blocks] # TODO - always self safe using equality
    if BASE:
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
        AtConfig(q1), #IsConfig(q1), IsConfig(q2),
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
        IsContained(r, p),
      ] , [
        InRegion(b, r),
    ])

# class SafeAxiom(STRIPSAxiom):
#   def __init__(self):
#     b1, p1, p2 = P('b1', BLOCK), P('p1', POSE), P('p2', POSE)
#     super(SafeAxiom, self).__init__([
#         AtPose(b1, p1),
#         IsCollisionFree(p1, p2),
#       ] , [
#         Safe(b1, p2),
#     ])

class SafeAxiom(Axiom):
  def __init__(self):
    b1, p1, p2 = P('b1', BLOCK), P('p1', POSE), P('p2', POSE)
    Axiom.__init__(self, Safe(b1, p2),
      Or(Holding(b1), # TODO - could also have equality here
        Exists([p1], And(
          AtPose(b1, p1),
          #Not(Equal(p1, p2)), # TODO - I could always immediately outlaw them having the same pose...
          IsCollisionFree(p1, p2)))))

##################################################

POSE_RANGE = [0, NUM_POSES]
CONFIG_RANGE = POSE_RANGE

def pose_generator():
  for n in irange(*POSE_RANGE):
    yield Pose(n),

def config_generator():
  for n in irange(*CONFIG_RANGE):
    yield Config(n),

# TODO - should these be objects themselves or just functions?
# TODO - maybe specify the same inference but allow different ways of generating it
# NOTE - would have specify all possible properties that they jointly specify and move some to preconditions and other to effects
class IKStream(CondStream):
  def __init__(self):
    pose, conf = P('p', POSE), P('q', CONFIG)
    super(IKStream, self).__init__([pose], [conf], [
      #IsPose(pose),
    ], [
      IsIK(pose, conf),
    ])
  # NOTE - StreamFn serves as a function from inputs to stream
  class StreamFn(FunctionStream): # NOTE - this doesn't depend on the object at all
    def function(self, (p,)):
      return [(Config(p.value),)]
  #stream_fn = StreamFn

class PoseStream(ConstCondStream):
  def __init__(self):
    pose = P('p', POSE)
    super(PoseStream, self).__init__([pose], [
      #IsPose(pose)
    ])
  #class StreamFn(GeneratorStream):
  #  generator_fn = pose_generator
  class StreamFn(StrictStream):
    def __init__(self, cond_stream, inputs):
      self.n = 0
      #super(StreamFn, self).__init__(cond_stream, inputs) # TODO - not working...
      Stream.__init__(self, cond_stream, inputs)
    def get_next(self, **kwargs):
      #(,) = self.inputs
      p = Pose(self.n)
      self.n += 1
      return [(p,)]

class CollisionFreeTest(TestCondStream):
  def __init__(self):
    p1, p2 = P('p1', POSE), P('p2', POSE)
    super(CollisionFreeTest, self).__init__([p1, p2], [], [
      IsCollisionFree(p1, p2)
    ])
  class StreamFn(TestStream):
    @staticmethod
    def test((p1, p2)):
      return p1.value != p2.value
# TODO - I should should just make this a conditional stream object

##################################################

class ConfigStream(ConstCondStream):
  def __init__(self):
    conf = P('q', CONFIG)
    super(ConfigStream, self).__init__([conf], [
    ])
  class StreamFn(GeneratorStream):
    def get_generator(self, _):
      return config_generator()

class IKStreamBad(ConstCondStream):
  def __init__(self):
    pose, conf = P('p', POSE), P('q', CONFIG)
    super(IKStreamBad, self).__init__([pose, conf], [
      IsIK(pose, conf),
    ])
  class StreamFn(GeneratorStream):
    def get_generator(self, _):
      #for p, g in product(pose_generator(), config_generator()):
      #  if p.value == g.value:
      #    yield p, g
      for p, in pose_generator(): # Could also do the other version of this
        yield p, Config(p.value)

class IKTest(TestCondStream):
  def __init__(self):
    pose, conf = P('p', POSE), P('q', CONFIG)
    super(IKTest, self).__init__([pose, conf], [], [
      IsIK(pose, conf),
    ])
  class StreamFn(TestStream):
    @staticmethod
    def test((pose, conf)):
      return pose.value == conf.value

##################################################

def generative_streams():
  return [
    PoseStream(),
    IKStream(),
    CollisionFreeTest(),
  ]

def constant_streams():
  return [
    IKStreamBad(),
    CollisionFreeTest(),
  ]

def implicit_streams():
  return [
    PoseStream(),
    ConfigStream(),
    IKTest(),
    CollisionFreeTest(),
  ]

def compile_problem(tamp_problem, stream_fn=generative_streams):
  blocks = [C(block, BLOCK) for block in tamp_problem.get_blocks()]

  initial_atoms = [
    AtConfig(Config(tamp_problem.initial_config)),
  ] + [
    AtPose(C(block, BLOCK), Pose(pose)) for block, pose in tamp_problem.initial_poses.iteritems()
  ]
  if tamp_problem.initial_holding is False:
    initial_atoms.append(HandEmpty())
  else:
    initial_atoms.append(Holding(C(tamp_problem.initial_holding, BLOCK)))

  goal_literals = []
  if tamp_problem.goal_holding is False:
    goal_literals.append(HandEmpty())
  elif tamp_problem.goal_holding is not None:
    goal_literals.append(Holding(C(tamp_problem.goal_holding, BLOCK)))
  for block, goal in tamp_problem.goal_poses.iteritems():
    goal_literals.append(AtPose(C(block, BLOCK), Pose(goal)))

  operators = [
    Move(),
    #H1Pick(),
    Pick(),
    Place(blocks),
    #InRegionAxiom(),
  ]
  if COLLISIONS:
    operators.append(SafeAxiom())

  objects = [ # Assorted objects
  ]

  return STRIPStreamProblem(initial_atoms, goal_literals, operators, stream_fn(), objects)

##################################################

def get_problem():
  region = [1, 2, 3]

  #mergeable_types = {BLOCK}

  block = C('block%s'%1, BLOCK)
  obstacle = C('block%s'%2, BLOCK)
  blocks = [block]
  #blocks = [block, obstacle]

  pose = Pose(10)
  config = Config(0)
  region = C('region%s'%1, REGION)

  initial_atoms = [
    AtConfig(config),
    HandEmpty(),
    AtPose(block, pose),
    #AtPose(obstacle, Pose(20)),
    #Unsafe(pose),
  ] + [
    #AtPose(obj, pose) for obj, pose in object_poses.iteritems()
  ]

  goal_literals = [
    Holding(block),
    #Not(HandEmpty(robot)), # NOTE - negative goal
    #Not(AtPose(obj, Pose(0))),
    #AtPose(block, Pose(20)),
    #AtPose(obj, Pose(4)),
    #AtConfig(Config(5)),
    #AtConfig(robot, Config(4)),
    #InRegion(block, region),
  ]

  operators = [
    Move(),
    Pick(),
    Place(blocks),
    #InRegionAxiom(),
  ]
  if COLLISIONS:
    operators.append(SafeAxiom())

  cond_streams = [
    PoseStream(),
    #IKStream(),
    CollisionFreeTest(),

    ConfigStream(),
    #IKStreamBad(),
    IKTest(),
  ]

  objects = [ # Assorted objects
  ]

  return STRIPStreamProblem(initial_atoms, goal_literals, operators, cond_streams, objects)

##################################################

ROWS, COLS = 1, 5
COLORS = ['red', 'orange', 'yellow', 'green', 'blue', 'violet']

def visualize_atoms(viewer, atoms):
  for atom in atoms:
    if isinstance(atom, Atom):
      if atom.predicate == AtConfig:
        q, = atom.args
        viewer.draw_robot(q.value)
      elif atom.predicate == AtPose:
        obj, p = atom.args
        index = int(obj.name[len('block'):]) # TODO - identify goal blocks and their colors
        viewer.draw_block(0, p.value, color=COLORS[index])
      elif atom.predicate == Holding:
        raise NotImplementedError()
        #obj = atom.args
        #viewer.draw_block(0, p.value)

def visualize_formula(viewer, formula):
  raise NotImplementedError()

def visualize_initial(problem):
  viewer = CountableTMPViewer(ROWS, COLS, title='Initial')
  visualize_atoms(viewer, problem.initial_atoms)
  return viewer

def visualize_goal(problem):
  viewer = CountableTMPViewer(ROWS, COLS, title='Goal')
  visualize_atoms(viewer, problem.goal_literals)
  return viewer