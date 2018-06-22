from manipulation.primitives.transforms import base_values_from_full_config
from manipulation.primitives.utils import Config
from manipulation.motion.cspace import CSpace
from manipulation.primitives.display import draw_node, draw_edge

from manipulation.motion.primitives import collision_fn, extend_fn, distance_fn, sample_fn
from stripstream.pddl.logic.predicates import Predicate, Function
from stripstream.pddl.logic.operations import Initialize, Cost
from stripstream.pddl.objects import Parameter, Constant
from stripstream.pddl.operators import STRIPSAction
from stripstream.pddl.streams import Stream, TestStream, FunctionStream, StrictStream
from stripstream.pddl.cond_streams import TestCondStream, ProducerCondStream
from stripstream.pddl.problem import STRIPStreamProblem
from stripstream.pddl.objects import EuclideanType, FiniteType, FixedType
from stripstream.pddl.logic.connectives import Not

# TODO - move to the intersection of two regions
# TODO - region that needs to be visited to turn on something
# TODO - multiple goals to choose from
# TODO - LTL goals
# TODO - random walk motion

EAGER_TESTS = True
EAGER_SAMPLE = False
EAGER_COLLISION = False

P = Parameter
C = Constant

CONF = EuclideanType('conf', norm=lambda a, b: 0)
STEER = EuclideanType('steer', norm=lambda a, b: 0) # NOTE - not used within the planner, but guide the expansion
REGION = FiniteType('region')
OBSTACLE = FixedType('obs', [])

AtConfig = Predicate('at_conf', [CONF])
IsCollisionFree = Predicate('is_collision_free', [CONF, CONF])
InRegion = Predicate('in_region', [CONF])

MoveCost = Function('move_cost', [CONF, CONF])

##################################################

# TODO - specialized PRM planner that only connects to new states and dynamically represents the state-space
# TODO - specialized RRT planner which takes advantage of metric space to do things

##################################################

class Conf(Constant):
  dictionary = {}
  def __init__(self, config):
    if config not in self.dictionary:
      self.dictionary[config] = 'q%s'%len(self.dictionary)
    super(Conf, self).__init__(self.dictionary[config], CONF)
    self.value = config

##################################################

# TODO - probably need to provide a temporary cost if something isn't instantiated

class Move(STRIPSAction):
  def __init__(self, oracle):
    params = (P('q1', CONF), P('q2', CONF))
    q1, q2 = params
    super(Move, self).__init__(
      self.__class__.__name__, params, [
        AtConfig(q1),
        #MoveCost(q1, q2),
        #Initialize(MoveCost(q1, q2)), # TODO - should I define this here?
        IsCollisionFree(q1, q2),
      ] , [
        AtConfig(q2),
        Not(AtConfig(q1)),
        Cost(MoveCost(q1, q2)),
    ])
    # TODO - how should I handle the function in the effect?

class CollisionFreeTest(TestCondStream):
  def __init__(self, oracle):
    self.oracle = oracle
    q1, q2 = P('q1', CONF), P('q2', CONF)
    super(CollisionFreeTest, self).__init__([q1, q2], [], [
      IsCollisionFree(q1, q2),
      IsCollisionFree(q2, q1),
      MoveCost(q1, q2), # TODO - need to specify the truth value
    ], eager=EAGER_COLLISION)
    self.collision = collision_fn(oracle.env, oracle.robot, check_self=True) # q_collision_fn
    self.extend = extend_fn(oracle.robot) # q_extend_fn
    self.distance = distance_fn(oracle.robot)
  class StreamFn(Stream):
    max_distance = .5
    handles = []
    scale = 100 # The cost must be a nonnegative integer
    def get_values(self, **kwargs):
      self.enumerated = True
      q1, q2 = self.inputs
      d = self.cond_stream.distance(q1.value.value, q2.value.value)
      if d > self.max_distance:
        return []
      for q in self.cond_stream.extend(q1.value.value, q2.value.value):
        if self.cond_stream.collision(q):
          return []
      self.handles.append(draw_edge(self.cond_stream.oracle.env, q1.value.value, q2.value.value, color=(1, 0, 0, .5)))
      cost = int(self.scale*d+1)
      #return [IsCollisionFree(q1, q2), Initialize(MoveCost(q1, q2), )]
      return [IsCollisionFree(q1, q2), IsCollisionFree(q2, q1),
              Initialize(MoveCost(q1, q2), cost), Initialize(MoveCost(q2, q1), cost)]
      # NOTE - doesn't help for preventing retests of infeasible edges

  """
  class StreamFn(TestStream):
    max_distance = .5
    handles = []
    def test(self, (q1, q2)):
      if q1 == q2:
        return False
      if self.cond_stream.distance(q1.value.value, q2.value.value) > self.max_distance:
        return False
      #with robot:
      #with collision_saver(env, openravepy_int.CollisionOptions.ActiveDOFs):
      for q in self.cond_stream.extend(q1.value.value, q2.value.value):
        if self.cond_stream.collision(q):
          return False
      self.handles.append(draw_edge(self.cond_stream.oracle.env, q1.value.value, q2.value.value, color=(1, 0, 0, .5)))
      return True
  """

class ConfStream(ProducerCondStream):
  def __init__(self, oracle):
    self.oracle = oracle
    q = P('q', CONF)
    super(ConfStream, self).__init__([], [q])
    self.sample = sample_fn(oracle.robot)
    self.collision = collision_fn(oracle.env, oracle.robot, check_self=True) # q_collision_fn
  class StreamFn(StrictStream):
    num = 10
    handles = []
    def get_next(self, **kwargs):
      samples = [self.cond_stream.sample() for _ in range(self.num)]
      samples = filter(lambda s: not self.cond_stream.collision(s), samples) # TODO - replace with more then?
      for sample in samples:
        self.handles.append(draw_node(self.cond_stream.oracle.env, sample, color=(1, 0, 0, .5)))
      return [(Conf(Config(sample)),) for sample in samples]
      #return [(Conf(Config(self.cond_stream.sample())),) for _ in range(self.num)]

class RegionStream(ProducerCondStream): # TODO - a version of this which only steers closet element
  def __init__(self, oracle):
    self.oracle = oracle
    r, q = P('r', REGION), P('q', CONF)
    super(RegionStream, self).__init__([r], [q]) # 1 | 4
  class StreamFn(FunctionStream):
    num_samples = 2
    def function(self, _):
      pass

##################################################

class SampleCollisionFreeStream(TestCondStream):
  pass # TODO - given poses of objects, sample collision free

class BatchCollisionFreeTest(TestCondStream):
  def __init__(self, oracle):
    self.oracle = oracle
    q1, q2 = P('q1', CONF), P('q2', CONF)
    o1, o2 = P('o1', OBSTACLE), P('o2', OBSTACLE)
    super(BatchCollisionFreeTest, self).__init__([q1, q2], [], [
      #IsCollisionFree(q1, q2, o1),
      #IsCollisionFree(q1, q2, o2),
      IsCollisionFree(q1, q2, o1, o2), # NOTE - could also do this
    ], eager=EAGER_TESTS)
  class StreamFn(TestStream):
    def test(self, (q1, q2)):
      pass

class NeighborTest(TestCondStream):
  def __init__(self, oracle):
    self.oracle = oracle
    q1, q2 = P('q1', CONF), P('q2', CONF)
    super(NeighborTest, self).__init__([q1, q2], [], [
      IsCollisionFree(q1, q2)
    ], eager=EAGER_TESTS)
  class StreamFn(TestStream):
    def test(self, (q1, q2)):
      pass

class SteeringTest(TestCondStream):
  def __init__(self, oracle):
    self.oracle = oracle
    q1, q2 = P('q1', CONF), P('q2', CONF)
    super(SteeringTest, self).__init__([q1, q2], [], [
      IsCollisionFree(q1, q2)
    ], eager=EAGER_TESTS)
  class StreamFn(TestStream):
    def test(self, (q1, q2)):
      pass

class SteeringStream(ProducerCondStream): # TODO - a version of this which only steers closet element
  def __init__(self, oracle):
    self.oracle = oracle
    q = P('q', CONF)
    super(SteeringStream, self).__init__([], [q]) # 1 | 4
  class StreamFn(FunctionStream):
    num_samples = 2
    def function(self, _):
      pass

##################################################

# Discrete planning

class NeighborsStream(ProducerCondStream): # TODO - a version of this which only steers closet element
  def __init__(self, oracle):
    self.oracle = oracle
    q1, q2 = P('q', CONF), P('q', CONF)
    super(NeighborsStream, self).__init__([q1], [q2]) # 1 | 4
  class StreamFn(FunctionStream):
    num_samples = 2
    def function(self, _):
      pass

# TODO - use the cost function to infer a heuristic
# TODO - object domain heuristics based on underlying norms

##################################################

# Random walk planning

##################################################

# Combinatorial Planning

# TODO - method that produces vertices from obstacles to obtain visibility graph
# TODO - obstacles are objects then
# NOTE - by treating obstacles as objects, we don't need to tailor the collision function

##################################################

def compile_problem(oracle):
  problem = oracle.problem
  CSpace.robot_base(oracle.robot).set_active()

  initial_config = Config(base_values_from_full_config(oracle.initial_config.value))
  initial_atoms = [
    AtConfig(Conf(initial_config)),
  ]
  print 'Initial', initial_config.value

  goal_literals = [AtConfig(Conf(problem.goal_config))] if problem.goal_config is not None else []
  print 'Goal', problem.goal_config.value

  actions = [
    Move(oracle),
  ]

  axioms = [
    #InRegionAxiom(),
  ]

  cond_streams = [
    CollisionFreeTest(oracle),
    ConfStream(oracle),
  ]

  objects = [
  ]

  return STRIPStreamProblem(initial_atoms, goal_literals, actions + axioms, cond_streams, objects)
