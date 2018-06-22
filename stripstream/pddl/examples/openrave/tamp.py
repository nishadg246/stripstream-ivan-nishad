from itertools import islice

from manipulation.grasps.grasps import get_grasps
from manipulation.pick_and_place import PickAndPlace
from manipulation.primitives.placements import random_region_placements
from manipulation.grasps.grasps import Holding as ObjGrasp
from manipulation.execute import Pickable, Placeable

from stripstream.pddl.logic.predicates import Predicate
from stripstream.pddl.logic.connectives import Not, And, Or
from stripstream.pddl.logic.quantifiers import Exists
from stripstream.pddl.objects import Type
from stripstream.pddl.objects import Parameter, Constant
from stripstream.pddl.operators import STRIPSAction, STRIPSAxiom, Axiom
from stripstream.pddl.streams import TestStream, FunctionStream, StrictStream
from stripstream.pddl.cond_streams import CondStream, TestCondStream
from stripstream.pddl.problem import STRIPStreamProblem

BASE = True
COLLISIONS = True
DO_MOTION = True
ACTION_COST = 1
EAGER_TESTS = True

####################

CONFIG = Type('conf')
BLOCK = Type('block')
POSE = Type('pose')
GRASP = Type('grasp')
REGION = Type('region')
TRAJ = Type('traj')

####################

AtConfig = Predicate('at_config', [CONFIG])
HandEmpty = Predicate('hand_empty', [])
AtPose = Predicate('at_pose', [BLOCK, POSE]) # TODO - probably don't even need block here...
#HasGrasp = Predicate('has_grasp', [BLOCK, GRASP])
HasGrasp = Predicate('has_grasp', [GRASP])
Holding = Predicate('holding', [BLOCK])

####################

#Holding = Predicate('holding', [BLOCK]) # NOTE - used to use an axiom, but now I update it explicitly
InRegion = Predicate('in_region', [BLOCK, REGION])
Safe = Predicate('safe', [BLOCK, TRAJ])

####################

IsPose = Predicate('is_pose', [BLOCK, POSE])
IsGrasp = Predicate('is_grasp', [BLOCK, GRASP])
#IsIK = Predicate('is_ik', [POSE, GRASP, CONFIG, TRAJ])
IsIK = Predicate('is_ik', [BLOCK, POSE, GRASP, CONFIG, TRAJ]) # NOTE - this ensures that the only achiever uses the block

IsContained = Predicate('is_contained', [REGION, POSE]) # TODO - should I make this include the block as well?
IsCollisionFree = Predicate('is_collision_free', [POSE, TRAJ])

####################

class Config(Constant):
  dictionary = {}
  def __init__(self, config):
    if config not in self.dictionary:
      self.dictionary[config] = 'q%s'%len(self.dictionary)
    super(Config, self).__init__(self.dictionary[config], CONFIG)
    self.value = config

# TODO - do this with object geometries

class Pose(Constant):
  dictionary = {}
  def __init__(self, obj, pose):
    if (obj, pose) not in self.dictionary:
      self.dictionary[(obj, pose)] = 'p%s'%len(self.dictionary) # TODO - include obj?
    super(Pose, self).__init__(self.dictionary[(obj, pose)], POSE)
    self.obj = obj
    self.value = pose

class Grasp(Constant):
  dictionary = {}
  def __init__(self, obj, grasp):
    if (obj, grasp) not in self.dictionary:
      self.dictionary[(obj, grasp)] = 'g%s'%len(self.dictionary)
    super(Grasp, self).__init__(self.dictionary[(obj, grasp)], GRASP)
    self.obj = obj
    self.value = grasp

class Trajectory(Constant):
  dictionary = {}
  def __init__(self, obj, pap):
    if (obj, pap) not in self.dictionary:
      self.dictionary[(obj, pap)] = 't%s'%len(self.dictionary)
    super(Trajectory, self).__init__(self.dictionary[(obj, pap)], TRAJ)
    self.obj = obj
    self.pap = pap

####################

P = Parameter
C = Constant

def collision_conditions(oracle, b, t):
  if not COLLISIONS:
    return []
  return [Safe(C(name, BLOCK), t) for name in oracle.get_objects()]
  # ob = P('ob', BLOCK)
  # return [ForAll([ob], Safe(ob, t))]
  # return [ForAll([ob], Or(Equal(b, ob), Safe(ob, t)))]
  # op = P('op', POSE)
  # return [ForAll([ob],
  #  Exists([op], And(
  #    IsPose(ob, op),
  #    AtPose(ob, op),
  #    IsCollisionFree(ob, t)),
  #  ))]

class ExecutablePick(Pickable):
  def __init__(self, object_name, pap):
    self.object_name = object_name
    self.pap = pap

class Pick(STRIPSAction):
  cost = ACTION_COST
  def __init__(self, oracle):
    params = (P('b', BLOCK), P('p', POSE), P('g', GRASP), P('q', CONFIG), P('t', TRAJ))
    b, p, g, q, t = params
    conditions = [
      AtPose(b, p),
      HandEmpty(),
      IsPose(b, p), # NOTE - might not need these
      IsGrasp(b, g),
      IsIK(b, p, g, q, t),
    ]
    if BASE:
      conditions.append(AtConfig(q))
    conditions += collision_conditions(oracle, b, t)
    effects = [
      #HasGrasp(b, g),
      Holding(b),
      HasGrasp(g),
      Not(HandEmpty()),
      Not(AtPose(b, p)),
    ]
    super(Pick, self).__init__(self.__class__.__name__, params, conditions, effects)
  def executable(self, args): # TODO - rethink this
    b, p, g, q, t = args
    return ExecutablePick(b.name, t.pap)

class ExecutablePlace(Placeable):
  def __init__(self, object_name, pap):
    self.object_name = object_name
    self.pap = pap

class Place(STRIPSAction):
  cost = ACTION_COST
  def __init__(self, oracle):
    params = (P('b', BLOCK), P('p', POSE), P('g', GRASP), P('q', CONFIG), P('t', TRAJ))
    b, p, g, q, t = params
    conditions = [
      #HasGrasp(b, g),
      Holding(b),
      HasGrasp(g),
      IsPose(b, p), # NOTE - might not need these
      IsGrasp(b, g),
      IsIK(b, p, g, q, t),
    ]
    if BASE:
      conditions.append(AtConfig(q))
    conditions += collision_conditions(oracle, b, t)
    effects = [
      AtPose(b, p),
      HandEmpty(),
      #Not(HasGrasp(b, g)),
      Not(Holding(b)),
      Not(HasGrasp(g)),
    ]
    super(Place, self).__init__(self.__class__.__name__, params, conditions, effects)
  def executable(self, args): # TODO - rethink this
    b, p, g, q, t = args
    return ExecutablePlace(b.name, t.pap)

#class ExecutablePlace(Moveable):
#  def __init__(self, object_name, pap):
#    pass

class Move(STRIPSAction):
  cost = ACTION_COST
  def __init__(self, oracle):
    params = (P('q1', CONFIG), P('q2', CONFIG))
    q1, q2 = params
    super(Move, self).__init__(
      self.__class__.__name__, params, [
        AtConfig(q1),
      ] , [
        AtConfig(q2),
        Not(AtConfig(q1)),
        #Cost(???), # TODO - include cost lowerbound first
        #Cost(???), # NOTE - PDDL uses the last cost specified
    ])

class MoveHolding(STRIPSAction):
  pass

####################

# class HoldingAxiom(STRIPSAxiom):
#   def __init__(self):
#     params = (P('b', BLOCK), P('g', GRASP))
#     b, g = params
#     super(HoldingAxiom, self).__init__([
#         HasGrasp(b, g),
#         IsGrasp(b, g),
#       ] , [
#         Holding(b),
#     ])

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

class SafeAxiom(Axiom): # TODO - it computes self collisions here when picking up an object...
  def __init__(self):
    b, p, t = P('b', BLOCK), P('p', POSE), P('t', TRAJ)
    Axiom.__init__(self, Safe(b, t),
      Or(Holding(b), # TODO - could also have equality here
        Exists([p], And(
          AtPose(b, p),
          #Not(Equal(p1, p2)), # TODO - I could always immediately outlaw them having the same pose...
          IsCollisionFree(p, t)))))

####################

class PoseStream(CondStream):
  def __init__(self, oracle):
    self.oracle = oracle
    b, p = P('b', BLOCK), P('p', POSE)
    super(PoseStream, self).__init__([b], [p], [
    ], [
      IsPose(b, p),
    ])
  class StreamFn(FunctionStream): # TODO - maybe find a better way of doing this?
    num_samples = 2
    avoid_initial = True
  #   def get_list(self, (b,)):
  #     oracle = self.cond_stream.oracle
  #     poses = random_region_placements(oracle, b.name, oracle.get_counters(), region_weights=True)
  #     return [(Pose(b.name, pose),) for pose in islice(poses, self.num_samples)]
    def function(self, (b,)):
      oracle = self.cond_stream.oracle
      with oracle.state_saver():
        if self.avoid_initial:
          oracle.set_all_object_poses(oracle.initial_poses)
        else:
          oracle.set_all_object_poses({b.name: oracle.initial_poses[b.name]})
        poses = random_region_placements(oracle, b.name, oracle.get_counters(), region_weights=True)
        #poses = cached_region_placements(self.oracle, self.obj, self.oracle.get_counters(), order=None, random=True)
        return [(Pose(b.name, pose),) for pose in islice(poses, self.num_samples)]

class GraspStream(CondStream):
  def __init__(self, oracle):
    self.oracle = oracle
    b, g = P('b', BLOCK), P('g', GRASP)
    super(GraspStream, self).__init__([b], [g], [
    ], [
      IsGrasp(b, g),
    ])
  class StreamFn(FunctionStream): # TODO - maybe find a better way of doing this?
    def function(self, (b,)):
      return [(Grasp(b.name, grasp),) for grasp in get_grasps(self.cond_stream.oracle, b.name)]

##########

class ContainedStream(CondStream):
  def __init__(self, oracle):
    self.oracle = oracle
    b, p, r = P('b', BLOCK), P('p', POSE), P('r', REGION)
    super(ContainedStream, self).__init__([b, r], [p], [
    ], [
      IsPose(b, p), IsContained(r, p)
    ])
  class StreamFn(FunctionStream): # TODO - maybe find a better way of doing this?
    num_samples = 2
    def function(self, (b, r)): # TODO - switch this to be a generator
      poses = random_region_placements(self.cond_stream.oracle, b.name, [r.name], region_weights=True) # TODO - save this generator
      return [(Pose(b.name, pose),) for pose in islice(poses, self.num_samples)]

class ContainedTest(TestCondStream):
  def __init__(self, oracle):
    self.oracle = oracle
    p, r = P('p', POSE), P('r', REGION) # TODO - should I put the object here?
    super(ContainedTest, self).__init__([p, r], [], [
      IsContained(r, p)
    ], eager=EAGER_TESTS)
  class StreamFn(TestStream):
    def test(self, (p, r)):
      return self.cond_stream.oracle.region_contains(r.name, p.obj, p.value)

##########

class IKStream(CondStream):
  def __init__(self, oracle):
    self.oracle = oracle
    b, p, g, q, t = (P('b', BLOCK), P('p', POSE), P('g', GRASP), P('q', CONFIG), P('t', TRAJ))
    super(IKStream, self).__init__([b, p, g], [q, t], [
      IsPose(b, p),
      IsGrasp(b, g),
    ], [
      IsIK(b, p, g, q, t),
    ])
  class StreamFn(StrictStream): # TODO - handle oracle
    max_failures = 50 # 10 | 20 | 40
    max_calls = 1 # 1 | INF
    def get_next(self, **kwargs):
      oracle = self.cond_stream.oracle
      b, p, g = self.inputs
      if self.calls >= self.max_calls:
        self.enumerated = True
      with oracle.state_saver():
        oracle.set_all_object_poses({b.name: p.value})
        if oracle.approach_collision(b.name, p.value, g.value):
          return []
        pap = PickAndPlace(oracle.get_geom_hash(b.name), p.value, g.value)
        if not pap.sample(oracle, b.name, max_failures=self.max_failures,
                          sample_vector=DO_MOTION, sample_arm=DO_MOTION, check_base=False):
          return []
        #self.oracle.add_pap(self.obj, pap)
        return [(Config(pap.approach_config), Trajectory(b.name, pap))]

##########

class CollisionFreeTest(TestCondStream):
  def __init__(self, oracle):
    self.oracle = oracle
    p, t = P('p', POSE), P('t', TRAJ)
    super(CollisionFreeTest, self).__init__([p, t], [], [
      IsCollisionFree(p, t)
    ], eager=EAGER_TESTS)
  class StreamFn(TestStream):
    def test(self, (p, t)):
      holding = ObjGrasp(t.obj, t.pap.grasp)
      #holding = Holding(self.oracle.get_body_name(pap.geom_hash), pap.grasp)
      #assert pose.obj != holding.object_name # NOTE - they cannot be the same for this check
      if DO_MOTION:
        return not self.cond_stream.oracle.traj_holding_collision(t.pap.approach_config, t.pap.trajs, p.obj, p.value, holding)
      return not self.cond_stream.oracle.holding_collision(t.pap.grasp_config, p.obj, p.value, holding)

####################

def compile_problem(oracle):
  problem = oracle.problem

  initial_config = Config(oracle.initial_config)
  block_poses = [(C(obj, BLOCK), Pose(obj, oracle.initial_poses[obj])) for obj in oracle.get_objects()]

  #region = C('region%s'%1, REGION)
  initial_atoms = [
    AtConfig(initial_config),
    HandEmpty(), # TODO - toggle
  ] + [
    AtPose(block, pose) for block, pose in block_poses
  ]+ [
    IsPose(block, pose) for block, pose in block_poses
  ]

  goal_literals = []
  if problem.goal_holding is not None:
    if problem.goal_holding is False:
      goal_literals.append(HandEmpty())
    elif isinstance(problem.goal_holding, ObjGrasp):
      goal_literals.append(HasGrasp(C(problem.goal_holding.object_name, BLOCK), C(problem.goal_holding.grasp, GRASP)))
    elif problem.goal_holding in oracle.get_objects():
      goal_literals.append(Holding(C(problem.goal_holding, BLOCK)))
    else:
      raise Exception()
  for obj in problem.goal_poses:
    if problem.goal_poses[obj] == 'initial':
      problem.goal_poses[obj] = oracle.initial_poses[obj]
    elif problem.goal_poses[obj] in oracle.initial_poses: # Goal names other object (TODO - compile with initial)
      problem.goal_poses[obj] = oracle.initial_poses[problem.goal_poses[obj]]
    goal_literals.append(AtPose(C(obj, BLOCK), Pose(obj, problem.goal_poses[obj])))
    initial_atoms.append(IsPose(C(obj, BLOCK), Pose(obj, problem.goal_poses[obj])))
  for obj, region in problem.goal_regions.iteritems():
  #for obj, region in problem.goal_regions.items()[:1]:
    goal_literals.append(InRegion(C(obj, BLOCK), C(region, REGION)))

  # NOTE - need to add the IsPose to everything a priori

  """
  block = C(oracle.get_objects()[0], BLOCK)
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
  """

  actions = [
    Move(oracle),
    Pick(oracle),
    Place(oracle),
  ]

  axioms = [
    #HoldingAxiom(),
    InRegionAxiom(),
    SafeAxiom(),
  ]

  cond_streams = [
    PoseStream(oracle),
    GraspStream(oracle),
    ContainedStream(oracle),
    #ContainedTest(oracle),
    IKStream(oracle),
  ]
  if COLLISIONS:
    cond_streams.append(CollisionFreeTest(oracle))

  objects = [
  ]

  return STRIPStreamProblem(initial_atoms, goal_literals, actions + axioms, cond_streams, objects)


##################################################

def convert_state(oracle, pddl_state):
  world_state = {
    'holding': False
  }
  for object_name in oracle.get_objects():
    world_state[object_name] = False
  for atom in pddl_state:
    if atom.predicate == AtConfig:
      config, = atom.args
      world_state['robot'] = config.value
    elif atom.predicate == AtPose:
      obj, pose = atom.args
      world_state[obj.name] = pose.value
    elif atom.predicate == HasGrasp:
      obj, grasp = atom.args
      world_state['holding'] = ObjGrasp(obj.name, grasp.value)
  return world_state

def executable_plan(oracle, plan):
  executable = []
  for operator, args in plan: # TODO - this is all a hack
    #instance = operator.instantiate(args)
    if hasattr(operator, 'executable'):
      executable.append(operator.executable(args))
  return executable
