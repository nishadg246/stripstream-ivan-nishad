from itertools import islice

from manipulation.grasps.grasps import get_grasps
from manipulation.pick_and_place import PickAndPlace
from manipulation.primitives.placements import random_region_placements
from manipulation.grasps.grasps import Holding as ObjGrasp

from stripstream.pddl.cond_streams import CondStream
from stripstream.pddl.logic.connectives import Not
from stripstream.pddl.logic.predicates import EasyPredicate
from stripstream.pddl.operators import STRIPSAxiom, STRIPSAction
from stripstream.pddl.streams import GeneratorStream, TestStream
from stripstream.pddl.utils import get_param_names
from stripstream.pddl.problem import STRIPStreamProblem
from stripstream.pddl.objects import EasyType

# TODO - figure out how to handle types in quantifiers? Infer from use in predicates?

Ty, Pred = EasyType, EasyPredicate

ACTION_COST = 1
EAGER_TESTS = True

DO_ARM_MOTION = False
DO_COLLISIONS = False
CHECK_BASE = False
AVOID_INITIAL = False

CONF, OBJ, POSE, GRASP, TRAJ, REG = Ty(), Ty(), Ty(), Ty(), Ty(), Ty()

ConfEq = Pred(CONF)
GraspFalse = Pred()
PoseEq = Pred(OBJ, POSE)
GraspEq = Pred(OBJ, GRASP)

LegalPose = Pred(OBJ, POSE)
LegalGrasp = Pred(OBJ, GRASP)
KinCon = Pred(OBJ, POSE, GRASP, CONF, TRAJ)
CFreeCon = Pred(POSE, TRAJ)
ContainedCon = Pred(REG, POSE)

Safe = Pred(OBJ, TRAJ)
InRegion = Pred(OBJ, REG)
Holding = Pred(OBJ)
#HandEmpty = Pr() # TODO - use an axiom for this?

o, p, g, q, t = OBJ(), POSE(), GRASP(), CONF(), TRAJ()
q1, q2, r = CONF(), CONF(), REG()

####################

class EasySTRIPSAction(STRIPSAction):
  def __init__(self, conditions, effects):
    param_names = get_param_names(conditions + effects)
    super(EasySTRIPSAction, self).__init__(self.__class__.__name__, param_names.values(), conditions, effects)

class Pick(EasySTRIPSAction):
  cost = ACTION_COST
  def __init__(self, oracle):
    super(Pick, self).__init__([
      PoseEq(o, p), GraspFalse(),
      KinCon(o, p, g, q, t), ConfEq(q),
    ] + [Safe(name, t) for name in oracle.get_objects()], [
      PoseEq(o, None), GraspEq(o, g),
      Not(GraspFalse()), Not(PoseEq(o, p))])

class Place(EasySTRIPSAction):
  cost = ACTION_COST
  def __init__(self, oracle):
    super(Place, self).__init__([
      PoseEq(o, None), GraspEq(o, g),
      KinCon(o, p, g, q, t), ConfEq(q),
    ] + [Safe(name, t) for name in oracle.get_objects()], [
      PoseEq(o, p), GraspFalse(),
      Not(PoseEq(o, None)), Not(GraspEq(o, g))])

class Move(EasySTRIPSAction):
  cost = ACTION_COST
  def __init__(self, oracle):
    super(Move, self).__init__([ConfEq(q1)], [ConfEq(q2), Not(ConfEq(q1))])

####################

class WrappedCondStream(CondStream):
  def __init__(self, inputs, outputs, conditions, effects, **kwargs):
    super(WrappedCondStream, self).__init__(inputs, outputs, conditions, effects, **kwargs)

class WrappedGenStream(WrappedCondStream):
  def generator(self, *inputs): raise NotImplementedError()
  class StreamFn(GeneratorStream):
    def get_generator(self, inputs):
      values = tuple(inp.type.get_value(inp) for inp in inputs)
      #for outputs_list in self.cond_stream.generator(*values):
      #  yield list(outputs_list)
      for output_list in self.cond_stream.generator(*values):
        assert type(output_list) == list
        new_output_list = []
        for outputs in output_list:
          if type(outputs) not in [list, tuple]:
            outputs = [outputs]
          assert len(outputs) == len(self.cond_stream.outputs)
          new_output_list.append(tuple(par.type(out) for par, out in zip(self.cond_stream.outputs, outputs)))
          #new_output_list.append(tuple(outputs))
        yield new_output_list

class WrappedTestStream(WrappedCondStream):
  def test(self, *inputs): raise NotImplementedError()
  class StreamFn(TestStream):
    def test(self, inputs):
      values = tuple(inp.type.get_value(inp) for inp in inputs)
      return self.cond_stream.test(*values)

####################

class PoseStream(WrappedGenStream):
  num_samples = 1
  def __init__(self, oracle):
    self.oracle = oracle
    super(PoseStream, self).__init__([o], [p], [], [LegalPose(o, p)])
  def generator(self, o):
    oracle = self.oracle
    while True:
      if AVOID_INITIAL:
        oracle.set_all_object_poses(oracle.initial_poses)
      else:
        oracle.set_all_object_poses({o: oracle.initial_poses[o]})
      poses = random_region_placements(oracle, o, oracle.get_counters(), region_weights=True)
      #poses = cached_region_placements(self.oracle, self.obj, self.oracle.get_counters(), order=None, random=True)
      fixed_poses = []
      for pose in islice(poses, self.num_samples):
        pose.obj = o
        fixed_poses.append(pose)
      yield fixed_poses

class GraspStream(WrappedGenStream):
  def __init__(self, oracle):
    self.oracle = oracle
    super(GraspStream, self).__init__([o], [g], [], [LegalGrasp(o, g)])
  def generator(self, o):
    grasps = get_grasps(self.oracle, o)
    for grasp in grasps:
      grasp.obj = o
    yield grasps

##########

class ContainedStream(WrappedGenStream):
  num_samples = 1
  def __init__(self, oracle):
    self.oracle = oracle
    super(ContainedStream, self).__init__([o, r], [p], [], [LegalPose(o, p), ContainedCon(r, p)])
  def generator(self, o, r):
    while True:
      self.oracle.set_all_object_poses({o: self.oracle.initial_poses[o]})
      poses = random_region_placements(self.oracle, o, [r], region_weights=True)
      fixed_poses = []
      for pose in islice(poses, self.num_samples):
        pose.obj = o
        fixed_poses.append(pose)
      yield fixed_poses

# class ContainedTest(EasyTestStream):
#   def __init__(self, oracle):
#     self.oracle = oracle
#     super(ContainedTest, self).__init__([o, p, r], [], [LegalPose(o, p)], [ContainedCon(r, p)
#     ], eager=EAGER_TESTS)
#   def test(self, o, p, r):
#     return isinstance(p, Pose) and self.oracle.region_contains(r, o, p)

##########

class IKStream(WrappedGenStream):
  max_failures = 50 # 10 | 20 | 40
  max_calls = 1 # 1 | INF
  def __init__(self, oracle):
    self.oracle = oracle
    super(IKStream, self).__init__([o, p, g], [q, t], [
      LegalPose(o, p), LegalGrasp(o, g),
    ], [
      KinCon(o, p, g, q, t)])
  def generator(self, o, p, g):
    oracle = self.oracle
    oracle.set_all_object_poses({o: p}) # TODO - saver for the initial state as well?
    if oracle.approach_collision(o, p, g):
      return
    for i in range(self.max_calls):
      pap = PickAndPlace(oracle.get_geom_hash(o), p, g)
      if not pap.sample(oracle, o, max_failures=self.max_failures,
                        sample_vector=DO_ARM_MOTION, sample_arm=DO_ARM_MOTION, check_base=CHECK_BASE):
        break
      pap.obj = o
      yield [(pap.approach_config, pap)]

##########

# class CollisionFreeTest(EasyTestStream):
#   def __init__(self, oracle):
#     self.oracle = oracle
#     super(CollisionFreeTest, self).__init__([o, p, t], [], [LegalPose(o, p)], [CFreeCon(p, t)], eager=EAGER_TESTS)
#   def test(self, o, p, t):
#     if p is None or o == t.obj:
#       return True
#     holding = ObjGrasp(t.obj, t.grasp)
#     #holding = Holding(self.oracle.get_body_name(pap.geom_hash), pap.grasp)
#     if not DO_ARM_MOTION:
#       return not self.oracle.holding_collision(t.grasp_config, o, p, holding)
#     return not self.oracle.traj_holding_collision(t.approach_config, t.trajs, o, p, holding)

class CollisionFreeTest(WrappedTestStream):
  def __init__(self, oracle):
    self.oracle = oracle
    super(CollisionFreeTest, self).__init__([p, t], [], [], [CFreeCon(p, t)], eager=EAGER_TESTS)
  def test(self, p, t):
    if p is None or p.obj == t.obj:
      return True
    holding = ObjGrasp(t.obj, t.grasp)
    #holding = Holding(self.oracle.get_body_name(pap.geom_hash), pap.grasp)
    if not DO_ARM_MOTION:
      return not self.oracle.holding_collision(t.grasp_config, p.obj, p, holding)
    return not self.oracle.traj_holding_collision(t.approach_config, t.trajs, p.obj, p, holding)

####################

def compile_problem(oracle):
  problem = oracle.problem
  for obj, pose in oracle.initial_poses.iteritems(): # NOTE - would be nice to undo this
    pose.obj = obj
  for obj in problem.goal_poses:
    if problem.goal_poses[obj] == 'initial':
      problem.goal_poses[obj] = oracle.initial_poses[obj]
    elif problem.goal_poses[obj] in oracle.initial_poses: # Goal names other object (TODO - compile with initial)
      raise NotImplementedError() # TODO - don't want to overwrite obj
      #problem.goal_poses[obj] = oracle.initial_poses[problem.goal_poses[obj]]
    problem.goal_poses[obj].obj = obj

  initial_atoms = [
    ConfEq(oracle.initial_config),
    GraspFalse(), # TODO - toggle
  ]
  for obj, pose in oracle.initial_poses.iteritems():
    initial_atoms += [
      PoseEq(obj, pose), LegalPose(obj, pose), #LegalPose(obj, None)
    ]

  goal_literals = []
  if problem.goal_holding is not None:
    if problem.goal_holding is False:
      goal_literals.append(GraspFalse())
    elif isinstance(problem.goal_holding, ObjGrasp):
      goal_literals.append(GraspEq(problem.goal_holding.object_name, problem.goal_holding.grasp))
    elif problem.goal_holding in oracle.get_objects():
      goal_literals.append(Holding(problem.goal_holding))
    else:
      raise Exception()
  for obj, pose in problem.goal_poses.iteritems():
    goal_literals.append(PoseEq(obj, pose))
    initial_atoms.append(LegalPose(obj, pose))
  for obj, region in problem.goal_regions.iteritems():
    goal_literals.append(InRegion(obj, region))

  actions = [
    Move(oracle),
    Pick(oracle),
    Place(oracle),
  ]

  axioms = [
    STRIPSAxiom([GraspEq(o, g), LegalGrasp(o, g)] , [Holding(o)]),
    STRIPSAxiom([PoseEq(o, p), ContainedCon(r, p)], [InRegion(o, r)]),
    STRIPSAxiom([PoseEq(o, p), CFreeCon(p, t)] , [Safe(o, t)]),
  ]

  cond_streams = [
    PoseStream(oracle),
    GraspStream(oracle),
    ContainedStream(oracle),
    #ContainedTest(oracle),
    IKStream(oracle),
    CollisionFreeTest(oracle),
  ]

  constants = [POSE(None)]

  return STRIPStreamProblem(initial_atoms, goal_literals, actions + axioms, cond_streams, constants)


##################################################

def convert_state(oracle, pddl_state):
  world_state = {
    'holding': False
  }
  for object_name in oracle.get_objects():
    world_state[object_name] = False
  for atom in pddl_state:
    if atom.predicate == ConfEq:
      config, = atom.args
      world_state['robot'] = config.value
    elif atom.predicate == PoseEq:
      obj, pose = atom.args
      world_state[obj.name] = pose.value
    elif atom.predicate == GraspEq:
      obj, grasp = atom.args
      world_state['holding'] = ObjGrasp(obj.name, grasp.value)
  return world_state
