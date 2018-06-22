from itertools import islice

from manipulation.grasps.grasps import get_grasps
from manipulation.pick_and_place import PickAndPlace
from manipulation.primitives.placements import random_region_placements
from manipulation.grasps.grasps import Holding as ObjGrasp
from manipulation.operators.pick import Pickable, Placeable
from manipulation.primitives.transforms import trans_from_pose, pose_from_trans
from manipulation.primitives import utils
import numpy as np

from stripstream.pddl.logic.predicates import *
from stripstream.pddl.objects import Parameter, Constant
from stripstream.pddl.operators import STRIPSAction, STRIPSAxiom, Axiom
from stripstream.pddl.streams import TestCondStream, TestStream, FunctionStream, StrictStream
from stripstream.pddl.cond_streams import CondStream, TestCondStream
from stripstream.pddl.problem import STRIPStreamProblem
from stripstream.pddl.logic.predicates import HashableObject




# TODO - treat tables as things that cannot move
# TODO - how to handle type hierarchy

# Should On be a derived predicate or an explicit effect?
# Can't really do negative stacking if making them derived
# Maybe I should just derive teh positions of stuff
# I could just store the relative position allowing us to move things

# How should tables work?
# - Forall blocks on tray (conditional), at new pose
# - The new pose is something computed from after the move
# TODO - how do you handle something being supported by multiple things

BASE = False
COLLISIONS = False
DO_MOTION = False
ACTION_COST = 1
EAGER_TESTS = True

P = Parameter
C = Constant

####################

CONFIG = Type('conf')
BODY = Type('body')

GLOB_POSE = Type('glob_pose')
REL_POSE = Type('rel_pose')

GRASP = Type('grasp')
REGION = Type('region')
TRAJ = Type('traj')

####################

AtConfig = Predicate('at_config', [CONFIG])
HandEmpty = Predicate('hand_empty', [])
HasGrasp = Predicate('has_grasp', [BODY, GRASP])
Holding = Predicate('holding', [BODY])
On = Predicate('on', [BODY, BODY]) # TODO - alternatively supporting

####################

#Holding = Predicate('holding', [BLOCK]) # NOTE - used to use an axiom, but now I update it explicitly
InRegion = Predicate('in_region', [BODY, REGION])
Safe = Predicate('safe', [BODY, TRAJ])

####################

IsGrasp = Predicate('is_grasp', [BODY, GRASP])
IsKin = Predicate('is_kin', [BODY, GLOB_POSE, GRASP, CONFIG, TRAJ]) # NOTE - this ensures that the only achiever uses the block

IsContained = Predicate('is_contained', [REGION, REL_POSE]) # TODO - should I make this include the block as well?
IsCollisionFree = Predicate('is_collision_free', [GLOB_POSE, TRAJ])

####################

# TODO - probably don't even need the blocks here...
AtGlobPose = Predicate('at_glob_pose', [BODY, GLOB_POSE])
IsGlobPose = Predicate('is_glob_pose', [BODY, GLOB_POSE])

AtRelPose = Predicate('at_rel_pose', [BODY, REL_POSE, BODY])
IsRelPose = Predicate('is_rel_pose', [BODY, REL_POSE, BODY])

# TODO - should I make the distinction between relative and global?

IsPickable = Predicate('is_pickable', [BODY])
IsPushable = Predicate('is_pushable', [BODY])
# TODO - attributes for whether something can be placed on something else (i.e. table, floor, etc...)
# TODO - fixed pose and temporary pose

AreStackable = Predicate('are_stackable', [BODY, BODY])

IsPoseProd = Predicate('is_pose_prod', [GLOB_POSE, REL_POSE, GLOB_POSE])


AreNearby = Predicate('are_nearby', [GLOB_POSE, CONFIG])

IsTop = Predicate('is_top', [BODY, REL_POSE])
IsBase = Predicate('is_base', [REL_POSE, BODY])

IsGraspable = Predicate('is_graspable', [BODY])

#IsGlobal = Predicate('is_global', [BODY])
#floor = C('floor', BODY)

AtFixedPose = Predicate('at_fixed_pose', [BODY, GLOB_POSE])

# TODO - if on floor then global and local

class Config(Constant):
  dictionary = {}
  def __init__(self, config):
    if config not in self.dictionary:
      self.dictionary[config] = 'q%s'%len(self.dictionary)
    super(Config, self).__init__(self.dictionary[config], CONFIG)
    self.value = config

# TODO - do this with object geometries

class GlobPose(Constant):
  dictionary = {}
  def __init__(self, obj, pose, parent_poses):
    if (obj, pose) not in self.dictionary:
      self.dictionary[(obj, pose)] = 'p%s'%len(self.dictionary) # TODO - include obj?
    super(GlobPose, self).__init__(self.dictionary[(obj, pose)], GLOB_POSE)
    self.obj = obj
    self.value = pose
    self.parent_poses = parent_poses

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

class RelPose(HashableObject): type = REL_POSE; prefix = 'rp'; value_repr = False

####################

def collision_conditions(oracle, b, t):
  if not COLLISIONS:
    return []
  return [Safe(C(name, BODY), t) for name in oracle.get_objects()]
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

# TODO - note need global pose here to do IK
# NOTE - also need relative pose for IK
# TODO - what if circular axiom where it keep producing new relative placements on top of itself

# NOTE - we don't need the bs pose here because we know we derived the correct globpose
class Pick(STRIPSAction):
  cost = ACTION_COST
  def __init__(self, oracle):
    b, p, g, q, t = P('b', BODY), P('p', GLOB_POSE), P('g', GRASP), P('q', CONFIG), P('t', TRAJ)
    rp, bs = P('rp', REL_POSE), P('bs', BODY)
    conditions = [
      AreStackable(b, bs),
      IsKin(b, p, g, q, t), # Implies IsPose, IsGrasp, and IsGraspable
      # TODO - there are even more of these I could put down...

      AtGlobPose(b, p),
      AtRelPose(b, rp, bs),
      HandEmpty(),
    ] + collision_conditions(oracle, b, t)
    if BASE:
      conditions.append(AtConfig(q))
    effects = [
      HasGrasp(b, g),
      Not(HandEmpty()),
      Not(AtRelPose(b, rp, bs)),
    ]
    super(Pick, self).__init__(self.__class__.__name__, [b, p, g, q, t, rp, bs], conditions, effects)
  def executable(self, (b, p, g, q, t, rp, bs)): # TODO - rethink this
    return ExecutablePick(b.name, t.pap)

class ExecutablePlace(Placeable):
  def __init__(self, object_name, pap):
    self.object_name = object_name
    self.pap = pap

# TODO - how do I handle placing because it won't yet be at the target pose?

class Place(STRIPSAction):
  cost = ACTION_COST
  def __init__(self, oracle):
    params = (P('b', BODY), P('p', GLOB_POSE), P('g', GRASP), P('q', CONFIG), P('t', TRAJ),
              P('rp', REL_POSE), P('bs', BODY), P('p2', GLOB_POSE))
    b, p, g, q, t, rp, bs, p2 = params
    conditions = [
      HasGrasp(b, g),
      IsKin(b, p, g, q, t), # Implies IsPose, IsGrasp, and IsGraspable

      AreStackable(b, bs),
      AtGlobPose(bs, p2),
      IsPoseProd(p, rp, p2),
    ] + collision_conditions(oracle, b, t)
    if BASE:
      conditions.append(AtConfig(q))
    effects = [
      AtRelPose(b, rp, bs),
      HandEmpty(),
      Not(HasGrasp(b, g)),
    ]
    super(Place, self).__init__(self.__class__.__name__, params, conditions, effects)
  def executable(self, (b, p, g, q, t)): # TODO - rethink this
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

class HoldingAxiom(STRIPSAxiom):
  def __init__(self):
    b, g = P('b', BODY), P('g', GRASP)
    super(HoldingAxiom, self).__init__([
        HasGrasp(b, g),
        IsGrasp(b, g),
      ] , [
        Holding(b),
    ])

# b1 on b2

# TODO - if at local ref is floor

class OnAxiom(STRIPSAxiom):
  def __init__(self):
    b1, rp, b2, = P('b1', BODY), P('rp', REL_POSE), P('b2', BODY)
    super(OnAxiom, self).__init__([
        #IsTop(b1, rp),
        #IsBase(rp, b2),
        AtRelPose(b1, rp, b2),
        IsRelPose(b1, rp, b2),
        #AreStackable(b1, b2),
      ] , [
        On(b1, b2),
    ])

"""
class PoseAxiom(STRIPSAxiom):
  def __init__(self):
    b1, p1, b2, rp, p2 = P('b1', BODY), P('p1', GLOB_POSE), P('b2', BODY), P('rp', GLOB_POSE), P('p1', GLOB_POSE)
    super(PoseAxiom, self).__init__([
        AtGlobPose(b1, p1),
        AtRelPose(b2, rp, b1),
        IsPoseProd(p1, rp, p2),
      ] , [
        AtGlobPose(b2, p2),
    ])
"""

# TODO - do I need b2 in AtRelPose? Yes, otherwise won't know which object it is on

# Fixed global pose
class PoseAxiom(Axiom):
  def __init__(self):
    b1, p1 = P('b1', BODY), P('p1', GLOB_POSE)
    rp, b2, p2 = P('rp', REL_POSE), P('b2', BODY), P('p2', GLOB_POSE)
    Axiom.__init__(self, AtGlobPose(b1, p1),
      Or(AtFixedPose(b1, p1),
        Exists([rp, b2, p2], And(
          IsPoseProd(p1, rp, p2),
          AtGlobPose(b2, p2),
          AtRelPose(b1, rp, b2)))))

class InRegionAxiom(STRIPSAxiom):
  def __init__(self):
    b, rp, bs, r = P('b', BODY), P('rp', REL_POSE), P('b', BODY), P('r', REGION)
    super(InRegionAxiom, self).__init__([
        IsTop(b, rp),
        IsBase(rp, bs),
        AtRelPose(b, rp, bs),
        IsContained(r, rp),
        #IsOn(r, bs),
      ] , [
        InRegion(b, r),
    ])

class SafeAxiom(Axiom): # TODO - it computes self collisions here when picking up an object...
  def __init__(self):
    b, p, t = P('b', BODY), P('p', GLOB_POSE), P('t', TRAJ)
    Axiom.__init__(self, Safe(b, t),
      Or(Holding(b), # TODO - could also have equality here
        Exists([p], And(
          AtGlobPose(b, p),
          #Not(Equal(p1, p2)), # TODO - I could always immediately outlaw them having the same pose...
          IsCollisionFree(p, t)))))

####################

class PoseStream(CondStream):
  def __init__(self, oracle):
    self.oracle = oracle
    b, rp, bs = P('b', BODY), P('rp', REL_POSE), P('bs', BODY)
    super(PoseStream, self).__init__([b, bs], [rp], [
      AreStackable(b, bs),
    ], [
      IsRelPose(b, rp, bs),
      #IsTop(b, rp),
      #IsBase(rp, bs),
    ])
  class StreamFn(FunctionStream): # TODO - maybe find a better way of doing this?
    num_samples = 2
    avoid_initial = True
    max_failures = 20 # INF
    def function(self, (b, bs)):
      oracle = self.cond_stream.oracle
      with oracle.state_saver():
        if self.avoid_initial:
          oracle.set_all_object_poses(oracle.initial_poses)
        else:
          oracle.set_all_object_poses({b.name: oracle.initial_poses[b.name]})
        assert bs.name in oracle.surface_poses

        poses = random_region_placements(oracle, b.name, [bs.name], region_weights=True, max_failures=self.max_failures)
        #poses = cached_region_placements(self.oracle, self.obj, self.oracle.get_counters(), order=None, random=True)
        r_pose = oracle.surface_poses[bs.name]
        combos = []
        for pose in islice(poses, self.num_samples):
          rel_pose = utils.Pose(pose_from_trans(np.dot(trans_from_pose(pose.value),
                                np.linalg.inv(trans_from_pose(r_pose.value))))) # p = r * t
          combos.append((RelPose((b.name, rel_pose, bs.name)),))
        return combos
        #return [(Pose(b.name, pose),) for pose in islice(poses, self.num_samples)]

class GraspStream(CondStream):
  def __init__(self, oracle):
    self.oracle = oracle
    b, g = P('b', BODY), P('g', GRASP)
    super(GraspStream, self).__init__([b], [g], [
      IsGraspable(b),
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
    b, rp, bs, r = P('b', BODY), P('rp', REL_POSE), P('bs', BODY), P('r', REGION)
    super(ContainedStream, self).__init__([b, r], [rp], [
    ], [
      #IsPoseProd(b, rp, bs),
      #IsPose(b, p),
      IsTop(b, rp),
      IsBase(rp, bs),
      IsContained(r, rp),
    ])
  class StreamFn(FunctionStream): # TODO - maybe find a better way of doing this?
    num_samples = 2
    def function(self, (b, r)): # TODO - switch this to be a generator
      poses = random_region_placements(self.cond_stream.oracle, b.name, [r.name], region_weights=True) # TODO - save this generator
      return [(GlobPose(b.name, pose),) for pose in islice(poses, self.num_samples)]

"""
# NOTE - I can store the relative pose of the region and use it here
class ContainedTest(TestCondStream):
  def __init__(self, oracle):
    self.oracle = oracle
    rp, bs, r = P('rp', REL_POSE), P('bs', BODY), P('r', REGION) # TODO - should I put the object here?
    super(ContainedTest, self).__init__([rp, bs, r], [
      IsBase(bs, rp),
      IsRegionOn(bs, r),
    ], [
      IsContained(r, rp)
    ], eager=EAGER_TESTS)
  class StreamFn(TestStream):
    def test(self, (rp, bs, r)):
      b1, p, b2 = rp.value
      if b2.value != self.cond_stream.oracle.region_supporters[r.value]:
        return False # TODO - can always return no if the relative pose is on the wrong table
      return self.cond_stream.oracle.region_contains(r.name, p.obj, p.value)

class ContainedTest2(TestCondStream):
  def __init__(self, oracle):
    self.oracle = oracle
    rp, r = P('rp', REL_POSE), P('r', REGION) # TODO - should I put the object here?
    super(ContainedTest, self).__init__([rp, r], [
    ], [
      IsContained(r, rp)
    ], eager=EAGER_TESTS)
  class StreamFn(TestStream):
    def test(self, (rp, r)):
      b1, p, b2 = rp.value
      if b2.value != self.cond_stream.oracle.region_supporters[r.value]:
        return False # TODO - can always return no if the relative pose is on the wrong table
      return self.cond_stream.oracle.region_contains(r.name, p.obj, p.value)
"""

##########

class IKStream(CondStream):
  def __init__(self, oracle):
    self.oracle = oracle
    b, p, g, q, t = (P('b', BODY), P('p', GLOB_POSE), P('g', GRASP), P('q', CONFIG), P('t', TRAJ))
    super(IKStream, self).__init__([b, p, g], [q, t], [
      IsGlobPose(b, p),
      IsGrasp(b, g),
    ], [
      IsKin(b, p, g, q, t),
    ], copies=1) # 2 | 6
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
    p, t = P('p', GLOB_POSE), P('t', TRAJ)
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

##########

class PoseProdStream(CondStream):
  def __init__(self, oracle):
    self.oracle = oracle
    o1 = P('o1', BODY)
    p1, rp, o2, p2 = P('p1', GLOB_POSE), P('rp', REL_POSE), P('o2', BODY), P('p2', GLOB_POSE)
    #p1, rp, p2 = P('p1', GLOB_POSE), P('rp', REL_POSE), P('p2', GLOB_POSE)
    super(PoseProdStream, self).__init__([o1, rp, o2, p2], [p1], [
      # TODO - assert that rp and p2 match?
      #IsBase(rp, o2),
      IsGlobPose(o2, p2),
      IsRelPose(o1, rp, o2),
    ], [
      #IsPose(o1, p1),
      IsGlobPose(o1, p1),
      IsPoseProd(p1, rp, p2), # TODO - assert that the objects match up
    ], eager=True)
  class StreamFn(FunctionStream):
    num_samples = 2
    def function(self, (o1, rp, o2, p2)):
      g1, p, g2 = rp.value
      assert g1 == o1.name and g2 == o2.name == p2.obj
      print o1, rp.value, o2, p2, p2.value
      # TODO - fail to make a pose if it goes past a max stacked height
      #if o2 != p2.obj: return []
      gp = utils.Pose(pose_from_trans(np.dot(trans_from_pose(p.value),
                                             trans_from_pose(p2.value.value))))
      return [(GlobPose(o1, gp, (rp, p2)),)]

####################

# TODO - for trays, can assume an optimistic collision region on top of them

def compile_problem(oracle):
  problem = oracle.problem

  #oracle.surface_poses = {surface: oracle.get_pose(surface) for surface in oracle.get_surfaces()}
  oracle.surface_poses = {surface: oracle.get_pose(surface) for surface in oracle.get_counters()}
  #floor = C('floor', BODY)

  initial_atoms = [
    AtConfig(Config(oracle.initial_config)),
    HandEmpty(), # TODO - toggle
    #IsGlobal(floor),
  ] + [
    #AtGlobPose(block, o_pose) for block, o_pose in block_poses
  ] + [
    IsGraspable(C(obj, BODY)) for obj in oracle.get_objects()
    #IsPose(block, o_pose) for block, o_pose in block_poses # NOTE - need to add the IsPose to everything a priori
  ] + [
    #AreStackable(C(obj, BODY), floor) for obj in oracle.get_floor_objects()
  ] + [
    AreStackable(C(obj, BODY), C(count, BODY)) for obj in oracle.get_counter_objects() for count in oracle.get_counters()
  ] + [
    #AtGlobPose(C(count, BODY), Pose(count, surface_poses[count])) for count in oracle.get_counters()
    #AtRelPose(C(count, BODY), RelPose((count, surface_poses[count], floor.name)), floor)
    #  for count in oracle.get_counters()
  ]

  for count in oracle.get_counters():
    body, pose = C(count, BODY), GlobPose(count, oracle.surface_poses[count], None)
    initial_atoms += [
      AtFixedPose(body, pose),
      IsGlobPose(body, pose),
    ] # NOTE - oracle.initial_poses[obj] != oracle.get_pose(obj)

  for obj in oracle.get_objects():
    o_pose = oracle.initial_poses[obj]
    supported = False
    for surface in oracle.get_surfaces():
      region = oracle.get_region(surface)
      if region.on(oracle.get_aabb(obj, trans_from_pose(o_pose.value))):
        assert not supported
        supported = True
        r_pose = oracle.surface_poses[surface]
        rel_pose = utils.Pose(pose_from_trans(np.dot(trans_from_pose(o_pose.value),
                              np.linalg.inv(trans_from_pose(r_pose.value))))) # p = r * t
        initial_atoms += [
          AtRelPose(C(obj, BODY), RelPose((obj, rel_pose, surface)), C(surface, BODY)),
          IsRelPose(C(obj, BODY), RelPose((obj, rel_pose, surface)), C(surface, BODY)),
          #IsPoseProd(Pose(obj, o_pose), RelPose((obj, rel_pose, surface)), Pose(surface, r_pose)),
          #IsTop(C(obj, BODY), RelPose((obj, rel_pose, surface))),
          #IsBase(RelPose((obj, rel_pose, surface)), C(surface, BODY)),
          #IsGlobPose(C(obj, BODY), Pose(obj, o_pose)),
        ]
        print obj, region
    assert supported

  oracle.region_supporters = {}
  for region in oracle.goal_regions:
    for surface in oracle.get_surfaces():
      if oracle.get_region(surface).region_on(oracle.get_region(region)):
        assert region not in oracle.region_supporters
        rel_pose = utils.Pose(pose_from_trans(np.dot(trans_from_pose(oracle.get_region(surface).pose()),
                              np.linalg.inv(trans_from_pose(oracle.surface_poses[surface].value)))))
        oracle.region_supporters[region] = (surface, rel_pose)
        print region, surface
    assert region in oracle.region_supporters

  goal_literals = []
  if problem.goal_holding is not None:
    if problem.goal_holding is False:
      goal_literals.append(HandEmpty())
    elif isinstance(problem.goal_holding, ObjGrasp):
      goal_literals.append(HasGrasp(C(problem.goal_holding.object_name, BODY), C(problem.goal_holding.grasp, GRASP)))
    elif problem.goal_holding in oracle.get_objects():
      goal_literals.append(Holding(C(problem.goal_holding, BODY)))
    else:
      raise Exception()
  """
  for obj in problem.goal_poses:
    if problem.goal_poses[obj] == 'initial':
      problem.goal_poses[obj] = oracle.initial_poses[obj]
    elif problem.goal_poses[obj] in oracle.initial_poses: # Goal names other object (TODO - compile with initial)
      problem.goal_poses[obj] = oracle.initial_poses[problem.goal_poses[obj]]
    goal_literals.append(AtGlobPose(C(obj, BODY), GlobPose(obj, problem.goal_poses[obj])))
    initial_atoms.append(IsGlobPose(C(obj, BODY), GlobPose(obj, problem.goal_poses[obj])))
  """
  for obj, region in problem.goal_regions.iteritems():
    goal_literals.append(InRegion(C(obj, BODY), C(region, REGION)))

  # TODO - how do we handle regions then?
  # Local to regions? Regions can't move anyways... Or can they? Pushing the table or something...
  # TODO - should regions be relative or fixed
  #goal_literals = [Holding(C('blue_box', BODY))]
  #goal_literals = [AtGlobPose(C('table1', BODY), Pose('table1', surface_poses['table1']))]
  goal_literals = [On(C('blue_box', BODY), C('table2', BODY))]

  actions = [
    Pick(oracle),
    Place(oracle),
  ]

  axioms = [
    HoldingAxiom(),
    PoseAxiom(),
    #InRegionAxiom(),
    OnAxiom(),
  ]

  cond_streams = [
    PoseProdStream(oracle),
    PoseStream(oracle),
    GraspStream(oracle),
    #ContainedStream(oracle),
    #ContainedTest(oracle),
    IKStream(oracle),
  ]
  if DO_MOTION:
    actions.append(Move(oracle))
  if COLLISIONS:
    cond_streams.append(CollisionFreeTest(oracle))
    axioms.append(SafeAxiom())

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
    elif atom.predicate == AtGlobPose:
      obj, pose = atom.args
      world_state[obj.name] = pose.value
    elif atom.predicate == HasGrasp:
      obj, grasp = atom.args
      world_state['holding'] = ObjGrasp(obj.name, grasp.value)
  return world_state
