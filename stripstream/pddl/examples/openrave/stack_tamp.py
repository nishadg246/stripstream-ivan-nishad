from itertools import islice

from manipulation.grasps.grasps import get_grasps
from manipulation.pick_and_place import PickAndPlace
from manipulation.primitives.placements import random_region_placements, center_stackings
from manipulation.grasps.grasps import Holding as ObjGrasp
from manipulation.operators.pick import Pickable, Placeable
from manipulation.primitives.transforms import trans_from_pose
from manipulation.bodies.bounding_volumes import aabb_stacked
from manipulation.constants import ACTIVE_LEFT, ACTIVE_RIGHT

from stripstream.pddl.logic.predicates import *
from stripstream.pddl.objects import Parameter, Constant
from stripstream.pddl.operators import STRIPSAction, STRIPSAxiom, Action, Axiom
from stripstream.pddl.streams import TestCondStream, Stream, TestStream, FunctionStream
from stripstream.pddl.cond_streams import CondStream, TestCondStream
from stripstream.pddl.problem import STRIPStreamProblem

BASE = False
COLLISIONS = False
DO_MOTION = False
ACTION_COST = 1
EAGER_TESTS = True

P = Parameter
C = Constant

####################

CONF = Type('conf')
MANIP = Type('manip') # left | right
BODY = Type('body')
POSE = Type('pose')
GRASP = Type('grasp')
REGION = Type('region')
TRAJ = Type('traj')

####################

AtConfig = Predicate('at_config', [CONF])
HandEmpty = Predicate('hand_empty', [MANIP])
AtPose = Predicate('at_pose', [BODY, POSE]) # TODO - probably don't even need block here...
HasGrasp = Predicate('has_grasp', [MANIP, BODY, GRASP])

####################

Holding = Predicate('holding', [BODY])
InRegion = Predicate('in_region', [BODY, REGION])
Safe = Predicate('safe', [BODY, TRAJ])

####################

IsPose = Predicate('is_pose', [BODY, POSE])
IsGrasp = Predicate('is_grasp', [BODY, GRASP])
IsKin = Predicate('is_kin', [BODY, POSE, MANIP, GRASP, CONF, TRAJ])

IsContained = Predicate('is_contained', [REGION, POSE]) # TODO - should I make this include the block as well?
IsCollisionFree = Predicate('is_collision_free', [POSE, TRAJ])

####################

On = Predicate('on', [BODY, BODY]) # NOTE - Could just infer this from them both being at their poses and supported
IsSupported = Predicate('is_supported', [BODY, POSE, BODY, POSE])
IsGraspable = Predicate('is_graspable', [BODY])
IsFixed = Predicate('is_fixed', [BODY])
AreStackable = Predicate('are_stackable', [BODY, BODY])

IsRegionOn = Predicate('is_region_on', [REGION, BODY])

####################

AreNearby = Predicate('are_nearby', [BODY, POSE, GRASP, CONF])
#AreNearby = Predicate('are_nearby', [POSE, CONF])

IsPush = Predicate('is_push', [BODY, POSE, POSE, CONF, TRAJ])

####################

class Config(Constant):
  dictionary = {}
  def __init__(self, config):
    if config not in self.dictionary:
      self.dictionary[config] = 'q%s'%len(self.dictionary)
    super(Config, self).__init__(self.dictionary[config], CONF)
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

# NOTE - could implement safe by quantifying over poses instead
def collision_conditions(oracle, b, t):
  if not COLLISIONS:
    return []
  #return [Safe(C(name, BLOCK), t) for name in oracle.get_objects()]
  ob = P('ob', BODY)
  #return [ForAll([ob], Safe(ob, t))]
  return [ForAll([ob], Or(IsFixed(ob), Safe(ob, t)))]
  #return [ForAll([ob], Or(Equal(b, ob), Safe(ob, t)))]
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

class Pick(Action):
  cost = ACTION_COST
  def __init__(self, oracle):
    b, p, g, q, t = P('b', BODY), P('p', POSE), P('g', GRASP), P('q', CONF), P('t', TRAJ)
    m = P('m', MANIP)
    #bs = P('bs', BODY)
    #ob = P('ob', BODY)
    conditions = [
      AtPose(b, p),
      HandEmpty(m),
      IsKin(b, p, m, g, q, t), # Implies IsPose, IsGrasp, and IsGraspable

      #AreStackable(b, bs),
      #On(b, bs),
      #ForAll([ob], Not(On(ob, b))), # NOTE - negative precondition
    ] + collision_conditions(oracle, b, t)
    if BASE:
      conditions.append(AtConfig(q))
    effects = [
      HasGrasp(m, b, g),
      Not(HandEmpty(m)),
      Not(AtPose(b, p)),
      #Not(On(b, bs)),
    ]
    super(Pick, self).__init__(self.__class__.__name__, [b, p, m, g, q, t],
                               And(*conditions), And(*effects))
  def executable(self, (b, p, m, g, q, t)): # TODO - rethink this
    return ExecutablePick(b.name, t.pap)

class ExecutablePlace(Placeable):
  def __init__(self, object_name, pap):
    self.object_name = object_name
    self.pap = pap

class Place(Action):
  cost = ACTION_COST
  def __init__(self, oracle):
    b, p, g, q, t = P('b', BODY), P('p', POSE), P('g', GRASP), P('q', CONF), P('t', TRAJ)
    m = P('m', MANIP)
    bs, ps = P('bs', BODY), P('ps', POSE)
    conditions = [
      HasGrasp(m, b, g),
      IsKin(b, p, m, g, q, t), # Implies IsPose, IsGrasp, and IsGraspable

      AtPose(bs, ps),
      IsPose(bs, ps),
      IsSupported(b, p, bs, ps), # Implies are stackable
    ] + collision_conditions(oracle, b, t)
    if BASE:
      conditions.append(AtConfig(q))
    effects = [
      AtPose(b, p),
      HandEmpty(m),
      #On(b, bs),
      Not(HasGrasp(m, b, g)),
    ]
    super(Place, self).__init__(self.__class__.__name__, [b, p, m, g, q, t, bs, ps],
                                And(*conditions), And(*effects))
  def executable(self, (b, p, m, g, q, t, bs, ps)): # TODO - rethink this
    return ExecutablePlace(b.name, t.pap)

#class ExecutablePlace(Moveable):
#  def __init__(self, object_name, pap):
#    pass

####################

class Move(STRIPSAction):
  cost = ACTION_COST
  def __init__(self, oracle):
    params = (P('q1', CONF), P('q2', CONF))
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

# TODO - should I decompose into separate actions for the approach and retreat?

class Push(Action):
  cost = ACTION_COST
  def __init__(self, oracle):
    b, p1, p2, q, t = P('b', BODY), P('p1', POSE), P('p2', POSE), P('q', CONF), P('t', TRAJ)
    conditions = [
      HandEmpty(),
      IsPush(b, p1, p2, q, t),
    ] #+ collision_conditions(oracle, b, t) # TODO - fill these in
    if BASE:
      conditions.append(AtConfig(q))
    effects = [
      AtPose(b, p2),
      Not(AtPose(b, p2)),
    ]
    super(Push, self).__init__(self.__class__.__name__, [b, p1, p2, q, t],
                                And(*conditions), And(*effects))
  def executable(self, (b, p, g, q, t, bs, ps)): # TODO - rethink this
    pass
    #return ExecutablePlace(b.name, t.pap)

####################

class HoldingAxiom(STRIPSAxiom):
  def __init__(self):
    m, b, g = P('m', MANIP), P('b', BODY), P('g', GRASP)
    super(HoldingAxiom, self).__init__([
        HasGrasp(m, b, g),
        IsGrasp(b, g),
      ] , [
        Holding(b),
    ])

# NOTE - this can either be an axiom or explicitly updated
class OnAxiom(STRIPSAxiom):
  def __init__(self):
    b, p, bs, ps = P('b', BODY), P('p', POSE), P('bs', BODY), P('ps', POSE)
    super(OnAxiom, self).__init__([
        AtPose(b, p),
        AtPose(bs, ps),
        IsSupported(b, p, bs, ps), # Implies AreStackable, IsPose, ...
      ] , [
        On(b, bs),
    ])

"""
class ClearAxiom(Axiom):
  def __init__(self):
    b, p = P('b', BODY), P('p', POSE)
    Axiom.__init__(self, Clear(b, p), # TODO - should the pose be included
    ForAll([o], Exists([po], And(
      AtPose(b, p),
      Not(IsSupported(o, po, p, p))))))
"""

class InRegionAxiom(STRIPSAxiom):
  def __init__(self):
    params = (P('b', BODY), P('p', POSE), P('r', REGION))
    b, p, r = params
    super(InRegionAxiom, self).__init__([
        AtPose(b, p),
        IsContained(r, p),
      ] , [
        InRegion(b, r),
    ])

# NOTE - alternatively, I could just make things in collision if stacked
class SafeAxiom(Axiom): # TODO - it computes self collisions here when picking up an object...
  def __init__(self):
    b, p, t = P('b', BODY), P('p', POSE), P('t', TRAJ)
    Axiom.__init__(self, Safe(b, t),
      Or(Holding(b), # TODO - could also have equality here
        Exists([p], And(
          AtPose(b, p),
          #Not(IsSupported(b, p, t.o, t.p)), TODO - stacking
          #Not(On(ob, b)),
          #Not(Equal(p1, p2)), # TODO - I could always immediately outlaw them having the same pose...
          IsCollisionFree(p, t)))))

####################

# TODO - a generic place on environment object (with the unit transform) which spawns predicates identifying objects it rests on
class PoseStream(CondStream):
  def __init__(self, oracle):
    self.oracle = oracle
    b, p = P('b', BODY), P('p', POSE)
    bs, ps = P('bs', BODY), P('ps', POSE)
    super(PoseStream, self).__init__([b, bs, ps], [p], [
      IsPose(bs, ps),
      AreStackable(b, bs), # TODO - make a fixed pose thing here?
    ], [
      IsPose(b, p),
      IsSupported(b, p, bs, ps),
    ])
  class StreamFn(FunctionStream): # TODO - maybe find a better way of doing this?
    num_samples = 2
    avoid_initial = True
    max_failures = 20 # INF
    # TODO - make the cost of the stream depend on the object below
    def function(self, (b, bs, ps)):
      oracle = self.cond_stream.oracle
      with oracle.state_saver():
        if self.avoid_initial:
          oracle.set_all_object_poses(oracle.initial_poses)
        else:
          oracle.set_all_object_poses({b.name: oracle.initial_poses[b.name]})
        if bs.name in oracle.get_counters():
          poses = random_region_placements(oracle, b.name, [bs.name], region_weights=True, max_failures=self.max_failures)
          #poses = cached_region_placements(self.oracle, self.obj, [bs.name], order=None, random=True)
        else:
          poses = center_stackings(oracle, b.name, [(bs.name, ps.value)], max_failures=self.max_failures)
        return [(Pose(b.name, pose),) for pose in islice(poses, self.num_samples)]

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

# TODO - reformat this with an "intentional" effect of just IsContained(r, p)
class ContainedStream(CondStream):
  def __init__(self, oracle):
    self.oracle = oracle
    b, p, r = P('b', BODY), P('p', POSE), P('r', REGION)
    bs, ps = P('bs', BODY), P('ps', POSE)
    super(ContainedStream, self).__init__([b, r, bs, ps], [p], [
      #IsGraspable(b),
      #Not(IsFixed(b)),
      IsPose(bs, ps),
      IsRegionOn(r, bs),
      AreStackable(b, bs), # TODO - make a fixed pose thing here?
    ], [
      IsPose(b, p),
      IsContained(r, p), # TODO - should probably put ps here...
      IsSupported(b, p, bs, ps),
    ])
  class StreamFn(FunctionStream): # TODO - maybe find a better way of doing this?
    num_samples = 2
    def function(self, (b, r, bs, bp)): # TODO - switch this to be a generator
      #print b, r, bs, bp
      #raw_input('Pause')
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
    b, p, g, q, t = P('b', BODY), P('p', POSE), P('g', GRASP), P('q', CONF), P('t', TRAJ)
    m = P('m', MANIP)
    super(IKStream, self).__init__([b, p, m, g], [q, t], [
      IsPose(b, p),
      IsGrasp(b, g),
    ], [
      IsKin(b, p, m, g, q, t),
    ])
  class StreamFn(Stream): # TODO - handle oracle
    max_failures = 50 # 10 | 20 | 40
    max_calls = 1 # 1 | INF
    def get_values(self, **kwargs):
      oracle = self.cond_stream.oracle
      b, p, m, g = self.inputs
      manip_name = m.name + 'arm'
      oracle.robot.SetActiveManipulator(manip_name) # TODO - reflect base pose for rightarm (although right now it is fairly symmetric
      #oracle.robot.GetActiveManipulator().SetIkSolver(oracle.ikmodels[manip_name].iksolver)
      #ikmodel = databases.inversekinematics.InverseKinematicsModel(robot=self.robot, iktype=IkParameterization.Type.Transform6D,
      #    forceikfast=True, freeindices=None, freejoints=None, manip=self.robot.GetManipulator(name))

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
        q, t = Config(pap.approach_config), Trajectory(b.name, pap)
        initial_atoms = []
        for obj in oracle.get_objects():
          if b.name != obj and obj not in oracle.introduced_objects:
            block, pose = C(obj, BODY), Pose(obj, oracle.initial_poses[obj])
            if are_colliding(pose, t, oracle):
              initial_atoms += get_object_initial_atoms(obj, oracle)
              #initial_atoms += [AtPose(block, pose), IsPose(block, pose)]
        return [IsKin(b, p, m, g, q, t)] + initial_atoms
        #return [q, t, IsIK(b, p, g, q, t)]

##########

def are_colliding(p, t, oracle):
  if p.obj == t.obj:
    return False
  holding = ObjGrasp(t.obj, t.pap.grasp)
  #holding = Holding(self.oracle.get_body_name(pap.geom_hash), pap.grasp)
  #assert pose.obj != holding.object_name # NOTE - they cannot be the same for this check
  #if aabb_stacked(aabb_from_body(oracle.get_body(p.obj)), aabb_from_body(oracle.get_body(t.obj))):
  #  return True
  if aabb_stacked(oracle.get_aabb(p.obj, trans_from_pose(p.value.value)),
                  oracle.get_aabb(t.obj, trans_from_pose(t.pap.pose.value))):
    return True
  if DO_MOTION:
    return oracle.traj_holding_collision(t.pap.approach_config, t.pap.trajs, p.obj, p.value, holding)
  return oracle.holding_collision(t.pap.grasp_config, p.obj, p.value, holding)

class CollisionFreeTest(TestCondStream):
  def __init__(self, oracle):
    self.oracle = oracle
    p, t = P('p', POSE), P('t', TRAJ)
    super(CollisionFreeTest, self).__init__([p, t], [], [
      IsCollisionFree(p, t)
    ], eager=EAGER_TESTS)
  class StreamFn(TestStream):
    def test(self, (p, t)):
      return not are_colliding(p, t, self.cond_stream.oracle)

####################

from manipulation.inverse_reachability.inverse_reachability import is_possible_fr_trans
from manipulation.bodies.robot import manip_trans_from_object_trans
from manipulation.primitives.transforms import trans_from_full_config

class NearbyTest(TestCondStream):
  def __init__(self, oracle):
    self.oracle = oracle
    b, p, g, q = P('b', BODY), P('p', POSE), P('g', GRASP), P('q', CONF)
    super(NearbyTest, self).__init__([b, p, g, q], [
      IsPose(b, p),
      IsGrasp(b, g),
    ], [
      AreNearby(b, p, g, q),
    ], eager=EAGER_TESTS)
  class StreamFn(TestStream):
    def test(self, (b, p, g, q)):
      object_trans = trans_from_pose(p.value.value)
      manip_trans = manip_trans_from_object_trans(object_trans, g.value.grasp_trans)
      base_trans = trans_from_full_config(q.value.value)
      answer = is_possible_fr_trans(self.cond_stream.oracle, base_trans, manip_trans)
      return answer
      #distance = np.linalg.norm((point_from_trans(base_trans) - point_from_trans(point_from_pose(p.value)))[:2])
      #return distance <= self.cond_stream.oracle.reachability_radius # NOTE - this is technically distance to manip_trans

# TODO - I really should just sample base values here and then use those for a generic test for IK

####################

# TODO - push aside, push to edge, push to point

# Block trajectory
# Approach and retreat trajectories
# Base pose
# TODO - do I need a separate trajectory object?

# TODO - object collisions to determine whether I actually want to do something

class PushStream(CondStream):
  def __init__(self, oracle):
    self.oracle = oracle
    b, p1, p2, q, t = P('b', BODY), P('p1', POSE), P('p2', POSE), P('q', CONF), P('t', TRAJ)
    #bs, ps = P('bs', BODY), P('ps', POSE)
    #super(PushStream, self).__init__([b, p1, p2], [q, t], [ # Produce moving in any direction
    super(PushStream, self).__init__([b, p1], [p2, q, t], [ # Produce trajectory moving in some direction

      IsPose(b, p1),
      IsPose(b, p2),
      #IsSupported(b, p1, bs, ps),
      #IsSupported(b, p2, bs, ps),
    ], [
      IsPush(b, p1, p2, q, t),
    ])
  class StreamFn(Stream): # TODO - handle oracle
    max_failures = 50 # 10 | 20 | 40
    max_calls = 1 # 1 | INF
    def get_values(self, **kwargs):
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
        q, t = Config(pap.approach_config), Trajectory(b.name, pap)
        initial_atoms = []
        for obj in oracle.get_objects():
          if b.name != obj and obj not in oracle.introduced_objects:
            block, pose = C(obj, BODY), Pose(obj, oracle.initial_poses[obj])
            if are_colliding(pose, t, oracle):
              initial_atoms += get_object_initial_atoms(obj, oracle)
              #initial_atoms += [AtPose(block, pose), IsPose(block, pose)]
        return [IsKin(b, p, g, q, t)] + initial_atoms
        #return [q, t, IsIK(b, p, g, q, t)]

"""
# TODO - finish the push sequence stream
class PushSeqStream(CondStream):
  def __init__(self, oracle):
    self.oracle = oracle
    b, p1, p2 = P('b', BODY), P('p1', POSE), P('p2', POSE)
    p, t = P('p2', POSE), P('t', TRAJ)
    super(PushSeqStream, self).__init__([b], [g], [
      IsPose(b, p1),
      IsPose(b, p2),
      # TODO - on same table, etc...
    ], [
      IsPush(b, p1, t, p2),
      IsPush(b, p1, t, t),
      IsPush(b, t, t, p2),
    ])
  class StreamFn(ListStream):
    def get_list(self, (b, p1, p2,)):
      return [(Grasp(b.name, grasp),) for grasp in get_grasps(self.cond_stream.oracle, b.name)]
"""

####################

def get_supporter(obj, pose, oracle):
  supporter = None
  for surface in oracle.get_surfaces():
  #for surface in oracle.get_objects(): # TODO - extend to all objects
    region = oracle.get_region(surface)
    if region.on(oracle.get_aabb(obj, trans_from_pose(pose.value))):
      assert supporter is None
      supporter = (surface, oracle.surface_poses[surface])
  for other in oracle.get_objects():
    if obj != other and aabb_stacked(oracle.get_aabb(obj, trans_from_pose(pose.value)),
        oracle.get_aabb(other, trans_from_pose(oracle.initial_poses[other].value))):
      assert supporter is None
      supporter = (other, oracle.initial_poses[other])
  assert supporter is not None
  return supporter

def get_object_initial_atoms(obj, oracle):
  oracle.introduced_objects.add(obj)
  initial_atoms = []
  body, pose = C(obj, BODY), Pose(obj, oracle.initial_poses[obj])
  surface, sp = get_supporter(body.name, pose.value, oracle)
  print 'Object supporter:', obj, surface
  s_body, s_pose = C(surface, BODY), Pose(surface, sp)
  initial_atoms += [ # NOTE - need to add this all when a new object happens
    IsGraspable(body),
    AtPose(body, pose),
    IsPose(body, pose),
    IsSupported(body, pose, s_body, s_pose), # NOTE - can just pass in dummy poses if I want
    #On(body, surface),
  ]
  if obj in oracle.get_counter_objects():
    initial_atoms += [AreStackable(body, C(c, BODY)) for c in oracle.get_counters()]
  return initial_atoms

####################

def compile_problem(oracle):
  problem = oracle.problem
  oracle.surface_poses = {surface: oracle.get_pose(surface) for surface in oracle.get_surfaces()}

  manips = []
  if ACTIVE_LEFT:
    manips.append(C('left', MANIP))
  if ACTIVE_RIGHT:
    manips.append(C('right', MANIP))

  initial_atoms = [
    AtConfig(Config(oracle.initial_config)),
  ]
  if problem.start_holding is False: # TODO - handle if a grasp happens
    initial_atoms += [HandEmpty(manip) for manip in manips]

  #for surface in oracle.get_surfaces():
  for surface in oracle.get_counters():
    body, pose = C(surface, BODY), Pose(surface, oracle.surface_poses[surface])
    initial_atoms += [
      IsFixed(body),
      AtPose(body, pose),
      IsPose(body, pose),
    ]

  goal_literals = []
  if problem.goal_holding is not None:
    if problem.goal_holding is False:
      goal_literals += [HandEmpty(manip) for manip in manips]
    elif isinstance(problem.goal_holding, ObjGrasp):
      # TODO - add the manipulator here
      goal_literals.append(HasGrasp(C(problem.goal_holding.object_name, BODY), C(problem.goal_holding.grasp, GRASP)))
    elif problem.goal_holding in oracle.get_objects():
      goal_literals.append(Holding(C(problem.goal_holding, BODY)))
    else:
      raise Exception()
  for obj in problem.goal_poses:
    if problem.goal_poses[obj] == 'initial':
      problem.goal_poses[obj] = oracle.initial_poses[obj]
    elif problem.goal_poses[obj] in oracle.initial_poses: # Goal names other object (TODO - compile with initial)
      problem.goal_poses[obj] = oracle.initial_poses[problem.goal_poses[obj]]
    goal_literals.append(AtPose(C(obj, BODY), Pose(obj, problem.goal_poses[obj])))
    initial_atoms += [
      IsPose(C(obj, BODY), Pose(obj, problem.goal_poses[obj])) # NOTE - need to add the IsPose to everything a priori
      # TODO - stuff here
    ]
  for obj, region in problem.goal_regions.iteritems():
    goal_literals.append(InRegion(C(obj, BODY), C(region, REGION)))
  for obj, base in problem.goal_stackings.items():
    goal_literals.append(On(C(obj, BODY), C(base, BODY)))
    initial_atoms.append(AreStackable(C(obj, BODY), C(base, BODY))) # NOTE - do I want to have this?
  # TODO - I guess I can handle this by making something only stackable when it needs to be
  # TODO - similarly, I could also make a condition that we only want to place in a region if it is a goal
  # TODO - make regions a type of stackable body
  # TODO - dynamically add/remove stacking
  #goal_literals = [Holding(C('blue_box', BODY))]
  goal_literals = [On(C('blue_box', BODY), C('table2', BODY))]

  goal_blocks = set()
  for atom in And(*goal_literals).get_atoms():
    for body in atom.args:
      if body.type == BODY and body.name in oracle.initial_poses:
        goal_blocks.add(body.name)
  #goal_blocks = oracle.get_objects()
  print 'Initial objects:', goal_blocks
  oracle.introduced_objects = set()
  for obj in goal_blocks:
    #for base in goal_blocks:
    #  if obj != base:
    #    initial_atoms.append(AreStackable(C(obj, BODY), C(base, BODY)))
    initial_atoms += get_object_initial_atoms(obj, oracle)

  oracle.region_supporters = {}
  for region in oracle.goal_regions:
    for surface in oracle.get_surfaces():
      if oracle.get_region(surface).region_on(oracle.get_region(region)):
        assert region not in oracle.region_supporters
        print 'Region supporter:', region, surface
        oracle.region_supporters[region] = surface
        initial_atoms.append(IsRegionOn(C(region, REGION), C(surface, BODY)))
    assert region in oracle.region_supporters

  actions = [
    Move(oracle),
    Pick(oracle),
    Place(oracle),
  ]

  axioms = [
    HoldingAxiom(),
    InRegionAxiom(),
    SafeAxiom(),
    OnAxiom(),
  ]

  cond_streams = [
    PoseStream(oracle),
    GraspStream(oracle),
    ContainedStream(oracle),
    #ContainedTest(oracle),
    IKStream(oracle),
    #NearbyTest(oracle),
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
