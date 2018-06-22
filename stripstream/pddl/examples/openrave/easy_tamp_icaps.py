from itertools import islice

from manipulation.grasps.grasps import get_grasps
from manipulation.pick_and_place import PickAndPlace
from manipulation.primitives.placements import random_region_placements
from manipulation.grasps.grasps import Holding as ObjGrasp

from stripstream.pddl.logic.quantifiers import ForAll, Exists
from stripstream.pddl.logic.connectives import Not, And, Or
from stripstream.pddl.logic.atoms import Equal
from stripstream.pddl.operators import Axiom, Action
from stripstream.pddl.problem import STRIPStreamProblem
from stripstream.pddl.cond_streams import EasyListGenStream, EasyTestStream
from stripstream.pddl.objects import EasyType, EasyParameter
from stripstream.pddl.logic.predicates import EasyPredicate

Type, Pred, Param = EasyType, EasyPredicate, EasyParameter
TestStream = EasyTestStream
GenStream = EasyListGenStream

ACTION_COST = 1
EAGER_TESTS = True

DO_ARM_MOTION = True
DO_COLLISIONS = True
CHECK_BASE = True
AVOID_INITIAL = False

if not DO_ARM_MOTION:
  print 'Warning: trajectories are disabled'
if not DO_COLLISIONS:
  print 'Warning: collisions are disabled'

####################

def compile_problem(oracle):
  problem = oracle.problem
  for obj in problem.goal_poses:
    if problem.goal_poses[obj] == 'initial':
      problem.goal_poses[obj] = oracle.initial_poses[obj]
    elif problem.goal_poses[obj] in oracle.initial_poses: # Goal names other object (TODO - compile with initial)
      problem.goal_poses[obj] = oracle.initial_poses[problem.goal_poses[obj]]

  ####################

  def sample_poses(o, num_samples=1):
    while True:
      if AVOID_INITIAL:
        oracle.set_all_object_poses(oracle.initial_poses)
      else:
        oracle.set_all_object_poses({o: oracle.initial_poses[o]})
      yield list(islice(random_region_placements(oracle, o, oracle.get_counters(), region_weights=True), num_samples))

  def sample_grasps(o):
    yield get_grasps(oracle, o)

  def sample_region(o, r, num_samples=1):
    while True:
      oracle.set_all_object_poses({o: oracle.initial_poses[o]})
      yield list(islice(random_region_placements(oracle, o, [r], region_weights=True), num_samples))

  def sample_motion(o, p, g, max_calls=1, max_failures=50):
    oracle.set_all_object_poses({o: p}) # TODO - saver for the initial state as well?
    if oracle.approach_collision(o, p, g):
      return
    for i in range(max_calls):
      pap = PickAndPlace(oracle.get_geom_hash(o), p, g)
      if not pap.sample(oracle, o, max_failures=max_failures,
                        sample_vector=DO_ARM_MOTION, sample_arm=DO_ARM_MOTION, check_base=CHECK_BASE):
        break
      pap.obj = o
      yield [(pap.approach_config, pap)]

  def collision_free(o, p, t):
    if p is None or o == t.obj:
      return True
    holding = ObjGrasp(t.obj, t.grasp)
    #holding = Holding(self.oracle.get_body_name(pap.geom_hash), pap.grasp)
    if not DO_ARM_MOTION:
      return not oracle.holding_collision(t.grasp_config, o, p, holding)
    return not oracle.traj_holding_collision(t.approach_config, t.trajs, o, p, holding)

  ####################

  # Types
  CONF, TRAJ, REG = Type(), Type(), Type()
  BLOCK, POSE, GRASP = Type(), Type(), Type()

  # Fluent predicates
  AtConfig = Pred(CONF)
  HandEmpty = Pred()
  AtPose = Pred(BLOCK, POSE)
  Holding = Pred(BLOCK, GRASP)

  # Static predicates
  IsPose = Pred(BLOCK, POSE)
  IsGrasp = Pred(BLOCK, GRASP)
  IsKin = Pred(BLOCK, POSE, GRASP, CONF, TRAJ)
  IsCollisionFree = Pred(BLOCK, POSE, TRAJ)
  IsContained = Pred(REG, BLOCK, POSE)

  # Derived predicates
  Safe = Pred(BLOCK, TRAJ)
  InRegion = Pred(BLOCK, REG)

  # Parameters
  O, P, G = Param(BLOCK), Param(POSE), Param(GRASP)
  Q, Q2, T = Param(CONF), Param(CONF), Param(TRAJ)
  OB, R = Param(BLOCK), Param(REG)

  actions = [
    Action(name='pick', parameters=[O, P, G, Q, T],
      condition=And(AtPose(O, P), HandEmpty(),
        IsKin(O, P, G, Q, T), AtConfig(Q),
        ForAll([OB], Or(Equal(O, OB), Safe(OB, T)))),
      effect=And(Holding(O, G),
        Not(HandEmpty()), Not(AtPose(O, P)))),
    Action(name='place', parameters=[O, P, G, Q, T],
      condition=And(Holding(O, G),
        IsKin(O, P, G, Q, T), AtConfig(Q),
        ForAll([OB], Or(Equal(O, OB), Safe(OB, T)))),
      effect=And(AtPose(O, P), HandEmpty(),
        Not(Holding(O, G)))),
    Action(name='move', parameters=[Q, Q2],
      condition=AtConfig(Q),
      effect=And(AtConfig(Q2),
        Not(AtConfig(Q))))]

  axioms = [
    Axiom(effect=InRegion(O, R), condition=Exists([P],
      And(AtPose(O, P), IsContained(R, O, P)))),
    Axiom(effect=Safe(O, T), condition=Exists([P],
      And(AtPose(O, P), IsCollisionFree(O, P, T))))]

  cond_streams = [
    GenStream(inputs=[O], outputs=[P],
      conditions=[],
      effects=[IsPose(O, P)],
      generator=sample_poses),
    GenStream(inputs=[O], outputs=[G],
      conditions=[],
      effects=[IsGrasp(O, G)],
      generator=sample_grasps),
    GenStream(inputs=[O, R], outputs=[P],
      conditions=[],
      effects=[IsPose(O, P), IsContained(R, O, P)],
      generator=sample_region),
    GenStream(inputs=[O, P, G], outputs=[Q, T],
      conditions=[IsPose(O, P), IsGrasp(O, G)],
      effects=[IsKin(O, P, G, Q, T)],
      generator=sample_motion),
    TestStream(inputs=[O, P, T],
      conditions=[IsPose(O, P)],
      effects=[IsCollisionFree(O, P, T)],
      test=collision_free)]

  ####################

  constants = [POSE(None)]

  initial_atoms = [AtConfig(oracle.initial_config)] # TODO - toggle
  holding = set()
  if problem.start_holding is not False:
    obj, grasp = problem.start_holding
    initial_atoms += [Holding(obj, grasp), AtPose(obj, None), IsGrasp(obj, grasp)]
    holding.add(obj)
  if not holding:
    initial_atoms.append(HandEmpty())
  for obj, pose in oracle.initial_poses.iteritems():
    if obj not in holding:
      initial_atoms += [AtPose(obj, pose), IsPose(obj, pose)]

  goal_literals = []
  if problem.goal_holding is not None:
    if problem.goal_holding is False:
      goal_literals.append(HandEmpty())
    elif isinstance(problem.goal_holding, ObjGrasp):
      goal_literals.append(Holding(problem.goal_holding.object_name, problem.goal_holding.grasp))
    elif problem.goal_holding in oracle.get_objects():
      goal_literals.append(Holding(problem.goal_holding))
    else:
      raise Exception()
  for obj, pose in problem.goal_poses.iteritems():
    goal_literals.append(AtPose(obj, pose))
    initial_atoms.append(IsPose(obj, pose))
  for obj, region in problem.goal_regions.iteritems():
    goal_literals.append(InRegion(obj, region))

  goal_formula = goal_literals

  return STRIPStreamProblem(initial_atoms, goal_formula, actions + axioms, cond_streams, constants)
