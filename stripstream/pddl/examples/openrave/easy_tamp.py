from itertools import islice

from manipulation.grasps.grasps import get_grasps
from manipulation.pick_and_place import PickAndPlace
from manipulation.primitives.placements import random_region_placements
from manipulation.grasps.grasps import Holding as ObjGrasp

from stripstream.pddl.logic.quantifiers import ForAll, Exists
from stripstream.pddl.logic.connectives import Not, And, Or
from stripstream.pddl.logic.atoms import Equal
from stripstream.pddl.operators import STRIPSAxiom, Axiom, STRIPSAction, Action
from stripstream.pddl.problem import STRIPStreamProblem
from stripstream.pddl.cond_streams import EasyListGenStream, EasyTestStream
from stripstream.pddl.objects import EasyType, EasyParameter
from stripstream.pddl.logic.predicates import EasyPredicate
from stripstream.pddl.utils import get_value, rename_easy
from stripstream.pddl.examples.openrave.tamp import ExecutablePick, ExecutablePlace
from stripstream.algorithms.hierarchy.utils import AbsCondition

Type, Pred, Param = EasyType, EasyPredicate, EasyParameter

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

CONF, OBJ, POSE, GRASP, TRAJ, REG = Type(), Type(), Type(), Type(), Type(), Type()

ConfEq = Pred(CONF)
HandEmpty = Pred()
PoseEq = Pred(OBJ, POSE)
GraspEq = Pred(OBJ, GRASP)

LegalPose = Pred(OBJ, POSE)
LegalGrasp = Pred(OBJ, GRASP)
Manip = Pred(OBJ, POSE, GRASP, CONF, TRAJ)
CFree = Pred(OBJ, POSE, TRAJ)
Contained = Pred(REG, OBJ, POSE)

#BASE_TRAJ = Type()
#Motion = Pred(CONF, BASE_TRAJ, CONF)

Safe = Pred(OBJ, TRAJ)
InRegion = Pred(OBJ, REG)
Holding = Pred(OBJ)

Cleaned = Pred(OBJ)
Cooked = Pred(OBJ)
IsSink = Pred(REG)
IsStove = Pred(REG)

rename_easy(locals())

####################

def compile_problem(oracle):
  problem = oracle.problem
  for obj in problem.goal_poses:
    if problem.goal_poses[obj] == 'initial':
      problem.goal_poses[obj] = oracle.initial_poses[obj]
    elif problem.goal_poses[obj] in oracle.initial_poses: # Goal names other object (TODO - compile with initial)
      problem.goal_poses[obj] = oracle.initial_poses[problem.goal_poses[obj]]

  ####################

  O, P, G, Q, T = Param(OBJ), Param(POSE), Param(GRASP), Param(CONF), Param(TRAJ)
  Q1, Q2, OB, R = Param(CONF), Param(CONF), Param(OBJ), Param(REG)
  #BT = Param(BASE_TRAJ)

  rename_easy(locals())

  actions = [
    Action(name='pick', parameters=[O, P, G, Q, T],
      condition=And(PoseEq(O, P), HandEmpty(),
        Manip(O, P, G, Q, T), ConfEq(Q), # NOTE - can remove ConfEq(Q)
        ForAll([OB], Or(Equal(O, OB), Safe(OB, T)))),
      effect=And(PoseEq(O, None), GraspEq(O, G),
        Not(HandEmpty()), Not(PoseEq(O, P)))),

    Action(name='place', parameters=[O, P, G, Q, T],
      condition=And(PoseEq(O, None), GraspEq(O, G),
        Manip(O, P, G, Q, T), ConfEq(Q), # NOTE - can remove ConfEq(Q)
        ForAll([OB], Or(Equal(O, OB), Safe(OB, T)))),
      effect=And(PoseEq(O, P), HandEmpty(),
        Not(PoseEq(O, None)), Not(GraspEq(O, G)))),

    Action('clean', parameters=[O, R],
      condition=And(InRegion(O, R), IsSink(R)),
      effect=Cleaned(O)),

    Action('cook', parameters=[O, R],
      condition=And(Cleaned(O), InRegion(O, R), IsStove(R)),
      effect=And(Cooked(O), Not(Cleaned(O)))),

    Action(name='move', parameters=[Q1, Q2],
      condition=ConfEq(Q1),
      effect=And(ConfEq(Q2), Not(ConfEq(Q1)))),
  ]

  axioms = [
    #Axiom(effect=Holding(O), condition=Exists([G], And(GraspEq(O, G), LegalGrasp(O, G)))),
    STRIPSAxiom(conditions=[GraspEq(O, G), LegalGrasp(O, G)], effects=[Holding(O)]),

    Axiom(effect=InRegion(O, R), condition=Exists([P], And(PoseEq(O, P), Contained(R, O, P)))),
    #STRIPSAxiom(conditions=[PoseEq(O, P), ContainedCon(R, O, P)], effects=[InRegion(O, R)]),

    Axiom(effect=Safe(O, T), condition=Exists([P], And(PoseEq(O, P), CFree(O, P, T)))),
    #STRIPSAxiom(conditions=[PoseEq(O, P), CFreeCon(O, P, T)], effects=[Safe(O, T)]),
  ]
  # TODO - include parameters in STRIPS axiom?

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

  cond_streams = [
    EasyListGenStream(inputs=[O], outputs=[P], conditions=[], effects=[LegalPose(O, P)], generator=sample_poses),

    EasyListGenStream(inputs=[O], outputs=[G], conditions=[], effects=[LegalGrasp(O, G)], generator=sample_grasps),

    EasyListGenStream(inputs=[O, R], outputs=[P], conditions=[],
                      effects=[LegalPose(O, P), Contained(R, O, P)], generator=sample_region),

    EasyListGenStream(inputs=[O, P, G], outputs=[Q, T], conditions=[LegalPose(O, P), LegalGrasp(O, G)],
                      effects=[Manip(O, P, G, Q, T)], generator=sample_motion),

    #MultiEasyGenStream(inputs=[Q1, Q2], outputs=[BT], conditions=[],
    #           effects=[Motion(Q1, BT, Q2)], generator=lambda q1, q2: [[None]]),

    EasyTestStream(inputs=[O, P, T], conditions=[LegalPose(O, P)], effects=[CFree(O, P, T)],
                test=collision_free, eager=EAGER_TESTS),
  ]

  ####################

  constants = [POSE(None)]

  initial_atoms = [ConfEq(oracle.initial_config)] # TODO - toggle
  holding = set()
  if problem.start_holding is not False:
    obj, grasp = problem.start_holding
    initial_atoms += [GraspEq(obj, grasp), PoseEq(obj, None), LegalGrasp(obj, grasp)]
    holding.add(obj)
  if not holding:
    initial_atoms.append(HandEmpty())
  for obj, pose in oracle.initial_poses.iteritems():
    if obj not in holding:
      initial_atoms += [PoseEq(obj, pose), LegalPose(obj, pose)]
  initial_atoms += [IsSink(region) for region in oracle.sinks]
  initial_atoms += [IsStove(region) for region in oracle.stoves]

  goal_literals = []
  if problem.goal_holding is not None:
    if problem.goal_holding is False:
      goal_literals.append(HandEmpty())
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
  for obj in problem.goal_cleaned:
    goal_literals.append(Cleaned(obj))
  for obj in problem.goal_cooked:
    goal_literals.append(Cooked(obj))

  goal_formula = goal_literals
  #goal_formula = And(*goal_literals)
  #goal_formula = AbsCondition(goal_literals) # TODO - bug where goals must have And

  return STRIPStreamProblem(initial_atoms, goal_formula, actions + axioms, cond_streams, constants)

##################################################

def convert_state(oracle, pddl_state):
  world_state = {
    'holding': False
  }
  for object_name in oracle.get_objects():
    world_state[object_name] = False
  for atom in pddl_state:
    if atom.predicate == ConfEq:
      config, = map(get_value, atom.args)
      world_state['robot'] = config
    elif atom.predicate == PoseEq:
      obj, pose = map(get_value, atom.args)
      world_state[obj] = pose
    elif atom.predicate == GraspEq:
      obj, grasp = map(get_value, atom.args)
      world_state['holding'] = ObjGrasp(obj, grasp)
  return world_state

def executable_plan(oracle, plan):
  executable = []
  for action, args in plan:
    #if action.name == 'move':
    #  q1, q2 = map(get_value, args)
    if action.name == 'pick':
      o, p, g, q, t = map(get_value, args)
      executable.append(ExecutablePick(o, t))
    elif action.name == 'place':
      o, p, g, q, t = map(get_value, args)
      executable.append(ExecutablePlace(o, t))
  return executable
