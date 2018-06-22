from itertools import islice

from manipulation.grasps.grasps import get_grasps
from manipulation.pick_and_place import PickAndPlace
from manipulation.primitives.placements import random_region_placements
from manipulation.grasps.grasps import Holding as ObjGrasp
from manipulation.primitives.utils import Config

from stripstream.pddl.logic.quantifiers import ForAll, Exists
from stripstream.pddl.logic.connectives import Not, And, Or
from stripstream.pddl.logic.atoms import Equal
from stripstream.pddl.operators import STRIPSAxiom, Axiom, STRIPSAction, Action
from stripstream.pddl.utils import rename_easy
from stripstream.pddl.problem import STRIPStreamProblem
from stripstream.pddl.cond_streams import EasyListGenStream, EasyTestStream, EasyGenStream
from stripstream.pddl.objects import EasyType, EasyParameter
from stripstream.pddl.logic.predicates import EasyPredicate
from stripstream.algorithms.hierarchy.operators import abs_action
from stripstream.algorithms.hierarchy.utils import AbsCondition

from manipulation.primitives.transforms import base_values_from_full_config, arm_from_full_config

# TODO - separate into approach movement, retreat and things
# NOTE - if I'm worried about the increased horizon, I can always separate into many components within an action

# Sample base, then IK, then motion trajectory

# Components
# - grasp config
# - approach config
# - approach vector
# - retreat vector
# - approach traj
# - retreat traj

# Probably don't need to separate vector and traj because other people won't

"""
from manipulation.primitives.transforms import point_from_pose
from manipulation.collision.collision_primitives import is_valid_point

def sample_approach_config(self, oracle):
  if not is_valid_point(oracle.robot, point_from_trans(self.base_trans)):
    return False
  manip_name = oracle.robot.GetActiveManipulator().GetName()
  default_config = oracle.default_left_arm_config if manip_name == 'leftarm' else oracle.default_right_arm_config
  set_config(oracle.robot, default_config, get_arm_indices(oracle))
  set_trans(oracle.robot, self.base_trans)
  if robot_collision(oracle, check_self=False):
    return False
  self.approach_config = oracle.get_robot_config()
  return True

class World(object):
  pass
"""

####################

Type, Pred, Param = EasyType, EasyPredicate, EasyParameter

ACTION_COST = 1
EAGER_TESTS = True

DO_ARM_MOTION = False
DO_COLLISIONS = False
CHECK_BASE = False
AVOID_INITIAL = False

####################

# TODO - predicate for whether it belongs to an arm or not

CONF, OBJ, POSE, GRASP, TRAJ, REG = Type(), Type(), Type(), Type(), Type(), Type()
BASE_CONF, MANIP_CONF = Type(), Type()
BASE_TRAJ, MANIP_TRAJ = Type(), Type()

# NOTE - I could also just make a static predicate

BaseEq = Pred(BASE_CONF)
ManipEq = Pred(MANIP_CONF)

HandEmpty = Pred()
PoseEq = Pred(OBJ, POSE)
GraspEq = Pred(OBJ, GRASP)

LegalPose = Pred(OBJ, POSE)
LegalGrasp = Pred(OBJ, GRASP)
Kin = Pred(OBJ, POSE, GRASP, BASE_CONF, MANIP_CONF)
CFree = Pred(OBJ, POSE, TRAJ)
Contained = Pred(REG, OBJ, POSE)

TransitManip = Pred(MANIP_CONF)

ManipMotion = Pred(MANIP_CONF, MANIP_CONF, BASE_CONF, MANIP_TRAJ) # Need BASE_CONF to avoid static collisions
#BaseMotion = Pred(BASE_CONF, BASE_CONF, MANIP_CONF, BASE_TRAJ) # Could ignore MANIP_CONF and just independently check collision
BaseMotion = Pred(BASE_CONF, BASE_CONF, BASE_TRAJ)
LegalConf = Pred(BASE_CONF, MANIP_CONF)

Safe = Pred(OBJ, TRAJ)
#SafeArm = Pred(OBJ, MANIP_CONF, TRAJ) # TODO - make a "object" for the fixed environment which isn't movable
SafeManip = Pred(BASE_TRAJ)

InRegion = Pred(OBJ, REG)
Holding = Pred(OBJ)

rename_easy(locals())

# NOTE - only need to check collisions when moving with current object or something
# - I guess I could make a manip pose be with respect to the base trajectory
# - It depends really on if I want to reuse these trajectories at all. I don't think I need to because

####################

def compile_problem(oracle):
  problem = oracle.problem
  for obj in problem.goal_poses:
    if problem.goal_poses[obj] == 'initial':
      problem.goal_poses[obj] = oracle.initial_poses[obj]
    elif problem.goal_poses[obj] in oracle.initial_poses: # Goal names other object (TODO - compile with initial)
      problem.goal_poses[obj] = oracle.initial_poses[problem.goal_poses[obj]]
  # oracle.initial_config = ??? # TODO - set the initial configuration

  transit_conf = Config(oracle.default_left_arm_config)

  ####################

  O, P, G, T = Param(OBJ), Param(POSE), Param(GRASP), Param(TRAJ)
  OB, R = Param(OBJ), Param(REG)

  BQ, MQ = Param(BASE_CONF), Param(MANIP_CONF)
  BQ2, MQ2 = Param(BASE_CONF), Param(MANIP_CONF)
  BT, MT = Param(BASE_TRAJ), Param(MANIP_TRAJ)

  rename_easy(locals())

  # Separate arm and base configs and trajectories?
  # Make a fixed configuration for arm stuff

  # If I separate arm and base poses, I can avoid worrying about collision when the base and things

  # Would I ever want to actually separate these and use the same grasp at a different config?
  # - Maybe for two arms?
  # - I don't want to reuse trajectories at different base configs. That wouldn't make sense
  # - Although if I manipulate two configs at once, maybe it would!

  # - Make constant arm transit configs?

  # TODO - maybe make an axiom to handle BT and MT when doing these things?
  # TODO - I could equip a manip conf with a base conf at all times if I really don't want them separate
  # NOTE - put then would need to update them whenever moving the base

  #actions = [
  #   Action(name='pick', parameters=[O, P, G, BQ, MQ],
  #     condition=And(PoseEq(O, P), HandEmpty(), Kin(O, P, G, BQ, MQ), BaseEq(BQ), ManipEq(MQ)),
  #       #ForAll([OB], Or(Equal(O, OB), Safe(OB, T)))),
  #     effect=And(GraspEq(O, G), Not(HandEmpty()), Not(PoseEq(O, P)))),
  #
  #   Action(name='place', parameters=[O, P, G, BQ, MQ],
  #     condition=And(GraspEq(O, G), Kin(O, P, G, BQ, MQ), BaseEq(BQ), ManipEq(MQ)),
  #       #ForAll([OB], Or(Equal(O, OB), Safe(OB, T)))),
  #     effect=And(PoseEq(O, P), HandEmpty(), Not(GraspEq(O, G)))),
  #
  #   Action(name='move_arm', parameters=[MQ, MQ2, BQ, MT],
  #     condition=And(ManipEq(MQ), ManipMotion(MQ, MQ2, BQ, MT)),
  #     effect=And(ManipEq(MQ2), Not(ManipEq(MQ)))),
  #
  #   Action(name='move_base', parameters=[BQ, BQ2, MQ, BT], # TODO - put the arm motion stuff in an axiom
  #     condition=And(BaseEq(BQ), TransitManip(MQ), BaseMotion(BQ, BQ2, BT)),
  #     effect=And(BaseEq(BQ2), Not(BaseEq(BQ)))),
  # ]

  # TODO - should I include the grasp in move backwards trajectory?

  actions = [
    abs_action(name='pick', parameters=[O, P, G, BQ, MQ],
      conditions=[
        And(PoseEq(O, P), HandEmpty(), Kin(O, P, G, BQ, MQ)), #ForAll([OB], Or(Equal(O, OB), Safe(OB, T)))),
        And(BaseEq(BQ), ManipEq(MQ))],
      effect=And(GraspEq(O, G), Not(HandEmpty()), Not(PoseEq(O, P)))),

    abs_action(name='place', parameters=[O, P, G, BQ, MQ],
      conditions=[
        And(GraspEq(O, G), Kin(O, P, G, BQ, MQ)), #ForAll([OB], Or(Equal(O, OB), Safe(OB, T)))),
        And(BaseEq(BQ), ManipEq(MQ))],
      effect=And(PoseEq(O, P), HandEmpty(), Not(GraspEq(O, G)))),

    # NOTE - the hierarchical versions of this
    abs_action(name='move_arm', parameters=[MQ, MQ2, BQ, MT], # TODO - Or(TransitManip(MQ), TransitManip(MQ2))
      conditions=[
        And(ManipEq(MQ), ManipMotion(MQ, MQ2, BQ, MT), BaseEq(BQ)),
        #And(ManipEq(MQ), ManipMotion(MQ, MQ2, BQ, MT), BaseEq(BQ), Or(TransitManip(MQ), TransitManip(MQ2))) # This does something odd
        #And(ManipEq(MQ), ManipMotion(MQ, MQ2, BQ, MT), BaseEq(BQ),
        #  Or(TransitManip(MQ), LegalConf(BQ, MQ)),
        #  Or(TransitManip(MQ2), LegalConf(BQ, MQ2))) # This does something odd
      ], # TODO - put an Or(Pair(MQ, BQ), Pair(MQ2, BQ)) or something
      effect=And(ManipEq(MQ2), Not(ManipEq(MQ)))),

    abs_action(name='move_base', parameters=[BQ, BQ2, BT],
      conditions=[
        And(BaseEq(BQ), SafeManip(BT), BaseMotion(BQ, BQ2, BT))],
      effect=And(BaseEq(BQ2), Not(BaseEq(BQ)))),

    #abs_action(name='move_base', parameters=[BQ, BQ2, MQ, BT],
    #  conditions=[
    #    And(BaseEq(BQ), TransitManip(MQ), BaseMotion(BQ, BQ2, BT))],
    #  effect=And(BaseEq(BQ2), Not(BaseEq(BQ)))),
  ]
  # NOTE - the parameters really aren't necessary

  # TODO - could remove any dependence on tests because there are so cheap with the evaluation (already can't use them with axioms)
  # TODO - could also just make the cost only depend on the introduced objects within the effects

  axioms = [
    Axiom(effect=Holding(O), condition=Exists([G], And(GraspEq(O, G), LegalGrasp(O, G)))),

    Axiom(effect=InRegion(O, R), condition=Exists([P], And(PoseEq(O, P), Contained(R, O, P)))),

    #Axiom(effect=Safe(O, T), condition=Exists([P], And(PoseEq(O, P), CFree(O, P, T)))),

    #Axiom(effect=SafeArm(O, MQ, T), condition=Exists([P, MQ], And(PoseEq(O, P), ManipEq(MQ), CFree(O, P, T)))),
    Axiom(effect=SafeManip(BT), condition=Exists([MQ], And(ManipEq(MQ), TransitManip(MQ)))), # TODO - add condition here
  ]
  # TODO - include parameters in STRIPS axiom?

  ####################

  def get_base_conf(conf):
    return Config(base_values_from_full_config(conf.value))

  def get_arm_conf(conf):
    return Config(arm_from_full_config(oracle.robot.GetActiveManipulator(), conf.value))

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
                        sample_vector=False, sample_arm=False, check_base=False):
        break
      #pap.obj = o
      yield [(get_base_conf(pap.grasp_config), get_arm_conf(pap.grasp_config))]

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

    #MultiEasyGenStream(inputs=[O, P, G], outputs=[BQ, MQ], conditions=[LegalPose(O, P), LegalGrasp(O, G)],
    #           effects=[Kin(O, P, G, BQ, MQ)], generator=sample_motion),
    EasyListGenStream(inputs=[O, P, G], outputs=[BQ, MQ], conditions=[LegalPose(O, P), LegalGrasp(O, G)],
                      effects=[Kin(O, P, G, BQ, MQ), LegalConf(BQ, MQ)], generator=sample_motion),
    # TODO - this doesn't work for constants

    # TODO - make a condition that MQ, MQ2 both work for BQ. Otherwise, it will still create a ton of streams
    #EasyGenStream(inputs=[MQ, MQ2, BQ], outputs=[MT], conditions=[],
    #           effects=[ManipMotion(MQ, MQ2, BQ, MT)], generator=lambda *args: [None], order=1),
    EasyGenStream(inputs=[MQ, MQ2, BQ], outputs=[MT], conditions=[TransitManip(MQ), LegalConf(BQ, MQ2)],
               effects=[ManipMotion(MQ, MQ2, BQ, MT)], generator=lambda *args: [None], order=1),
    EasyGenStream(inputs=[MQ, MQ2, BQ], outputs=[MT], conditions=[LegalConf(BQ, MQ), TransitManip(MQ2)],
               effects=[ManipMotion(MQ, MQ2, BQ, MT)], generator=lambda *args: [None], order=1),
    #EasyGenStream(inputs=[MQ, MQ2, BQ], outputs=[MT], conditions=[LegalConf(BQ, MQ), LegalConf(BQ, MQ2)],
    #           effects=[ManipMotion(MQ, MQ2, BQ, MT)], generator=lambda *args: [None], order=1),

    EasyGenStream(inputs=[BQ, BQ2], outputs=[BT], conditions=[],
               effects=[BaseMotion(BQ, BQ2, BT)], generator=lambda *args: [None], order=1),
               #effects=[BaseMotion(BQ, BQ2, BT)], generator=lambda *args: [None], order=2), # NOTE - causes a bug!

    #EasyTestStream(inputs=[O, P, T], conditions=[LegalPose(O, P)], effects=[CFree(O, P, T)],
    #            test=collision_free, eager=EAGER_TESTS),
  ]

  ####################

  #print transit_conf.value
  #print oracle.initial_config.value

  initial_base = get_base_conf(oracle.initial_config)
  initial_manip = get_arm_conf(oracle.initial_config)
  print initial_base.value
  print initial_manip.value

  constants = []

  initial_atoms = [
    BaseEq(initial_base),
    ManipEq(initial_manip),
    HandEmpty(),
    TransitManip(transit_conf),
    LegalConf(initial_base, initial_manip),
  ]
  for obj, pose in oracle.initial_poses.iteritems():
    initial_atoms += [PoseEq(obj, pose), LegalPose(obj, pose)]

  # TODO - serialize goals by object name
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

  #goal_formula = And(*goal_literals)
  #goal_formula = AbsCondition(map(And, goal_literals)) # TODO - bug where goals must have And
  goal_formula = AbsCondition(goal_literals) # TODO - bug where goals must have And

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
      config, = atom.args
      world_state['robot'] = config.value
    elif atom.predicate == PoseEq:
      obj, pose = atom.args
      world_state[obj.name] = pose.value
    elif atom.predicate == GraspEq:
      obj, grasp = atom.args
      world_state['holding'] = ObjGrasp(obj.name, grasp.value)
  return world_state

def executable_plan(oracle, plan):
  executable = []
  for action, args in plan:
    if action.name == 'move':
      q1, q2 = map(get_value, args)
    elif action.name == 'pick':
      o, p, g, q, t = map(get_value, args)
      executable.append(ExecutablePick(o, t))
    elif action.name == 'place':
      o, p, g, q, t = map(get_value, args)
      executable.append(ExecutablePlace(o, t))
  return executable
