from itertools import islice

from manipulation.grasps.grasps import get_grasps
from manipulation.pick_and_place import PickAndPlace
from manipulation.operators.move import Moveable
from manipulation.primitives.placements import random_region_placements
from manipulation.grasps.grasps import Holding as ObjGrasp, Grasp as GraspTform
from manipulation.operators.pick import Pickable, Placeable
from manipulation.primitives.utils import Pose
from manipulation.motion.cspace import CSpace
from manipulation.motion.single_query import motion_plan
from manipulation.primitives.transforms import base_values_from_full_config

from stripstream.pddl.constraint import VarType, nX, X, U, Par, Var, Eq, get_value, ConType
from stripstream.pddl.constraint import Action
from stripstream.pddl.constraint import Sampler
from stripstream.pddl.constraint import ConstraintProblem

DO_BASE_MOTION = True
DO_ARM_MOTION = True
DO_COLLISIONS = True
CHECK_BASE = True
AVOID_INITIAL = True

# TODO - I could return and hash tuples instead of doing this...
# TODO - do I want to consider alerting the algorithm when it is done?

# TODO - make the params and types has to the same value if they are the same themselves
# TODO - include functions for variables

CONF = VarType('CONF')
POSE = VarType('POSE')
GRASP = VarType('GRASP')
ARM_TRAJ = VarType('ARM_TRAJ')
BASE_TRAJ = VarType('BASE_TRAJ')

R, P, G, AT, BT = 'R', 'P', 'G', 'AT', 'BT'

p, g = Par(POSE), Par(GRASP)
at, bt = Par(ARM_TRAJ), Par(BASE_TRAJ)
q, nq = Par(CONF), Par(CONF)

#####################################

def tamp_problem(oracle):
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

  ##########

  def collision_free(p, pap):
    if not DO_COLLISIONS or p is None or p.obj == pap.obj:
      return True
    holding = ObjGrasp(pap.obj, pap.grasp)
    #holding = Holding(self.oracle.get_body_name(pap.geom_hash), pap.grasp)
    #assert pose.obj != holding.object_name # NOTE - they cannot be the same for this check
    if not DO_ARM_MOTION:
      return not oracle.holding_collision(pap.grasp_config, p.obj, p, holding)
    return not oracle.traj_holding_collision(pap.approach_config, pap.trajs, p.obj, p, holding)

  ##########

  def sample_grasp(o):
    grasps = get_grasps(oracle, o)
    for grasp in grasps:
      grasp.obj = o
    yield grasps

  max_failures = 50 # 10 | 20 | 40
  max_calls = 1 # 1 | INF
  def sample_ik(o, g, p):
    #saver = oracle.state_saver()
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
      #saver.Restore()

  def sample_region(o, r, num_samples=1):
    while True:
      oracle.set_all_object_poses({o: oracle.initial_poses[o]})
      poses = random_region_placements(oracle, o, [r], region_weights=True)
      fixed_poses = []
      for pose in islice(poses, num_samples):
        pose.obj = o
        fixed_poses.append(pose)
      yield fixed_poses

  def sample_pose(o, num_samples=2):
    #with oracle.state_saver():
    while True:
      if AVOID_INITIAL:
        oracle.set_all_object_poses(oracle.initial_poses)
      else:
        oracle.set_all_object_poses({o: oracle.initial_poses[o]})
      poses = random_region_placements(oracle, o, oracle.get_counters(), region_weights=True)
      #poses = cached_region_placements(self.oracle, self.obj, self.oracle.get_counters(), order=None, random=True)
      fixed_poses = []
      for pose in islice(poses, num_samples):
        pose.obj = o
        #yield pose
        fixed_poses.append(pose)
      yield fixed_poses
  # Exception IndexError: 'vector' in <generator object sample_pose at 0x127d0dbe0> ignored

  def sample_motion(q1, q2):
    if not DO_BASE_MOTION:
      yield [None]
    #with oracle.robot: # NOTE - this causes an segfault when freeing the memory
    oracle.set_robot_config(q1)
    goal = base_values_from_full_config(q2.value)
    traj = motion_plan(oracle.env, CSpace.robot_base(oracle.robot), goal)
    if traj is not None:
      traj.obj = None
      yield [traj]

  ##########

  OBJ = VarType('OBJ', domain=oracle.get_objects())
  REG = VarType('REG', domain=set(problem.goal_regions.values()))

  Motion = ConType([CONF, BASE_TRAJ, CONF], name='Motion')
  IsStable = ConType([OBJ, POSE], name='IsStable', satisfying=oracle.initial_poses.items() + problem.goal_poses.items())
  #IsStable = ConType([OBJ, POSE], name='IsStable', test=lambda o, p: p is not None and p.obj == o, plannable=False)
  IsGrasp = ConType([OBJ, GRASP], name='IsGrasp')
  Kin = ConType([OBJ, GRASP, POSE, CONF, ARM_TRAJ], name='Kin')
  CFree = ConType([POSE, ARM_TRAJ], name='CFree', test=collision_free)
  Contained = ConType([POSE, REG], name='Contained')
  IsGoal = ConType([OBJ, REG], name='IsGoal', satisfying=problem.goal_regions.items())

  state_vars = [Var(R, CONF), Var(P, POSE, args=[OBJ]), Var(G, GRASP, args=[OBJ])]
  control_vars = [Var(AT, ARM_TRAJ), Var(BT, BASE_TRAJ)]

  ##########

  o, r = Par('o', OBJ), Par('r', REG)
  actions = [
    Action([Motion(X[R], U[BT], nX[R])], name='move'),
    Action([IsStable(o, X[P,o]), IsGrasp(o, nX[G,o]), Kin(o, nX[G,o], X[P,o], X[R], U[AT]),
      Eq(X[G,o], None), Eq(nX[P,o], None)] +
      [CFree(X[P,ob], U[AT]) for ob in oracle.get_objects()] +
      [Eq(X[G,ob], None) for ob in oracle.get_objects()], name='pick'),
    Action([IsGrasp(o, X[G,o]), IsStable(o, nX[P,o]), Kin(o, X[G,o], nX[P,o], X[R], U[AT]),
      Eq(X[P,o], None), Eq(nX[G,o], None)] +
      [CFree(X[P,ob], U[AT]) for ob in oracle.get_objects()], name='place')]

  ##########

  samplers = [
    Sampler([IsStable(o, p), Contained(p, r)], gen=sample_region, inputs=[o, r], domain=[IsGoal(o, r)], name='Contained'),
    Sampler([IsStable(o, p)], gen=sample_pose, inputs=[o], name='IsStable'),
    Sampler([IsGrasp(o, g)], gen=sample_grasp, inputs=[o], name='IsGrasp'),
    Sampler([Motion(q, bt, nq)], gen=sample_motion, inputs=[q, nq], name='Motion', order=1),
    Sampler([Kin(o, g, p, q, at)], gen=sample_ik, inputs=[o, g, p], domain=[IsGrasp(o, g), IsStable(o, p)], name='Kin'),
  ]

  ##########

  initial_state = [Eq(X[R], oracle.initial_config)] + \
    [Eq(X[G,obj], None) for obj in oracle.get_objects()] + \
    [Eq(X[P,obj], pose) for obj, pose in oracle.initial_poses.iteritems()]

  goal_constraints = []
  for obj in problem.goal_poses:
    goal_constraints.append(Eq(X[P,obj], problem.goal_poses[obj]))
  for obj, region in problem.goal_regions.iteritems():
    goal_constraints.append(Contained(X[P,obj], region))

  return ConstraintProblem(state_vars, control_vars, actions, samplers,
                           initial_state, goal_constraints)

#####################################

def convert_state(oracle, state):
  world_state = {
    'holding': False,
    'robot': oracle.initial_config,
  }
  for var, val in state.iteritems():
    name = var[0]
    if name == R:
      world_state['robot'] = val
    elif name == P and val is not None:
      _, obj = var
      assert isinstance(val, Pose)
      world_state[obj] = val
    elif name == G and val is not None:
      _, obj = var
      assert isinstance(val, GraspTform)
      world_state['holding'] = ObjGrasp(obj, val)
    #else:
    #  raise ValueError(name)
  return world_state

# TODO - automatically convert result?

class Pick(Pickable):
  def __init__(self, object_name, pap):
    self.object_name = object_name
    self.pap = pap

class Place(Placeable):
  def __init__(self, object_name, pap):
    self.object_name = object_name
    self.pap = pap

class Move(Moveable):
  def __init__(self, base_trajs, end_config):
    self.base_trajs = base_trajs
    self.end_config = end_config

# NOTE - this still is a STRIPS action

def convert_plan(plan):
  executables = []
  for operator, constants in plan: # TODO - this is all a hack
    clause = operator.clause
    args = [get_value(const) for const in constants]
    param_to_arg = dict(zip(operator.parameters, args))
    if clause.name == 'pick':
      objs = [arg for arg in args if isinstance(arg, str)]
      assert len(objs) == 1
      obj = objs[0] # TODO - more principled way of doing this
      executables.append(Pick(obj, param_to_arg[clause.parameter_map[U[AT]]]))
    elif clause.name == 'place':
      objs = [arg for arg in args if isinstance(arg, str)]
      assert len(objs) == 1
      obj = objs[0] # TODO - more principled way of doing this
      executables.append(Place(obj, param_to_arg[clause.parameter_map[U[AT]]]))
    elif clause.name == 'move':
      pass
    else:
      raise ValueError(clause)
  return executables