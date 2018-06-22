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


# TODO - I could return and hash tuples instead of doing this...
# TODO - separate base and arm trajectories
# TODO - need to return multiple values
# TODO - do I want to consider alerting the algorithm when it is done?

# TODO - make the params and types has to the same value if they are the same themselves
# TODO - include functions for variables

CONF = VarType('CONF')
TRAJ = VarType('TRAJ')
BOOL = VarType('BOOL', domain=[True, False])
POSE = VarType('POSE')

R, O, H, T = 'R', 'O', 'H', 'T'

q1, t, q2 = Par(CONF), Par(TRAJ), Par(CONF)
p, g = Par(POSE), Par(POSE)

#####################################

def tamp_problem(oracle):
  DO_ARM_MOTION = True
  DO_COLLISIONS = True
  DO_BASE_MOTION = True
  CHECK_BASE = False

  problem = oracle.problem
  for obj, pose in oracle.initial_poses.iteritems(): # NOTE - would be nice to undo this
    pose.obj = obj
  for obj in problem.goal_poses:
    if problem.goal_poses[obj] == 'initial':
      problem.goal_poses[obj] = oracle.initial_poses[obj]
    elif problem.goal_poses[obj] in oracle.initial_poses: # Goal names other object (TODO - compile with initial)
      raise NotImplementedError() # TODO - don't want to overwrite obj
      problem.goal_poses[obj] = oracle.initial_poses[problem.goal_poses[obj]]
    problem.goal_poses[obj].obj = obj

  ##########

  def region_contains(p, r): # TODO - this is kind of strange
    return isinstance(p, Pose) and oracle.region_contains(r, p.obj, p)

  def collision_free(p, pap):
    if not DO_COLLISIONS: return True
    if pap is None: return True
    #if p is None: return True
    if isinstance(p, GraspTform): return True
    if p.obj == pap.obj: return True
    if pap.obj is None:
      return True # TODO - trajectory collisions here
    holding = ObjGrasp(pap.obj, pap.grasp)
    #holding = Holding(self.oracle.get_body_name(pap.geom_hash), pap.grasp)
    #assert pose.obj != holding.object_name # NOTE - they cannot be the same for this check
    if not DO_ARM_MOTION:
      return not oracle.holding_collision(pap.grasp_config, p.obj, p, holding)
    return not oracle.traj_holding_collision(pap.approach_config, pap.trajs, p.obj, p, holding)

  ##########

  # TODO - support running None or something for these generators to return fewer than one value

  #def sample_grasp(b):
  #  for grasp in get_grasps(oracle, b):
  #    grasp.obj = b
  #    yield [grasp]

  def sample_grasp(b):
    grasps = get_grasps(oracle, b)
    for grasp in grasps:
      grasp.obj = b
    yield grasps

  max_failures = 50 # 10 | 20 | 40
  max_calls = 1 # 1 | INF
  def sample_ik(b, g, p):
    saver = oracle.state_saver()
    oracle.set_all_object_poses({b: p}) # TODO - saver for the initial state as well?
    if oracle.approach_collision(b, p, g):
      return
    for i in range(max_calls):
      pap = PickAndPlace(oracle.get_geom_hash(b), p, g)
      if not pap.sample(oracle, b, max_failures=max_failures,
                        sample_vector=DO_ARM_MOTION, sample_arm=DO_ARM_MOTION, check_base=CHECK_BASE):
        break
      pap.obj = b
      yield [(pap.approach_config, pap)]
      saver.Restore()

  def sample_motion(q1, q2):
    if not DO_BASE_MOTION:
      yield [None]
    with oracle.robot:
      oracle.set_robot_config(q1)
      goal = base_values_from_full_config(q2.value)
      traj = motion_plan(oracle.env, CSpace.robot_base(oracle.robot), goal)
      if traj is not None:
        traj.obj = None
        yield [traj]

  def sample_region(o, r, max_samples=2):
    poses = random_region_placements(oracle, o, [r], region_weights=True)
    for pose in islice(poses, max_samples):
      pose.obj = o
      yield [pose]

  NUM_POSES = 2
  AVOID_INITIAL = True
  def sample_pose(b):
    with oracle.state_saver():
      if AVOID_INITIAL:
        oracle.set_all_object_poses(oracle.initial_poses)
      else:
        oracle.set_all_object_poses({b: oracle.initial_poses[b]})
      poses = random_region_placements(oracle, b, oracle.get_counters(), region_weights=True)
      #poses = cached_region_placements(self.oracle, self.obj, self.oracle.get_counters(), order=None, random=True)
      fixed_poses = []
      for pose in islice(poses, NUM_POSES):
        pose.obj = b
        #yield pose
        fixed_poses.append(pose)
      yield fixed_poses

  ##########

  OBJ = VarType('OBJ', domain=oracle.get_objects())
  REG = VarType('REG', domain=oracle.goal_regions)

  Motion = ConType([CONF, TRAJ, CONF], name='Motion') # NOTE - types are really just for safety more than anything...
  Stable = ConType([OBJ, POSE], name='Stable', satisfying=oracle.initial_poses.items() + problem.goal_poses.items())
  Grasp = ConType([OBJ, POSE], name='Grasp')
  # NOTE - I could just evaluate these on initial values or something
  # NOTE - this would be like saying all new values must come from a generator (not some external source)
  Kin = ConType([OBJ, POSE, CONF, POSE, TRAJ], name='Kin')
  #Contained = ConType([REG, POSE], name='Contained', test=region_contains)
  #Contained = ConType([OBJ, POSE, REG], name='Contained', test=region_contains, domain=[Stable(o, p)])
  CFree = ConType([POSE, TRAJ], name='CFree', test=collision_free)

  Contained = ConType([POSE, REG], name='Contained')
  #Contained = ConType([POSE, REG], name='Contained', test=region_contains)
  #IsGoal = ConType([OBJ, REG], name='IsGoal', test=lambda o, r: problem.goal_regions.get(o, None) == r)
  IsGoal = ConType([OBJ, REG], name='IsGoal', satisfying=problem.goal_regions.items())

  state_vars = [Var(R, CONF), Var(O, POSE, args=[OBJ]), Var(H, BOOL, args=[OBJ])] #+ [Var(HE, BOOL)]
  control_vars = [Var(T, TRAJ)]

  ##########

  o, r = Par('o', OBJ), Par('r', REG)
  actions = [
    Action([Motion(X[R], U[T], nX[R])], name='move'),
    #Action([Identity(nX[R])], name='move'),
    Action([Stable(o, X[O,o]), Grasp(o, nX[O,o]), Kin(o, nX[O,o], X[R], X[O,o], U[T]), Eq(nX[H,o], True)] +
      [CFree(X[O,ob], U[T]) for ob in oracle.get_objects()] +
      [Eq(X[H,ob], False) for ob in oracle.get_objects()], name='pick'),
    Action([Grasp(o, X[O,o]), Stable(o, nX[O,o]), Kin(o, X[O,o], X[R], nX[O,o], U[T]),
      Eq(X[H,o], True), Eq(nX[H,o], False)] +
      [CFree(X[O,ob], U[T]) for ob in oracle.get_objects()], name='place')]
  # TODO - could also just do holding as an axiom on other poses that they are placed

  ##########

  samplers = [
    #Sampler([Motion(q1, t, q2)], inputs=[q1, q2], gen=lambda a,b: iter([None])),
    Sampler([Motion(q1, t, q2)], gen=sample_motion, inputs=[q1, q2], name='Motion'),
    Sampler([Stable(o, p), Contained(p, r)], gen=sample_region, inputs=[o, r], domain=[IsGoal(o, r)], name='Contained'),
    Sampler([Stable(o, p)], gen=sample_pose, inputs=[o], name='Stable'),
    Sampler([Grasp(o, g)], gen=sample_grasp, inputs=[o], name='Grasp'),
    Sampler([Kin(o, g, q1, p, t)], gen=sample_ik, inputs=[o, g, p], domain=[Stable(o, p), Grasp(o, g)], name='Kin'),
  ]

  ##########

  initial_state = [Eq(X[R], oracle.initial_config)] + \
    [Eq(X[O,obj], pose) for obj, pose in oracle.initial_poses.iteritems()] + \
    [Eq(X[H,obj], False) for obj in oracle.get_objects()]

  obj0 = oracle.get_objects()[0]
  #reg0 = oracle.goal_regions[0]
  goal_constraints = [
    #Eq(x[R], oracle.initial_config),
    #Eq(x[R], None),
    #Contained(reg0, X[O,obj0])
    #Eq(X[H,obj0], True),
  ]

  # if problem.goal_holding is not None:
  #   if problem.goal_holding is False:
  #     goal_constraints.append(HandEmpty())
  #   elif isinstance(problem.goal_holding, ObjGrasp):
  #     goal_constraints.append(HasGrasp(C(problem.goal_holding.object_name, BLOCK), C(problem.goal_holding.grasp, GRASP)))
  #   elif problem.goal_holding in oracle.get_objects():
  #     goal_constraints.append(Holding(C(problem.goal_holding, BLOCK)))
  #   else:
  #     raise Exception()
  for obj in problem.goal_poses:
    goal_constraints.append(Eq(X[O,obj], problem.goal_poses[obj]))
  for obj, region in problem.goal_regions.iteritems():
    goal_constraints.append(Contained(X[O,obj], region))

  return ConstraintProblem(state_vars, control_vars, actions, samplers,
                           initial_state, goal_constraints)

#####################################

def convert_state(state):
  world_state = {'holding': False}
  for var, val in state.iteritems():
    name = var[0]
    if name == R:
      world_state['robot'] = val
    elif name == O:
      _, obj = var
      if isinstance(val, Pose):
        world_state[obj] = val
      elif isinstance(val, GraspTform):
        world_state['holding'] = ObjGrasp(obj, val)
      else:
        raise ValueError(val)
    elif name == H:
      pass
    else:
      raise ValueError(name)
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
    if clause.name == 'move':
      pass
      #executables.append(Move(param_to_arg[U[T]], param_to_arg[nX[R]]))
    elif clause.name == 'pick':
      objs = [arg for arg in args if isinstance(arg, str)]
      assert len(objs) == 1
      obj = objs[0] # TODO - more principled way of doing this
      executables.append(Pick(obj, param_to_arg[clause.parameter_map[U[T]]]))
    elif clause.name == 'place':
      objs = [arg for arg in args if isinstance(arg, str)]
      assert len(objs) == 1
      obj = objs[0] # TODO - more principled way of doing this
      executables.append(Place(obj, param_to_arg[clause.parameter_map[U[T]]]))
    else:
      raise ValueError(clause)
  return executables