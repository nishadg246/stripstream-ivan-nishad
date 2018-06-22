from itertools import islice

from manipulation.grasps.grasps import get_grasps
from manipulation.pick_and_place import PickAndPlace
from manipulation.execute import Moveable, Pickable, Placeable
from manipulation.primitives.placements import random_region_placements
from manipulation.grasps.grasps import Holding as ObjGrasp, Grasp as GraspTform
from manipulation.primitives.utils import Pose

from stripstream.fts.constraint import Eq, ConType
from stripstream.fts.variable import VarType, Par, get_value, Var, X, U, nX
from stripstream.fts.clause import Clause
from stripstream.fts.sampler import Sampler
from stripstream.fts.problem import FTSProblem

DO_ARM_MOTION = True
DO_COLLISIONS = True
CHECK_BASE = True
AVOID_INITIAL = True

if not DO_ARM_MOTION:
  print 'Warning: trajectories are disabled'
if not DO_COLLISIONS:
  print 'Warning: collisions are disabled'

# TODO - I could return and hash tuples instead of doing this...
# TODO - separate base and arm trajectories
# TODO - need to return multiple values
# TODO - do I want to consider alerting the algorithm when it is done?

# TODO - make the params and types has to the same value if they are the same themselves
# TODO - include functions for variables

TRAJ = VarType('TRAJ')
BOOL = VarType('BOOL', domain=[True, False])
POSE = VarType('POSE')

O, H, T = 'O', 'H', 'T'

t, p, g = Par(TRAJ), Par(POSE), Par(POSE)

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
      yield [pap]
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

  ##########

  OBJ = VarType('OBJ', domain=oracle.get_objects())
  REG = VarType('REG', domain=set(problem.goal_regions.values()))

  Stable = ConType([OBJ, POSE], name='Stable', satisfying=oracle.initial_poses.items() + problem.goal_poses.items())
  Grasp = ConType([OBJ, POSE], name='Grasp')
  Kin = ConType([OBJ, POSE, POSE, TRAJ], name='Kin')
  CFree = ConType([POSE, TRAJ], name='CFree', test=collision_free)
  Contained = ConType([POSE, REG], name='Contained')
  IsGoal = ConType([OBJ, REG], name='IsGoal', satisfying=problem.goal_regions.items())
  # NOTE - I could just evaluate these on initial values or something
  # NOTE - this would be like saying all new values must come from a generator (not some external source)

  state_vars = [Var(O, POSE, args=[OBJ]), Var(H, BOOL, args=[OBJ])] #+ [Var(HE, BOOL)]
  control_vars = [Var(T, TRAJ)]

  ##########

  o, r = Par('o', OBJ), Par('r', REG)
  actions = [
    #Action([Stable(o, X[O,o]), Grasp(o, nX[O,o]), Kin(o, nX[O,o], X[O,o], U[T]), Eq(nX[H,o], True)] + # NOTE - the compilation makes False a parameter
    Clause([Stable(o, X[O,o]), Grasp(o, nX[O,o]), Kin(o, nX[O,o], X[O,o], U[T]), Eq(X[H,o], False), Eq(nX[H,o], True)] +
      [CFree(X[O,ob], U[T]) for ob in oracle.get_objects()] +
      [Eq(X[H,ob], False) for ob in oracle.get_objects()], name='pick'),
    Clause([Grasp(o, X[O,o]), Stable(o, nX[O,o]), Kin(o, X[O,o], nX[O,o], U[T]),
      Eq(X[H,o], True), Eq(nX[H,o], False)] +
      [CFree(X[O,ob], U[T]) for ob in oracle.get_objects()], name='place')]
  # TODO - could also just do holding as an axiom on other poses that they are placed

  ##########

  samplers = [
    Sampler([Stable(o, p), Contained(p, r)], gen=sample_region, inputs=[o, r], domain=[IsGoal(o, r)], name='Contained'),
    Sampler([Stable(o, p)], gen=sample_pose, inputs=[o], name='Stable'),
    Sampler([Grasp(o, g)], gen=sample_grasp, inputs=[o], name='Grasp'),
    Sampler([Kin(o, g, p, t)], gen=sample_ik, inputs=[o, g, p], domain=[Stable(o, p), Grasp(o, g)], name='Kin'),
  ]

  ##########

  initial_state = [Eq(X[H,obj], False) for obj in oracle.get_objects()] + \
    [Eq(X[O,obj], pose) for obj, pose in oracle.initial_poses.iteritems()]


  goal_constraints = []
  for obj in problem.goal_poses:
    goal_constraints.append(Eq(X[O,obj], problem.goal_poses[obj]))
  for obj, region in problem.goal_regions.iteritems():
    goal_constraints.append(Contained(X[O,obj], region))

  return FTSProblem(state_vars, control_vars, actions, samplers,
                           initial_state, goal_constraints)

#####################################

def convert_state(oracle, state):
  world_state = {
    'holding': False,
    'robot': oracle.initial_config,
  }
  for var, val in state.iteritems():
    name = var[0]
    if name == O:
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

def convert_plan(oracle, plan):
  executables = []
  last_config = oracle.initial_config
  for operator, constants in plan: # TODO - this is all a hack
    clause = operator.clause
    args = [get_value(const) for const in constants]
    param_to_arg = dict(zip(operator.parameters, args))
    pap = param_to_arg[clause.parameter_map[U[T]]]

    #path = oracle.grow_base_roadmap(pap.approach_config)
    base_trajs = oracle.plan_base_roadmap(last_config, pap.approach_config)
    executables.append(Move(base_trajs, pap.approach_config))
    last_config = pap.approach_config

    if clause.name == 'pick':
      objs = [arg for arg in args if isinstance(arg, str)]
      assert len(objs) == 1
      obj = objs[0] # TODO - more principled way of doing this
      executables.append(Pick(obj, pap))
    elif clause.name == 'place':
      objs = [arg for arg in args if isinstance(arg, str)]
      assert len(objs) == 1
      obj = objs[0] # TODO - more principled way of doing this
      executables.append(Place(obj, pap))
    else:
      raise ValueError(clause)
  return executables