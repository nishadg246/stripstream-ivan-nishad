from itertools import islice
from random import choice

from manipulation.grasps.grasps import get_grasps
from manipulation.pick_and_place import PickAndPlace
from manipulation.primitives.placements import random_region_placements
from manipulation.grasps.grasps import Holding as ObjGrasp, Grasp as GraspTform
from manipulation.primitives.utils import Pose

from stripstream.fts.constraint import Eq, ConType
from stripstream.fts.variable import VarType, Par, Var, X, U, nX
from stripstream.fts.sampler import Sampler

# Only primitive samples are x and u
# How do I sample in a goal or something
# We only sample the forward model
# Only constraint is the dynamics constraint which just, given a state, samples an action and result
# Special samplers for partial goals
# Well I can't even have partial goals, I need to have combined goals which we definitely can't sample for multiple things
# Well I could always separate the constraint into things on the combined state, but individual

STATE, CONTROL = VarType('STATE'), VarType('CONTROL')
OBJ, REG, POSE = VarType('OBJ'), VarType('REG'), VarType('POSE')

S, A = 'S', 'A' # TODO - allow no specification if desired
H, P = 'H', 'P'

s1, a, s2 = Par(STATE), Par(CONTROL), Par(STATE)

#####################################

def tamp_problem(oracle):
  DO_ARM_MOTION = True
  DO_COLLISIONS = True
  CHECK_BASE = True

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

  def sample_region(o, r, max_samples=2):
    oracle.set_all_object_poses({o: oracle.initial_poses[o]})
    poses = random_region_placements(oracle, o, [r], region_weights=True)
    for pose in islice(poses, max_samples):
      pose.obj = o
      yield [pose]

  NUM_POSES = 2
  AVOID_INITIAL = True
  def sample_pose(o):
    #with oracle.state_saver():
    while True:
      if AVOID_INITIAL:
        oracle.set_all_object_poses(oracle.initial_poses)
      else:
        oracle.set_all_object_poses({o: oracle.initial_poses[o]})
      poses = random_region_placements(oracle, o, oracle.get_counters(), region_weights=True)
      #poses = cached_region_placements(self.oracle, self.obj, self.oracle.get_counters(), order=None, random=True)
      fixed_poses = []
      for pose in islice(poses, NUM_POSES):
        pose.obj = o
        #yield pose
        fixed_poses.append(pose)
      yield fixed_poses
  # Exception IndexError: 'vector' in <generator object sample_pose at 0x127d0dbe0> ignored

  ##########

  def test_goal(s):
    state = dict(s)
    for obj, pose in problem.goal_poses.iteritems():
      if state[(P, obj)] != pose:
        return False
    for obj, region in problem.goal_regions.iteritems():
      if not region_contains(state[(P, obj)], region):
        return False
    return True

  def get_successor(state, obj, pose, holding):
    new_state = state.copy()
    new_state[(H, obj)] = holding
    new_state[(P, obj)] = pose
    return frozenset(new_state.items())

  # TODO - how many successors to sample?
  # TODO - can either automatically sample goal or make special sampler
  # TODO - how do I signal to the algorithm that I achieve a goal? Only do implicitly
  def forward_dynamics(s):
    state = dict(s)
    holding = None
    for obj in oracle.get_objects():
      if state[(H, obj)]:
        holding = obj
    # TODO - I should just make an alternating cycle of these generators or something
    while True:
      if holding is None: # TODO - why does this termainte on the first state sometimes?????
        obj = choice(oracle.get_objects())
        pose = state[(P, obj)]
        #for grasp in islice(sample_grasp(obj), 1):
        for grasp in next(sample_grasp(obj)):
          for traj in next(sample_ik(obj, grasp, pose)):
            if all(collision_free(state[(P, obst)], traj) for obst in oracle.get_objects()):
              yield [(traj, get_successor(state, obj, grasp, True))]
      else:
        obj = holding
        grasp = state[(P, obj)]
        pose_generators = [sample_pose(obj)]
        if obj in problem.goal_poses:
          pose_generators.append(iter([problem.goal_poses[obj]]))
        if obj in problem.goal_regions:
          pose_generators.append(sample_region(obj, problem.goal_regions[obj]))

        # TODO - I should cycle using this
        for generator in pose_generators: # NOTE - this is generous
          #for pose in islice(generator, 1):
          for pose in next(generator):
            for traj in next(sample_ik(obj, grasp, pose)):
              if all(collision_free(state[(P, obst)], traj) for obst in oracle.get_objects()):
                yield [(traj, get_successor(state, obj, pose, False))]
        #for pose in islice(sample_pose(obj), 1):
        #  for traj in sample_ik(obj, grasp, pose):
        #    if all(collision_free(state[(P, obst)], traj) for obst in oracle.get_objects()):
        #      yield [(traj, get_successor(state, obj, pose, False))]


  ##########

  Transition = ConType([STATE, CONTROL, STATE], name='Transition')
  #GoalPose = ConType([OBJ, STATE], name='GoalPose')
  #GoalRegion = ConType([OBJ, STATE], name='GoalRegion')
  Goal = ConType([STATE], name='Goal', test=test_goal)

  state_vars = [Var(S, STATE)]
  control_vars = [Var(A, CONTROL)]

  ##########

  actions = [
    Action([Transition(X[S], U[A], nX[S])], name='transition'),
  ]

  ##########

  samplers = [
    Sampler([Transition(s1, a, s2)], gen=forward_dynamics, inputs=[s1], name='forward'),
  ]

  ##########

  S0 = frozenset([((H, obj), False) for obj in oracle.get_objects()] + \
    [((P, obj), pose) for obj, pose in oracle.initial_poses.iteritems()])
  initial_state = [Eq(X[S], S0)]

  goal_constraints = [Goal(X[S])]
  #for obj, pose in problem.goal_poses.iteritems():
  #  goal_constraints.append(GoalPose(obj, pose, X[S]))
  #for obj, region in problem.goal_regions.iteritems():
  #  goal_constraints.append(GoalRegion(obj, region, X[S]))

  return ConstraintProblem(state_vars, control_vars, actions, samplers,
                           initial_state, goal_constraints)