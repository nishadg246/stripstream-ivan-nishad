from factored_constraint import *

def stuff():
  #CONF, POSE, BOOL, TRAJ = DType(), DType(), DType(), DType()
  CONF, POSE, BOOL, TRAJ = Ty('CONF'), Ty('POSE'), Ty('BOOL'), Ty('TRAJ')

  # Motion = ConstraintForm(name='Motion') # NOTE - types are really just for safety more than anything...
  # HoldingMotion = ConstraintForm(name='HoldingMotion')
  # Traversable = ConstraintForm(name='Traversable')
  # #HoldTraversable = ConstraintForm(name='HoldTraversable')
  # Stable = ConstraintForm(name='Stable')
  # Grasp = ConstraintForm(name='Grasp')
  # Kin = ConstraintForm(name='Kin')

  R, O, H, HE, T = 'R', 'O', 'H', 'HE', 'T'
  #r, h, t = 'r', 'h', 't' # TODO - make different than parameter


  objects = ['o%s'%i for i in range(3)]
  OBJ = Ty('OBJ', domain=objects)

  Motion = ConType([CONF, TRAJ, CONF], name='Motion') # NOTE - types are really just for safety more than anything...
  HoldingMotion = ConType([CONF, TRAJ, CONF, POSE], name='HoldingMotion')
  Traversable = ConType([TRAJ, POSE], name='Traversable')
  Stable = ConType([OBJ, POSE], name='Stable')
  Grasp = ConType([OBJ, POSE], name='Grasp')
  Kin = ConType([OBJ, POSE, CONF, POSE], name='Kin')

  o = Par('o', OBJ)
  #reg = Par('reg')

  #state_vars = {R: CONF, H: BOOL, (O, OBJECT): POSE} # TODO - make this the variable description
  #state_vars = {r: CONF, h: BOOL, o: POSE} # TODO - maybe I can implicitly infer the domain of o from parameters
  #control_vars = {T: TRAJ}

  state_vars = [Var(R, CONF), Var(H, BOOL), Var(O, POSE, args=[OBJ])]
  control_vars = [Var(T, TRAJ)]

  #predicates = predicate_from_variable(state_vars)
  #print predicates

  # NOTE - I still can't easily convert to lifted STRIPS with equality
  # TODO - in future extend to when the constraints can themselves be in a logical form

  initial = [
    Eq(X[H], None),
  ]

  initial = {R: None, H: None} # NOTE - could also implement this as a map

  #initial_state = {
  #  x[R]: oracle.initial_config,
  #  x[H]: None,
  #}

  goal = [
    # Constraints here
    Eq(nX[H], None),
    #GoalRegion(region[], None)
  ]

  # TODO - make an at predicate for each data type (and then just include name if only one)?

  # TODO - should I include the index in the sampler? I think I need to...
  # NOTE - x1 and x2 are more shortcuts for a parameter
  actions = [
    Action([Motion(X[R], U[T], nX[R]), Eq(X[H], None)] +
           [Traversable(U[T], X[O,ob]) for ob in objects], name='move'),
    Action([HoldingMotion(X[R], U[T], nX[R], X[O,o])] +
           [Traversable(U[T], X[O,ob]) for ob in objects], name='moveh'),
    Action([Stable(o, X[O,o]), Grasp(o, nX[O,o]), Kin(o, nX[O,o], X[R], X[O,o]),
            Eq(X[H], None), Eq(nX[H], o)], name='pick'),
    Action([Grasp(o, X[O,o]), Stable(o, nX[O,o]), Kin(o, X[O,o], X[R], nX[O,o]),
            Eq(X[H], o), Eq(nX[H], None)], name='place')]

  tests = [
    #Test(Motion(x1[r], u[t], x2[r]), test=None),
  ] # TODO - just define tests up with the constraints (no point doing it here)

  # NOTE - should never need more than two per variable...
  # samplers = [
  #   Sampler([Motion(x1[R], u[T], x2[R])], inputs=[x1[R], x2[R]]),
  #   Sampler([Stable(x[o])]), # TODO - I don't likely want to specify the reverse for each of these...
  #   Sampler([Grasp(x[o])]), # NOTE - maybe specify just one
  #   Sampler([Kin(x[o], x[R], x[o])], inputs=[x1[o], x2[o]], domain=[Stable(x1[o]), Grasp(x2[o])]),
  # ]

  q1, t, q2 = Par(CONF), Par(TRAJ), Par(CONF)
  p, g = Par(POSE), Par(POSE)
  samplers = [
    Sampler([Motion(q1, t, q2)], inputs=[q1, q2]),
    Sampler([Stable(o, p)]), # TODO - I don't likely want to specify the reverse for each of these...
    Sampler([Grasp(o, g)]), # NOTE - maybe specify just one
    Sampler([Kin(o, g, q1, p)], inputs=[o, g, p], domain=[Stable(o, p), Grasp(o, g)]),
  ]

  # TODO - only make axioms for variables not already needed in the action

  #state_var_map = {var.name: var for var in state_vars}
  var_map = {var.name: var for var in state_vars + control_vars}
  # TODO - domain conditions for constraint?



# TODO - how do I handle testing initial literals satisfy things?
# TODO - just assume eager test until shown to be false?
# NOTE - could also reuse poses if I make the relative to the base of the object


from itertools import product, count, islice

from manipulation.grasps.grasps import get_grasps
from manipulation.pick_and_place import PickAndPlace, current_base_iterator, custom_base_iterator
from manipulation.operators.pick import Pickable, Placeable
from manipulation.operators.move import Moveable
from manipulation.primitives.placements import random_region_placements, cached_region_placements
from manipulation.grasps.grasps import Holding as ObjGrasp, Grasp as GraspTform
from manipulation.operators.pick import Pickable, Placeable
from manipulation.primitives.utils import Pose

def constraint_problem(oracle):

  def region_contains(o, p, r): # TODO - this is kind of strange
    return isinstance(p, Pose) and oracle.region_contains(r, o, p)

  ##########

  # TODO - support running None or something for these generators to return fewer than one value

  def sample_grasp(b):
    return iter(get_grasps(oracle, b))

  DO_MOTION = False
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
                        sample_vector=DO_MOTION, sample_arm=DO_MOTION, check_base=False):
        break
      yield pap.approach_config
      #yield (pap.approach_config, pap)
      saver.Restore()

  def motion_plan(q1, q2):
    yield None

  def sample_region(o, r, max_samples=2):
    poses = random_region_placements(oracle, o, [r], region_weights=True)
    for pose in islice(poses, max_samples):
      yield (pose,)

  ##########


  #CONF, POSE, BOOL, TRAJ = DType(), DType(), DType(), DType()
  CONF, TRAJ = Ty('CONF'), Ty('TRAJ')
  BOOL = Ty('BOOL', domain=[True, False])

  FIXED_POSES = True
  if FIXED_POSES:
    # TODO - assert that all the same geometry
    poses = oracle.problem.known_poses + oracle.initial_poses.values() # TODO - add goal poses
    # TODO - this is also kind of bad
    grasps =  get_grasps(oracle, oracle.get_objects()[0])
    POSE = Ty('POSE', domain=poses+grasps)
  else:
    POSE = Ty('POSE')

  R, O, H, HE, T = 'R', 'O', 'H', 'HE', 'T'
  #r, h, t = 'r', 'h', 't' # TODO - make different than parameter

  OBJ = Ty('OBJ', domain=oracle.get_objects())
  REG = Ty('REG', domain=oracle.goal_regions)

  Motion = ConType([CONF, TRAJ, CONF], name='Motion') # NOTE - types are really just for safety more than anything...
  Stable = ConType([OBJ, POSE], name='Stable', test=lambda o, p: isinstance(p, Pose))
  Grasp = ConType([OBJ, POSE], name='Grasp', test=lambda o, p: isinstance(p, GraspTform)) # TODO - shouldn't need this
  # NOTE - I could just evaluate these on initial values or something
  # NOTE - this would be like saying all new values must come from a generator (not some external source)
  Kin = ConType([OBJ, POSE, CONF, POSE], name='Kin')
  #Contained = ConType([REG, POSE], name='Contained', test=region_contains)
  #Contained = ConType([OBJ, POSE, REG], name='Contained', test=region_contains, domain=[Stable(o, p)])
  Contained = ConType([OBJ, POSE, REG], name='Contained', test=region_contains)

  # TODO - could specify axiom creation here

  state_vars = [Var(R, CONF), Var(O, POSE, args=[OBJ]), Var(H, BOOL, args=[OBJ])] #+ [Var(HE, BOOL)]
  control_vars = [Var(T, TRAJ)]

  ##########

  o = Par('o', OBJ)
  # actions = [
  #   Action([Motion(x[R], u[T], nx[R])], name='move'),
  #   Action([Stable(o, x[O,o]), Grasp(o, nx[O,o]), Kin(o, nx[O,o], x[R], x[O,o]),
  #           Eq(x[H], None), Eq(nx[H], o)], name='pick'),
  #   Action([Grasp(o, x[O,o]), Stable(o, nx[O,o]), Kin(o, x[O,o], x[R], nx[O,o]),
  #           Eq(x[H], o), Eq(nx[H], None)], name='place')]
  #actions = [
  #  Action([Motion(x[R], u[T], nx[R])], name='move'),
  #  Action([Stable(o, x[O,o]), Grasp(o, nx[O,o]), Kin(o, nx[O,o], x[R], x[O,o]),
  #          Eq(x[HE], True), Eq(nx[HE], False), Eq(x[H,o], False), Eq(nx[H,o], True)], name='pick'),
  #  Action([Grasp(o, x[O,o]), Stable(o, nx[O,o]), Kin(o, x[O,o], x[R], nx[O,o]),
  #          Eq(x[HE], False), Eq(nx[HE], True), Eq(x[H,o], True), Eq(nx[H,o], False)], name='place')]
  actions = [
    Action([Motion(X[R], U[T], nX[R])], name='move'),
    Action([Stable(o, X[O,o]), Grasp(o, nX[O,o]), Kin(o, nX[O,o], X[R], X[O,o]), Eq(nX[H,o], True)] + \
      [Eq(X[H,ob], False) for ob in oracle.get_objects()], name='pick'),
    Action([Grasp(o, X[O,o]), Stable(o, nX[O,o]), Kin(o, X[O,o], X[R], nX[O,o]),
            Eq(X[H,o], True), Eq(nX[H,o], False)], name='place')]

  ##########

  q1, t, q2 = Par(CONF), Par(TRAJ), Par(CONF)
  r, p, g = Par(REG), Par(POSE), Par(POSE)
  samplers = [
    #Sampler([Motion(q1, t, q2)], inputs=[q1, q2], gen=lambda a,b: iter([None])),
    Sampler([Motion(q1, t, q2)], gen=motion_plan, inputs=[q1, q2]),
    #Sampler([Stable(o, p)]),
    Sampler([Kin(o, g, q1, p)], gen=sample_ik, inputs=[o, g, p], domain=[Stable(o, p), Grasp(o, g)]),
  ]
  if not FIXED_POSES:
    samplers += [
      Sampler([Contained(o, p, r), Stable(o, p)], gen=sample_region, inputs=[o, r]),
      Sampler([Grasp(o, g)], gen=sample_grasp, inputs=[o]),
    ]

  # NOTE - discard any sampler not mentioned in start or goal, etc

  ##########

  #initial_state = [Eq(x[R], oracle.initial_config), Eq(x[H], None)] + \
  #  [Eq(x[O,obj], pose) for obj, pose in oracle.initial_poses.iteritems()]
  #initial_state = [Eq(x[R], oracle.initial_config), Eq(x[HE], False)] + \
  initial_state = [Eq(X[R], oracle.initial_config)] + \
    [Eq(X[O,obj], pose) for obj, pose in oracle.initial_poses.iteritems()] + \
    [Eq(X[H,obj], False) for obj in oracle.get_objects()]

  # NOTE - can specify facts in the initial state
  #initial_state += [Stable(obj, pose) for obj, pose in oracle.initial_poses.iteritems()]
  # TODO - should I instead just have a generic test?

  obj0 = oracle.get_objects()[0]
  #reg0 = oracle.goal_regions[0]
  goal_constraints = [
    #Eq(x[R], oracle.initial_config),
    #Eq(x[R], None),
    #Contained(reg0, X[O,obj0])
    #Eq(X[H,obj0], True),
  ]

  problem = oracle.problem
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
    if problem.goal_poses[obj] == 'initial':
      problem.goal_poses[obj] = oracle.initial_poses[obj]
    elif problem.goal_poses[obj] in oracle.initial_poses: # Goal names other object (TODO - compile with initial)
      problem.goal_poses[obj] = oracle.initial_poses[problem.goal_poses[obj]]
    goal_constraints.append(Eq(X[O,obj], problem.goal_poses[obj]))
  for obj, region in problem.goal_regions.iteritems():
    goal_constraints.append(Contained(obj, X[O,obj], region))

  return ConstraintProblem(state_vars, control_vars, actions, samplers,
                           initial_state, goal_constraints)

############################################################

# TODO - should I just pick things up vertically or move them interestingly
# NOTE - I don't even need poses if I take movements

def dantam_problem(oracle):
  problem = oracle.problem
  obj_name = oracle.get_objects()[0]
  grasp = get_grasps(oracle, obj_name)[0]
  assert problem.initial_poses is not None and problem.known_poses is not None
  oracle.initial_poses = problem.initial_poses # TODO - hack to avoid duplicating poses

  ##########

  DO_MOTION = False
  max_failures = 50 # 10 | 20 | 40
  max_calls = 1 # 1 | INF
  iterator = custom_base_iterator # current_base_iterator | custom_base_iterator
  def sample_ik(p):
    saver = oracle.state_saver()
    oracle.set_all_object_poses({obj_name: p}) # TODO - saver for the initial state as well?
    if oracle.approach_collision(obj_name, p, grasp):
      return
    for i in range(max_calls):
      pap = PickAndPlace(oracle.get_geom_hash(obj_name), p, grasp)
      if not pap.sample(oracle, obj_name, base_iterator_fn=iterator,
                        max_failures=max_failures,
                        sample_vector=DO_MOTION, sample_arm=DO_MOTION, check_base=False):
        break
      yield pap.approach_config
      #yield (pap.approach_config, pap)
      saver.Restore()

  def motion_plan(q1, q2):
    yield None

  ##########

  CONF, TRAJ = Ty('CONF'), Ty('TRAJ')

  # TODO - assert that all objects have the same geometry
  poses = oracle.problem.known_poses + oracle.initial_poses.values() # TODO - add goal poses
  grasps = [None]
  POSE = Ty('POSE', domain=poses+grasps)
  OBJ = Ty('OBJ', domain=oracle.get_objects())

  R, O, T = 'R', 'O', 'T'

  Motion = ConType([CONF, TRAJ, CONF], name='Motion')
  Placed = ConType([POSE], name='Placed', test=lambda p: isinstance(p, Pose))
  Kin = ConType([POSE, CONF], name='Kin')
  PoseCFree = ConType([POSE, POSE], name='PCFree', test=lambda p1, p2: p1 is None or p2 is None or p1 != p2)

  # TODO - could specify axiom creation here
  state_vars = [Var(R, CONF), Var(O, POSE, args=[OBJ])]
  control_vars = [Var(T, TRAJ)]

  ##########

  o = Par('o', OBJ)
  actions = [
    Action([Motion(X[R], U[T], nX[R])], name='move'),
    Action([Kin(X[O,o], X[R]),
            Eq(nX[O,o], None)] + \
      [Placed(X[O,ob]) for ob in oracle.get_objects()], name='pick'),
    Action([Eq(X[O,o], None), Kin(nX[O,o], X[R]),
            Placed(nX[O,o])] + \
           [PoseCFree(nX[O,o], X[O,ob]) for ob in oracle.get_objects()],
           name='place')]

  ##########

  q1, t, q2 = Par(CONF), Par(TRAJ), Par(CONF)
  p = Par(POSE)
  samplers = [
    #Sampler([Motion(q1, t, q2)], inputs=[q1, q2], gen=lambda a,b: iter([None])),
    Sampler([Motion(q1, t, q2)], gen=motion_plan, inputs=[q1, q2]),
    #Sampler([Stable(o, p)]),
    Sampler([Kin(p, q1)], gen=sample_ik, inputs=[p], domain=[Placed(p)]),
  ]

  ##########

  initial_state = [Eq(X[R], oracle.initial_config)] + \
    [Eq(X[O,obj], pose) for obj, pose in oracle.initial_poses.iteritems()]

  obj0 = oracle.get_objects()[0]
  goal_constraints = [
    #Eq(x[R], oracle.initial_config),
    #Eq(x[R], None),
    #Contained(reg0, X[O,obj0])
    #Eq(X[H,obj0], True),
  ]
  for obj in problem.goal_poses:
    assert problem.goal_poses[obj] != 'initial' and problem.goal_poses[obj] not in oracle.initial_poses
    goal_constraints.append(Eq(X[O,obj], problem.goal_poses[obj]))
  assert len(problem.goal_regions) == 0

  return ConstraintProblem(state_vars, control_vars, actions, samplers,
                           initial_state, goal_constraints)
