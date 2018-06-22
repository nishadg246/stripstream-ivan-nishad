from factored_constraint import ConType, Ty as Type, Var, Eq, \
  X, U, nX, Action as Clause, Sampler, Par as Param
from manipulation import rrt, sample_pose, sample_grasp, \
  inverse_kin, traj_collision, objects

# from factored_constraint import ConType, Ty, Var, Eq, \
#   X, U, nX, Action as Cl, Sampler, Par
# from manipulation import rrt, sample_pose, sample_grasp, \
#   inverse_kin, traj_collision, objects
# CONF, BOOL, TRAJ = Ty(), Ty(), Ty()
# OBJ, POSE = Ty(domain=objects), Ty()
# xvars = [Var('R',CONF),Var('H',BOOL),Var('O',POSE,args=[OBJ])]
# uvars = [Var('T',TRAJ)]
# q1, t, q2 = Par(CONF), Par(TRAJ), Par(CONF)
# o, p, g = Par(OBJ), Par(POSE), Par(POSE)
#
# Motion = ConType([CONF,TRAJ,CONF])
# MotionH = ConType([CONF,TRAJ,CONF,POSE])
# CFree = ConType([TRAJ, POSE], test=traj_collision)
# Stable, Grasp = ConType([OBJ,POSE]), ConType([OBJ,POSE])
# Kin = ConType([OBJ,POSE,CONF,POSE])
# transitions = [
#   Cl([Motion(X['R'],U['T'],nX['R']),Eq(X['H'],None)] +
#     [CFree(U['T'],X['O',ob]) for ob in objects]),#Move
#   Cl([MotionH(X['R'],U['T'],nX['R'],X['O',o])] +
#     [CFree(U['T'],X['O',ob]) for ob in objects]),#MoveH
#   Cl([Stable(o,X['O',o]),Grasp(o,nX['O',o]),Eq(X['H'],None),
#     Eq(nX['H'],o),Kin(o,nX['O',o],X['R'],X['O',o])]),#Pick
#   Cl([Grasp(o,X['O',o]),Stable(o,nX['O',o]),Eq(X['H'],o),
#     Eq(nX['H'],None),Kin(o,X['O',o],X['R'],nX['O',o])])]#Place
# samplers = [
#   Sampler([Motion(q1,t,q2)], inputs=[q1,q2], gen=rrt),
#   Sampler([Stable(o,p)], gen=sample_pose),
#   Sampler([Grasp(o,g)], gen=sample_grasp),
#   Sampler([Kin(o,g,q1,p)], inputs=[o,g,p],
#     domain=[Stable(o,p), Grasp(o,g)], gen=inverse_kin)]

#from factored_constraint import ConType, Type, Var, Eq, \
#  X, U, nX, Clause, Sampler, Param

# def collision_free(p, t): return ...
# def sample_grasp(o): yield ...
# def sample_motion(o, g, p): yield ...
# def sample_pose(o): yield ...
# def sample_ik(o, p, g): yield ...

def collision_free(p, t): pass
def sample_grasp(o): pass
def sample_motion(o, g, p): pass
def sample_pose(o): pass
def sample_ik(o, p, g): pass

def pick_and_place(objects):
  CONF, BOOL, TRAJ = Type(), Type(), Type()
  OBJ, POSE = Type(domain=objects), Type()
  state_vars = [Var('R', CONF), Var('H', BOOL),Var('O', POSE, args=[OBJ])]
  control_vars = [Var('T', TRAJ)]
  q1, t, q2 = Param(CONF), Param(TRAJ), Param(CONF)
  o, p, g = Param(OBJ), Param(POSE), Param(POSE)

  Motion = ConType([CONF, TRAJ, CONF])
  MotionH = ConType([CONF, TRAJ, CONF, POSE])
  CFree = ConType([TRAJ, POSE], test=collision_free)
  Stable, Grasp = ConType([OBJ, POSE]), ConType([OBJ, POSE])
  Kin = ConType([OBJ, POSE, CONF, POSE])
  transitions = [
    Clause([Motion(X['R'], U['T'], nX['R']),Eq(X['H'], None)] +
      [CFree(U['T'], X['O',ob]) for ob in objects]), #Move
    Clause([MotionH(X['R'], U['T'], nX['R'], X['O',o])] +
      [CFree(U['T'], X['O',ob]) for ob in objects]), #MoveH
    Clause([Stable(o, X['O',o]), Grasp(o, nX['O',o]), Eq(X['H'],None),
      Eq(nX['H'],o), Kin(o,nX['O',o], X['R'], X['O',o])]), #Pick
    Clause([Grasp(o, X['O',o]), Stable(o,nX['O',o]),Eq(X['H'],o),
      Eq(nX['H'], None), Kin(o, X['O',o], X['R'], nX['O',o])])] #Place
  samplers = [
    Sampler([Motion(q1, t, q2)], inputs=[q1,q2], gen=sample_motion),
    Sampler([Stable(o, p)], gen=sample_pose),
    Sampler([Grasp(o,g)], gen=sample_grasp),
    Sampler([Kin(o, g, q1, p)], inputs=[o,g,p],
      domain=[Stable(o, p), Grasp(o, g)], gen=sample_ik)]

#####################################

# from factored_constraint import ConType, Type, Var, Eq, \
#   X, U, nX, Clause, Sampler, Param
#
# def collision_free(p, t): return ...
# def sample_grasp(o): yield ...
# def sample_combined_motion(o, g, p): yield ...
# def sample_region(o, r): yield ...
# def sample_pose(o): yield ...

def collision_free(p, t): pass
def sample_grasp(o): pass
def sample_combined_motion(o, g, p): pass
def sample_region(o, r): pass
def sample_pose(o): pass

def pr2_pick_and_place(objects, regions, initial_poses, goal_poses, goal_regions):
  POSE, TRAJ = Type(), Type()
  BOOL = Type(domain=[True, False])
  OBJ = Type(domain=objects)
  REG = Type(domain=regions)
  state_vars = [Var('O', POSE, args=[OBJ]), Var('H', BOOL, args=[OBJ])]
  control_vars = [Var('T', TRAJ)]
  t, p, g = Param(TRAJ), Param(POSE), Param(POSE)
  o, r = Param('o', OBJ), Param('r', REG)

  Stable = ConType([OBJ, POSE], satisfying=initial_poses.items() + goal_poses.items())
  Grasp = ConType([OBJ, POSE])
  CombinedMotion = ConType([OBJ, POSE, POSE, TRAJ])
  CFree = ConType([POSE, TRAJ], test=collision_free)
  Contained = ConType([POSE, REG])
  IsGoal = ConType([OBJ, REG], satisfying=goal_regions.items())
  transitions = [
    Clause([Stable(o, X['O',o]), Grasp(o, nX['O',o]), CombinedMotion(o, nX['O',o], X['O',o], U['T']),
              Eq(X['H',o], False), Eq(nX['H',o], True)] +
           [CFree(X['O',ob], U['T']) for ob in objects] +
           [Eq(X['H',ob], False) for ob in objects]), # Move, Pick, MoveH
    Clause([Grasp(o, X['O',o]), Stable(o, nX['O',o]), CombinedMotion(o, X['O',o], nX['O',o], U['T']),
              Eq(X['H',o], True), Eq(nX['H',o], False)] +
           [CFree(X['O',ob], U['T']) for ob in objects])] # MoveH, Place, Move
  samplers = [
    Sampler([Stable(o, p), Contained(p, r)], gen=sample_region,
            inputs=[o, r], domain=[IsGoal(o, r)]),
    Sampler([Stable(o, p)], gen=sample_pose, inputs=[o]),
    Sampler([Grasp(o, g)], gen=sample_grasp, inputs=[o]),
    Sampler([CombinedMotion(o, g, p, t)], gen=sample_combined_motion,
            inputs=[o, g, p], domain=[Stable(o, p), Grasp(o, g)])]

#####################################

# from factored_constraint import ConType, Type, Var, Eq, \
#   X, U, nX, Clause, Sampler, Param
#
# def collision_free(q1, q2): return ...
# def sample_config(): yield ...
def collision_free(q1, q2): pass
def sample_config(): pass

def motion_planning():
  CONF = Type()
  state_vars = [Var('Q', CONF)]
  control_vars = []
  q1, q2, g = Param(CONF), Param(CONF)

  CFree = ConType([CONF, CONF], test=collision_free)
  transitions = [Clause([CFree(X['Q'], nX['Q'])])]
  samplers = [Sampler([q1], gen=sample_config)]