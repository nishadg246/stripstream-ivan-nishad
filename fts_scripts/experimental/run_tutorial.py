#!/usr/bin/env python2

from stripstream.fts.constraint import Eq, ConType, Unconstrained
from stripstream.fts.variable import VarType, Par, Var, X, U, nX
from stripstream.fts.clause import Clause
from stripstream.fts.sampler import Sampler
from stripstream.fts.problem import FTSProblem
from stripstream.fts.stripstream_conversion import constraint_to_stripstream
from stripstream.fts.utils import convert_plan, rename_variables

from stripstream.algorithms.search.fast_downward import get_fast_downward
from stripstream.algorithms.incremental.incremental_planner import incremental_planner

# TODO - make the name of the variable a parameter?

def create_problem():
  """
  Creates the 1D task and motion planning FTSProblem problem.

  :return: a :class:`.FTSProblem`
  """

  blocks = ['block%i'%i for i in range(2)]
  num_poses = pow(10, 10)

  initial_config = 0 # the initial robot configuration is 0
  initial_poses = {block: i for i, block in enumerate(blocks)} # the initial pose for block i is i

  goal_poses = {block: i+1 for i, block in enumerate(blocks)} # the goal pose for block i is i+1
  #goal_poses = {blocks[0]: 1}

  ####################

  # TODO - rethink the naming...
  R_Q, B_P, B_H = 'R_Q', 'B_P', 'B_H'
  R_T = 'R_T'

  # NOTE - these should really just be param type
  CONF = VarType()
  BOOL = VarType(domain=[True, False])
  POSE = VarType()
  BLOCK = VarType(domain=blocks) # TODO - VarType vs ParamType?
  TRAJ = VarType()

  B, Q, P = Par(BLOCK), Par(CONF), Par(POSE)
  T, Q2 = Par(TRAJ), Par(CONF)

  LegalKin = ConType([POSE, CONF])
  CollisionFree = ConType([POSE, POSE], test=lambda p1, p2: None in (p1, p2) or p1 != p2)
  Motion = ConType([CONF, TRAJ, CONF])

  rename_variables(locals()) # Trick to make debugging easier

  # NOTE - can make a holding variable for each object or just one holding variable

  # TODO - maybe declare the variables upfront
  state_vars = [Var(R_Q, CONF), Var(B_P, POSE, args=[BLOCK]), Var(B_H, BOOL, args=[BLOCK])]
  control_vars = [Var(R_T, TRAJ)]

  ##########

  # NOTE - didn't I have some kind of bug where things had to be effects for this to work?
  # NOTE - this is because I have to write things in the form where we only mention things that change....
  # THus, I need the identify constraint or something

  transition = [
    Clause([LegalKin(X[B_P, B], X[R_Q]), Eq(nX[B_P, B], None), Eq(X[B_H, B], False), Eq(nX[B_H, B], True)] +
      [Eq(X[B_H, block], False) for block in blocks], name='pick'),

    #Clause([LegalKin(X[B_P, B], X[R_Q]), Eq(nX[B_P, B], None), Eq(nX[B_H, B], True)] +
    #  [Eq(X[B_H, block], False) for block in blocks], name='pick'), # NOTE - this makes bool free params internally

    Clause([LegalKin(nX[B_P, B], X[R_Q]), Eq(X[B_P, B], None), Eq(X[B_H, B], True), Eq(nX[B_H, B], False)] +
      [CollisionFree(X[B_P, block], nX[B_P, B]) for block in blocks], name='place'),

    #Clause([LegalKin(nX[B_P, B], X[R_Q]), Eq(X[B_P, B], None), Eq(nX[B_H, B], False)], # +
    ##Clause([LegalKin(nX[B_P, B], X[R_Q]), Eq(X[B_H, B], True), Eq(nX[B_H, B], False)], # +
    #  [CollisionFree(X[B_P, block], nX[B_P, B]) for block in blocks], name='place'),

    #Clause([Unconstrained(nX[R_Q])], name='move'), # NOTE - only write what changes
    Clause([Motion(X[R_Q], U[R_T], nX[R_Q])], name='move'),
  ]

  ##########

  # TODO - expand so we don't need to return lists of lists
  samplers = [
    Sampler([P], gen=lambda: ([(p,)] for p in xrange(num_poses)), inputs=[]),
    Sampler([LegalKin(P, Q)], gen=lambda p: [[(p,)]] if p is not None else [], inputs=[P]),
    Sampler([Motion(Q, T, Q2)], gen=lambda q1, q2: [[((q1, q2),)]], inputs=[Q, Q2]), # TODO - avoid this much nesting
  ]

  ##########

  initial_state = [Eq(X[R_Q], initial_config)] + \
                  [Eq(X[B_H, block], False) for block in blocks] + \
                  [Eq(X[B_P, block], pose) for block, pose in initial_poses.iteritems()]
  goal_constraints = [Eq(X[B_P, block], pose) for block, pose in goal_poses.iteritems()]
  #goal_constraints = [Eq(X[R_Q], 1)]

  return FTSProblem(state_vars, control_vars, transition, samplers,
                           initial_state, goal_constraints)

##################################################

def main():
  """
  Creates and solves the 1D task and motion planning FTSProblem problem.
  """

  constraint_problem = create_problem()
  print
  print constraint_problem

  stream_problem = constraint_to_stripstream(constraint_problem)
  print
  print stream_problem

  search_fn = get_fast_downward('eager') # 'dijkstra | astar | wastar1 | wastar2 | wastar3 | eager | lazy

  plan, _ = incremental_planner(stream_problem, search=search_fn)
  print
  print 'Plan:', convert_plan(plan)

# TODO - visualize by applying actions to env state

if __name__ == '__main__':
  main()
