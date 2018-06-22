#!/usr/bin/env python

from stripstream.pddl.logic.predicates import EasyPredicate as Pred
from stripstream.pddl.objects import EasyType as Type, EasyParameter as Param
from stripstream.pddl.logic.connectives import Not
from stripstream.pddl.operators import STRIPSAction, STRIPSAxiom
from stripstream.pddl.cond_streams import EasyGenStream as GeneratorStream, EasyTestStream as TestStream
from stripstream.pddl.utils import rename_easy
from stripstream.pddl.problem import STRIPStreamProblem

# TODO - make this executable within the API?

def create_problem():
  """
  Creates the 1D task and motion planning STRIPStream problem.

  :return: a :class:`.STRIPStreamProblem`
  """

  blocks = ['block%i'%i for i in range(3)]
  num_poses = pow(10, 10)

  initial_config = 0 # the initial robot configuration is 0
  initial_poses = {block: i for i, block in enumerate(blocks)} # the initial pose for block i is i

  goal_poses = {block: i+1 for i, block in enumerate(blocks)} # the goal pose for block i is i+1

  ####################

  VALUE = Type()

  # Fluent predicates
  AtConf = Pred(VALUE)
  AtPose = Pred(VALUE, VALUE)
  HandEmpty = Pred()
  Holding = Pred(VALUE)

  # Derived predicates
  Safe = Pred(VALUE, VALUE, VALUE)

  # Static predicates
  IsBlock = Pred(VALUE)
  IsPose = Pred(VALUE)
  IsConf = Pred(VALUE)
  LegalKin = Pred(VALUE, VALUE)
  CollisionFree = Pred(VALUE, VALUE, VALUE, VALUE)

  # Free parameters
  B1, B2 = Param(VALUE), Param(VALUE)
  P1, P2 = Param(VALUE), Param(VALUE)
  Q1, Q2 = Param(VALUE), Param(VALUE)

  rename_easy(locals()) # Trick to make debugging easier

  ####################

  # NOTE - it would be easier to just update an in hand pose

  actions = [
    STRIPSAction(name='pick', parameters=[B1, P1, Q1],
      conditions=[IsBlock(B1), IsPose(P1), IsConf(Q1), LegalKin(P1, Q1),
                  AtPose(B1, P1), HandEmpty(), AtConf(Q1)],
      effects=[Holding(B1), Not(AtPose(B1, P1)), Not(HandEmpty())]),

    STRIPSAction(name='place', parameters=[B1, P1, Q1],
      conditions=[IsBlock(B1), IsPose(P1), IsConf(Q1), LegalKin(P1, Q1),
                  Holding(B1), AtConf(Q1)] + [Safe(b2, B1, P1) for b2 in blocks],
      effects=[AtPose(B1, P1), HandEmpty(), Not(Holding(B1))]),

    STRIPSAction(name='move', parameters=[Q1, Q2],
      conditions=[IsConf(Q1), IsConf(Q2), AtConf(Q1)],
      effects=[AtConf(Q2), Not(AtConf(Q1))]),
  ]

  axioms = [ # TODO - need to combine axioms
    #Axiom(effect=Safe(B2, B1, P1),
    #      condition=Exists([P2], And(AtPose(B2, P2), CollisionFree(B1, P1, B2, P2)))), # Infers B2 is at a safe pose wrt B1 at P1
  ]

  ####################

  # Conditional stream declarations
  cond_streams = [
    GeneratorStream(inputs=[], outputs=[P1], conditions=[], effects=[IsPose(P1)],
                    generator=lambda: xrange(num_poses)), # Enumerating all the poses

    GeneratorStream(inputs=[P1], outputs=[Q1], conditions=[IsPose(P1)], effects=[IsConf(Q1), LegalKin(P1, Q1)],
                    generator=lambda p: [p]), # Inverse kinematics

    TestStream(inputs=[B1, P1, B2, P2], conditions=[IsBlock(B1), IsPose(P1), IsBlock(B2), IsPose(P2)],
               effects=[CollisionFree(B1, P1, B2, P2)], test=lambda b1, p1, b2, p2: p1 != p2, eager=True), # Collision checking
  ]

  ####################

  constants = []

  initial_atoms = [
    IsConf(initial_config),
    AtConf(initial_config),
    HandEmpty()
  ]
  for block, pose in initial_poses.iteritems():
    initial_atoms += [
      IsBlock(block),
      IsPose(pose),
      AtPose(block, pose),
    ]
  goal_literals = []
  for block, pose in goal_poses.iteritems():
    initial_atoms += [
      IsBlock(block),
      IsPose(pose),
    ]
    goal_literals.append(AtPose(block, pose))

  problem = STRIPStreamProblem(initial_atoms, goal_literals, actions + axioms, cond_streams, constants)

  return problem

##################################################

from stripstream.algorithms.incremental.incremental_planner import incremental_planner
from stripstream.pddl.utils import convert_plan

def main():
  """
  Creates and solves the 1D task and motion planning STRIPStream problem.
  """

  problem = create_problem()
  print problem
  plan, _ = incremental_planner(problem)
  print
  print 'Plan:', convert_plan(plan)

# TODO - visualize by applying actions to env state

if __name__ == '__main__':
  main()
