#!/usr/bin/env python

from stripstream.pddl.logic.predicates import EasyPredicate as Pred
from stripstream.pddl.objects import EasyType as Type, EasyParameter as Param
from stripstream.pddl.logic.connectives import Not, Or, And
from stripstream.pddl.logic.quantifiers import Exists, ForAll
from stripstream.pddl.logic.atoms import Equal
from stripstream.pddl.operators import Action, Axiom
from stripstream.pddl.cond_streams import EasyGenStream as GeneratorStream, EasyTestStream as TestStream
from stripstream.pddl.utils import rename_easy
from stripstream.pddl.problem import STRIPStreamProblem
from stripstream.algorithms.search.fast_downward import get_fast_downward

# TODO - make this executable within the API?

# TODO: debug mode that prints the value of things instead of the index

def create_problem(n=40):
  """
  Creates the 1D task and motion planning STRIPStream problem.

  :return: a :class:`.STRIPStreamProblem`
  """

  blocks = ['block%i'%i for i in xrange(n)]
  num_poses = pow(10, 10)

  initial_config = 0 # the initial robot configuration is 0
  initial_poses = {block: i for i, block in enumerate(blocks)} # the initial pose for block i is i

  goal_poses = {blocks[0]: 1} # the goal pose for block i is i+1

  ####################

  # Data types
  CONF, BLOCK, POSE = Type(), Type(), Type()

  # Fluent predicates
  AtConf = Pred(CONF)
  AtPose = Pred(BLOCK, POSE)
  IsPose = Pred(BLOCK, POSE)
  HandEmpty = Pred()
  Holding = Pred(BLOCK)

  # Derived predicates
  Safe = Pred(BLOCK, POSE)
  Unsafe = Pred(BLOCK, POSE)

  # Static predicates
  Kin = Pred(POSE, CONF)
  CFree = Pred(POSE, POSE)
  Collision = Pred(POSE, POSE)

  # Free parameters
  B1, B2 = Param(BLOCK), Param(BLOCK)
  P1, P2 = Param(POSE), Param(POSE)
  Q1, Q2 = Param(CONF), Param(CONF)

  rename_easy(locals()) # Trick to make debugging easier

  actions = [
    Action(name='move', parameters=[Q1, Q2],
      condition=AtConf(Q1),
      effect=And(AtConf(Q2), Not(AtConf(Q1)))),
  ]

  axioms = [
    #Axiom(effect=Safe(B2, P1),
    #      condition=Or(Holding(B2), Exists([P2], And(AtPose(B2, P2), IsPose(B2, P2), Not(Collision(P1, P2)))))),

    Axiom(effect=Unsafe(B2, P1),
                condition=Exists([P2], And(AtPose(B2, P2), IsPose(B2, P2), Collision(P1, P2)))),
  ]

  ####################

  # Conditional stream declarations
  cond_streams = [
    #GeneratorStream(inputs=[B1], outputs=[P1], conditions=[], effects=[IsPose(B1, P1)],
    #                generator=lambda b: ((p,) for p in xrange(n, num_poses))),

    GeneratorStream(inputs=[P1], outputs=[Q1], conditions=[], effects=[Kin(P1, Q1)],
                    generator=lambda p: [(p,)]), # Inverse kinematics

    TestStream(inputs=[P1, P2], conditions=[], effects=[Collision(P1, P2)],
               test=lambda p1, p2: p1 == p2, eager=True),
  ]

  # TODO: this actually isn't slow at all. What was the problem then?
  # Maybe it was that I was creating many predicates?
  # It could have also been the lack of IsPose predicates

  for b in blocks:
    axioms += [
      # TODO: to do this, need to make b a parameter and set it using inequality
      #Axiom(effect=Unsafe(b, P1),
      #            condition=Exists([P2], And(AtPose(b, P2), IsPose(b, P2), Collision(P1, P2)))),
    ]
    actions += [
      Action(name='pick-{}'.format(b), parameters=[P1, Q1],
             condition=And(AtPose(b, P1), HandEmpty(), AtConf(Q1), IsPose(b, P1), Kin(P1, Q1)),
             effect=And(Holding(b), Not(AtPose(b, P1)), Not(HandEmpty()))),
      Action(name='place-{}'.format(b), parameters=[P1, Q1],
             condition=And(Holding(b), AtConf(Q1), IsPose(b, P1), Kin(P1, Q1),
                           #*[Safe(b2, P1) for b2 in blocks if b2 != b]),
                           *[Not(Unsafe(b2, P1)) for b2 in blocks if b2 != b]),
             effect=And(AtPose(b, P1), HandEmpty(), Not(Holding(b)))),
    ]
    cond_streams += [
      GeneratorStream(inputs=[], outputs=[P1], conditions=[], effects=[IsPose(b, P1)],
                      generator=lambda: ((p,) for p in xrange(n, num_poses))),
    ]


  ####################

  constants = [
    CONF(initial_config) # Any additional objects
  ]

  initial_atoms = [
    AtConf(initial_config),
    HandEmpty(),
  ] + [
    AtPose(block, pose) for block, pose in initial_poses.items()
  ]  + [
    IsPose(block, pose) for block, pose in (initial_poses.items() + goal_poses.items())
  ]

  goal_literals = [AtPose(block, pose) for block, pose in goal_poses.iteritems()]

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

  search = get_fast_downward('eager', max_time=10, verbose=True) # dijkstra | astar
  plan, _ = incremental_planner(problem, search=search, max_time=30, optimal=False)
  print
  print 'Plan:', convert_plan(plan)

# TODO - visualize by applying actions to env state

if __name__ == '__main__':
  main()
