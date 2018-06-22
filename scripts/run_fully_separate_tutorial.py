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
from stripstream.algorithms.search.fast_forward import get_fast_forward

# TODO - make this executable within the API?

# TODO: debug mode that prints the value of things instead of the index

def create_problem(n=50):
  """
  Creates the 1D task and motion planning STRIPStream problem.

  :return: a :class:`.STRIPStreamProblem`
  """

  blocks = ['block%i'%i for i in xrange(n)]
  num_poses = pow(10, 10)

  initial_config = 0 # the initial robot configuration is 0
  initial_poses = {block: i for i, block in enumerate(blocks)} # the initial pose for block i is i

  goal_poses = {blocks[1]: 2} # the goal pose for block i is i+1

  ####################

  # TODO: the last version of this would be to make a separate pose type per object (I think I did do this)

  CONF = Type()
  HandEmpty = Pred()
  AtConf = Pred(CONF)
  Q1, Q2 = Param(CONF), Param(CONF)

  #POSE = Type()
  #Kin = Pred(POSE, CONF)
  #Collision = Pred(POSE, POSE)
  #P1, P2 = Param(POSE), Param(POSE)

  #rename_easy(locals()) # Trick to make debugging easier

  actions = [
    Action(name='move', parameters=[Q1, Q2],
      condition=AtConf(Q1),
      effect=And(AtConf(Q2), Not(AtConf(Q1)))),
  ]
  axioms = []

  cond_streams = [
    #GeneratorStream(inputs=[P1], outputs=[Q1], conditions=[], effects=[Kin(P1, Q1)],
    #                generator=lambda p: [(p,)]), # Inverse kinematics

    #TestStream(inputs=[P1, P2], conditions=[], effects=[Collision(P1, P2)],
    #           test=lambda p1, p2: p1 == p2, eager=True),
  ]

  initial_atoms = [
    AtConf(initial_config),
    HandEmpty(),
  ]
  goal_literals = []

  ####################

  # TODO: I think thinking invariants gets harder with many predicates. Can cap this time I believe though
  #153 initial candidates
  #Finding invariants: [2.250s CPU, 2.263s wall - clock]

  for b in blocks:
    # TODO: can toggle using individual pose types
    POSE = Type()
    Kin = Pred(POSE, CONF)
    P1 = Param(POSE)

    AtPose = Pred(POSE)
    IsPose = Pred(POSE)
    Holding = Pred()
    #Unsafe = Pred(BLOCK, POSE)

    initial_atoms += [
      AtPose(initial_poses[b]),
      IsPose(initial_poses[b]),
    ]
    if b in goal_poses:
      goal_literals += [AtPose(goal_poses[b])]
      initial_atoms += [IsPose(goal_poses[b])]

    axioms += [
      # TODO: to do this, need to make b a parameter and set it using inequality
      #Axiom(effect=Unsafe(b, P1),
      #            condition=Exists([P2], And(AtPose(b, P2), IsPose(b, P2), Collision(P1, P2)))),
    ]
    actions += [
      Action(name='pick-{}'.format(b), parameters=[P1, Q1],
             condition=And(AtPose(P1), HandEmpty(), AtConf(Q1), IsPose(P1), Kin(P1, Q1)),
             effect=And(Holding(), Not(AtPose(P1)), Not(HandEmpty()))),
      Action(name='place-{}'.format(b), parameters=[P1, Q1],
             condition=And(Holding(), AtConf(Q1), IsPose(P1), Kin(P1, Q1)),
                           #*[Safe(b2, P1) for b2 in blocks if b2 != b]),
                           #*[Not(Unsafe(b2, P1)) for b2 in blocks if b2 != b]),
             effect=And(AtPose(P1), HandEmpty(), Not(Holding()))),
    ]
    cond_streams += [
      GeneratorStream(inputs=[], outputs=[P1], conditions=[], effects=[IsPose(P1)],
                      generator=lambda: ((p,) for p in xrange(n, num_poses))),
      GeneratorStream(inputs=[P1], outputs=[Q1], conditions=[], effects=[Kin(P1, Q1)],
                      generator=lambda p: [(p,)]), # Inverse kinematics
    ]

  problem = STRIPStreamProblem(initial_atoms, goal_literals, actions + axioms, cond_streams, [])

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

  # TODO: maybe the problem was with FastForward previously?
  #search = get_fast_forward()
  search = get_fast_downward('eager', max_time=10, verbose=True) # dijkstra | astar
  plan, _ = incremental_planner(problem, search=search, max_time=30, optimal=False)
  print
  print 'Plan:', convert_plan(plan)

# TODO - visualize by applying actions to env state

if __name__ == '__main__':
  main()
