#!/usr/bin/env python2

from robotics.motion2D import get_distance, is_collision_free, \
  create_box, sample_box, is_region, draw_solution, sample, inf_sequence

from stripstream.pddl.logic.predicates import EasyPredicate as Pred, EasyFunction as Func
from stripstream.pddl.objects import EasyType as Type, EasyParameter as Param
from stripstream.pddl.logic.connectives import Not, And
from stripstream.pddl.logic.operations import Cost
from stripstream.pddl.operators import STRIPSAction, STRIPSAxiom, Action
from stripstream.pddl.cond_streams import EasyGenStream as GeneratorStream, EasyTestStream as TestStream, \
  EasyCostStream as CostStream
from stripstream.pddl.utils import rename_easy
from stripstream.pddl.problem import STRIPStreamProblem

from stripstream.algorithms.incremental.incremental_planner import incremental_planner
from stripstream.algorithms.search.fast_downward import get_fast_downward
from stripstream.pddl.utils import convert_plan

def create_problem(goal, obstacles=(), distance=.25, digits=3):
  """
  Creates a Probabilistic Roadmap (PRM) motion planning problem.

  :return: a :class:`.STRIPStreamProblem`
  """

  # Data types
  POINT = Type()
  REGION = Type()

  # Fluent predicates
  AtPoint = Pred(POINT)

  # Derived predicates
  InRegion = Pred(REGION)

  # Stream predicates
  AreNearby = Pred(POINT, POINT)
  IsEdge = Pred(POINT, POINT)
  Contained = Pred(POINT, REGION)

  # Functions
  Distance = Func(POINT, POINT)

  # Free parameters
  P1, P2 = Param(POINT), Param(POINT)
  R = Param(REGION)

  rename_easy(locals()) # Trick to make debugging easier

  ####################

  actions = [
    #STRIPSAction(name='move', parameters=[P1, P2],
    #  conditions=[AtPoint(P1), IsEdge(P1, P2)],
    #  effects=[AtPoint(P2), Not(AtPoint(P1))], cost=1), # Fixed cost

    #STRIPSAction(name='move', parameters=[P1, P2],
    #  conditions=[AtPoint(P1), IsEdge(P1, P2)],
    #  effects=[AtPoint(P2), Not(AtPoint(P1)), Cost(Distance(P1, P2))]), # Cost depends on parameters
    Action(name='move', parameters=[P1, P2],
                 condition=And(AtPoint(P1), IsEdge(P1, P2)),
                 effect=And(AtPoint(P2), Not(AtPoint(P1))),
                 costs=[1, Distance(P1, P2)]),
  ]

  axioms = [
    #Axiom(effect=GoalReached(), condition=Exists([P1], And(AtPos(P1), Contained(P1)))),
    STRIPSAxiom(conditions=[AtPoint(P1), Contained(P1, R)], effects=[InRegion(R)])
  ]

  ####################

  # Conditional stream declarations
  cond_streams = [
    GeneratorStream(inputs=[], outputs=[P1], conditions=[], effects=[],
                    generator=lambda: ((sample(digits),) for _ in inf_sequence())), # NOTE - version that only generators collision-free points

    GeneratorStream(inputs=[R], outputs=[P1], conditions=[], effects=[Contained(P1, R)],
                    generator=lambda r: ((sample_box(r),) for _ in inf_sequence())),

    #TestStream(inputs=[P1, P2], conditions=[], effects=[AreNearby(P1, P2), AreNearby(P2, P1)],
    #           test=lambda p1, p2: get_distance(p1, p2) <= distance, eager=True),

    #TestStream(inputs=[P1, P2], conditions=[AreNearby(P1, P2)], effects=[IsEdge(P1, P2), IsEdge(P2, P1)],
    #           test=lambda p1, p2: is_collision_free((p1, p2), obstacles), eager=True),

    TestStream(inputs=[P1, P2], conditions=[], effects=[IsEdge(P1, P2), IsEdge(P2, P1)],
               #test=lambda p1, p2: is_collision_free((p1, p2), obstacles), eager=True),
               test = lambda p1, p2: (get_distance(p1, p2) <= distance) and
                                     is_collision_free((p1, p2), obstacles), eager = True),

    #TestStream(inputs=[P1, P2], conditions=[], effects=[IsEdge(P1, P2), IsEdge(P2, P1)],
    #           test=lambda p1, p2: get_distance(p1, p2) <= distance and is_collision_free((p1, p2), obstacles), eager=True),

    #TestStream(inputs=[P1, R], conditions=[], effects=[Contained(P1, R)],
    #           test=lambda p, r: contains(p, r), eager=True),

    CostStream(inputs=[P1, P2], conditions=[], effects=[Distance(P1, P2), Distance(P2, P1)],
               function=get_distance, scale=100, eager=True),
  ]

  ####################

  constants = []

  initial_atoms = [
    AtPoint((0, 0)),
  ]

  goal_literals = []
  if is_region(goal):
    goal_literals.append(InRegion(goal))
  else:
    goal_literals.append(AtPoint(goal))

  problem = STRIPStreamProblem(initial_atoms, goal_literals, actions + axioms, cond_streams, constants)

  return problem

##################################################

# TODO - algorithms that take advantage of metric space (RRT)

def main(optimal=True, max_time=20):
  """
  Creates and solves the 2D motion planning problem.
  """

  obstacles = [
    create_box((.5, .5), .2, .2)
  ]
  #goal = (1, 1)
  goal = create_box((.8, .8), .4, .4)

  problem = create_problem(goal, obstacles)
  print problem
  search = get_fast_downward('ff-astar', max_time=10, verbose=True) # dijkstra | astar
  plan, _ = incremental_planner(problem, search=search, frequency=10, waves=False,
                                optimal=optimal, max_time=max_time)
  plan = convert_plan(plan)
  print 'Plan:', plan

  draw_solution(plan, goal, obstacles)
  raw_input('Finish?')

if __name__ == '__main__':
  main()
