#!/usr/bin/env python2

from random import random
from robotics.motion2D import get_distance, is_collision_free, sample, \
  create_box, sample_box, is_region, draw_solution, inf_sequence, draw_roadmap

from stripstream.pddl.logic.predicates import EasyPredicate as Pred
from stripstream.pddl.objects import EasyType as Type, EasyParameter as Param
from stripstream.pddl.logic.connectives import Not, And, Or
from stripstream.pddl.logic.quantifiers import Exists
from stripstream.pddl.operators import Action, Axiom
from stripstream.pddl.cond_streams import EasyGenStream as GeneratorStream, EasyTestStream as TestStream, \
  EasyListGenStream
from stripstream.pddl.utils import rename_easy
from stripstream.pddl.problem import STRIPStreamProblem

from stripstream.algorithms.incremental.incremental_planner import incremental_planner
from stripstream.algorithms.search.fast_downward import get_fast_downward
from stripstream.pddl.utils import convert_plan

# TODO - FFRob version of this that uses axioms to shorten the horizon

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
  #IsReachable = Pred(POINT, POINT)
  IsReachable = Pred(POINT)

  # Stream predicates
  IsEdge = Pred(POINT, POINT)
  Contained = Pred(POINT, REGION)

  # Free parameters
  P1, P2 = Param(POINT), Param(POINT)
  R = Param(REGION)

  rename_easy(locals()) # Trick to make debugging easier

  ####################

  actions = [
    Action(name='move', parameters=[P1, P2],
      condition=And(AtPoint(P1), IsReachable(P2)),
      effect=And(AtPoint(P2), Not(AtPoint(P1))))
  ]

  axioms = [
    Axiom(effect=InRegion(R),
          condition=Exists([P1], And(AtPoint(P1), Contained(P1, R)))),
    Axiom(effect=IsReachable(P2),
          condition=Or(AtPoint(P2),  Exists([P1], And(IsReachable(P1), IsEdge(P1, P2))))),
  ]

  ####################

  def sampler():
    for _ in inf_sequence():
      yield [(sample(digits),) for _ in range(10)]

  roadmap = set()
  def test(p1, p2):
    if not (get_distance(p1, p2) <= distance and
           is_collision_free((p1, p2), obstacles)):
      return False
    roadmap.add((p1, p2))
    return True

  ####################

  # Conditional stream declarations
  cond_streams = [
    EasyListGenStream(inputs=[], outputs=[P1], conditions=[], effects=[],
                      generator=sampler), # NOTE - version that only generators collision-free points

    GeneratorStream(inputs=[R], outputs=[P1], conditions=[], effects=[Contained(P1, R)],
                    generator=lambda r: (sample_box(r) for _ in inf_sequence())),

    TestStream(inputs=[P1, P2], conditions=[], effects=[IsEdge(P1, P2), IsEdge(P2, P1)],
               test=test, eager=True),
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

  return problem, roadmap

##################################################

# TODO - algorithms that take advantage of metric space (RRT)

def main():
  """
  Creates and solves the 2D motion planning problem.
  """

  obstacles = [
    create_box((.5, .5), .2, .2)
  ]
  #goal = (1, 1)
  goal = create_box((.8, .8), .4, .4)

  problem, roadmap = create_problem(goal, obstacles)
  print problem
  search = get_fast_downward('astar') # dijkstra | astar
  plan, _ = incremental_planner(problem, search=search, frequency=1, waves=False)
  plan = convert_plan(plan)
  print 'Plan:', plan
  print 'Roadmap', len(roadmap)

  draw_roadmap(roadmap, goal, obstacles)
  raw_input('Continue?')
  draw_solution(plan, goal, obstacles)
  raw_input('Finish?')

if __name__ == '__main__':
  main()
