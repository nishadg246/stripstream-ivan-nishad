#!/usr/bin/env python

from stripstream.pddl.logic.predicates import EasyPredicate as Pred
from stripstream.pddl.objects import EasyType as Type, EasyParameter as Param
from stripstream.pddl.logic.connectives import Not, Or, And
from stripstream.pddl.logic.quantifiers import Exists, ForAll
from stripstream.pddl.logic.atoms import Equal
from stripstream.pddl.operators import Action, Axiom
from stripstream.pddl.cond_streams import EasyGenStream as GeneratorStream, EasyTestStream as TestStream, \
  EasyFnStream as FunctionStream
from stripstream.pddl.utils import rename_easy
from stripstream.pddl.problem import STRIPStreamProblem

# TODO - could also combine the acceleration in the state

def create_problem(p_init=0, v_init=0, a_init=0,
                   dt=.5, max_a=10, min_a=-10, p_goal=10):
  """
  Creates a 1D car STRIPStream problem.

  https://github.com/KCL-Planning/SMTPlan/blob/master/benchmarks/car_nodrag/car_domain_nodrag.pddl

  :return: a :class:`.STRIPStreamProblem`
  """

  # Data types
  STATE, ACCEL, TIME = Type(), Type(), Type()

  # Fluent predicates
  AtState = Pred(STATE)
  AtAccel = Pred(ACCEL)
  NewTime = Pred()

  # Fluent predicates
  Running = Pred()
  Stopped = Pred()
  EngineBlown = Pred()
  TransmissionFine = Pred()
  GoalReached = Pred()

  # Static predicates
  Delta1 = Pred(ACCEL, ACCEL) # A2 - A1 = 1
  Dynamics = Pred(STATE, ACCEL, STATE)
  Contained = Pred(STATE)

  # Free parameters
  A1, A2 = Param(ACCEL), Param(ACCEL)
  S1 = Param(STATE)
  S2 = Param(STATE)

  rename_easy(locals()) # Trick to make debugging easier

  ####################

  actions = [
    Action(name='accelerate', parameters=[A1, A2],
      condition=And(Running(), NewTime(), AtAccel(A1), Delta1(A1, A2)),
      effect=And(AtAccel(A2), Not(NewTime()), Not(AtAccel(A1)))),

    Action(name='decelerate', parameters=[A1, A2],
      condition=And(Running(), NewTime(), AtAccel(A1), Delta1(A2, A1)),
      effect=And(AtAccel(A2), Not(NewTime()), Not(AtAccel(A1)))),

    Action(name='simulate', parameters=[S1, A1, S2],
      condition=And(AtState(S1), AtAccel(A1), Dynamics(S1, A1, S2)),
      effect=And(NewTime(), AtState(S2), Not(AtState(S1)))),
  ]

  axioms = [
    Axiom(effect=GoalReached(),
          condition=Exists([S1], And(AtState(S1), Contained(S1)))),
  ]

  ####################

  # Conditional stream declarations
  cond_streams = [
    FunctionStream(inputs=[S1, A1], outputs=[S2], conditions=[], effects=[Dynamics(S1, A1, S2)],
                    function=lambda (p1, v1), a1: (p1 + v1*dt + .5*a1*dt**2, v1 + a1*dt)),

    GeneratorStream(inputs=[A1], outputs=[A2], conditions=[], effects=[Delta1(A1, A2)],
                    generator=lambda a1: [a1+1] if a1+1 <= max_a else []),
    GeneratorStream(inputs=[A2], outputs=[A1], conditions=[], effects=[Delta1(A2, A1)],
                    generator=lambda a2: [a2+1] if a2-1 >- min_a else []),

    TestStream(inputs=[S1], conditions=[], effects=[Contained(S1)],
               test=lambda (p1, v1): p1 > p_goal, eager=True),
  ]

  ####################

  constants = []

  initial_atoms = [
    AtState((p_init, v_init)),
    AtAccel(a_init),
    NewTime(),
    Running(),
  ]

  goal_literals = [
    GoalReached()
    #AtAccel(0),
  ]

  problem = STRIPStreamProblem(initial_atoms, goal_literals, actions + axioms, cond_streams, constants)

  return problem

##################################################

from stripstream.algorithms.experimental.simultaneous import simultaneous
from stripstream.algorithms.incremental.incremental_planner import incremental_planner
from stripstream.algorithms.focused.focused_planner import focused_planner
from stripstream.algorithms.experimental.state_space import progression
from stripstream.algorithms.search.fast_downward import get_fast_downward
from stripstream.pddl.utils import convert_plan

def main():
  """
  Creates and solves the 1D car STRIPStream problem.
  """

  problem = create_problem(p_goal=10)
  #plan, _ = simultaneous(problem, frequency=10)
  plan, _ = progression(problem, frequency=10, useful_actions=True, verbose=False)
  search = get_fast_downward('eager') # dijkstra | astar | wastar1 | wastar2 | wastar3 | eager | lazy
  #plan, _ = incremental_planner(problem, search=search, frequency=1, waves=True)
  #plan, _ = focused_planner(problem, search=search, greedy=False)
  print
  print 'Plan:', convert_plan(plan)

# TODO - visualize by applying actions to env state

if __name__ == '__main__':
  main()
