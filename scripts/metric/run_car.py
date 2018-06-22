#!/usr/bin/env python

from stripstream.pddl.logic.predicates import EasyPredicate as Pred
from stripstream.pddl.objects import EasyType as Type, EasyParameter as Param
from stripstream.pddl.logic.connectives import Not, Or, And
from stripstream.pddl.logic.quantifiers import Exists, ForAll
from stripstream.pddl.logic.atoms import Equal
from stripstream.pddl.operators import Action, Axiom, STRIPSAction, STRIPSAxiom
from stripstream.pddl.cond_streams import EasyGenStream as GeneratorStream, EasyTestStream as TestStream, \
  EasyFnStream as FunctionStream
from stripstream.pddl.utils import rename_easy
from stripstream.pddl.problem import STRIPStreamProblem
from stripstream.utils import INF

def create_problem(p_init=0, v_init=0, a_init=0,
                   dt=.5, max_a=10, min_a=-10, p_goal=10):
  """
  Creates a 1D car STRIPStream problem.

  https://github.com/KCL-Planning/SMTPlan/blob/master/benchmarks/car_nodrag/car_domain_nodrag.pddl

  :return: a :class:`.STRIPStreamProblem`
  """

  # Data types
  POS, VEL, ACCEL, TIME = Type(), Type(), Type(), Type()

  # Fluent predicates
  AtPos = Pred(POS)
  AtVel = Pred(VEL)
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
  Dynamics = Pred(POS, VEL, ACCEL, POS, VEL)
  Contained = Pred(POS)

  # Free parameters
  A1, A2 = Param(ACCEL), Param(ACCEL)
  P1, V1 = Param(POS), Param(VEL)
  P2, V2 = Param(POS), Param(VEL)

  rename_easy(locals()) # Trick to make debugging easier

  ####################

  actions = [
    #Action(name='accelerate', parameters=[A1, A2],
    #  #condition=And(Running(), NewTime(), AtAccel(A1), Delta1(A1, A2)),
    #  condition=And(NewTime(), AtAccel(A1), Delta1(A1, A2)),
    #  effect=And(AtAccel(A2), Not(NewTime()), Not(AtAccel(A1))), cost=5),

    STRIPSAction(name='accelerate', parameters=[A1, A2],
      conditions=[NewTime(), AtAccel(A1), Delta1(A1, A2)],
      effects=[AtAccel(A2), Not(NewTime()), Not(AtAccel(A1))], cost=5),

    #Action(name='decelerate', parameters=[A1, A2],
    #  condition=And(Running(), NewTime(), AtAccel(A1), Delta1(A2, A1)),
    #  condition=And(NewTime(), AtAccel(A1), Delta1(A2, A1)),
    #  effect=And(AtAccel(A2), Not(NewTime()), Not(AtAccel(A1)))),

    STRIPSAction(name='decelerate', parameters=[A1, A2],
      conditions=[NewTime(), AtAccel(A1), Delta1(A2, A1)],
      effects=[AtAccel(A2), Not(NewTime()), Not(AtAccel(A1))]),

    #Action(name='simulate', parameters=[P1, V1, A1, P2, V2],
    #  condition=And(AtPos(P1), AtVel(V1), AtAccel(A1), Dynamics(P1, V1, A1, P2, V2)),
    #  effect=And(NewTime(), AtPos(P2), AtVel(V2), Not(AtPos(P1)), Not(AtVel(V1))), cost=1),

    STRIPSAction(name='simulate', parameters=[P1, V1, A1, P2, V2],
      conditions=[AtPos(P1), AtVel(V1), AtAccel(A1), Dynamics(P1, V1, A1, P2, V2)],
      effects=[NewTime(), AtPos(P2), AtVel(V2), Not(AtPos(P1)), Not(AtVel(V1))], cost=1),

    #STRIPSAction(name='goal', parameters=[P1],
    #  conditions=[AtPos(P1), Contained(P1)],
    #  effects=[GoalReached()], cost=None),
  ]

  axioms = [
    #Axiom(effect=GoalReached(), condition=Exists([P1], And(AtPos(P1), Contained(P1)))),
    STRIPSAxiom(conditions=[AtPos(P1), Contained(P1)], effects=[GoalReached()])
  ]

  ####################

  # Conditional stream declarations
  cond_streams = [
    FunctionStream(inputs=[P1, V1, A1], outputs=[P2, V2], conditions=[], effects=[Dynamics(P1, V1, A1, P2, V2)],
                    function=lambda p1, v1, a1: (p1 + v1*dt + .5*a1*dt**2, v1 + a1*dt)),

    FunctionStream(inputs=[A1], outputs=[A2], conditions=[], effects=[Delta1(A1, A2)],
                    function=lambda a1: (a1+1,) if a1+1 <= max_a else None),
    #GeneratorStream(inputs=[A1], outputs=[A2], conditions=[], effects=[Delta1(A1, A2)],
    #                generator=lambda a1: [a1+1] if a1+1 <= max_a else []),


    FunctionStream(inputs=[A2], outputs=[A1], conditions=[], effects=[Delta1(A2, A1)],
                    function=lambda a2: (a2+1,) if a2-1 >= min_a else None),
    #GeneratorStream(inputs=[A2], outputs=[A1], conditions=[], effects=[Delta1(A2, A1)],
    #                generator=lambda a2: [a2+1] if a2-1 >- min_a else []),

    TestStream(inputs=[P1], conditions=[], effects=[Contained(P1)],
               test=lambda p1: p1 >= p_goal, eager=True),
  ]

  ####################

  constants = []

  initial_atoms = [
    AtPos(p_init),
    AtVel(v_init),
    AtAccel(a_init),
    NewTime(),
    #Running(),
  ]

  goal_literals = [
    GoalReached(),
    AtAccel(0),
  ]

  problem = STRIPStreamProblem(initial_atoms, goal_literals, actions + axioms, cond_streams, constants)

  return problem

##################################################

from stripstream.algorithms.incremental.incremental_planner import incremental_planner
from stripstream.algorithms.experimental.simultaneous import simultaneous, simultaneous_strips
from stripstream.algorithms.experimental.state_space import progression
from stripstream.algorithms.focused.focused_planner import focused_planner
from stripstream.algorithms.search.fast_downward import get_fast_downward
from stripstream.pddl.utils import convert_plan

def main():
  """
  Creates and solves the 1D car STRIPStream problem.
  """

  problem = create_problem(p_goal=5) # 0 | 1 | 2 | 5 | 10
  #print problem, '\n'
  #problem.replace_axioms()
  #print problem

  #plan, _ = progression(problem, frequency=10, useful_actions=True, verbose=False)
  #plan, _ = simultaneous(problem, frequency=10, verbose=False, debug=False)
  #plan, _ = simultaneous_strips(problem, frequency=10, verbose=False, debug=False)

  from misc.profiling import run_profile, str_profile
  #solve = lambda: progression(problem, frequency=INF, useful_actions=True, verbose=False) # frequency = 10 | INF
  #solve = lambda: simultaneous(problem, frequency=10, verbose=False, debug=False)
  #solve = lambda: simultaneous_strips(problem, frequency=10, verbose=False, debug=False)
  #(plan, _), prof = run_profile(solve)
  #print str_profile(prof)

  search = get_fast_downward('eager') # dijkstra | astar | wastar1 | wastar2 | wastar3 | eager | lazy
  plan, _ = incremental_planner(problem, search=search, frequency=1, waves=True)
  #plan, _ = incremental_planner(problem, search=search, frequency=100, waves=False)
  #plan, _ = focused_planner(problem, search=search, greedy=False)
  print
  print 'Plan:', convert_plan(plan)

# TODO - visualize by applying actions to env state

if __name__ == '__main__':
  main()
