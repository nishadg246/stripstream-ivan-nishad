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

def create_problem(dt, GoalTest, ClearTest, CalcDiffSystem, InitialState):
  """
  Creates a generic non-holonomic motion planning problem
  :return: a :class:`.STRIPStreamProblem`
  """

  # Data types
  X, DX, U = Type(), Type(), Type()

  # Fluent predicates
  AtX = Pred(X)

  # Fluent predicates
  GoalReached = Pred()

  # Static predicates
  ComputeDX = Pred(X,U,DX)
  Dynamics = Pred(X, DX, X)
  AtGoal = Pred(X)
  Clear = Pred(X, X)

  # Free parameters
  X1, X2 = Param(X), Param(X)
  DX1 = Param(DX)
  U1 = Param(U)
  rename_easy(locals()) # Trick to make debugging easier

  ####################

  actions = [
    Action(name='simulate', parameters=[X1,U1,X2,DX1],
      condition=And(AtX(X1),ComputeDX(X1,U1,DX1),Dynamics(X1,DX1,X2),Clear(X1,X2)),
      effect=And(Not(AtX(X1)),AtX(X2))),
  ]

  axioms = [
    Axiom(effect=GoalReached(),
          condition=Exists([X1], AtGoal(X1))),
  ]

  ####################

  # Conditional stream declarations
  def ComputeDXCalculator(X1F, DX1F):
    X2F = X1F[:]
    for i in range(len(X2F)):
      X2F[i] = X1F[i] + dt*DX1F[i]
    return X2F

  cond_streams = [
    FunctionStream(inputs=[X1, DX1], outputs=[X2], conditions=[], effects=[Dynamics(X1, DX1, X2)],
                    function=ComputeDXCalculator), 
    FunctionStream(inputs=[X1, U1], outputs=[DX1], conditions=[], effects=[ComputeDX(X1, U1, DX1)],
                    function=CalcDiffSystem),
    
    TestStream(inputs=[X1,X2], conditions=[], effects=[Clear(X1,X2)],
               test=ClearTest, eager=True),
    TestStream(inputs=[X1], conditions=[], effects=[AtGoal(X1)],
               test=GoalTest, eager=True),
  ]

  ####################

  constants = [
    # TODO - need to declare control inputs (or make a stream for them)
  ]

  initial_atoms = [
    AtX(InitialState),
  ]

  goal_literals = [GoalReached()]

  problem = STRIPStreamProblem(initial_atoms, goal_literals, actions + axioms, cond_streams, constants)

  return problem

##################################################

from stripstream.algorithms.incremental.incremental_planner import incremental_planner
from stripstream.algorithms.focused.focused_planner import focused_planner
from stripstream.algorithms.search.fast_downward import get_fast_downward
from stripstream.pddl.utils import convert_plan

"""
1D test case
"""

def DGoalTest(X1):
  return X1[0] > 5

def DClearTest(X1,X2):
    return True

def DCalcDiffSystem(X1,U1):
  DX1 = X1[:]
  DX1[0] = X1[1] # TODO - tuple does not support object assignment
  DX1[1] = U1[0]
  return DX1



def main():
  """
  Creates and solves a generic non-holonomic motion planning problem
  """

  init_state = (0,0)
  problem = create_problem(.5,DGoalTest,DClearTest,DCalcDiffSystem,init_state)

  search = get_fast_downward('eager') # dijkstra | astar | wastar1 | wastar2 | wastar3 | eager | lazy
  plan, _ = incremental_planner(problem, search=search, frequency=1, waves=True, debug=True)
  #plan, _ = focused_planner(problem, search=search, greedy=False)
  print
  print 'Plan:', convert_plan(plan)

# TODO - visualize by applying actions to env state

if __name__ == '__main__':
  main()