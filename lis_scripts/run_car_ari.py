#!/usr/bin/env python
import numpy as np
from stripstream.pddl.logic.predicates import EasyPredicate as Pred
from stripstream.pddl.objects import EasyType as Type, EasyParameter as Param
from stripstream.pddl.logic.connectives import Not, Or, And
from stripstream.pddl.logic.quantifiers import Exists, ForAll
from stripstream.pddl.logic.atoms import Equal
from stripstream.pddl.operators import Action, Axiom
from stripstream.pddl.cond_streams import EasyGenStream as GeneratorStream, EasyTestStream as TestStream
from stripstream.pddl.utils import rename_easy
from stripstream.pddl.problem import STRIPStreamProblem

def create_problem(dt=.5, goalx=10,goaly=10, epsilon=0.5):
  """
  Creates a 1D car STRIPStream problem.

  https://github.com/KCL-Planning/SMTPlan/blob/master/benchmarks/car_nodrag/car_domain_nodrag.pddl

  :return: a :class:`.STRIPStreamProblem`
  """

  # Data types
  #POS, VEL, ACCEL, TIME = Type(), Type(), Type(), Type()
  #POSX, POSY, SPEED, STEER, TIME = [Type()]*5 # XXX - this uses the same time for every value
  POSX, POSY, SPEED, STEER, TIME = Type(), Type(), Type(), Type(), Type()

  # Fluent predicates
  AtPosX = Pred(POSX)
  AtPosY = Pred(POSY)
  AtSpeed = Pred(SPEED)
  AtSteer = Pred(STEER)
  NewTime = Pred()

  # Fluent predicates
  Running = Pred()
  GoalReached = Pred()

  # Static predicates
  DeltaSteer = Pred(STEER,STEER) # S2 - S1 = 1
  DeltaSpeed = Pred(SPEED,SPEED) # S2 - S1 = 1
  Dynamics = Pred(POSX, POSY, SPEED, STEER, POSX,POSY)
  Contained = Pred(POSX, POSY)

  # Free parameters
  SP1,SP2 = Param(SPEED), Param(SPEED)
  ST1,ST2 = Param(STEER), Param(STEER)
  PX1, PX2 = Param(POSX), Param(POSX)
  PY1, PY2 = Param(POSY), Param(POSY)

  rename_easy(locals())

  ####################

  actions = [
    Action(name='accelerate', parameters=[SP1, ST1, SP2, ST2],
      condition=And(\
              Running(),
              NewTime(),
              AtSpeed(SP1),
              AtSteer(ST1), 
              DeltaSteer(ST2, ST1), 
              DeltaSpeed(SP2,SP1)
              ),\
      effect=And(\
            AtSpeed(SP2),
            AtSteer(ST2), 
            Not(NewTime()), 
            Not(AtSpeed(SP1)), 
            Not(AtSteer(ST1)))
            ),

    Action(name='simulate', parameters=[PX1, PY1, SP1, ST1, PX2, PY2],
      condition=And(\
              AtPosX(PX1),
              AtPosY(PY1),
              AtSpeed(SP1),
              AtSteer(ST1),
              Dynamics(PX1, PY1, SP1, ST1, PX2, PY2)),
      effect=And(\
              NewTime(), 
              AtPosX(PX2),
              AtPosY(PY2),
              Not(AtPosX(PX1)),
              Not(AtPosY(PY1))
              ))
  ]

  axioms = [
    Axiom(effect=GoalReached(),
          condition=Exists([PX1, PY1], And(\
                  #Exists(PX1),
                  #Exists(PY1),
                  AtPosX(PX1),
                  AtPosY(PY1), 
                  Contained(PX1, PY1)
                  )))]

  ####################

  #def simulate(p1, v1, a1):
  #  p2, v2 = p1 + v1*dt + .5*a1*dt**2, v1 + a1*dt
  #  print p1, v1, a1, p2, v2
  #  return [(p2, v2)]

  # Conditional stream declarations
  cond_streams = [
    GeneratorStream(inputs=[PX1,PY1,SP1,ST1], outputs=[PX2,PY2], conditions=[], effects=[Dynamics(PX1, PY1, SP1, ST1, PX2, PY2)],
                    #generator=simulate),
                    generator=lambda px1,py1,sp1,st1:
                    [   [px1+sp1*dt*np.cos(st1), py1+sp1*dt*np.sin(st1)]]), # XXX - must return pairs of values
     
    GeneratorStream(inputs=[ST1], outputs=[ST2], conditions=[], effects=[DeltaSteer(ST1, ST2)], generator=lambda st1: [st1-1]),

    GeneratorStream(inputs=[SP1], outputs=[SP2], conditions=[], effects=[DeltaSpeed(SP1, SP2)], generator=lambda sp1: [sp1-.1]),


    GeneratorStream(inputs=[ST1], outputs=[ST2], conditions=[], effects=[DeltaSteer(ST1, ST2)], generator=lambda st1: [st1+1]),

    GeneratorStream(inputs=[SP1], outputs=[SP2], conditions=[], effects=[DeltaSpeed(SP1, SP2)], generator=lambda sp1: [sp1+.1]),

    TestStream(inputs=[PX1,PY1], conditions=[], effects=[Contained(PX1,PY1)],
               test= lambda px1, py1: (\
                    (px1 >= (goalx-epsilon)) and\
                    (px1 <= (goalx+epsilon)) and \
                    (py1 >= (goaly-epsilon)) and\
                    (py1 <= (goaly+epsilon))\

                    ), eager=True),
  ]

  ####################

  constants = []

  initial_atoms = [
    AtPosX(0),
    AtPosY(0),
    AtSpeed(0),
    AtSteer(0),
    NewTime(),
    Running(),
  ]

  goal_literals = [GoalReached()]

  problem = STRIPStreamProblem(initial_atoms, goal_literals, actions + axioms, cond_streams, constants)

  return problem

##################################################

from stripstream.algorithms.incremental.incremental_planner import incremental_planner
from stripstream.algorithms.focused.focused_planner import focused_planner
from stripstream.algorithms.search.fast_downward import get_fast_downward
from stripstream.pddl.utils import convert_plan

def main():
  """
  Creates and solves the 1D car STRIPStream problem.
  """

  problem = create_problem(goalx=.6, goaly=.6)
  search = get_fast_downward('eager') # dijkstra | astar | wastar1 | wastar2 | wastar3 | eager | lazy
  plan, _ = incremental_planner(problem, search=search, frequency=1, waves=True, debug=True)
  #plan, _ = focused_planner(problem, search=search, greedy=False)
  print
  print 'Plan:', convert_plan(plan)

# TODO - visualize by applying actions to env state

if __name__ == '__main__':
  main()
