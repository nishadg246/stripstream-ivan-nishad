#!/usr/bin/env python
import pdb
import math
import random

from stripstream.pddl.logic.predicates import EasyPredicate as Pred
from stripstream.pddl.objects import EasyType as Type, EasyParameter as Param
from stripstream.pddl.logic.connectives import Not, Or, And
from stripstream.pddl.logic.quantifiers import Exists, ForAll
from stripstream.pddl.logic.atoms import Equal
from stripstream.pddl.operators import Action, Axiom
from stripstream.pddl.cond_streams import EasyGenStream as GeneratorStream, EasyTestStream as TestStream
from stripstream.pddl.problem import STRIPStreamProblem

# To run this:  python -m scripts.simple_continuous_belief

# I think at this point I'm going to leave this example for you to improve on.  It would be nice to be able to get it to do some examples where:
# - It starts with its mean near the goal with high variance
# - It has some, but relatively low, odometry error
# - There is a beacon location
# - It has to move near the beacon, observe, and then move home
#
# If that works, then a more complex case with obstacles and more beacons would be awesome.

# Helper functions
def distance(p1, p2):
    result = math.sqrt(sum([(a - b)**2 for (a,b) in zip(p1, p2)]))
    return [result]

def lt(a, b):
    return a < b

def create_problem(initRobotPos = (0.5, 0.5),
                   initRobotVar = 0.01,
                   maxMoveDist = 5.0,
                   beaconPos = (1, 1),
                   homePos = (0, 0),
                   goalPosEps = 0.1,
                   goalVar = 0.1,
                   odoErrorRate = 0.1,
                   obsVarPerDistFromSensor = 10.0,
                   minObsVar = 0.001,
                   domainSize = 20,
                   verboseFns = True):
  """
  :return: a :class:`.STRIPStreamProblem`
  """
  # Data types
  POS = Type()   # 2D position
  VAR = Type()   # 2D variance
  DIST = Type()  # positive scalar

  # Fluent predicates
  RobotPos = Pred(POS)
  RobotVar = Pred(VAR)

  # Derived predicates
  KnowYouAreHome = Pred()

  # Static predicates
  DistBetween = Pred(POS, POS, DIST) # Distance between two positions (function)
  LessThanV = Pred(VAR, VAR) # Less than, on distances
  LessThanD = Pred(DIST, DIST) # Less than, on distances
  OdometryVar = Pred(VAR, VAR, DIST) # How does odometry variance increase?
  SensorVar = Pred(VAR, VAR, DIST) # How does an observation decrease variance?
  LegalPos = Pred(POS) # Any legal robot pos

  # Free parameters
  RPOS1, RPOS2, RVAR1, RVAR2 = Param(POS), Param(POS), Param(VAR), Param(VAR)
  DIST1, DIST2 = Param(DIST), Param(DIST)

  def odoVarFun(rv, d):
    odoVar = (d * odoErrorRate)**2
    result = rv + odoVar
    if verboseFns: print 'ovf:', rv, d, result
    return [result]

  def sensorVarFun(rv, d):
    obsVar = max(d / obsVarPerDistFromSensor, minObsVar)
    result = 1.0 / ((1.0 / rv) + (1.0 / obsVar))
    if verboseFns: print 'svf:', rv, d, result
    return [result]

  def randPos():
    while True:
      result = (random.random() * domainSize, random.random() * domainSize)
      print 'rp:', result
      yield [result]

  def legalTest(rp):
    (x, y) = rp
    result = (0 <= x <= domainSize) and (0 <= y <= domainSize)
    if not result: print 'not legal:', rp
    return result

  actions = [
    Action(name='Move',
           parameters=[RPOS1, RPOS2, RVAR1, RVAR2, DIST1],
           condition = And(RobotPos(RPOS1),
                           RobotVar(RVAR1),
                           LegalPos(RPOS2), # Generate an intermediate pos
                           DistBetween(RPOS1, RPOS2, DIST1),
                           LessThanD(DIST1, maxMoveDist),
                           OdometryVar(RVAR1, RVAR2, DIST1)),
           effect = And(RobotPos(RPOS2),
                        RobotVar(RVAR2),
                        Not(RobotPos(RPOS1)),
                        Not(RobotVar(RVAR1)))),

    # Action(name='Look',
    #        parameters=[RPOS1, RVAR1, RVAR2, DIST1],
    #        condition = And(RobotPos(RPOS1),
    #                        RobotVar(RVAR1),
    #                        DistBetween(RPOS1, beaconPos, DIST1),
    #                        SensorVar(RVAR1, RVAR2, DIST1)),
    #        effect = And(RobotVar(RVAR2),
    #                     Not(RobotVar(RVAR1))))

  ]

  axioms = [
    Axiom(effect = KnowYouAreHome(),
          condition = Exists([RPOS1, RVAR1, DIST1], 
                             And(RobotPos(RPOS1),
                                 RobotVar(RVAR1),
                                 DistBetween(RPOS1, homePos, DIST1),
                                 LessThanD(DIST1, goalPosEps),
                                 LessThanV(RVAR1, goalVar))))
  ]

  # Conditional stream declarations
  cond_streams = [
    TestStream(inputs = [RPOS1],
               conditions = [],
               effects = [LegalPos(RPOS1)],
               test = legalTest,
               eager = True),

    GeneratorStream(inputs = [],
                    outputs = [RPOS1],
                    conditions = [],
                    effects = [LegalPos(RPOS1)],
                    generator = randPos),

    GeneratorStream(inputs = [RPOS1, RPOS2],
                    outputs = [DIST1],
                    conditions = [],
                    effects = [DistBetween(RPOS1, RPOS2, DIST1)],
                    generator = lambda rp1, rp2: [distance(rp1, rp2)]),

    GeneratorStream(inputs = [RVAR1, DIST1],
                    outputs = [RVAR2],
                    conditions = [],
                    effects = [OdometryVar(RVAR1, RVAR2, DIST1)],
                    generator = odoVarFun),

    # GeneratorStream(inputs = [RVAR1, DIST1],
    #                 outputs = [RVAR2],
    #                 conditions = [],
    #                 effects = [SensorVar(RVAR1, RVAR2, DIST1)],
    #                 generator = sensorVarFun),

    TestStream(inputs = [DIST1, DIST2],
               conditions = [],
               effects = [LessThanD(DIST1, DIST2)],
               test = lt,
               eager = True),

    TestStream(inputs = [RVAR1, RVAR2],
               conditions = [],
               effects = [LessThanV(RVAR1, RVAR2)],
               test = lt, eager = True)

  ]

  ####################

  constants = [
  ]

  initial_atoms = [
    RobotPos(initRobotPos),
    RobotVar(initRobotVar),
    LegalPos(homePos),
    LegalPos((.1, .1)),
    LegalPos((3.0, .1)),
    LegalPos((6.0, .1))
  ]

  goal_literals = [
    KnowYouAreHome()
    ]

  problem = STRIPStreamProblem(initial_atoms, goal_literals, actions + axioms,
                                 cond_streams, constants)

  return problem

##################################################

from stripstream.algorithms.incremental.incremental_planner import incremental_planner
from stripstream.algorithms.focused.focused_planner import focused_planner
from stripstream.algorithms.search.bfs import get_bfs
from stripstream.algorithms.search.fast_downward import get_fast_downward
from stripstream.pddl.utils import convert_plan
from time import time
import sys

testArgs = \
  {
   # Just needs to move
   0 : {},
   
   # Needs to look twice
   1 : {'initRobotVar' : 1.0,
        'initRobotPos' : (0.0, 0.0)},
        
   # Needs to look and move
   2 : {'initRobotVar' : 0.5,
        'initRobotPos' : (1.0, 1.0)},
        
   # Needs to move twice, low odo
   3 : {'initRobotVar' : 0.0000001,
        'odoErrorRate' : 0.0000001,
        'domainSize'   : 10.0,
        'initRobotPos' : (6.0, 0.01),
        'homePos'      : (0.01, 0.01)},
        
   # Move near beacon, look, drive back; very small odo error
   4 : {'initRobotVar' : 1.0,
        'initRobotPos' : (0.0, 0.0),
        'beaconPose'   : (2.0, 2.0),
        'odoErrorRate' : 0.00001}
  }
 
def main(argv):
  testNum = int(argv[0]) if argv else 0

  print 'Test number', testNum
  print 'Args = ', testArgs[testNum]
  
  problem = create_problem(**testArgs[testNum])
  # dijkstra | astar | wastar1 | wastar2 | wastar3 | eager | lazy
  search = get_fast_downward('eager')

  #plan, _ = incremental_planner(problem, search=search,
  #                                frequency=10, waves=False)
  plan, _ = focused_planner(problem, search=search, greedy=False)

  print '\nPlan:', convert_plan(plan)

if __name__ == '__main__':
  main(sys.argv[1:])
