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
from stripstream.pddl.utils import rename_easy
from stripstream.pddl.problem import STRIPStreamProblem

# To run this:  python -m scripts.simple_continuous_belief

# I think at this point I'm going to leave this example for you to improve on.  It would be nice to be able to get it to do some examples where:
# - It starts with its mean near the goal with high variance
# - It has some, but relatively low, odometry error
# - There is a beacon location
# - It has to move near the beacon, observe, and then move home
#
# If that works, then a more complex case with obstacles and more beacons would be awesome.

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
                   verboseFns = False):
  """
  :return: a :class:`.STRIPStreamProblem`
  """
  # Data types
  POS = Type()   # 2D position
  VAR = Type()   # 2D variance
  BEACON = Type()  # 2D position

  # Fluent predicates
  RobotPos = Pred(POS)
  RobotVar = Pred(VAR)

  # Derived predicates
  KnowYouAreHome = Pred()

  # Static predicates
  Odometry = Pred(POS, VAR, POS, VAR)
  Sensor = Pred(POS, VAR, BEACON, VAR)
  LegalPosVar = Pred(POS, VAR)
  NearbyPos = Pred(POS, POS)
  NearGoalPos = Pred(POS)
  NearGoalVar = Pred(VAR)

  # Free parameters
  RPOS1, RPOS2 = Param(POS), Param(POS)
  RVAR1, RVAR2 = Param(VAR), Param(VAR)
  BPOS = Param(BEACON)

  rename_easy(locals())

  # Helper functions
  def distance(p1, p2):
    return math.sqrt(sum([(a - b)**2 for (a,b) in zip(p1, p2)]))

  def odoVarFun(rp1, rv, rp2):
    d = distance(rp1, rp2)
    odoVar = (d * odoErrorRate)**2
    result = rv + odoVar
    if verboseFns: print 'ovf:', rv, d, result
    return [result]

  def sensorVarFun(rp, rv, bp):
    d = distance(rp, bp)
    obsVar = max(d / obsVarPerDistFromSensor, minObsVar)
    #result = round(1.0 / ((1.0 / rv) + (1.0 / obsVar)), 5) # Causes zero variance which is bad
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

  # TODO - combine variance and positions

  actions = [
    Action(name='Move',
           parameters=[RPOS1, RVAR1, RPOS2, RVAR2],
           condition = And(RobotPos(RPOS1),
                           RobotVar(RVAR1),
                           Odometry(RPOS1, RVAR1, RPOS2, RVAR2)),
           effect = And(RobotPos(RPOS2),
                        RobotVar(RVAR2),
                        Not(RobotPos(RPOS1)),
                        Not(RobotVar(RVAR1)))),

    Action(name='Look',
           parameters=[RPOS1, RVAR1, BPOS, RVAR2],
           condition = And(RobotPos(RPOS1),
                           RobotVar(RVAR1),
                           Sensor(RPOS1, RVAR1, BPOS, RVAR2)),
           effect = And(RobotVar(RVAR2),
                        Not(RobotVar(RVAR1))))
  ]

  axioms = [
  ]

  # Conditional stream declarations
  cond_streams = [
     GeneratorStream(inputs = [],
                     outputs = [RPOS1],
                     conditions = [],
                     effects = [],
                     generator = randPos),

    # TODO - the number of variances grows incredibly but we just want to consider variances possible with the move
    # Each var only has one pos because moving changes
    GeneratorStream(inputs = [RPOS1, RVAR1, RPOS2],
                    outputs = [RVAR2],
                    conditions = [LegalPosVar(RPOS1, RVAR1), NearbyPos(RPOS1, RPOS2)],
                    effects = [Odometry(RPOS1, RVAR1, RPOS2, RVAR2), LegalPosVar(RPOS2, RVAR2)],
                    generator = odoVarFun),

    GeneratorStream(inputs = [RPOS1, RVAR1, BPOS],
                    outputs = [RVAR2],
                    conditions = [LegalPosVar(RPOS1, RVAR1)],
                    effects = [Sensor(RPOS1, RVAR1, BPOS, RVAR2), LegalPosVar(RPOS1, RVAR2)],
                    generator = sensorVarFun),

    TestStream(inputs = [RPOS1, RPOS2],
               conditions = [],
               effects = [NearbyPos(RPOS1, RPOS2)],
               test = lambda rp1, rp2: distance(rp1, rp2) < maxMoveDist,
               eager = True),

    TestStream(inputs = [RPOS1],
               conditions = [],
               effects = [NearGoalPos(RPOS1)],
               test = lambda rp1: distance(rp1, homePos) < goalPosEps,
               eager = True),

    TestStream(inputs = [RVAR1],
               conditions = [],
               effects = [NearGoalVar(RVAR1)],
               test = lambda a: a < goalVar,
               eager = True)

    # NOTE - the problem seems to be the fast that this is eager

  ]

  ####################

  constants = [
    BEACON(beaconPos),
    #POS((.1, .1)),
    #POS((3., .1)),
    #POS((6., .1)),
    POS(homePos),
  ]

  initial_atoms = [
    RobotPos(initRobotPos),
    RobotVar(initRobotVar),
    LegalPosVar(initRobotPos, initRobotVar), # TODO - I might as well keep track of pairs. Make this a legal on both
  ]

  goal_formula = Exists([RPOS1, RVAR1],
                             And(RobotPos(RPOS1),
                                 RobotVar(RVAR1),
                                 LegalPosVar(RPOS1, RVAR1),
                                 NearGoalPos(RPOS1),
                                 NearGoalVar(RVAR1)))

  problem = STRIPStreamProblem(initial_atoms, goal_formula, actions + axioms,
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
        'beaconPos'   : (2.0, 2.0),
        'odoErrorRate' : 0.00001}
  }
 
def main(argv):
  testNum = int(argv[0]) if argv else 0

  print 'Test number', testNum
  print 'Args = ', testArgs[testNum]
  
  problem = create_problem(**testArgs[testNum])
  # dijkstra | astar | wastar1 | wastar2 | wastar3 | eager | lazy
  search = get_fast_downward('eager', verbose=False)

  #plan, _ = incremental_planner(problem, search=search,
  #                                frequency=10, waves=False, debug=False)
  #plan, _ = incremental_planner(problem, search=search,
  #                                frequency=1, waves=True, debug=True)
  plan, _ = focused_planner(problem, search=search, greedy=False)

  print '\nPlan:', convert_plan(plan)

if __name__ == '__main__':
  main(sys.argv[1:])
