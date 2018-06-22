from stripstream.pddl.objects import EasyType as Type, EasyParameter as Param
from stripstream.pddl.logic.predicates import EasyPredicate as Predicate
from stripstream.pddl.operators import Action
from stripstream.pddl.logic.connectives import And, Not
from stripstream.algorithms.incremental.incremental_planner import incremental_planner
from stripstream.algorithms.search.fast_downward import get_fast_downward
from stripstream.pddl.utils import convert_plan, rename_easy
from stripstream.pddl.problem import STRIPStreamProblem
from stripstream.pddl.examples.belief.problems import *

from toyTest import glob, makeOperators, Bd, ObjState, ObjLoc

OBJ, LOC = Type(), Type()

At = Predicate(OBJ, LOC)
Clear = Predicate(LOC)
Clean = Predicate(OBJ)
WetPaint = Predicate(OBJ)
DryPaint = Predicate(OBJ)

IsDryer = Predicate(LOC)
IsPainter = Predicate(LOC)
IsWasher = Predicate(LOC)

O, L1, L2 = Param(OBJ), Param(LOC), Param(LOC)

actions = [
  Action(name='transport', parameters=[O, L1, L2],
    condition=And(At(O, L1), Clear(L2)),
    effect=And(At(O, L2), Clear(L1), Not(At(O, L1)), Not(Clear(L2)))), # NOTE - Leslie and Tomas call this Move

  Action(name='wash', parameters=[O, L1],
    condition=And(At(O, L1), IsWasher(L1)),
    #effect=And(Clean(O)))
    effect=And(Clean(O), Not(WetPaint(O)), Not(DryPaint(O)))),

  Action(name='paint', parameters=[O, L1],
    condition=And(At(O, L1), Clean(O), IsPainter(L1)),
    #effect=And(WetPaint(O))),
    effect=And(WetPaint(O), Not(Clean(O)))),

  Action(name='dry', parameters=[O, L1],
    condition=And(At(O, L1), WetPaint(O), IsDryer(L1)),
    #effect=And(DryPaint(O))),
    effect=And(DryPaint(O), Not(WetPaint(O)))),
]

rename_easy(locals())

def observable_problem(env, start, goal):
  locations = start.details.occupancies.keys()

  initial_atoms = []
  constants = []
  occupied = set()
  for obj in env.objects:
    for attr in env.objects[obj]:
      if attr == 'clean':
        initial_atoms.append(Clean(obj))
      elif attr == 'dirty':
        pass
      elif attr in locations:
        initial_atoms.append(At(obj, attr))
        occupied.add(attr)
      else:
        raise NotImplementedError(attr)
  for loc in locations:
    if loc not in occupied:
      initial_atoms.append(Clear(loc))
    constants.append(LOC(loc))
    if loc == 'washer':
      initial_atoms.append(IsWasher(loc))
    if loc == 'painter':
      initial_atoms.append(IsPainter(loc))
    if loc == 'dryer':
      initial_atoms.append(IsDryer(loc))

  goal_literals = []
  for fluent in goal.fluents:
    if isinstance(fluent, Bd):
      literal, arg, prob = fluent.args
      if isinstance(literal, ObjLoc):
        goal_literals.append(At(literal.args[0], literal.value))
      elif isinstance(literal, ObjState) and arg == 'clean':
        goal_literals.append(Clean(literal.args[0]))
      elif isinstance(literal, ObjState) and arg == 'wetPaint':
        goal_literals.append(WetPaint(literal.args[0]))
      elif isinstance(literal, ObjState) and arg == 'dryPaint':
        goal_literals.append(DryPaint(literal.args[0]))
      else:
        raise NotImplementedError(literal)
    else:
      raise NotImplementedError(fluent)

  return STRIPStreamProblem(initial_atoms, goal_literals, actions, [], constants)

##################################################

#operator.__dict__.keys() = ['configurationCache', 'prim', 'conditionOnPreconds', 'delayBinding', 'results', 'cost', 'instanceCost',
# 'subPlans', 'functions', 'sideEffects', 'rebindPenalty', 'concreteAbstractionLevel', 'argsToPrint', 'parent',
# 'abstractionLevel', 'args', 'metaOperator', 'preconditions', 'closedLoopPrim', 'ignorableArgs', 'name',
# 'f', 'specialRegress', 'num']

def run_observable():
  problem_fn = test8 # test0 | test1 | test8
  env, start, goal = problem_fn()
  global operatorDict
  operatorDict = makeOperators(glob.failProbs)

  print
  for name, operator in operatorDict.iteritems():
    print 'Name:', name
    print 'Args:', [arg for i, arg in enumerate(operator.args) if i not in operator.ignorableArgs]
    print 'Pre:', operator.preconditions
    print 'Eff:', operator.results
    print 'Side Eff:', operator.sideEffects
    print

  problem = observable_problem(env, start, goal)

  search = get_fast_downward('eager') # dijkstra | astar | wastar1 | wastar2 | wastar3 | eager | lazy
  plan, _ = incremental_planner(problem, search=search, frequency=1)
  print convert_plan(plan)