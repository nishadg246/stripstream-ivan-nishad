from stripstream.pddl.objects import EasyType as Type, EasyParameter as Param
from stripstream.pddl.logic.predicates import EasyPredicate as Predicate
from stripstream.pddl.operators import Action, Axiom
from stripstream.pddl.logic.connectives import And, Not
from stripstream.algorithms.incremental.incremental_planner import incremental_planner
from stripstream.algorithms.search.fast_downward import get_fast_downward
from stripstream.pddl.utils import convert_plan, rename_easy
from stripstream.pddl.problem import STRIPStreamProblem
from stripstream.pddl.examples.belief.problems import *
from stripstream.pddl.examples.belief.utils import *
from stripstream.pddl.examples.belief.unknown import OPERATOR_MAP
from stripstream.pddl.logic.quantifiers import Exists, ForAll
from stripstream.pddl.logic.atoms import Equal

from toyTest import glob, makeOperators, Bd, ObjState, ObjLoc

OBJ, LOC, STATE = Type(), Type(), Type()

At = Predicate(OBJ, LOC)
HasState = Predicate(OBJ, STATE)

Clear = Predicate(LOC)
Safe = Predicate(OBJ, LOC)

IsDryer = Predicate(LOC)
IsPainter = Predicate(LOC)
IsWasher = Predicate(LOC)

O, O2, L1, L2, S = Param(OBJ), Param(OBJ), Param(LOC), Param(LOC), Param(STATE)

actions = [
  Action(name='transport', parameters=[O, L1, L2],
    condition=And(At(O, L1), Clear(L2)),
    effect=And(At(O, L2), Not(At(O, L1)))), # NOTE - Leslie and Tomas call this Move

  Action(name='wash', parameters=[O, L1, S],
    condition=And(At(O, L1), HasState(O, S), IsWasher(L1)),
    effect=And(HasState(O, clean), Not(HasState(O, S)))),

  Action(name='paint', parameters=[O, L1],
    condition=And(At(O, L1), HasState(O, clean), IsPainter(L1)),
    effect=And(HasState(O, wet), Not(HasState(O, clean)))),

  Action(name='dry', parameters=[O, L1],
    condition=And(At(O, L1), HasState(O, wet), IsDryer(L1)),
    effect=And(HasState(O, dry), Not(HasState(O, wet)))),

  Axiom(effect=Clear(L2), condition=ForAll([O2], Safe(O2, L2))),

  Axiom(effect=Safe(O, L1), condition=Exists([L2], And(At(O, L2), Not(Equal(L1, L2))))),
]

rename_easy(locals())

##################################################

def make_goal_literals(goal):
  goal_literals = []
  for fluent in goal.fluents:
    if isinstance(fluent, Bd):
      literal, arg, prob = fluent.args
      if isinstance(literal, ObjLoc):
        goal_literals.append(At(literal.args[0], literal.value))
      elif isinstance(literal, ObjState):
        goal_literals.append(HasState(literal.args[0], arg))
      else:
        raise NotImplementedError(literal)
    else:
      raise NotImplementedError(fluent)
  return goal_literals

def make_initial_atoms(locations, states, occupancies):
  initial_atoms = []
  initial_atoms += [At(*pair) for pair in locations.iteritems()]
  initial_atoms += [HasState(*pair) for pair in states.iteritems()]
  for loc in occupancies:
    if loc == 'washer':
      initial_atoms.append(IsWasher(loc))
    if loc == 'painter':
      initial_atoms.append(IsPainter(loc))
    if loc == 'dryer':
      initial_atoms.append(IsDryer(loc))
  return initial_atoms

def make_problem((l, s, o), goal):
  initial_atoms = make_initial_atoms(l, s, o)
  goal_literals = make_goal_literals(goal)
  constants = map(LOC, o)
  return STRIPStreamProblem(initial_atoms, goal_literals, actions, [], constants)

##################################################

def observable_policy(operators, goal):
  def fn(belief):
    problem = make_problem(maximum_likelihood_obs(belief), goal)
    #problem = make_problem(sample_obs(belief), goal)
    #problem = make_problem(sample_consistent_obs(belief), goal)
    #print problem
    search = get_fast_downward('astar', verbose=False) # dijkstra | astar | wastar1 | wastar2 | wastar3 | eager | lazy
    plan, universe = incremental_planner(problem, search=search, frequency=1)
    plan = convert_plan(plan)
    print 'Plan:', plan
    print 'Length:', len(plan)
    if plan is None or not plan:
      return None
    action, params = plan[0]
    print 'Action:', action, params
    print
    return OPERATOR_MAP[action.name](operators, *params)
  return fn

##################################################

#operator.__dict__.keys() = ['configurationCache', 'prim', 'conditionOnPreconds', 'delayBinding', 'results', 'cost', 'instanceCost',
# 'subPlans', 'functions', 'sideEffects', 'rebindPenalty', 'concreteAbstractionLevel', 'argsToPrint', 'parent',
# 'abstractionLevel', 'args', 'metaOperator', 'preconditions', 'closedLoopPrim', 'ignorableArgs', 'name',
# 'f', 'specialRegress', 'num']

def run_observable_2():
  problem_fn = test8 # test0 | test1 | test8
  env, start, goal = problem_fn()
  print
  print convert_start(env, start)
  print convert_goal(goal)

  #operators = makeOperators(glob.failProbs)
  #for name, operator in operators.iteritems():
  #  print 'Name:', name
  #  print 'Args:', [arg for i, arg in enumerate(operator.args) if i not in operator.ignorableArgs]
  #  print 'Pre:', operator.preconditions
  #  print 'Eff:', operator.results
  #  print 'Side Eff:', operator.sideEffects
  #  print

  #problem = make_problem(convert_start(env, start), goal)
  problem = make_problem(maximum_likelihood_obs(start.details), goal)
  #problem = make_problem(sample_obs(start.details), goal)
  #problem = make_problem(sample_consistent_obs(start.details), goal)

  print problem

  search = get_fast_downward('dijkstra') # dijkstra | astar | wastar1 | wastar2 | wastar3 | eager | lazy
  plan, _ = incremental_planner(problem, search=search, frequency=1)
  print convert_plan(plan)