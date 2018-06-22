from stripstream.pddl.objects import EasyType as Type, EasyParameter as Param
from stripstream.pddl.logic.predicates import EasyPredicate as Predicate
from stripstream.pddl.operators import Action, Axiom
from stripstream.pddl.logic.connectives import And, Not
from stripstream.algorithms.incremental.incremental_planner import incremental_planner
from stripstream.algorithms.search.fast_downward import get_fast_downward
from stripstream.pddl.utils import convert_plan, rename_easy
from stripstream.pddl.problem import STRIPStreamProblem
from stripstream.pddl.examples.belief.problems import *
from stripstream.pddl.logic.predicates import Function
from stripstream.pddl.logic.operations import Initialize, Cost
from stripstream.algorithms.plan import plan_cost
from stripstream.pddl.logic.quantifiers import Exists, ForAll
from stripstream.pddl.logic.atoms import Equal
from stripstream.pddl.examples.belief.utils import *

from toyTest import glob, makeOperators, Bd, ObjState, ObjLoc
import toyTest

OPERATOR_MAP = {
  'find': make_look,
  'inspect_loc': make_look_clear,
  'inspect_state': make_look_state,
  'transport': make_transport,
  'wash': make_wash,
  'paint': make_paint,
  'dry': make_dry,
}

COST_SCALE = 10

OBJ, LOC, STATE = Type(), Type(), Type()

At = Predicate(OBJ, LOC)
HasState = Predicate(OBJ, STATE)

Clear = Predicate(LOC)
Safe = Predicate(OBJ, LOC)

IsDryer = Predicate(LOC)
IsPainter = Predicate(LOC)
IsWasher = Predicate(LOC)

UnsureLoc = Predicate(OBJ) # NOTE - can also just make these objects
UnsureState = Predicate(OBJ)
UnsureClear = Predicate(LOC)

NotAtLoc = Predicate(OBJ, LOC)

FindCost = Function('find_cost', [OBJ, LOC])
InspectStateCost = Function('inspect_state_cost', [OBJ, STATE])

O, O2, L1, L2, S = Param(OBJ), Param(OBJ), Param(LOC), Param(LOC), Param(STATE)

rename_easy(locals())

actions = [
  Action(name='find', parameters=[O, L1], # Policy to find the object or an inspection?
    condition=And(UnsureLoc(O), Not(NotAtLoc(O, L1))),
    effect=And(At(O, L1), Not(UnsureLoc(O)), Cost(FindCost(O, L1)))), # TODO - can even adjust the cost of these to bias search towards more likely

  Action(name='inspect_state', parameters=[O, L1, S],
    condition=And(UnsureState(O), At(O, L1)), # Probably should know where it is before you worry about its state
    #condition=And(UnsureState(O), Not(UnsureLoc(O))), # Probably should know where it is before you worry about its state
    effect=And(HasState(O, S), Not(UnsureState(O)), Cost(InspectStateCost(O, S)))), # TODO - should I factor loc in the cost?

  Action(name='transport', parameters=[O, L1, L2], # NOTE - Leslie and Tomas call this Move
    #condition=And(At(O, L1)),
    condition=And(At(O, L1), Clear(L2)),
    effect=And(At(O, L2), Not(At(O, L1))),
    cost=COST_SCALE*1),

  Action(name='wash', parameters=[O, L1, S],
    condition=And(At(O, L1), HasState(O, S), IsWasher(L1)),
    effect=And(HasState(O, clean), Not(HasState(O, S))),
    cost=COST_SCALE*1),

  Action(name='paint', parameters=[O, L1],
    condition=And(At(O, L1), HasState(O, clean), IsPainter(L1)),
    effect=And(HasState(O, wet), Not(HasState(O, clean))),
    cost=COST_SCALE*1),

  Action(name='dry', parameters=[O, L1],
    condition=And(At(O, L1), HasState(O, wet), IsDryer(L1)),
    effect=And(HasState(O, dry), Not(HasState(O, wet))),
    cost=COST_SCALE*1),

  Axiom(effect=Clear(L2), condition=ForAll([O2], Safe(O2, L2))),

  Axiom(effect=Safe(O, L1), condition=Exists([L2], And(At(O, L2), Not(Equal(L1, L2))))), # NOTE - automatically includes UnsureLoc
]

# TODO - do I need lower confidence bound that the object isn't there to prevent it from doing nothing or impossible things?

LOC_CONFIDENCE = .95
STATE_CONFIDENCE = LOC_CONFIDENCE
CLEAR_CONFIDENCE = STATE_CONFIDENCE

MIN_CONFIDENCE = .001 # TODO - how does this work in continuous domains?
MIN_P = 1e-6

def observable_problem(belief, goal, costs=True): # NOTE - costs is really important here
  #objects = belief.objLoc.keys()
  locations = belief.occupancies.keys()
  states = [dirty, clean, dry, wet]

  initial_atoms = []
  occupied = set()
  for obj in belief.objLoc.keys():
    dist = belief.objLocDist(obj)
    loc, p = dist.computeMPE() # TODO - concentration inequalities for Gaussian
    if p >= LOC_CONFIDENCE:
      initial_atoms.append(At(obj, loc))
      occupied.add(loc)
    else:
      initial_atoms.append(UnsureLoc(obj))
    for loc in locations:
      p = dist.prob(loc)
      cost = int(COST_SCALE*1./max(p, MIN_P)) if costs else COST_SCALE*1 # TODO - set to infinity if too large (equivalent to blocking)
      initial_atoms.append(Initialize(FindCost(OBJ(obj), LOC(loc)), cost))
      #if p < MIN_CONFIDENCE:
      #  initial_atoms.append(NotAtLoc(obj, loc))

  for obj in belief.objState.keys():
    dist = belief.objState[obj]
    state, p = dist.computeMPE()
    if p >= STATE_CONFIDENCE:
      initial_atoms.append(HasState(obj, state))
    else:
      initial_atoms.append(UnsureState(obj))
    for state in states:
      p = dist.prob(state)
      cost = int(COST_SCALE*1./max(p, MIN_P)) if costs else COST_SCALE*1
      initial_atoms.append(Initialize(InspectStateCost(OBJ(obj), STATE(state)), cost))
      #if p < MIN_CONFIDENCE:
      #  initial_atoms.append(NotHasState(obj, loc))

  for loc in belief.occupancies.keys():
    if loc == 'washer':
      initial_atoms.append(IsWasher(loc))
    elif loc == 'painter':
      initial_atoms.append(IsPainter(loc))
    elif loc == 'dryer':
      initial_atoms.append(IsDryer(loc))

  goal_literals = []
  for fluent in goal.fluents:
    if isinstance(fluent, Bd):
      literal, arg, prob = fluent.args
      if isinstance(literal, ObjLoc):
        goal_literals.append(At(literal.args[0], literal.value))
      elif isinstance(literal, ObjState):
        goal_literals.append(HasState(literal.args[0], arg))
      elif isinstance(literal, toyTest.Clear):
        goal_literals.append(Clear(literal.args[0]))
      else:
        raise NotImplementedError(literal)
    else:
      raise NotImplementedError(fluent)

  return STRIPStreamProblem(initial_atoms, goal_literals, actions, [], [])

##################################################

def unknown_policy(operators, goal, max_cost=10000):
  def fn(belief):
    problem = observable_problem(belief, goal)
    #print problem
    # NOTE - the cost sensitivity starts to factor into this...
    # NOTE - max cost prevents extremely large cost actions
    search = get_fast_downward('astar', verbose=False, max_cost=max_cost) # dijkstra | astar | wastar1 | wastar2 | wastar3 | eager | lazy
    plan, universe = incremental_planner(problem, search=search, frequency=1)
    cost = plan_cost(universe, plan)
    plan = convert_plan(plan)
    print 'Plan:', plan
    print 'Cost:', cost
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

def run_unknown():
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
    print operator.__dict__
    print

  problem = observable_problem(start.details, goal)
  print problem

  search = get_fast_downward('eager') # dijkstra | astar | wastar1 | wastar2 | wastar3 | eager | lazy
  plan, _ = incremental_planner(problem, search=search, frequency=1)
  print convert_plan(plan)
