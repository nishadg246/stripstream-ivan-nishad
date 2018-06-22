from stripstream.pddl.streams import Stream
from stripstream.pddl.cond_streams import CondStream

from toyTest import Belief, glob, BBhAddBackBSetNew, UniformDist, DD, State, Bd, ObjLoc, makeOperators, objDictCopy, World, hpn, ObjState
#from bhpn.hpn.fbch import *
#import hpn

from stripstream.pddl.objects import EasyType as Type, EasyParameter as Param
from stripstream.pddl.logic.predicates import EasyPredicate as Predicate
from stripstream.pddl.operators import Action, Axiom
from stripstream.pddl.logic.connectives import And, Not
from stripstream.pddl.logic.quantifiers import Exists
from stripstream.pddl.cond_streams import EasyGenStream as GeneratorStream, EasyTestStream as TestStream
from stripstream.algorithms.incremental.incremental_planner import incremental_planner
from stripstream.algorithms.focused.focused_planner import focused_planner
from stripstream.algorithms.focused.simple_focused import simple_focused
from stripstream.algorithms.search.fast_downward import get_fast_downward
from stripstream.pddl.utils import convert_plan, rename_easy
from stripstream.pddl.problem import STRIPStreamProblem
from stripstream.pddl.logic.predicates import Function
from stripstream.pddl.logic.operations import Initialize, Cost
from stripstream.algorithms.utils import plan_cost
from stripstream.algorithms.plan import plan_length, plan_cost

from stripstream.pddl.examples.belief.utils import *
from stripstream.pddl.examples.belief.unknown_no_occup import OPERATOR_MAP
from stripstream.pddl.utils import get_value

UNIT = False
FOCUSED = True
COST_SCALE = 100 # TODO - need to adjust default cost

# TODO - can simulate by applying the belief space functions and passing a custom observation or result
# TODO - Encode pose in it

OBJ, LOC, BELIEF = Type(), Type(), Type()
PROB, CONCENTRATION = Type(), Type()

#concentration = CONCENTRATION(('i1', .95))

##########

BAt = Predicate(OBJ, BELIEF)
BAtAbove = Predicate(OBJ, LOC, PROB) # TODO - should OBJ go in here?

BSatisfies = Predicate(BELIEF, LOC, PROB)
IsLookUpdate = Predicate(BELIEF, LOC, BELIEF)
IsMoveUpdate = Predicate(LOC, BELIEF, LOC, BELIEF)

#IsBelief = Predicate(OBJ, BELIEF)

LookCost = Function('look_cost', [BELIEF, LOC])
MoveCost = Function('move_cost', [LOC, BELIEF])

##########

O = Param(OBJ)
B, B2 = Param(BELIEF), Param(BELIEF)
L, L2 = Param(LOC), Param(LOC)
P = Param(PROB)

rename_easy(locals())

##################################################

class LookUpdate(CondStream): # NOTE - this is pretty bad because there are so many combinations of locations
  def __init__(self):
    super(LookUpdate, self).__init__([B, L], [B2], [
    ], [
      IsLookUpdate(B, L, B2),
      #LookCost(B, L), # How do I signal this?
    ])
  class StreamFn(Stream):
    def get_values(self, **kwargs):
      self.enumerated = True
      prior, loc = map(get_value, self.inputs)
      post, prob = look_update(loc, prior, True)
      if prior.prob(loc) >= .95:
        return [] # Set a maximum probability update
      #post = BELIEF(post)
      cost = int(COST_SCALE*geom_cost(1, prob))
      return [IsLookUpdate(prior, loc, post), Initialize(LookCost(*self.inputs), cost)]

class PerfectLookUpdate(CondStream):
  def __init__(self):
    super(PerfectLookUpdate, self).__init__([B, L], [B2], [], [
      IsLookUpdate(B, L, B2),
      #LookCost(B, L), # How do I signal this?
    ])
  class StreamFn(Stream):
    def get_values(self, **kwargs):
      self.enumerated = True
      prior, loc = map(get_value, self.inputs)
      _, prob = look_update(loc, prior, True)
      perfect = DDist({loc: 1.})
      cost = int(COST_SCALE*geom_cost(1, prob)) if not UNIT else 1
      return [IsLookUpdate(prior, loc, perfect),
              Initialize(LookCost(*self.inputs), cost)] # Could include satisfies here...

##################################################

class LookCostFn(CondStream):
  def __init__(self):
    super(LookCostFn, self).__init__([B, L], [], [], [], eager=True)
  class StreamFn(Stream):
    def get_values(self, **kwargs):
      self.enumerated = True
      prior, loc = map(get_value, self.inputs)
      #_, prob = look_update(loc, prior, True)
      prob = prior.prob(loc) #*(1 - glob.failProbs['Look'])
      cost = int(COST_SCALE*geom_cost(1, prob)) if not UNIT else 1
      return [Initialize(LookCost(*self.inputs), cost)]

class MoveCostFn(CondStream):
  def __init__(self):
    super(MoveCostFn, self).__init__([L, B], [], [], [], eager=True)
  class StreamFn(Stream):
    def get_values(self, **kwargs):
      self.enumerated = True
      loc, prior = map(get_value, self.inputs)
      #loc1, prior, loc2 = map(get_value, self.inputs)
      #post, prob = move_update(loc1, prior, loc2)
      prob = prior.prob(loc) # *(1 - glob.failProbs['Move'])
      cost = int(COST_SCALE*geom_cost(1, prob)) if not UNIT else 1
      return [Initialize(MoveCost(*self.inputs[:2]), cost)]

##################################################

# TODO - allow exclusion of B2?

class LookPlan(CondStream):
  def __init__(self):
    super(LookPlan, self).__init__([B, L, P], [B2], [], [
      IsLookUpdate(B, L, B2), # Need for focused
      BSatisfies(B2, L, P),
    ])
  class StreamFn(Stream):
    def get_values(self, **kwargs):
      self.enumerated = True # NOTE - could return one action that does the full look
      prior, loc, p_goal = map(get_value, self.inputs)
      post = prior
      facts = []
      while post.prob(loc) < p_goal: # NOTE - could also move one step
        post, prob = look_update(loc, post, True)
        cost = int(COST_SCALE*geom_cost(1, prob))
        facts += [IsLookUpdate(prior, loc, post),
              Initialize(LookCost(*self.inputs[:2]), cost)]
      facts.append(BSatisfies(post, loc, p_goal))
      return facts

##################################################

# NOTE
# - the problem with the focused algorithm is the sharing of the abstract constant which satisfies the goal
# - why isn't this a problem before? I think the dependence on the object helped separate?
# - maybe the grounding version of this would be better
# - L is a control parameter which isn't tied to the actual state

class LookJump(CondStream):
  def __init__(self):
    super(LookJump, self).__init__([B, L, P], [B2], [], [
      IsLookUpdate(B, L, B2), # Need for focused
      BSatisfies(B2, L, P),
    ])
  class StreamFn(Stream):
    def get_values(self, **kwargs):
      self.enumerated = True # NOTE - could return one action that does the full look
      prior, loc, p_goal = map(get_value, self.inputs)
      if prior.prob(loc) >= p_goal:
        return [BSatisfies(prior, loc, p_goal)]
      post, cost = prior, 0
      while post.prob(loc) < p_goal: # NOTE - could also move one step
        post, prob = look_update(loc, post, True)
        cost += int(COST_SCALE*geom_cost(1, prob))
      return [IsLookUpdate(prior, loc, post),
              BSatisfies(post, loc, p_goal),
              Initialize(LookCost(*self.inputs[:2]), cost)]

class PerfectLookJump(CondStream):
  def __init__(self):
    super(PerfectLookJump, self).__init__([B, L, P], [B2], [], [
      IsLookUpdate(B, L, B2), # Need for focused
      BSatisfies(B2, L, P),
    ])
  class StreamFn(Stream):
    def get_values(self, **kwargs):
      self.enumerated = True
      prior, loc, p_goal = map(get_value, self.inputs)
      _, p_obs = look_update(loc, prior, True)
      perfect = DDist({loc: 1.})
      #cost = int(COST_SCALE*geom_cost(1, p_obs)) if not UNIT else 1
      return [IsLookUpdate(prior, loc, perfect),
              BSatisfies(perfect, loc, p_goal),
              #Initialize(LookCost(*self.inputs[:2]), cost)
              ] # Could include satisfies here...

# If you don't know the dynamics, looking anywhere could possibly increase prob
# Something still isn't right... it isn't cost sensitive because the cost is inferred after the plan
# Even if I make it plan on the next iteration, there always is a cheaper action? Why does it take the same thing?
# I think the problem is that it keeps adding belief arguments B which will prove that the abstract thing works
# If I excluded them before I added them to the pool, this wouldn't happen
# Don't worry too much, choosing a spot to look at won't really happen in the Gaussian case?

class PerfectLookJump2(CondStream):
  def __init__(self):
    super(PerfectLookJump2, self).__init__([B, L2, P], [L, B2], [], [
      IsLookUpdate(B, L, B2), # Need for focused
      BSatisfies(B2, L2, P),
    ])
  class StreamFn(Stream):
    def get_values(self, **kwargs):
      self.enumerated = True
      prior, loc, p_goal = map(get_value, self.inputs)
      _, p_obs = look_update(loc, prior, True)
      perfect = DDist({loc: 1.})
      #cost = int(COST_SCALE*geom_cost(1, p_obs)) if not UNIT else 1
      return [IsLookUpdate(prior, loc, perfect),
              BSatisfies(perfect, loc, p_goal),
              #Initialize(LookCost(*self.inputs[:2]), cost)
              ] # Could include satisfies here...

##################################################

class MoveUpdate(CondStream):
  def __init__(self):
    super(MoveUpdate, self).__init__([L, B, L2], [B2], [], [IsMoveUpdate(L, B, L2, B2)])
  class StreamFn(Stream):
    def get_values(self, **kwargs):
      self.enumerated = True
      loc1, prior, loc2 = map(get_value, self.inputs)
      post, prob = move_update(loc1, prior, loc2)
      cost = int(COST_SCALE*geom_cost(1, prob)) if not UNIT else 1
      return [IsMoveUpdate(loc1, prior, loc2, post),
              Initialize(MoveCost(*self.inputs[:2]), cost)] # TODO - make this a test stream

##################################################

# NOTE - the location here is like a control parameter

actions = [
  Action(name='transport', parameters=[O, L, L2, B, B2], # NOTE - Leslie and Tomas call this Move
    condition=And(BAt(O, B), IsMoveUpdate(L, B, L2, B2)),
    effect=And(BAt(O, B2), Not(BAt(O, B)), #)),
               Cost(MoveCost(L, B)))),

  #Action(name='transport', parameters=[O, B, L, B2], # NOTE - I could include belief preconditions for safety or effectiveness
  #  condition=And(BAt(O, B), BAtAbove(L, B2, ), IsMoveUpdate(B, L, B2)),
  #  effect=And(BAt(O, B2), Not(BAt(O, B)))), # NOTE - Leslie and Tomas call this Move

  Action(name='find', parameters=[O, L, B, B2],
    condition=And(BAt(O, B), IsLookUpdate(B, L, B2)),
    effect=And(BAt(O, B2), Not(BAt(O, B)), #)),
               Cost(LookCost(B, L)))),

  Axiom(effect=BAtAbove(O, L, P), condition=Exists([B], And(BAt(O, B), BSatisfies(B, L, P)))),
]

##################################################

def is_above(dist, loc, prob):
  #print dist, loc, prob, dist.prob(loc) >= prob
  return dist.prob(loc) >= prob

def compile_belief(belief, goal):
  constants = map(OBJ, belief.objLoc.keys()) + map(LOC, belief.occupancies.keys())
  #constants = []
  initial_atoms = []
  initial_atoms += [BAt(obj, belief.objLocDist(obj)) for obj in belief.objLoc] # NOTE - objLocDist != objLoc
  goal_literals = []
  for fluent in goal.fluents:
    if isinstance(fluent, Bd):
      literal, arg, prob = fluent.args
      if isinstance(literal, ObjLoc):
        goal_literals.append(BAtAbove(literal.args[0], literal.value, prob))
      #elif isinstance(literal, ObjState) and arg == 'clean':
      #  goal_literals.append(BClean(literal.args[0], prob))
      else:
        raise NotImplementedError(literal)
    else:
      raise NotImplementedError(fluent)

  cond_streams = [
    # Forward streams
    #LookUpdate(),
    #PerfectLookUpdate(),

    # Oriented streams
    #LookPlan(),
    #LookJump(),
    PerfectLookJump(),
    #PerfectLookJump2(),

    # Independent costs
    LookCostFn(),
    MoveCostFn(),

    MoveUpdate(),
    #GeneratorStream(inputs=[L, B, L2], outputs=[B2], conditions=[], effects=[IsMoveUpdate(L, B, L2, B2)],
    #                generator=lambda l, b, l2: [move_update(l, b, l2)[0]]),

    #TestStream(inputs=[B, L, P], conditions=[], effects=[BSatisfies(B, L, P)],
    #           test=is_above, eager=True),
  ]

  return STRIPStreamProblem(initial_atoms, goal_literals, actions, cond_streams, constants)

##################################################

# TODO - make a max size parameter?
# NOTE - need to pre-compute costs for the focused algorithm
# NOTE - with precomputed costs, the actions are must be optimal (true with admissible costs in general)
# Need to provide a cost estimate for all "abstract actions". Don't want zero cost because no cost

# NOTE - it might be currently adding an extra-precondition or something for action costs...

# TODO - the focused algorithm is sampling random things with the intention of just generating a new value which will optimistically satisfy

def forward_policy(operators, goal): # TODO - avoid translating/save some work
  def fn(belief):
    problem = compile_belief(belief, goal)
    if FOCUSED:
      # TODO - try the plan_focused algorithm here
      search = get_fast_downward('astar', verbose=True) # dijkstra | astar | wastar1 | wastar2 | wastar3 | eager | lazy
      #plan, universe = focused_planner(problem, search=search, stream_cost=0, verbose=True, optimal=True, debug=False) # stream_cost = 0 instead of None
      plan, universe = simple_focused(problem, search=search, stream_cost=0, max_level=0, verbose=True, optimal=True, debug=False)
    else:
      search = get_fast_downward('wastar1') # dijkstra | astar | wastar1 | wastar2 | wastar3 | eager | lazy
      plan, universe = incremental_planner(problem, search=search, frequency=1, optimal=True, waves=True, debug=False, max_calls=5000)
    print
    print 'Cost:', plan_cost(universe, plan)
    print 'Length:', plan_length(universe, plan)
    plan = convert_plan(plan)
    print 'Plan:', plan
    if plan is None or not plan:
      return None
    action, params = plan[0]
    return OPERATOR_MAP[action.name](operators, *params)
  return fn
