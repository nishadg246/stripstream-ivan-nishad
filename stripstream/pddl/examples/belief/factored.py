from stripstream.pddl.logic.operations import Initialize
from stripstream.pddl.streams import Stream
from stripstream.pddl.cond_streams import TestCondStream

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
from stripstream.algorithms.search.fast_downward import get_fast_downward
from stripstream.pddl.utils import convert_plan, rename_easy
from stripstream.pddl.problem import STRIPStreamProblem

from stripstream.pddl.examples.belief.utils import *

OBJ, POSE, BELIEF = Type(), Type(), Type()
DIST = Type()

At = Predicate(OBJ, POSE)
BAt = Predicate(OBJ, POSE, BELIEF)
BAtAbove = Predicate(OBJ, POSE, BELIEF)
Above = Predicate(BELIEF, BELIEF)

IsUpdate = Predicate(BELIEF, BELIEF)
IsPossible = Predicate(BELIEF, BELIEF)

IsClean = Predicate(DIST, DIST)

BClean = Predicate(OBJ, DIST)
BDirty = Predicate(OBJ, DIST) # TODO - I could just process this as one parameter

rename_easy(locals())

# NOTE - this is like transforming each position dist into a bernoulli distribution (don't do this for others)
# - this naturally ignores the interactions between objects which is good
# - well they still are slightly different. Gaussians have spacial properties. This leads to shadow objects
# - the spacial distribution of uncertainty is important in both. For Gaussians, the covariance. For us, the special cells

# 3 distribution update options
# - Always update all dists after an observation
# - Update one object's full dist
# - Update the mode and ignore the rest

# TODO - two versions. One reasons about object distributions directly and another reasons about free space

# Plans a sequence of look actions to get between two beliefs

"""
class BoundaryBelief(TestCondStream):
  def __init__(self, oracle):
    self.oracle = oracle
    q1, q2 = P('q1', CONF), P('q2', CONF)
    super(CollisionFreeTest, self).__init__([q1, q2], [], [
      IsCollisionFree(q1, q2),
      IsCollisionFree(q2, q1),
      MoveCost(q1, q2), # TODO - need to specify the truth value
    ], eager=EAGER_COLLISION)
    self.collision = collision_fn(oracle.env, oracle.robot, check_self=True) # q_collision_fn
    self.extend = extend_fn(oracle.robot) # q_extend_fn
    self.distance = distance_fn(oracle.robot)
  class StreamFn(Stream):
    max_distance = .5
    handles = []
    scale = 100 # The cost must be a nonnegative integer
    def get_values(self, **kwargs):
      self.enumerated = True
      q1, q2 = self.inputs
      d = self.cond_stream.distance(q1.value.value, q2.value.value)
      if d > self.max_distance:
        return []
      for q in self.cond_stream.extend(q1.value.value, q2.value.value):
        if self.cond_stream.collision(q):
          return []
      self.handles.append(draw_edge(self.cond_stream.oracle.env, q1.value.value, q2.value.value, color=(1, 0, 0, .5)))
      cost = int(self.scale*d+1)
      #return [IsCollisionFree(q1, q2), Initialize(MoveCost(q1, q2), )]
      return [IsCollisionFree(q1, q2), IsCollisionFree(q2, q1),
              Initialize(MoveCost(q1, q2), cost), Initialize(MoveCost(q2, q1), cost)]
"""

class PerfectLook(Action):
  def __init__(self):
    obj, p, prior = Param(OBJ), Param(POSE), Param(BELIEF)
    post = 1
    super(PerfectLook, self).__init__(self.__class__.__name__, [obj, p, prior],
      And(
        #At(obj, p),
        BAt(obj, p, prior)),
      And(
        BAt(obj, p, 1),
        Not(BAt(obj, p, prior))))
  def executable(self, operators, (obj, p, _)):
    executable_op = make_look(operators, obj, p)
    executable_op.parent = self
    return executable_op

# TODO - should I update all the probabilities?
# NOTE - make approximation that only want to believe more things and would not look intentionally to incidentally learn something
# TODO - combine the pose and belief as an object
# TODO - maybe I should instead just do forward search and guide using a determinized heuristic

class Look(Action):
  def __init__(self, p_obs_t, p_obs_f=None, log_cost=False):
    #self.p_obs_t = p_obs_t
    #if p_obs_f is None: self.p_obs_f = 1 - p_obs_t
    #self.log_cost = log_cost
    cost = prob_cost(p_obs_t, log_cost=log_cost) # Conditions on successful observation
    cost = None

    obj, p, prior, post = Param(OBJ), Param(POSE), Param(BELIEF), Param(BELIEF)
    super(Look, self).__init__(self.__class__.__name__, [obj, p, prior, post],
      And(
        #At(obj, p),
        BAt(obj, p, prior),
        IsUpdate(prior, post)),
      And(
        BAt(obj, p, post),
        Not(BAt(obj, p, prior))), cost)
  def executable(self, operators, (obj, p, prior, post)):
    executable_op = make_look(operators, obj, p)
    executable_op.parent = self
    return executable_op # NOTE - because the look involves a particular object

class BackwardLook(Action):
  def __init__(self, p_obs_t):
    obj, p, prior, post = Param(OBJ), Param(POSE), Param(BELIEF), Param(BELIEF)
    super(BackwardLook, self).__init__(self.__class__.__name__, [obj, p, prior, post],
      And(
        #At(obj, p),
        BAt(obj, p, prior),
        #BAtAbove(obj, p, prior), # Problem where we want to delete the old belief, but we need to include the reference belief
        IsPossible(prior, post)),
      And(
        BAt(obj, p, post),
        Not(BAt(obj, p, prior))))

  def executable(self, operators, (obj, p, prior, post)):
    executable_op = make_look(operators, obj, p)
    executable_op.parent = self
    return executable_op

# NOTE
# - Transition should move all the probability mass to first states
# - Well then how do you sample a belief state to move towards?
# - Yeah I'll have to factor the location and belief

# NOTE
# - I could just consider the movement of probability mass between the first and second state
# - The problem is that the full state needs to be updated when we do this

class Transport(Action): # NOTE - optimistic that no belief is lost
  def __init__(self, min_safe_p=.95):
    obj, p1, p2, b = Param(OBJ), Param(POSE), Param(POSE), Param(BELIEF)
    Action.__init__(self, self.__class__.__name__, [obj, p1, p2, b],
      And(
        #At(obj, p1),
        BAt(obj, p1, b),
        BAtAbove(obj, p1, min_safe_p)),
      And(
        #At(obj, p2),
        BAt(obj, p2, b),
        #Not(At(obj, p1)),
        Not(BAt(obj, p1, b))), cost=1)
  def executable(self, operators, (obj, p1, p2, _)):
    executable_op = make_transport(operators, obj, p1, p2)
    executable_op.parent = self
    return executable_op

class Clean(Action):
  def __init__(self, min_safe_p=.95):
    obj, d1, d2 = Param(OBJ), Param(DIST), Param(DIST)
    Action.__init__(self, self.__class__.__name__, [obj, d],
      And(
        BClean(obj, d1),
        IsClean(d1, d2),
        #BAtAbove(obj, p1, min_safe_p)
      ),
      And(
        BClean(obj, d2),
        #Not(At(obj, p1)),
        Not(BClean(obj, d1))), cost=1)
  def executable(self, operators, (obj, p1, p2, _)):
    executable_op = make_wash(operators, obj)
    executable_op.parent = self
    return executable_op

##################################################

# Collision free values

# TODO - could also test if something is the MLO using a deferred literal
# TODO - allow choice of which world we live in as opposed to max likelihood

# TODO - sample belief state?
# TODO - do I want a deferred literal to test things or do I want to specify them all upfront?

def compile_belief(belief, goal):
  locations, _, _ = maximum_likelihood_obs(belief)

  constants = map(OBJ, belief.objLoc.keys()) + map(POSE, belief.occupancies.keys())
  initial_atoms = []
  #initial_atoms += [At(obj, p) for obj, p in locations.iteritems()]
  initial_atoms += [BAt(obj, p, round(belief.objLocDist(obj).prob(p), 3)) for obj, p in locations.iteritems()] # NOTE - using the maximum belief here
  goal_literals = []
  for fluent in goal.fluents:
    if isinstance(fluent, Bd):
      literal, arg, prob = fluent.args
      if isinstance(literal, ObjLoc):
        goal_literals.append(BAtAbove(literal.args[0], literal.value, prob))
      elif isinstance(literal, ObjState) and arg == 'clean':
        goal_literals.append(BClean(literal.args[0], prob))
      else:
        raise NotImplementedError(literal)
    else:
      raise NotImplementedError(fluent)

  O, P, B = Param(OBJ), Param(POSE), Param(BELIEF)
  B1, B2 = Param(BELIEF), Param(BELIEF)

  p_obs_t = 1 - glob.failProbs['Look']
  p_obs_f = 1 - p_obs_t
  #cost = prob_cost(p_obs_t) # This isn't informative
  #cost = prob_cost(float(prior.name)*p_obs_t) # TODO - need to automatically derive the costs

  actions = [
    #PerfectLook(),
    Look(p_obs_t),
    #BackwardLook(p_obs_t),
    Transport(),
  ]
  axioms = [
    Axiom(BAtAbove(O, P, B2), Exists([B1], And(BAt(O, P, B1), Above(B1, B2)))),
    #Axiom(IsPossible(B1, B2), Exists([B], And(IsUpdate(B, B2), Above(B1, B)))), # NOTE - this only uses static facts
  ]

  cond_streams = [
    GeneratorStream(inputs=[B1], outputs=[B2], conditions=[], effects=[IsUpdate(B1, B2)],
                    generator=lambda b1: [round(forward_belief(b1, p_obs_t, p_obs_f), 3)]),
    #GeneratorStream(inputs=[B2], outputs=[B1], conditions=[], effects=[IsUpdate(B1, B2)],
    #                generator=lambda b2: [round(inverse_belief(b2, p_obs_t, p_obs_f), 3)]), # NOTE - I previously used a Deferred Literal to produce the initial belief

    #TestStream(inputs=[O, P, B1], conditions=[], effects=[BAt(O, P, B1)],
    #           test=lambda o, p, b: belief.objLocDist(o).prob(p) >= b, eager=True),

    TestStream(inputs=[B1, B2], conditions=[], effects=[Above(B1, B2)],
               test=lambda b1, b2: b1 >= b2, eager=True),
    #TestStream(inputs=[B1, B2, B], conditions=[IsUpdate(B, B2), Above(B1, B)], effects=[IsPossible(B1, B2)],
    #           test=lambda *args: True, eager=True),
  ]

  return STRIPStreamProblem(initial_atoms, goal_literals, actions + axioms, cond_streams, constants)

##################################################

def online_policy(operators, goal, only_first=True): # TODO - avoid translating/save some work
  online_policy.last_plan = None
  # TODO - might need to store the last dynamic problem
  def fn(belief):
    problem = compile_belief(belief, goal)
    search = get_fast_downward('eager') # dijkstra | astar | wastar1 | wastar2 | wastar3 | eager | lazy
    plan, _ = incremental_planner(problem, search=search, frequency=1)
    print convert_plan(plan)
    # TODO - need to apply axioms to do this
    # TODO - need to also be careful that constants contains all the necessary constants if using quantifiers
    """
    if goal_formula.holds(initial_atoms, constants):
      return []
    if bias_plan is not None:
      # NOTE - testing if any subplan works is NP-Complete
      #candidate_plans = [bias_plan[1:], bias_plan]
      candidate_plans = [bias_plan[1:]]
      for candidate_plan in candidate_plans:
        instantiated_plan = [action.instantiate(args) for action, args in candidate_plan]
        if len(instantiated_plan) >= 1:
          if not isinstance(instantiated_plan[0], RefinableOperator) and \
              is_valid_plan(initial_atoms, goal_formula, constants, instantiated_plan):
            return candidate_plan
    """
    plan = convert_plan(plan)
    if plan is None or not plan:
      return None
    online_policy.last_plan = plan # TODO - use the next action on the last plan if it still is valid
    action, params = plan[0]
    return action.executable(operators, params)
  return fn
