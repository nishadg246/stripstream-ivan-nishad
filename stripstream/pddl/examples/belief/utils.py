from toyTest import glob, makeOperators, Bd, ObjState, ObjLoc, Clear
from util.dist import DDist
from copy import deepcopy
from itertools import product

import math

# TODO - what is PR? - is it probability?
# TODO - the args are really just for TLPK's operators. Put something generic in?

dirty, clean, dry, wet = 'dirty', 'clean', 'dryPaint', 'wetPaint'
STATES = [dirty, clean, dry, wet]

# TODO - instantiate with probabilities to get preconditions for safety

def make_look(operators, obj, p, *args):
  return operators['Look'].applyBindings({'Obj': obj, 'Loc': p}, rename=False)

def make_look_state(operators, obj, p, state, *args):
  return operators['LookState'].applyBindings({'Obj': obj, 'Loc': p, 'State': state}, rename=False)

def make_look_clear(operators, obj, p):
  return operators['LookToClear'].applyBindings({'Obj': obj, 'Loc': p}, rename=False) # TODO - why does this have an obj argument?

def make_transport(operators, obj, p1, p2, *args):
  return operators['Move'].applyBindings({'Obj': obj, 'Start': p1, 'Dest': p2}, rename=False)

def make_wash(operators, obj, *args):
  return operators['Wash'].applyBindings({'Obj': obj}, rename=False)

def make_paint(operators, obj, *args):
  return operators['Paint'].applyBindings({'Obj': obj}, rename=False)

def make_dry(operators, obj, *args):
  return operators['Dry'].applyBindings({'Obj': obj}, rename=False)

# TODO - instantiate with probabilities to get preconditions for safety
def get_operator_instances(operators, belief):
  objects = belief.objLoc.keys()
  locations = belief.occupancies.keys()
  operators = [] + \
    [make_transport(operators, *args) for args in product(objects, locations, locations)] + \
    [make_look(operators, *args) for args in product(objects, locations)] + \
    [make_look_state(operators, *args) for args in product(objects, locations, STATES)] #+ \
    #[make_wash(operators, *args) for args in product(objects)] + \
    #[make_paint(operators, *args) for args in product(objects)] + \
    #[make_dry(operators, *args) for args in product(objects)] + \
    #[make_look_clear(operators, *args) for args in product(objects, locations)] #+ \
  return operators

##################################################

def convert_start(env, start):
  locations = {}
  states = {}
  occupancies = {loc: False for loc in start.details.occupancies}
  for obj in env.objects:
    for attr in env.objects[obj]:
      if attr in STATES:
        states[obj] = attr
      elif attr in occupancies:
        locations[obj] = attr
        occupancies[attr] = True
      else:
        raise NotImplementedError(attr)
  return locations, states, occupancies

def convert_goal(goal):
  locations, states, occupancies = {}, {}, {}
  for fluent in goal.fluents:
    if isinstance(fluent, Bd):
      literal, arg, prob = fluent.args
      if isinstance(literal, ObjLoc):
        obj, loc = literal.args[0], literal.value
        locations[obj] = loc
      elif isinstance(literal, ObjState):
        obj, state = literal.args[0], arg
        states[obj] = state
      elif isinstance(literal, Clear):
        loc, occ = literal.args[0], False
        occupancies[loc] = occ
      else:
        raise NotImplementedError(literal)
    else:
      raise NotImplementedError(fluent)
  return locations, states, occupancies

def holds(pre, state):
  #return pre.valueInDetails(state.details) == True
  return state.fluentValue(pre) == True

def belief_satisfies(belief, goal):
  #return all(state.fluentValue(fluent) == True for fluent in goal.fluents)
  return all(fluent.valueInDetails(belief) == True for fluent in goal.fluents)

def real_satisfies(env, state, goal):
  return all(set(g.values()) <= set(s.values()) for s, g in zip(convert_start(env, state), convert_goal(goal)))

def op_pre(op):
  for level in op.preconditions:
    for pre in op.preconditions[level]:
      yield pre

def applicable_operator(state, op): # NOTE - need to pass probability arguments to make this work
  return all(holds(pre, state) for pre in op_pre(op))

def execute_operator(env, state, op):
  #return executePrim(op, state, env, None)
  return env.executePrim(op, op.params)

def apply_operator_observation(state, op, obs):
  new_state = deepcopy(state)
  new_state.updateStateEstimate(op, obs, draw=False)
  return new_state

##################################################

def prob_cost(p, log_cost=False):
  if log_cost:
    return -math.log(p)
  #return 1./p
  return int(1./p + 1)

def log_cost(p, min_p=1e-6):
  return -math.log(max(p, min_p))

def geom_cost(cost, p, min_p=1e-6):
  return cost/max(p, min_p)

# Cost for MDP where failures remain in the same state
def mdp_cost(success_cost, failure_cost, p, min_p=1e-6):
  # Equals geom_cost(cost, p) == mdp_cost(cost, cost, p)
  return success_cost + failure_cost*(1./max(p, min_p) - 1.)

def expected_cost(success_cost, failure_cost, p):
  # Equals geom_cost(cost, p) == mdp_cost(cost, cost, p)
  return p*success_cost + failure_cost*(1-p)

#dist.transitionUpdate
#dist.obsUpdate
#dist.copy()

from toyFactoryOps import lookObsModelTarget


# f = lookStateBProgress, is the transition function
# prim = lookPrim, is the observation function

# lookPrim returns obj and loc
# defaultPrim returns None

# Within executePrim
# obs = env.executePrim(op, params)
# env.executePrim is defined within toyFactory
# updateStateEstimate calls f

# lookBProgress
def look_update(look_loc, prior, obs):
  obs_model = lambda loc: lookObsModelTarget(loc, look_loc, glob.failProbs['Look'])
  posterior = prior.copy()
  prob_obs = posterior.obsUpdate(obs_model, obs)
  return posterior, prob_obs # returns the probability of the observation as well

# lookStateBProgress
# lookNotBProgress (same as lookBProgress and not used)

# TODO - the full transition model is complicated
# moveBProgress


from operator import mul

from toyFactoryOps import moveTransModel

# NOTE - need to reason about the poses of other objects to declare success...
# Cost is based on all the other objects
# Could pass in occupancy probabilities then I would have it update like every state as well...

def move_update(start_loc, prior, dest_loc):
  p_move = 1 - glob.failProbs['Move']
  #objClearProbs = dict((o, 1 - d.prob(dest_loc)) for (o, d) in prior.iteritems())
  #clearProb = reduce(mul, [objClearProbs[o] for o in prior if o != obj], 1)
  #p_failure = 1 - (p_move * clearProb)     # move wins, dest is clear

  p_failure = 1 - p_move
  transModel = lambda oldLoc: moveTransModel(oldLoc, start_loc, dest_loc, p_failure)
  posterior = prior.copy()
  #print
  #print start_loc, prior, dest_loc
  posterior.transitionUpdate(transModel)
  #raw_input('Continue')

  #dp = dirtyProb * dist.prob(start)
  return posterior, prior.prob(start_loc)*p_move # Modeling the probability that everything goes as planned (no accidental successes)

# Optimistically assumes that the object is there and observation is true
# p_obs_t = P(O | X = True), p_obs_t = P(O | X = False)
def forward_belief(p_prior, p_obs_t, p_obs_f=None):
  if p_obs_f is None: p_obs_f = 1 - p_obs_t # Symmetric
  return p_obs_t * p_prior / (p_obs_t*p_prior + p_obs_f*(1 - p_prior))

def inverse_belief(p_post, p_obs_t, p_obs_f=None):
  if p_obs_f is None: p_obs_f = 1 - p_obs_t # Symmetric
  print p_obs_f * p_post / (p_obs_f * p_post + p_obs_t*(1 - p_post)), p_post
  return p_obs_f * p_post / (p_obs_f * p_post + p_obs_t*(1 - p_post))

#p_prior = .3, p_obs_t = .9
#assert inverse_belif(forward_belief(p_prior, p_obs_t), p_obs_t) == p_prior

def mlo_bernoulli(dist):
  mlo = dist.computeMPE()[0]
  return dist.project(lambda x: x if x == mlo else None)

##################################################

def consistent_locations(locations):
  return len(locations) == len(set(locations.values()))

def mlo_locations(belief):
  for obj in belief.objLoc:
    dist = belief.objLocDist(obj)
    print dist
    print mlo_bernoulli(dist)
  raw_input('Pause')
  return {obj: belief.objLocDist(obj).computeMPE()[0] for obj in belief.objLoc}

def sample_locations(belief):
  return {obj: belief.objLocDist(obj).draw() for obj in belief.objLoc}

def mlo_states(belief):
  return {obj: belief.objState[obj].computeMPE()[0] for obj in belief.objState}

def sample_states(belief):
  return {obj: belief.objState[obj].draw() for obj in belief.objState}

def infer_occupancies(belief, locations):
  occupied = set(locations.values())
  return {loc: loc in occupied for loc in belief.occupancies}

# TODO - allow the planner to choose the initial state but include the cost of each
def maximum_likelihood_obs(belief): # TODO - randomly break ties
  locations = mlo_locations(belief)
  states = mlo_states(belief)
  #occupancies = {loc: belief.locOccDist(loc).computeMPE()[0] for loc in belief.occupancies} # TODO - sample or infer?
  occupancies = infer_occupancies(belief, locations)
  return locations, states, occupancies

def sample_obs(belief):
  locations = sample_locations(belief)
  states = sample_states(belief)
  occupancies = {loc: belief.locOccDist(loc).draw() for loc in belief.occupancies} # TODO - could maintain the fog as well
  return locations, states, occupancies

def sample_consistent_obs(belief):
  while True:
    locations = sample_locations(belief)
    if not consistent_locations(locations):
      continue
    return locations, sample_states(belief), infer_occupancies(belief, locations)
