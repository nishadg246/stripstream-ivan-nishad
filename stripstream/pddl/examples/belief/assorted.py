from random import choice
from toyTest import glob, State

from stripstream.pddl.examples.belief.utils import *

OPERATOR_OBS = {
  'Move': ['whiff', 'bump', None],
  'Look': [True, False],
  'LookToClear': [True, False],
  'LookState': STATES,
  'Wash': [None],
  'Paint': [None],
  'Dry': [None],
}

HELPFUL_OPERATOR_OBS = {
  'Move': lambda op: [None], #[op.args[2]] if op.args[1] != op.args[2] else [],
  'Look': lambda op: [True],
  'LookToClear': lambda op: [True], # NOTE - false?
  'LookState': lambda op: [op.args[2]],
}

def get_operator_cost(state, op):
  #cost = op.cost(None, op.args, current_state.details) # NOTE - this requires probabilities
  belief = state.details
  cost = 1
  if op.name == 'Move':
    obj, l1, l2 = op.args[:3]
    p_obj_l1 = belief.objLoc[obj].prob(l1) # objLocDist(obj)?
    p_l2_free = belief.locOccDist(l2).prob(False)
    p_success = 1 - glob.failProbs['Move']
    return geom_cost(cost, p_obj_l1*p_l2_free*p_success)
  if op.name == 'Look':
    obj, loc = op.args[:2]
    p_obj_loc = belief.objLoc[obj].prob(loc)
    p_success = 1 - glob.failProbs['Look']
    return geom_cost(cost, p_obj_loc*p_success)
  #if op.name == 'LookToClear':
  #  obj, loc = op.args[:2]
  #  p_obj_loc = 1 - belief.objLoc[obj].prob(loc) # TODO - is this right?
  #  p_success = 1 - glob.failProbs['Look']
  #  return geom_cost(cost, p_obj_loc*p_success)
  if op.name == 'LookState':
    obj, loc, state = op.args[:3]
    p_obj_loc = belief.objLoc[obj].prob(loc)
    p_obj_state = belief.objState[obj].prob(state)
    p_success = 1 - glob.failProbs['Look']
    return geom_cost(cost, p_obj_loc*p_obj_state*p_success)
  if op.name == 'Wash': # TODO - should you know where the object is to clean it?
    #obj, = op.args[:1]
    #p_obj_state = belief.objState[obj].prob('dirty')
    p_obj_state = 1. # NOTE - you can clean from anywhere
    p_success = 1 - glob.failProbs['Look']
    return geom_cost(cost, p_obj_state*p_success)
  if op.name == 'Paint':
    obj, = op.args[:1]
    p_obj_state = belief.objState[obj].prob(clean)
    p_success = 1 - glob.failProbs['Look']
    return geom_cost(cost, p_obj_state*p_success)
  if op.name == 'Dry':
    obj, = op.args[:1]
    p_obj_state = belief.objState[obj].prob(wet)
    p_success = 1 - glob.failProbs['Look']
    return geom_cost(cost, p_obj_state*p_success)
  raise NotImplementedError(op.name)
  #return geom_cost(cost, 1)

def belief_satisfies(belief, goal):
  #return all(state.fluentValue(fluent) == True for fluent in goal.fluents)
  return all(fluent.valueInDetails(belief) == True for fluent in goal.fluents)

from heapq import heappop, heappush
from copy import deepcopy

def bfs_policy(operators, initial_belief, goal, pre=False, unit=False):
  instances = get_operator_instances(operators, initial_belief)
  def fn(belief):
    queue = [(0, State([], belief), [])]
    while queue:
      plan_cost, current_state, history = heappop(queue)
      print '\nCost: %s | Length: %s'%(plan_cost, len(history))
      if belief_satisfies(current_state.details, goal):
        print history
        raw_input('Continue?')
        return history[0]
      for op in instances:
        if pre and not applicable_operator(current_state, op):
          continue
        observations = [None]
        if op.name in HELPFUL_OPERATOR_OBS:
          observations = HELPFUL_OPERATOR_OBS[op.name](op)
        else:
          raise NotImplementedError(op.name)
        for obs in observations:
          #print 'Op:', op, ' | Obs:', obs
          cost = get_operator_cost(current_state, op) if not unit else 1
          new_state = apply_operator_observation(current_state, op, obs)
          #new_state.details.draw()
          #env.reset(state.details) # This just passes
          #hCacheReset() # For the heuristic
          #writePrimRefinement(f, op)
          heappush(queue, (plan_cost+cost, new_state, history+[op]))
    return None
  return fn

##################################################

# TODO - MDP solver https://github.com/sawcordwell/pymdptoolbox

def random_policy(operators, belief): # TODO - weight the actions
  instances = get_operator_instances(operators, belief)
  return lambda _: choice(instances)

def random_safe_policy(start):
  raise NotImplementedError() # TODO - a random policy that chooses "safe" actions randomly

# TODO - POMCP - https://github.com/pemami4911/POMDPy

def open_loop_policy(plan):
  def fn(_):
    if plan is None or not plan:
      return None
    action = plan.pop(0)
    return action
  return fn

def localize_policy(obj):
  pass # TODO - make a localize action that runs the localize plan