import sys

sys.path.append('/Users/caelan/Programs/LIS/git/MP')
sys.path.append('/Users/caelan/Programs/LIS/git/bhpn/non_robot_domains') # NOTE - for some reason need to add to the python path
sys.path.append('/Users/caelan/Programs/LIS/git/bhpn/hpn')

from toyTest import Belief, glob, BBhAddBackBSetNew, UniformDist, DD, State, Bd, ObjLoc, makeOperators, objDictCopy, World, hpn, ObjState
#from bhpn.hpn.fbch import *
from fbch import executePrim
#import hpn

# TODO - clean this up
# from stripstream.pddl.examples.belief.observable import run_observable
# from stripstream.pddl.examples.belief.observable_no_occup import run_observable
#
# from stripstream.pddl.examples.belief.observable_2 import run_observable_2, observable_policy
# from stripstream.pddl.examples.belief.observable_no_occup_2 import run_observable_2, observable_policy
#
# from stripstream.pddl.examples.belief.unknown import run_unknown, unknown_policy
# from stripstream.pddl.examples.belief.unknown_no_occup import run_unknown, unknown_policy
#
# from stripstream.pddl.examples.belief.factored import online_policy
# from stripstream.pddl.examples.belief.forward import forward_policy
from stripstream.pddl.examples.belief.forward_latent import forward_latent_policy

from stripstream.pddl.examples.belief.problems import *
from stripstream.pddl.examples.belief.assorted import bfs_policy, random_policy, open_loop_policy
from stripstream.pddl.examples.belief.utils import belief_satisfies, real_satisfies, convert_start, convert_goal
from stripstream.utils import SEPARATOR, INF

# TODO - maybe I should just go to the combined search? The problem is the heuristic...
# Dropping preconditions will keep the heuristic admissible
# I can avoid "side-effects" when doing relaxed planning
# Every fluent will be new in a width based planner
# Intentional effects and side effects...
# I could make two versions of an operator for the fast-downward planning, the real one and the relaxed one
# And I could consider a forward search where we move the state forward

# TODO - collisions
# - Model the uncertainty fog through space as an obstacle
# - If I just use the object poses, then could have low probability of collision because uncertainty if anything is there
# - One could try to "delocalize" the object in order to lower the probability of collision

# TODO - "discrete Gaussian" with a Bernoulli distribution over the current state or something else
# TODO - I could also do the version of this which uses a "fake" belief space representation
# Learning a heuristic for search for these simple problems
# We include the target object here because this includes the desired outcomes

def simulate(env, state, goal, policy, belief=True, debug=False):
  history = []
  while not (belief and belief_satisfies(state.details, goal)) and \
      not (not belief and real_satisfies(env, state, goal)):
    op = policy(state.details) # Right now the algorithm chooses when it is done
    print 'Step %s: %s'%(len(history), op)
    if debug: raw_input('Before execution. Continue?')
    if op is None:
      return state, history
    history.append(op)
    executePrim(op, state, env, None)
    if debug: raw_input('After execution. Continue?')
  return state, history

# TODO - could have observation whether you are in the goal state

##################################################

# TODO - run all of these at once

def run_belief():
  #random.seed(0)
  #numpy.random.seed(0)
  #hpn.fbch.flatPlan = True # TODO - do I need this?

  problem_fn = test0 # test0 | test1 | test2 | test3 | test8s
  print problem_fn
  env, start, goal = problem_fn()
  operators = makeOperators(glob.failProbs)
  print operators.values()

  #print env
  #print start.details
  print
  print convert_start(env, start)
  print convert_goal(goal)
  print goal

  #print start.copy()
  #print
  #print compile_belief(start.details, goal)

  #policy = bfs_policy(operators, start.details, goal)
  #policy = online_policy(operators, goal)
  #policy = forward_policy(operators, goal)
  policy = forward_latent_policy(operators, goal)
  #policy = unknown_policy(operators, goal)
  #policy = observable_policy(operators, goal)
  #policy = random_policy(operators, start.details)
  #policy = open_loop_policy([
  #  make_transport(operators, 'a', 'i2', 'i0'),
  #  make_look(operators, 'a', 'i0'),
  #  make_look(operators, 'a', 'i0'),
  #])

  belief = True
  state, history = simulate(env, start, goal, policy, belief=belief, debug=True)
  print SEPARATOR
  #print state.details
  print convert_start(env, state)
  print 'Belief:', belief
  print 'Env success:', real_satisfies(env, state, goal)
  print 'Belief success:', belief_satisfies(state.details, goal)
  print 'History:'
  for i, action in enumerate(history):
    print i, action
  #print 'Cost:', sum(action.cost for action in history) # NOTE - these are Leslie and Tomas's objects

def main():
  #run_observable()
  #run_observable_2()
  run_belief()

if __name__ == '__main__':
  main()
