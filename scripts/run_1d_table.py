#!/usr/bin/env python

from collections import namedtuple

from stripstream.algorithms.hierarchy.utils import preimage_sequence
from stripstream.algorithms.plan import get_states
from stripstream.algorithms.search.fast_downward import get_fast_downward
from stripstream.pddl.cond_streams import EasyGenStream as GeneratorStream, \
  EasyListFnStream as ListStream
from stripstream.pddl.logic.connectives import Not, And
from stripstream.pddl.logic.predicates import EasyPredicate as Pred
from stripstream.pddl.objects import EasyType as Type, EasyParameter as Param
from stripstream.pddl.operators import Action
from stripstream.pddl.problem import STRIPStreamProblem
from stripstream.pddl.utils import rename_easy
from stripstream.utils import SEPARATOR

Interval = namedtuple('Interval', ['low', 'up']) # inclusive, exclusive
Conf = namedtuple('Conf', ['x', 'y'])
Pose = namedtuple('Pose', ['ty', 'sur', 'x'])
Grasp = namedtuple('Grasp', ['ty', 'y'])
State = namedtuple('State', ['surfaces', 'poses', 'conf', 'holding'])
Goal = namedtuple('Goal', ['on', 'holding'])

class State(namedtuple('State', ['surfaces', 'poses', 'conf', 'holding'])):
  def __repr__(self):
    return self.__class__.__name__ + repr((self.poses, self.conf, self.holding))


def print_plan(plan):
  if plan is None:
    print plan
    return
  for i, (action, args) in enumerate(plan):
    print i, action, args

def get_problem0():
  tables = {
    'table1': Interval(10, 20),
    'table2': Interval(30, 40)}
  initial_poses = {
    'block1': Pose('block1', 'table1', 0),
    'block2': Pose('block2', 'table2', 0)}

  initial_config = Conf(0, 0)
  holding = None

  return State(tables, initial_poses, initial_config, holding)

def get_problem(unique=False, n=5):
  tables = {}
  initial_poses = {}
  for i in xrange(n):
    table = 'table{}'.format(i)
    block = 'block{}'.format(i)
    tables[table] = Interval(10 * i, 10 * (i + 1))
    ty = block if unique else 'block'
    initial_poses[block] = Pose(ty, table, 10 * i)

  initial_config = Conf(0, 0)
  holding = None
  initial = State(tables, initial_poses, initial_config, holding)
  #goal = Goal({}, 'block0')
  goal = Goal({
    'block0': 'table0',
    'block1': 'table2',
    'block2': 'table3'
  }, None)

  return initial, goal

# TODO: later can reincorporate types (instead of assuming all unique)

# Data types
CONF = Type()
BLOCK = Type()
SURFACE = Type()
POSE = Type()
GRASP = Type()

# Fluent predicates
AtConf = Pred(CONF)
AtPose = Pred(BLOCK, POSE)
HasGrasp = Pred(BLOCK, GRASP)
HandEmpty = Pred()

# Derived predicates
On = Pred(BLOCK, SURFACE)
Holding = Pred(BLOCK)
NearSurface = Pred(SURFACE) # Nearby
Safe = Pred(BLOCK, BLOCK, POSE)

# Static predicates
IsSupported = Pred(POSE, SURFACE)
IsPose = Pred(BLOCK, POSE)
IsGrasp = Pred(BLOCK, GRASP)
IsKin = Pred(POSE, GRASP, CONF)
#CFree = Pred(BLOCK, POSE, BLOCK, POSE)

# Free parameters
B, B2 = Param(BLOCK), Param(BLOCK)
P, P2 = Param(POSE), Param(POSE)
G = Param(GRASP)
Q, Q2 = Param(CONF), Param(CONF)
S, S2 = Param(SURFACE), Param(SURFACE)

# TODO: trajectories

rename_easy(locals()) # Trick to make debugging easier

def create_abstract_problem(state, goal):
  # Goal serialization is basically like this
  # Necessary and sufficient?

  tables, initial_poses, initial_config, holding = state

  # TODO: connect this to goals on the actual states
  # The quantification of poses and grasps is independent
  # Conditions are derived or atoms
  actions = [
    Action(name='pick', parameters=[B, S],
           condition=And(On(B, S), HandEmpty()),  #, AtConf(Q)),
           #condition=And(On(B, S), HandEmpty(), Not(Holding(B))),  # , AtConf(Q)),
           effect=And(Holding(B), Not(On(B, S)), Not(HandEmpty()))),
    Action(name='place', parameters=[B, S],
      condition=And(Holding(B)), #, AtConf(Q)),
      effect=And(On(B, S), HandEmpty(), Not(Holding(B)))),
    #Action(name='move', parameters=[S1, Q2],
    #  condition=AtConf(Q),
    #  effect=And(AtConf(Q2), Not(AtConf(Q)))),
  ]

  axioms = []
  cond_streams = []

  initial_atoms = [
    #AtConf(initial_config),
    HandEmpty()
  ] + [
    On(block, pose.sur) for block, pose in initial_poses.items()
  ]

  goal_literals = [On(b, s) for b, s in goal.on.items()]
  if goal.holding is not None:
    goal_literals.append(Holding(goal.holding))

  problem = STRIPStreamProblem(initial_atoms, goal_literals, actions + axioms, cond_streams)

  return problem


def create_real_problem(state, goal):
  tables, initial_poses, initial_config, holding = state

  # TODO: surface parameter here?
  # TODO: what happens if I drop preconditions here and things
  # TODO: add on here as well? Drop all preconditions mentioning continuous params
  # TODO: can then drop all preconditions and effects associated
  # Well I won't have these in the effects
  actions = [
    #Action(name='pick', parameters=[B, P, G, Q],
    #  condition=And(AtPose(B, P), HandEmpty(), AtConf(Q),
    #                IsPose(B, P), IsKin(P, G, Q)),
    #  effect=And(HasGrasp(B, G), Not(AtPose(B, P)), Not(HandEmpty()))),
    #Action(name='place', parameters=[B, P, G, Q],
    #  condition=And(HasGrasp(B, G), AtConf(Q),
    #                IsPose(B, P), IsKin(P, G, Q)),
    #    #ForAll([B2], Or(Equal(B, B2), Safe(B2, B, P)))),
    #  effect=And(AtPose(B, P), HandEmpty(), Not(HasGrasp(B, G)))),
    Action(name='pick', parameters=[B, S, P, G, Q],
           condition=And(AtPose(B, P), HandEmpty(), AtConf(Q),
                         IsPose(B, P), IsKin(P, G, Q), IsSupported(P, S)),
           effect=And(HasGrasp(B, G), Holding(B), Not(On(B, S)), Not(AtPose(B, P)), Not(HandEmpty()))),
    Action(name='place', parameters=[B, S, P, G, Q],
           condition=And(HasGrasp(B, G), AtConf(Q),
                         IsPose(B, P), IsKin(P, G, Q), IsSupported(P, S)),
           # ForAll([B2], Or(Equal(B, B2), Safe(B2, B, P)))),
           effect=And(AtPose(B, P), Not(Holding(B)), HandEmpty(), On(B, S), Not(HasGrasp(B, G)))),
    Action(name='move', parameters=[Q, Q2],
      condition=AtConf(Q),
      effect=And(AtConf(Q2), Not(AtConf(Q)))),
    # TODO: can do a similar thing with is near here
    # TODO: should also probably have two costs so one can be dropped
  ]
  # TODO: can relax preconditions as normal to make a strictly easier problem
  # If you remove all preconditions, can also remove effects that aren't needed and then parameters

  axioms = [
    #Axiom(effect=Safe(B2, B, P),
    #      condition=Exists([P2], And(AtPose(B2, P2), CFree(B, P, B2, P2)))),
    #Axiom(effect=On(B, S),
    #      condition=Exists([P], And(IsPose(B, P), AtPose(B, P), IsSupported(P, S)))),
    #Axiom(effect=Holding(B),
    #      condition=Exists([G], And(IsGrasp(B, G), HasGrasp(B, G)))),
    # TODO: one for the robot near a surface as well
  ]
  # TODO: axioms make it less clear what is going on for hierarchy

  ####################

  # Conditional stream declarations
  cond_streams = [
    GeneratorStream(inputs=[B, S], outputs=[P],
                    conditions=[],
                    effects=[IsPose(B, P), IsSupported(P, S)],
                    #generator=lambda b, s: ((Pose(b, randint(*tables[s])),) for _ in xrange(10))),
                    generator = lambda b, s: ((Pose(b, s, x),) for x in xrange(*tables[s]))),
    GeneratorStream(inputs=[B, P, G], outputs=[Q],
                    conditions=[IsPose(B, P), IsGrasp(B, G)],
                    effects=[IsKin(P, G, Q)],
                    generator=lambda _, p, g: [(Conf(p.x, g.y),)]),
    ListStream(inputs=[B], outputs=[G], conditions=[], effects=[IsGrasp(B, G)],
                    function=lambda b: [(Grasp(b, v),) for v in [-1, +1]]),
    #TestStream(inputs=[B, P, B2, P2], conditions=[], effects=[CFree(B, P, B2, P2)],
    #           test=lambda b1, p1, b2, p2: p1 != p2, eager=True), # Collision checking
  ]

  ####################

  # TODO: need to add surfaces as constants
  initial_atoms = [
    AtConf(initial_config),
  ] + [
    AtPose(block, pose) for block, pose in initial_poses.items()
  ] + [
    IsPose(block, pose) for block, pose in initial_poses.items()
  ] + [
    IsSupported(pose, pose.sur) for pose in initial_poses.values()
  ] + [
    On(block, pose.sur) for block, pose in initial_poses.items()
  ]
  if state.holding is None:
    initial_atoms.append(HandEmpty())
  else:
    block, grasp = state.holding
    initial_atoms += [HasGrasp(block, grasp), Holding(block)]

  goal_literals = [On(b, s) for b, s in goal.on.items()]
  if goal.holding is not None:
    goal_literals.append(Holding(goal.holding))

  return STRIPStreamProblem(initial_atoms, goal_literals, actions + axioms, cond_streams)

##################################################

def execute(initial, plan):
  surfaces, poses, conf, holding = initial
  for action, args in plan:
    if action.name == 'move':
      q, q2 = args
      conf = q2
    elif action.name == 'pick':
      b, s, p, g, q = args
      assert (b in poses) and (holding is None)
      del poses[b]
      holding = (b, g)
    elif action.name == 'place':
      b, s, p, g, q = args
      assert (b not in poses) and (holding == (b, g))
      poses[b] = p
      holding = None
  return State(surfaces, poses, conf, holding)

##################################################

# TODO: can avoid calling streams if nothing would be used (or solvable immediately)

def flat_replan(initial, goal):
  # Plan with operators with mixed levels of abstraction (dropped conditions)
  # The abstract operators aren't actually executable
  # Need to do something concrete before can do
  # Can be thought of as serialization of the front of the hierarchy from bottom up?
  # Unfortunately, it will likely do something suboptimal just to use more abstract operators
  # Well I need to be high quality to do anything anyways
  # The cost of the abstract operator should be what relative to the actual operator?
  # - if more, then would prefer to find concrete than to use abstract
  # - if less, then might do something silly just to be able to use abstract
  # Use derived predicates flexibly as fluents and
  # TODO: a key point of the hierarchy is to get rid of stream calls
  # This requires sequentially filling in the problem
  pass

# TODO: version of the focused algorithm that uses shared objects as a placeholder
# TODO: the goal serialization version subplan is slow because of the replan
# TODO: need to shorten the horizon in order to focus sampling and search
# TODO: goal serialization has to do with assumption that can always satisfy future goals
# Goal serialization identifies a sequence of goal states that in which all plans between all pairs of states
# Can both design samplers that only deal with the subgoal. Although the normal samplers are also good because shorter horizon
# Can keep around all facts that are achieved
# All the hierarchy things make an assumption or pragmatically an assumption to do this kind of stuff
# This assumption can be applied recursively and enables execution while planning

def hierarchy(initial, goal):
  abstract_problem = create_abstract_problem(initial, goal)
  abstract_plan, abstract_universe = incremental_planner(abstract_problem, search=get_fast_downward(
    config='ff-astar', remove=False, verbose=False))
  print_plan(convert_plan(abstract_plan))

  goal_sequence = preimage_sequence(abstract_universe, abstract_plan)
  #goal_sequence = [action.instantiate(args).condition for action, args in abstract_plan[1:]] + \
  #                [abstract_problem.goal_literals]
  # TODO: this doesn't really work because it doesn't accomplish the necessary subgoals
  #goal_sequence = list(And(*atoms) for atoms in get_states(abstract_universe, abstract_plan))[1:]

  # TODO: note that the subgoal actually has quite a few literals but most of them are satisfied
  # Each abstract operator reflects out confidence that each abstract operator represents the ability to go between conditions

  state = initial
  history = []
  for goal_formula in goal_sequence:
    print SEPARATOR
    problem = create_real_problem(state, goal)
    problem.goal_literals = goal_formula
    print state
    print problem.goal_literals
    plan, universe = incremental_planner(problem, search=get_fast_downward(
      config='ff-astar', remove=False, verbose=False))
    print_plan(convert_plan(plan))
    if plan is None:
      return None
    state = execute(state, convert_plan(plan)) # TODO: would be better if directly applied the actions
    history += plan
  return convert_plan(history)

def replan_hierarchy(initial, goal):
  pass

def replan(initial, goal):
  state = initial
  history = []
  while True:
    problem = create_real_problem(state, goal)
    search = get_fast_downward(config='ff-astar', remove=False, verbose=False)
    plan, _ = incremental_planner(problem, search=search)
    plan = convert_plan(plan)
    print_plan(plan)
    raw_input('awefwaef')
    if plan is None:
      return None
    if not plan:
      return convert_plan(history)
    plan_prefix = plan[:1]
    state = execute(state, plan_prefix)
    history += plan_prefix
    print SEPARATOR

def flat_plan(initial, goal):
  #problem = create_abstract_problem(initial, goal)
  problem = create_real_problem(initial, goal)
  print problem
  search = get_fast_downward(config='ff-astar', remove=False, verbose=False)
  plan, _ = incremental_planner(problem, search=search)
  return convert_plan(plan)

##################################################

from stripstream.algorithms.incremental.incremental_planner import incremental_planner
from stripstream.pddl.utils import convert_plan

def main():
  """
  Creates and solves the 1D task and motion planning STRIPStream problem.
  """

  initial, goal = get_problem()
  #plan = replan(initial, goal)
  #plan = flat_plan(initial, goal)
  plan = hierarchy(initial, goal)
  print SEPARATOR
  print_plan(plan)

if __name__ == '__main__':
  main()
