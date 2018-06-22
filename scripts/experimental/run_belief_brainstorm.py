#!/usr/bin/env python

from stripstream.pddl.logic.predicates import EasyPredicate as Pred
from stripstream.pddl.objects import EasyType as Type, EasyParameter as Param
from stripstream.pddl.logic.connectives import Not, Or, And
from stripstream.pddl.logic.quantifiers import Exists, ForAll
from stripstream.pddl.logic.atoms import Equal
from stripstream.pddl.operators import Action, Axiom
from stripstream.pddl.cond_streams import EasyGenStream as GeneratorStream, EasyTestStream as TestStream
from stripstream.pddl.utils import rename_easy
from stripstream.pddl.problem import STRIPStreamProblem

# Preconditions (in general) represent locations when applying an action has infinite cost
# Thus, belief preconditions represent actions with too great of a cost from the belief state
# The goal state is an action to terminate and be in a certain state
# Optimal policy for MDP where only rewards are stochastic are representable
# Optimal policy for MDP where actions return to current state is representable
# Want to solve a POMDP represented as factored belief state MDP
# Can design a POMDP with high cost to something unexpected happening
# Can use the expected number of applications before advance trick r/p
# Can also use -log(p) sum trick
# In order to act, the parameters from the next action should be obtainable from the visible current state (no "imaginary" parameters)
# Move to look before we pick
# Can sample straight line base pose in order to get a better sample
# Time budget when running the planner
# What is the relation between the abstract objects and the online planning stuff
# Recall that I both had the action to generate one observation as well as many
# Observations update everything that has the same type?
# Could also specify policy in a reactive manner

# Optimality important in POMDP to keep making progress


# Start with finite number of tables
# Then extend to uncertainty in tables
# Could also just start with one object per type

# Just use filter cloud from Moveit
# Course occupancy grid for the world
# Fine grid based on more recent scan (for table manipulation)


def prior_from_goal(goal):
  p_table = [0.5, 0.5, 0.5] # Tables in descending likelihood of existing
  #p_red_block = [0.0]

  # Strong improvement in prior if you have observed something

  #p_table = stuff

  return goal

# Could have explicit belief or just have the uncertain belief as I did before
# Can look at multiple points on the table (and sample them) in order to view behind something
# In which case, I would want to have belief fluents to adjust the feasibilty of seeing anything
# Looking at the center of the table is just one choice for these


# Assume 1 room (but an other room), 1 table, and 1 block
# Hierarchical belief - belief becomes more specific
# P(T in R) = 0.9, P(T not in R) = 0.1
# Look

# Only uncertainty on state to start


# Tables are one square long
# One object per table
# Can only sense when on top of it
# Alternatively could be +/- 1 for poses

class WorldState(object):
  def __init__(self, robot_conf, table_poses, object_poses):
    self.robot_conf = robot_conf
    self.table_poses = table_poses
    self.object_poses = object_poses

class BeliefState(object):
  def __init__(self, robot_conf, visible_tables, visible_objects, known_tables, known_objects):
    self.robot_conf = robot_conf
    self.visible_tables = visible_tables
    self.visible_objects = visible_objects
    self.known_tables = known_tables
    self.known_objects = known_objects

def execute(world, action):
  pass


def create_problem():
  table = 'table'
  room = 'room'
  block = 'block'
  table_pose = None
  block_pose = None

  if table_pose is None:
    table_belief = frozenset({(room, 0.5), (None, 0.5)}) # Discrete distribution over poses
  else:
    table_belief = frozenset({(table_pose, 1.0)}) # Gaussian

  if block_pose is None:
    block_belief = frozenset({(table, 0.5), (None, 0.5)}) # Particle filter
  else:
    block_belief = frozenset({(table_pose, 1.0)}) # Gaussian

  # I definitely am implicitly using belief conditions by asserting we will know the resultant pose

  # Tables and Objects have three beliefs
  # 1) Unknown
  # 2) Coarse
  # 3) Fine (with respect to base pose). Or could just add LowVariance condition when true

  # Data types
  CONF = Type()
  ROOM = Type()
  TABLE = Type() # Difference between fixed and movable objects
  BLOCK = Type()
  POSE = Type()

  # Fluent predicates
  AtConf = Pred(CONF)
  HandEmpty = Pred()
  Holding = Pred(BLOCK)

  # Static predicates
  LegalKin = Pred(POSE, CONF)

  # Know that each block is at one pose at once (but don't know which one). Well
  # Tables can be at only one pose. Only need to have argument for whether localized
  UncertainT = Pred(TABLE)
  UncertainB = Pred(BLOCK) # Has an internal distribution in it
  AtPoseT = Pred(TABLE) # Has a fixed pose / convex hull in it
  AtPoseB = Pred(BLOCK, POSE)
  LocalizedT = Pred(TABLE)
  LocalizedB = Pred(BLOCK)
  #Scanned = Pred(ROOM)
  #IsReal = Pred(POSE) # Could also specify all the fake values upfront

  # Free parameters
  B1, B2 = Param(BLOCK), Param(BLOCK)
  P1, P2 = Param(POSE), Param(POSE)
  Q1, Q2 = Param(CONF), Param(CONF)

  R, T = Param(ROOM), Param(TABLE)

  rename_easy(locals()) # Trick to make debugging easier

  actions = [
    Action(name='pick', parameters=[B1, P1, Q1],
      condition=And(AtPoseB(B1, P1), LocalizedB(B1), HandEmpty(), AtConf(Q1), LegalKin(P1, Q1)),
      effect=And(Holding(B1), Not(AtPoseB(B1, P1)), Not(HandEmpty()), Not(LocalizedB(B1)))),

    Action(name='place', parameters=[B1, P1, Q1], # Localize table?
      condition=And(Holding(B1), AtConf(Q1), LegalKin(P1, Q1)),
      effect=And(AtPoseB(B1, P1), HandEmpty(), Not(Holding(B1)))),

    Action(name='move_base', parameters=[Q1, Q2],
      condition=AtConf(Q1),
      effect=And(AtConf(Q2), Not(AtConf(Q1)), ForAll([B1], Not(LocalizedB(B1))))), # Set all known poses to be high uncertainty

    #Action(name='scan', parameters=[R, T],
    #       condition=And(InRoom(R), AtConf(Q1)), # Should have a trajectory really
    # condition=And(Believe(T), Not(Scanned(R))), # Scan from anywhere in the room
    #       effect=And(T)),

    Action(name='move_head', parameters=[Q1, Q2], # Head conf, base conf, manip conf?
      condition=AtConf(Q1),
      effect=And(AtConf(Q2), Not(AtConf(Q1)))), # Should I undo localized if I move the head at all?

    Action(name='scan_room', parameters=[T],
           condition=UncertainT(T),
           effect=And(AtPoseT(T), Not(UncertainT(T)))),

    Action(name='scan_table', parameters=[T, B1, P1, Q1],
            condition=And(AtPoseT(T), AtConf(Q1)),
            effect=And(AtPoseB(B1, P1), Not(UncertainB(B1)))),

    Action(name='look_table', parameters=[T, Q1],
             condition=And(AtPoseT(T), AtConf(Q1)),
             effect=LocalizedT(T)),

    Action(name='look_block', parameters=[B1, P1, Q1],
           condition=And(AtPoseB(B1, P1), AtConf(Q1)), # Visibility constraint
           effect=LocalizedB(B1)),

    #Action(name='stop', parameters=[T, Q1],
    #       condition=And(AtPoseT(T), AtConf(Q1)),
    #       effect=LocalizedT(T)),
  ]

  axioms = [
    #Axiom(effect=InRoom(R),
    #      condition=Exists([Q1], And(AtConf(Q1), ConfIn(Q1, R)))), # Infers B2 is at a safe pose wrt B1 at P1
  ]

  # TODO: partially observable version of this

  def inverse_kinematics(pose): # TODO: list stream that uses ending info
    if type(pose) == str:
      yield (pose + '_conf',) # Represents a hypothetical
    yield (pose,)

  def sample_table(table):
    if not localized:
      yield # Stuff
    yield (pose,)

  streams = [
    GeneratorStream(inputs=[P1], outputs=[Q1], conditions=[], effects=[LegalKin(P1, Q1)],
                    generator=inverse_kinematics),
  ]

  constants = [
    POSE('pose'), # Strings denote fake values
  ]

  initial_atoms = [
    AtConf(1),
    HandEmpty(),
    UncertainT(table),
    UncertainB(block),
  ]

  goal_literals = [Holding(block)]

  problem = STRIPStreamProblem(initial_atoms, goal_literals, actions + axioms, streams, constants)

  return problem


def create_problem2():
  """
  Creates the 1D task and motion planning STRIPStream problem.

  :return: a :class:`.STRIPStreamProblem`
  """

  # How would I specify table position
  # From goal specification can derive prior
  # Everything of same object type should be one variable? Otherwise, how would I update?
  # I do actually have limits on the number of things
  # Doing with would relive the strangeness when you have to update the others
  # The strange thing is that we would like to distinguish the clusters in space when we do find them

  p_table = 0.9
  p_hit_exists = 0.99
  p_miss_notexists = p_hit_exists
  # Could use count based things or could just indicate confidence in sensor model

  # Goal, object in hand
  # Object starts out with high probability that its on a surface


  surfaces = ['table%i'%i for i in range(3)]

  # Different predicates for course belief and fine belief?
  # Do I want to expose blocks as objects to belief?


  # The probability that another table exists drops immensely once we find 3
  # I think I always have to fix this number
  # I suppose I could make a stream that generates new objects if desired
  # Decrease the likelihood of later numbered objects
  # Maybe I just use one table and allow it not to integrate to one?

  # Why does the online deferral to use objects in the focused algorithm work?
  # We often have streams for continuous values and these are the ones we want to defer
  # Could I do this for discrete objects as well?
  # Sure, just make a stream to generate them
  # This is all about hte optimistic, I think there is a pose but I don't actually know it stuff
  # Should the imaginary pose be explicit then?
  # Maybe I should find a true plan but allow some objects to be imaginary
  # Simultaneous actions to look and observe multiple things


  blocks = ['block%i'%i for i in range(3)]
  num_poses = pow(10, 10)

  initial_config = 0 # the initial robot configuration is 0
  initial_poses = {block: i for i, block in enumerate(blocks)} # the initial pose for block i is i

  goal_poses = {block: i+1 for i, block in enumerate(blocks)} # the goal pose for block i is i+1

  ####################

  # Data types
  CONF, BLOCK, POSE = Type(), Type(), Type()
  ROOM = Type()


  # Fluent predicates
  AtConf = Pred(CONF)
  AtPose = Pred(BLOCK, POSE)
  HandEmpty = Pred()
  Holding = Pred(BLOCK)

  # Derived predicates
  Safe = Pred(BLOCK, BLOCK, POSE)

  # Static predicates
  LegalKin = Pred(POSE, CONF)
  CollisionFree = Pred(BLOCK, POSE, BLOCK, POSE)

  # Free parameters
  B1, B2 = Param(BLOCK), Param(BLOCK)
  P1, P2 = Param(POSE), Param(POSE)
  Q1, Q2 = Param(CONF), Param(CONF)

  rename_easy(locals()) # Trick to make debugging easier

  ####################

  actions = [
    Action(name='pick', parameters=[B1, P1, Q1],
      condition=And(AtPose(B1, P1), HandEmpty(), AtConf(Q1), LegalKin(P1, Q1)),
      effect=And(Holding(B1), Not(AtPose(B1, P1)), Not(HandEmpty()))),

    Action(name='place', parameters=[B1, P1, Q1],
      condition=And(Holding(B1), AtConf(Q1), LegalKin(P1, Q1),
        ForAll([B2], Or(Equal(B1, B2), Safe(B2, B1, P1)))), # TODO - convert to finite blocks case?
      effect=And(AtPose(B1, P1), HandEmpty(), Not(Holding(B1)))),

    Action(name='move', parameters=[Q1, Q2],
      condition=AtConf(Q1),
      effect=And(AtConf(Q2), Not(AtConf(Q1)))),

    Action(name='scan', parameters=[Q1], # Looks at a particular object. Discount costs for subsequent looks from that spot
           condition=AtConf(Q1),
           effect=And()),

    Action(name='look', parameters=[Q1, O], # Look at surface vs object
            condition=AtConf(Q1),
            effect=And()),
  ]

  axioms = [
    Axiom(effect=Safe(B2, B1, P1),
          condition=Exists([P2], And(AtPose(B2, P2), CollisionFree(B1, P1, B2, P2)))), # Infers B2 is at a safe pose wrt B1 at P1
  ]

  ####################

  # Conditional stream declarations
  cond_streams = [
    GeneratorStream(inputs=[], outputs=[P1], conditions=[], effects=[],
                    generator=lambda: ((p,) for p in xrange(num_poses))), # Enumerating all the poses

    GeneratorStream(inputs=[P1], outputs=[Q1], conditions=[], effects=[LegalKin(P1, Q1)],
                    generator=lambda p: [(p,)]), # Inverse kinematics

    TestStream(inputs=[B1, P1, B2, P2], conditions=[], effects=[CollisionFree(B1, P1, B2, P2)],
               test=lambda b1, p1, b2, p2: p1 != p2, eager=True), # Collision checking
  ]

  ####################

  constants = [
    CONF(initial_config) # Any additional objects
  ]

  initial_atoms = [
    AtConf(initial_config),
    HandEmpty()
  ] + [
    AtPose(block, pose) for block, pose in initial_poses.iteritems()
  ]

  goal_literals = [AtPose(block, pose) for block, pose in goal_poses.iteritems()]

  problem = STRIPStreamProblem(initial_atoms, goal_literals, actions + axioms, cond_streams, constants)

  return problem

def compile_observable_problem(world, task):
  # Data types
  CONF = Type()
  TABLE = Type()  # Difference between fixed and movable objects
  BLOCK = Type()
  POSE = Type()

  # Fluent predicates
  AtConf = Pred(CONF)
  HandEmpty = Pred()
  Holding = Pred(BLOCK)
  AtPose = Pred(BLOCK, POSE)

  # Static predicates
  LegalKin = Pred(POSE, CONF)

  # Free parameters
  Q1, Q2 = Param(CONF), Param(CONF)
  T = Param(TABLE)
  B1, B2 = Param(BLOCK), Param(BLOCK)
  P1, P2 = Param(POSE), Param(POSE)

  rename_easy(locals())  # Trick to make debugging easier

  actions = [
    Action(name='pick', parameters=[B1, P1, Q1],
           condition=And(AtPose(B1, P1),  HandEmpty(), AtConf(Q1), LegalKin(P1, Q1)),
           effect=And(Holding(B1), Not(AtPose(B1, P1)), Not(HandEmpty()))),

    # Action(name='place', parameters=[B1, P1, Q1], # Localize table?
    #  condition=And(Holding(B1), AtConf(Q1), LegalKin(P1, Q1)),
    #  effect=And(AtPoseB(B1, P1), HandEmpty(), Not(Holding(B1)))),

    Action(name='move', parameters=[Q1, Q2],
           condition=AtConf(Q1),
           effect=And(AtConf(Q2), Not(AtConf(Q1)))),
  ]

  axioms = [
    # Axiom(effect=InRoom(R),
    #      condition=Exists([Q1], And(AtConf(Q1), ConfIn(Q1, R)))), # Infers B2 is at a safe pose wrt B1 at P1
  ]

  def inverse_kinematics(pose): # TODO: list stream that uses ending info
    yield (pose,)

  #def sample_table(table):
  #  yield (pose,)

  streams = [
    GeneratorStream(inputs=[P1], outputs=[Q1], conditions=[], effects=[LegalKin(P1, Q1)],
                    generator=inverse_kinematics),
  ]

  initial_atoms = [AtConf(world.robot_conf)]
  if world.holding is None:
    initial_atoms.append(HandEmpty())
  else:
    initial_atoms.append(Holding(world.holding))
  for obj, pose in world.object_poses:
    initial_atoms.append(AtPose(obj, pose))

  goal_literals = []
  if task.robot_conf is not False:
    goal_literals.append(AtConf(task.robot_conf))
  if task.holding is None:
    goal_literals.append(HandEmpty())
  elif task.holding:
    goal_literals.append(Holding(task.holding))
  #for obj, pose in task.object_poses.iteritems():
  #  goal_literals.append(AtPoseB(obj, pose))
  goal_formula = And(*goal_literals)

  problem = STRIPStreamProblem(initial_atoms, goal_formula, actions + axioms, streams, [])

  return problem

##################################################

from stripstream.algorithms.incremental.incremental_planner import incremental_planner
from stripstream.pddl.utils import convert_plan

def main():
  """
  Creates and solves the 1D task and motion planning STRIPStream problem.
  """

  problem = create_problem()
  print problem
  plan, _ = incremental_planner(problem)
  print
  print 'Plan:', convert_plan(plan)

# TODO - visualize by applying actions to env state

if __name__ == '__main__':
  main()
