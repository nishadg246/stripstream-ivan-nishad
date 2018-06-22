#!/usr/bin/env python

from stripstream.pddl.logic.predicates import EasyPredicate as Pred, EasyFunction as Func
from stripstream.pddl.objects import EasyType as Type, EasyParameter as Param, OBJECT
from stripstream.pddl.logic.connectives import Not, Or, And
from stripstream.pddl.logic.quantifiers import Exists, ForAll
from stripstream.pddl.operators import Action, Axiom
from stripstream.pddl.cond_streams import EasyGenStream as GeneratorStream, EasyTestStream as TestStream
from stripstream.pddl.utils import rename_easy
from stripstream.pddl.problem import STRIPStreamProblem
from stripstream.pddl.logic.operations import Cost
from stripstream.pddl.cond_streams import EasyCostStream as CostStream

from stripstream.algorithms.incremental.incremental_planner import incremental_planner
from stripstream.pddl.utils import convert_plan, get_value
from stripstream.algorithms.utils import get_fast_downward
from stripstream.algorithms.plan import plan_cost, plan_length
from stripstream.pddl.cond_streams import expected_cost, mdp_cost

from copy import copy
from collections import defaultdict

import math
import numpy as np

class WorldState(object):
  def __init__(self, robot_conf, holding, table_poses, object_poses):
    self.robot_conf = robot_conf
    self.holding = holding
    self.table_poses = table_poses
    self.object_poses = object_poses
  def __repr__(self):
    return self.__class__.__name__ + repr((self.robot_conf, self.holding, self.table_poses, self.object_poses))

class Task(object):
  def __init__(self, holding=False, object_surfaces=[]):
    #self.robot_conf = robot_conf
    self.holding = holding
    self.object_surfaces = object_surfaces
  def __repr__(self):
    return self.__class__.__name__ + repr((self.holding, self.object_surfaces))

class Observation(object):
  def __init__(self, table_poses=None, object_poses=None):
    self.table_poses = table_poses
    self.object_poses = object_poses
  def __repr__(self):
    return self.__class__.__name__ + repr((self.table_poses, self.object_poses))

#####

def log_odds_from_prob(p):
  return math.log(p/(1 - p))

def prob_from_log_odds(l):
  return 1. / (1. + math.exp(-l))

class OctoMap(object):
  def __init__(self, prior, p_occ_hit, p_occ_miss, min_p, max_p, p_threshold): # Inverse sensor model
    self.log_prior = log_odds_from_prob(prior)
    self.log_hit = log_odds_from_prob(p_occ_hit)
    self.log_miss = log_odds_from_prob(p_occ_miss)
    self.log_min_p = log_odds_from_prob(min_p)
    self.log_max_p = log_odds_from_prob(max_p)
    #print prob_from_log_odds(self.log_min_p), prob_from_log_odds(self.log_max_p)
    self.p_threshold = p_threshold
    self.log_odds = {}

  def observe(self, cell, hit):
    self.log_odds[cell] = self.log_odds.get(cell, self.log_prior) + (self.log_hit if hit else self.log_miss)
    self.log_odds[cell] = max(self.log_min_p, min(self.log_odds[cell], self.log_max_p))

  def set_prob(self, cell, p):
    self.log_odds[cell] = log_odds_from_prob(p)

  def set_prob_min(self, cell):
    self.log_odds[cell] = self.log_min_p

  def set_prob_max(self, cell):
    self.log_odds[cell] = self.log_max_p

  def get_prob(self, cell):
    return prob_from_log_odds(self.log_odds.get(cell, self.log_prior))

  def is_occupied(self, cell):
    return self.p_threshold <= self.get_prob(cell)

  def get_occupied(self):
    return filter(self.is_occupied, self.log_odds)

  def __repr__(self):
    return '{}({}, [{}])'.format(self.__class__.__name__, prob_from_log_odds(self.log_prior),
           ', '.join('{}: {:.3f}'.format(c, self.get_prob(c)) for c in sorted(self.log_odds)))

class Estimator(object):
  def __init__(self, task, surfaces, objects, poses, robot_conf, holding):
    self.task = task
    self.poses = poses
    self.robot_conf = robot_conf
    self.holding = holding
    #self.confident = False
    self.surface_octomaps = {}
    # TODO: priors on the number of objects in the room
    expected_surfaces = 1.0
    surface_prior = expected_surfaces / len(poses)
    for s in surfaces:
      #self.surface_octomaps[s] = OctoMap(0.5, 0.7, 0.4, 0.12, 0.97, 0.7)
      self.surface_octomaps[s] = OctoMap(surface_prior, 0.9, 0.1, 0.01, 0.99, 0.5)
    self.object_octomaps = {}

    #self.object_prior = {'block': {'table': 0.9, 'shelf': 0.1}}
    self.object_prior = {}
    for o in objects:
      #self.object_octomaps[o] = OctoMap(0.5, 0.7, 0.4, 0.12, 0.97, 0.7)
      self.object_octomaps[o] = OctoMap(0.5, 0.9, 0.1, 0.001, 0.99, 0.7)
    self.command_history = []
    self.obs_history = []
  def update(self, command, obs):
    self.command_history.append(command)
    self.obs_history.append(obs)
    action, args = command
    if action == simulate_move:
      self.robot_conf, = args
      assert type(self.robot_conf) != str
      #self.confident = False
    elif action == simulate_scan:
      for surface, octomap in self.surface_octomaps.iteritems():
        hits = {p for s, p in obs.table_poses if s == surface}
        for pose in self.poses:
          octomap.observe(pose, pose in hits)
    elif action == simulate_look:
      for obj, octomap in self.object_octomaps.iteritems():
        hits = {p for o, p in obs.object_poses if o == obj}
        x, _ = self.robot_conf
        octomap.observe(x, (x in hits))
      #if action in ('look_table', 'look_block'): # TODO: inspect?
      #  self.confident = True
    elif action == simulate_pick:
      self.holding, = args
      x, _ = self.robot_conf
      self.object_octomaps[self.holding].set_prob_min(x)
    elif action == simulate_place:
      x, _ = self.robot_conf
      self.object_octomaps[self.holding].set_prob_max(x)
      self.holding = None
    else:
      raise ValueError(action)

    # TODO: on each state estimate the split
    # TODO: have some probability of a scan producing a new generic surface to place things (just in case)

  def __repr__(self):
    s = '{}({}, {})'.format(self.__class__.__name__, self.robot_conf, self.holding)
    for surface in sorted(self.surface_octomaps.keys()):
      s += '\n{}: {}'.format(surface, self.surface_octomaps[surface])
    for obj in sorted(self.object_octomaps.keys()):
      s += '\n{}: {}'.format(obj, self.object_octomaps[obj])
    return s

#####

# Automatic observation (or deliberate)?

def simulate_move(world, conf):
  next_world = copy(world)
  #if random.random() < 0.9: # Doesn't really make sense to have moving error quite like this
  next_world.robot_conf = conf
  observation = None
  return next_world, observation

def simulate_scan(world):
  #if random.random() < 0.9: # TODO: independently fail on each
  observation = Observation(table_poses=world.table_poses[:])
  return copy(world), observation

def simulate_look(world):
  x, y = world.robot_conf
  adjacent_surfaces = filter(lambda (_, p): p == x, world.table_poses)
  adjacent_objects = filter(lambda (_, p): p == x, world.object_poses)
  observation = Observation(table_poses=adjacent_surfaces, object_poses=adjacent_objects)
  return copy(world), observation

#def look_block(world):
#  adjacent_objects = filter(lambda (_, p): p == world.robot_conf, world.object_poses)
#  observation = Observation(object_poses=adjacent_objects)
#  return copy(world), observation

# TODO: multiple objects in same region?
def simulate_pick(world, obj):
  x, y = world.robot_conf
  next_world = copy(world)
  if world.holding is None:
    for i, (o, p) in enumerate(world.object_poses):
      if (p == x) and (o == obj):
        next_world.holding = o
        next_world.object_poses.pop(i)
        break
    else:
      raise ValueError()
  else:
    # TODO: drop the current object?
    raise NotImplementedError()
  return next_world, Observation()

def simulate_place(world):
  x, y = world.robot_conf
  next_world = copy(world)
  if world.holding is not None:
    next_world.holding = None
    next_world.object_poses.append((world.holding, x))
  return next_world, Observation()

N_POSES = 10

#####

# TODO: to start just consider poses [0, N_POSES) with None as not in room
# TODO: what if just know there is a surface in the room?

# TODO: version of this using a 2D grid where the robot must stand adjacent to the table

def entities_from_task(task):
  task_surfaces = defaultdict(int)
  task_objects = defaultdict(int)
  if task.holding not in (None, False):
    task_objects[task.holding] += 1
  for obj, surface in task.object_surfaces:
    task_objects[obj] += 1  # We have prior that cannot be in two places at once
    task_surfaces[surface] = 1  # Doesn't prevent use in other goals
  #print dict(task_objects), dict(task_surfaces)
  return task_objects, task_surfaces

def compile_problem(estimator, task):
  # Data types
  CONF = Type()
  SURFACE = Type()  # Difference between fixed and movable objects
  ITEM = Type()
  POSE = Type()
  CLASS = Type()

  # Fluent predicates
  AtConf = Pred(CONF)
  HandEmpty = Pred()
  Holding = Pred(ITEM)
  AtPose = Pred(ITEM, POSE)
  Supported = Pred(POSE, SURFACE) # Fluent
  Localized = Pred(OBJECT)
  Measured = Pred(OBJECT)

  # Static predicates
  IsKin = Pred(POSE, CONF)
  IsClass = Pred(OBJECT, CLASS)
  IsVisible = Pred(SURFACE, CONF)
  IsSupported = Pred(POSE, SURFACE) # Static

  # Functions
  ScanRoom = Func(SURFACE)
  #ScanTable = Func(SURFACE, TYPE)
  ScanTable = Func(SURFACE, ITEM) # TODO: could include more specific vantage point costs
  Distance = Func(CONF, CONF)

  # Derived
  On = Pred(ITEM, SURFACE)
  ComputableP = Pred(POSE)
  ComputableQ = Pred(CONF)

  # Free parameters
  Q1, Q2 = Param(CONF), Param(CONF)
  S1 = Param(SURFACE)
  B1, B2 = Param(ITEM), Param(ITEM)
  P1, P2 = Param(POSE), Param(POSE)

  rename_easy(locals())  # Trick to make debugging easier

  # TODO: could just do easier version of this that doesn't require localized to start

  actions = [
    Action(name='pick', parameters=[B1, P1, Q1], # TODO: Visibility constraint
           condition=And(Localized(B1), AtPose(B1, P1), HandEmpty(), AtConf(Q1), IsKin(P1, Q1)),
           effect=And(Holding(B1), Not(AtPose(B1, P1)), Not(HandEmpty()))),

    Action(name='place', parameters=[B1, P1, Q1],
      condition=And(Holding(B1), AtConf(Q1), IsKin(P1, Q1)),
      effect=And(AtPose(B1, P1), HandEmpty(), Not(Holding(B1)))),

    Action(name='move', parameters=[Q1, Q2],
           condition=And(AtConf(Q1), ComputableQ(Q2)),
           effect=And(AtConf(Q2), Not(AtConf(Q1)), Cost(Distance(Q1, Q2)))),

    Action(name='scan_room', parameters=[S1],
           condition=Not(Localized(S1)),
           effect=And(Localized(S1), Cost(ScanRoom(S1)))),
    # TODO: need to set later poses to be usable or not to constrain order

    Action(name='scan_table', parameters=[S1, B1, P1, Q1],
           condition=And(Localized(S1), AtConf(Q1), IsVisible(S1, Q1), Not(Localized(B1))),
           effect = And(Localized(B1), Measured(P1), Supported(P1, S1), Cost(ScanTable(S1, B1)))),
  ]

  axioms = [
    # TODO: axiom for on? Might need a stream that generates initial fluents for On
    # TODO: axiom that says that all fake values depending on a certain one now are usable
    # TODO: could use stream predicates as fluents (as long as it doesn't break anything...)
    Axiom(effect=On(B1, S1),
          condition = Exists([P1], And(AtPose(B1, P1), Or(
            IsSupported(P1, S1), Supported(P1, S1))))),

    # TODO: compile automatically
    Axiom(effect=ComputableQ(Q1),
          condition=Or(Measured(Q1),
            Exists([P1], And(IsKin(P1, Q1), ComputableP(P1))),
            Exists([S1], And(IsVisible(S1, Q1), Localized(S1))))),
    Axiom(effect=ComputableP(P1),
          condition=Or(Measured(P1),
            Exists([S1], And(IsSupported(P1, S1), Localized(S1))))),
  ]

  #####

  surface_types = estimator.surface_octomaps.keys()
  item_types = estimator.object_octomaps.keys()
  names_from_type = defaultdict(list)
  known_poses = {}
  holding = None

  def add_type(cl):
    name = '{}{}'.format(cl, len(names_from_type[cl]))
    names_from_type[cl].append(name)
    return name

  # TODO: this is all very similar to the generic open world stuff
  if estimator.holding is not None:
    holding = add_type(estimator.holding)
  for cl, octomap in estimator.surface_octomaps.items(): # TODO: generic surface object
    for pose in octomap.get_occupied():
      known_poses[add_type(cl)] = pose
  for cl, octomap in estimator.object_octomaps.items():
    for pose in octomap.get_occupied():
      known_poses[add_type(cl)] = pose
  print dict(names_from_type), known_poses

  # Human tells you to move block -> at least one block
  # At least one block -> at least one surface
  # TODO: generate fake properties about these fake values?
  goal_objects, goal_surfaces = entities_from_task(task)
  for cl in surface_types:
    add_type(cl)
  #for cl in goal_surfaces:
  #  for i in xrange(len(names_from_type[cl]), goal_surfaces[cl]):
  #    add_type(cl)
  #for cl in item_types:
  for cl in goal_objects:
    for i in xrange(len(names_from_type[cl]), goal_objects[cl]):
      add_type(cl)

  #####

  initial_atoms = [
    AtConf(estimator.robot_conf),
    Measured(CONF(estimator.robot_conf))]
  if holding is None:
    initial_atoms.append(HandEmpty())
  else:
    initial_atoms.append(Holding(holding))

  class_from_name = {name: ty for ty in names_from_type for name in names_from_type[ty]}
  for name, ty in class_from_name.iteritems():
    ENTITY = SURFACE if ty in surface_types else ITEM
    initial_atoms.append(IsClass(ENTITY(name), ty))
    if name in known_poses:
      initial_atoms.append(Localized(ENTITY(name)))
      if ENTITY == ITEM:
        pose = known_poses[name]
        initial_atoms += [AtPose(name, pose), Measured(POSE(pose))]
    else:
      if ENTITY == ITEM:
        pose = 'p_init_{}'.format(name)  # The object should always be at this pose (we just can't do anything about it yet)
        initial_atoms += [AtPose(name, pose)]

  goal_literals = []
  if task.holding is None:
    goal_literals.append(HandEmpty())
  elif task.holding is not False:
    goal_literals.append(Exists([B1], And(Holding(B1), IsClass(B1, task.holding))))
  for obj, surface in task.object_surfaces:
    goal_literals.append(Exists([B1, S1], And(On(B1, S1), IsClass(B1, obj), IsClass(S1, surface))))
  goal_formula = And(*goal_literals)

  ####################

  TOLERANCE = 0.1

  def is_visible(table, conf):
    x, y = conf
    pose = known_poses[table]
    #return (pose == x) and (y == 2)
    #return (pose == x) and (y == 2)
    return (pose == x) and (abs(y - 2) < TOLERANCE)

  def is_kinematic(pose, conf):
    x, y = conf
    #return (pose == x) and (y == 1)
    return (pose == x) and (abs(y - 1) < TOLERANCE)

  ####################

  def sample_visible(table):  # TODO: could generically do this with poses
    if table in known_poses:
      y = 2
      #y += round(uniform(-TOLERANCE, TOLERANCE), 3)
      conf = (known_poses[table], y)
      assert is_visible(table, conf)
    else:
      conf = 'q_vis_{}'.format(table)
    yield (conf,)

  def inverse_kinematics(pose):  # TODO: list stream that uses ending info
    # TODO: only do if localized as well?
    # TODO: is it helpful to have this even if the raw value is kind of wrong (to steer the search)
    if type(pose) != str:
      y = 1
      #y += round(uniform(-TOLERANCE, TOLERANCE), 3)
      conf = (pose, y)
      assert is_kinematic(pose, conf)
    else:
      conf = 'q_ik_{}'.format(pose)
    yield (conf,)

  def sample_table(table):
    if table in known_poses:
      pose = known_poses[table]
    else:
      pose = 'p_{}'.format(table)
    yield (pose,)

  ####################

  MAX_DISTANCE = 10

  # TODO: maybe I don't need to worry about normalizing. I can just pretend non-parametric again for planning
  def scan_surface_cost(surface, obj): # TODO: what about multiple scans of the belief?
    fail_cost = 100
    surface_cl = class_from_name[surface]
    obj_cl = class_from_name[obj]
    prob = 1.0
    if obj_cl in estimator.object_prior:
      prob *= estimator.object_prior[obj_cl].get(surface_cl, 0)
    if surface in known_poses:
      prob *= estimator.object_octomaps[obj_cl].get_prob(known_poses[surface])
    else:
      prob *= 0.1 # Low chance if you don't even know the table exists
      # TODO: could even include the probability the table exists
    #return expected_cost(1, fail_cost, prob)
    return mdp_cost(1, fail_cost, prob)

  def scan_room_cost(surface):
    # TODO: try to prove some sort of bound on the cost to recover will suffice?
    fail_cost = 100
    cl = class_from_name[surface]
    occupied_poses = {known_poses[n] for n in names_from_type[cl] if n in known_poses}
    p_failure = 1.0
    for pose in estimator.poses:
      if pose not in occupied_poses:
        p_failure *= (1 - estimator.surface_octomaps[cl].get_prob(pose))
    return 1*(1-p_failure) + fail_cost*p_failure

  def distance_cost(q1, q2):
    if str in (type(q1), type(q2)):
      return MAX_DISTANCE # TODO: take the max possible pose distance
    # TODO: can use the info encoded within these to obtain better bounds
    return np.linalg.norm(np.array(q2) - np.array(q1))

  ####################

  # TODO: could add measured as the output to these
  streams = [
    GeneratorStream(inputs=[P1], outputs=[Q1], conditions=[], effects=[IsKin(P1, Q1)],
                    generator=inverse_kinematics),

    GeneratorStream(inputs=[S1], outputs=[Q1], conditions=[], effects=[IsVisible(S1, Q1)],
                    generator=sample_visible),

    GeneratorStream(inputs=[S1], outputs=[P1], conditions=[], effects=[IsSupported(P1, S1)],
                    generator=sample_table),

    CostStream(inputs=[S1, B1], conditions=[], effects=[ScanTable(S1, B1)],
               function=scan_surface_cost),
    CostStream(inputs=[Q1, Q2], conditions=[], effects=[Distance(Q1, Q2), Distance(Q2, Q1)],
               function=distance_cost),
    CostStream(inputs=[S1], conditions=[], effects=[ScanRoom(S1)],
               function=scan_room_cost),

    # TODO: make an is original precondition and only apply these to original values?
    # I suppose I could apply to all concrete things but that's likely not useful
    #TestStream(inputs=[S1, Q1], conditions=[IsOriginal(Q1)], effects=[IsVisible(S1, Q1)],
    #           test=is_visible, eager=True),
    #TestStream(inputs=[P1, Q1], conditions=[IsOriginal(Q1), IsOriginal(Q1)], effects=[IsKin(P1, Q1)],
    #           test=is_kinematic, eager=True),

    #GeneratorStream(inputs=[P1], outputs=[Q1], conditions=[], effects=[IsVisible(P1, Q1)],
    #                generator=sample_visible),
  ]

  problem = STRIPStreamProblem(initial_atoms, goal_formula, actions + axioms, streams, [])

  def command_from_action((action, args)):
    if action.name == 'scan_room':
      return simulate_scan, []
    if action.name in ('scan_table', 'look_block'):
      return simulate_look, []
    if action.name == 'move':
      q1, q2 = map(get_value, args)
      return simulate_move, [q2]
    if action.name == 'pick':
      o, p, q = map(get_value, args)
      return simulate_pick, [class_from_name[o]]
    if action.name == 'place':
      return simulate_place, []
    raise ValueError(action.name)

  return problem, command_from_action

def policy_from_task(task):
  def policy(estimator):
    problem, command_from_action = compile_problem(estimator, task)
    print problem
    plan, universe = incremental_planner(problem, search=get_fast_downward('astar'),
                                  frequency=100, waves=False, optimal=True, debug=False) # TODO: need to make sure I do all costs (they are always eager)
    print 'Plan: {}\nLength: {} | Cost: {}'.format(convert_plan(plan), plan_length(universe, plan), plan_cost(universe, plan))
    if plan is None:
      raise RuntimeError('Unable to find a plan')
    if not plan:
      return None
    return command_from_action(plan[0])
  return policy

##################################################

def get_pick_task1():
  pose = 1
  world = WorldState(robot_conf=(0, 3),
                     holding=None,
                    table_poses=[('table', pose)],
                    object_poses=[('block', pose)])
  task = Task(holding='block')
  return world, task

def get_pick_task2():
  pose = 5
  world = WorldState(robot_conf=(0, 3),
                     holding=None,
                    table_poses=[('table', pose), ('table', 1), ('table', 9)],
                    object_poses=[('block', pose)])
  task = Task(holding='block')
  return world, task

def get_pick_task3():
  world = WorldState(robot_conf=(0, 3),
                     holding=None,
                     table_poses=[('shelf', 1), ('table', 5)],
                     object_poses=[('block', 5)])
  task = Task(holding='block')
  return world, task

def get_place_task():
  world = WorldState(robot_conf=(0, 3),
                     holding=None,
                     table_poses=[('shelf', 1), ('table', 5)],
                     object_poses=[('block', 5)])
  task = Task(object_surfaces=[('block', 'shelf')])
  return world, task

##################################################

def main():
  #task_fn = get_pick_task1
  #task_fn = get_pick_task2
  #task_fn = get_pick_task3
  task_fn = get_place_task
  debug = False

  world, task = task_fn()
  surfaces = {s for s, _ in world.table_poses} # The robot knows what possible objects could be invovled
  objects = {o for o, _ in world.object_poses}
  # TODO: generic surface type (that doesn't actually exist)

  print task
  estimator = Estimator(task=task, surfaces=surfaces, objects=objects, poses=range(10),
                        robot_conf=world.robot_conf, holding=world.holding)
  policy = policy_from_task(task)
  action_history = []
  obs_history = []
  while True:
    print
    print world
    print estimator
    command = policy(estimator)
    if command is None:
      break
    action_history.append(command)
    action, args = command
    world, observation = action(world, *args)
    obs_history.append(obs_history)
    estimator.update(command, observation)
    if debug:
      raw_input('Continue?')
  print 'Done!'

if __name__ == '__main__':
  main()
