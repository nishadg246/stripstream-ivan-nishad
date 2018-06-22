from math import sqrt

from stripstream.pddl.logic.connectives import Not
from stripstream.pddl.logic.predicates import Predicate
from stripstream.pddl.objects import Parameter, Constant, Type, HashableObject
from stripstream.pddl.operators import STRIPSAction, STRIPSAxiom
from stripstream.pddl.streams import GeneratorStream, TestStream, FunctionStream
from stripstream.pddl.cond_streams import CondStream, TestCondStream
from stripstream.pddl.problem import STRIPStreamProblem

P = Parameter
C = Constant

POSITION = Type('pos')
#VELOCITY = Type('vel')
STATE = Type('state')
ACCELERATION = Type('accel')
FORCE = Type('force')
MASS = Type('mass')
TIME = Type('time')
SATELLITE = Type('satellite')
#ROCKET = Type('rocket')

G = -9.8
# TODO - mass of the packages affects things

######

#AtState = Predicate('at_state', [POSITION, VELOCITY])
AtState = Predicate('at_state', [STATE])
Above = Predicate('above', [POSITION])

AtOrbit = Predicate('at_orbit', [SATELLITE, POSITION])
Carrying = Predicate('carrying', [SATELLITE])

Flying = Predicate('flying', [])
Landed = Predicate('landed', [])

######

#IsBurst = Predicate('is_burst', [POSITION, VELOCITY, ACCELERATION, TIME, POSITION, VELOCITY]) # Alternatively constant of zero
IsBurst = Predicate('is_burst', [STATE, ACCELERATION, TIME, STATE])
IsGlide = Predicate('is_glide', [STATE, TIME, STATE])

#IsFloat = Predicate('at_state', [POSITION, VELOCITY, TIME, POSITION, VELOCITY])
IsCrashed = Predicate('is_crashed', [POSITION])
#IsAbove = Predicate('is_above', [POSITION, POSITION])
IsAbove = Predicate('is_above', [STATE, POSITION])

ArePair = Predicate('are_pair', [STATE, POSITION])

##################################################

class Position(HashableObject): type = POSITION; prefix = 'p'
#class Velocity(HashableObject): type = VELOCITY; prefix = 'v'
class State(HashableObject): type = STATE; prefix = 'x'
class Acceleration(HashableObject): type = ACCELERATION; prefix = 'a'
class Time(HashableObject): type = TIME; prefix = 't'

##################################################

def in_range(x, (l, u)):
  return l < x < u

# NOTE - a crash is defined as an intersection with the ground at any time
def crashes(p0, v0, a, t): # TODO - later make its own predicate
  if a == 0:
    if v0 == 0:
      return p0 == 0
    t0 = -p0/v0
    return in_range(t0, [0, t])
  det = v0**2 - 4*a*p0
  if det < 0:
    return False
  t0 = (-v0 + sqrt(det))/a
  t1 = (-v0 - sqrt(det))/a
  return in_range(t0, [0, t]) or in_range(t1, [0, t])

##################################################

ACTION_COST = 1

# class Burst(STRIPSAction):
#   cost = ACTION_COST
#   def __init__(self):
#     params = (P('p1', POSITION), P('v1', VELOCITY),
#               P('a', ACCELERATION), P('t', TIME),
#               P('p2', POSITION), P('v2', VELOCITY))
#     p1, v1, a, t, p2, v2 = params
#     super(Burst, self).__init__(self.__class__.__name__, params, [
#       AtState(p1, v1),
#       IsBurst(p1, v1, a, t, p2, v2),
#     ], [
#       AtState(p1, v1),
#     ])

class Burst(STRIPSAction):
  cost = ACTION_COST
  def __init__(self):
    params = (P('x1', STATE),  P('a', ACCELERATION), P('t', TIME), P('x2', STATE))
    x1, a, t, x2 = params
    super(Burst, self).__init__(self.__class__.__name__, params, [
      AtState(x1),
      IsBurst(x1, a, t, x2),
      Flying(),
    ], [
      #Cost(), # Delta fuel
      AtState(x2),
      Not(AtState(x1)),
    ])

class Glide(STRIPSAction):
  cost = ACTION_COST
  def __init__(self):
    params = (P('x1', STATE), P('t', TIME), P('x2', STATE))
    x1, t, x2 = params
    super(Glide, self).__init__(self.__class__.__name__, params, [
      AtState(x1),
      IsGlide(x1, t, x2),
      Flying(),
    ], [
      AtState(x2),
      Not(AtState(x1)),
    ])

class Deploy(STRIPSAction): # At some state with zero velocity to safely deploy
  cost = ACTION_COST
  def __init__(self):
    params = (P('x', STATE), P('p', POSITION), P('s', SATELLITE))
    x, p, s = params
    super(Deploy, self).__init__(self.__class__.__name__, params, [
      Carrying(s),
      AtState(x),
      ArePair(x, p),
    ], [
      AtOrbit(s, p),
      Not(Carrying(s)),
    ])

class TakeOff(STRIPSAction): # At some state with zero velocity to safely deploy
  cost = ACTION_COST
  def __init__(self):
    super(TakeOff, self).__init__(self.__class__.__name__, [], [
      Landed(),
      AtState(State((0, 0))),
    ], [
      Flying(),
      Not(Landed()),
    ])

class Land(STRIPSAction): # At some state with zero velocity to safely deploy
  cost = ACTION_COST
  def __init__(self):
    super(Land, self).__init__(self.__class__.__name__, [], [
      Flying(),
      AtState(State((0, 0))),
    ], [
      Landed(),
      Not(Flying()),
    ])

##################################################

# class AboveAxiom(STRIPSAxiom):
#   def __init__(self):
#     p, v, h = P('p', POSITION), P('v', VELOCITY), P('h', POSITION)
#     super(AboveAxiom, self).__init__([
#         AtState(p, v),
#         IsAbove(p, h),
#       ] , [
#         Above(h),
#     ])

class AboveAxiom(STRIPSAxiom):
  def __init__(self):
    x, h = P('x', STATE), P('h', POSITION)
    super(AboveAxiom, self).__init__([
        AtState(x),
        IsAbove(x, h),
      ] , [
        Above(h),
    ])

##################################################

ACCELERATION_RANGE = [-10., 20.]
#ACCELERATION_RANGE = [0., 20.]
#TIME_RANGE = [0., 1.]
TIME_RANGE = [0., 5.]

# TODO - I could just combine p, v as a single x state

# class ForwardBurstStream(CondStream): # NOTE - can treat the interval as a region itself
#   def __init__(self):
#     p1, v1 = P('p1', POSITION), P('v1', VELOCITY)
#     a, t = P('a', ACCELERATION), P('t', TIME)
#     p2, v2 = P('p2', POSITION), P('v2', VELOCITY)
#     super(ForwardBurstStream, self).__init__([p1, v1], [a, t, p2, v2], [
#     ], [
#       IsBurst(p1, v1, a, t, p2, v2),
#     ])
#   class StreamFn(GeneratorStream):
#     def get_generator(self, (p1, v1)):
#       while True:
#         a = uniform(*ACCELERATION_RANGE) # NOTE - this should be u because we have gravity
#         t = uniform(*TIME_RANGE)
#         v2 = v1.value + (a-G)*t
#         p2 = .5+(a-G)*t**2 + v1.value*t + p1.value
#         yield (Acceleration(a), Time(t), Position(p2), Velocity(v2),)
#
# class AboveTest(TestCondStream): # NOTE - can treat the interval as a region itself
#   def __init__(self):
#     p, h = P('p', POSITION), P('h', POSITION)
#     super(AboveTest, self).__init__([p, h], [
#     ], [
#       IsAbove(p, h),
#     ])
#   class StreamFn(TestStream):
#     def test(self, (p, h)):
#       return p.value >= h.value

##################################################

class ForwardBurstStream(CondStream):
  def __init__(self):
    x1, a, t, x2 = P('x1', STATE),  P('a', ACCELERATION), P('t', TIME), P('x2', STATE)
    super(ForwardBurstStream, self).__init__([x1], [a, t, x2], [
    ], [
      IsBurst(x1, a, t, x2),
    ])
  class StreamFn(GeneratorStream):
    def get_generator(self, (x1,)):
      p1, v1 = x1.value
      #while True:
      #  a = uniform(*ACCELERATION_RANGE) # NOTE - this should be u because we have gravity
      #  t = uniform(*TIME_RANGE)
      for a in ACCELERATION_RANGE + [-G]:
        t = 1.0 # Small time step for the controlled burst
        v2 = v1 + (a+G)*t
        p2 = .5+(a+G)*t**2 + v1*t + p1
        if not crashes(p1, v1, a+G, t):
          yield (Acceleration(a), Time(t), State((p2, v2)))

class ForwardGlideStream(CondStream):
  def __init__(self):
    x1, t, x2 = P('x1', STATE), P('t', TIME), P('x2', STATE)
    super(ForwardGlideStream, self).__init__([x1], [t, x2], [
    ], [
      IsGlide(x1, t, x2),
    ])
  class StreamFn(FunctionStream):
    def function(self, (x1,)):
      p1, v1 = x1.value
      t = 1.0 # Small time step for the controlled glide
      v2 = v1 + G*t
      p2 = .5+G*t**2 + v1*t + p1
      yield (Time(t), State((p2, v2)))

"""
class FixedBurstStream(CondStream):
  def __init__(self):
    x1, a, t, x2 = P('x1', STATE),  P('a', ACCELERATION), P('t', TIME), P('x2', STATE)
    #super(FixedBurstStream, self).__init__([x1, a, t], [x2], [
    super(FixedBurstStream, self).__init__([x1, a], [x2], [
    ], [
      IsBurst(x1, a, Time(TIME_RANGE[1]), x2),
    ])
  class StreamFn(ListStream):
    def get_list(self, (x1, a)):
      p1, v1 = x1.value
      t = TIME_RANGE[1]
      v2 = v1 + (a.value-G)*t
      p2 = .5+(a.value-G)*t**2 + v1*t + p1
      return [(State((p2, v2)),)]
"""

class TargetBurstStream(CondStream):
  def __init__(self):
    x1, a, t, x2 = P('x1', STATE),  P('a', ACCELERATION), P('t', TIME), P('x2', STATE)
    super(TargetBurstStream, self).__init__([x1, x2], [a, t], [
    ], [
      IsBurst(x1, a, t, x2),
    ])
  class StreamFn(FunctionStream):
    def function(self, (x1, x2)):
      p1, v1 = x1.value
      p2, v2 = x2.value
      if v2 + v1 == 0:
        return []
      t = 2.*(p2 - p1)/(v2 + v1)
      if not in_range(t, TIME_RANGE) or t == 0:
        return []
      a = (v2 - v1)/t - G
      if not in_range(a, ACCELERATION_RANGE):
        return []
      if crashes(p1, v1, a+G, t):
        return []
      return [(Acceleration(a), Time(t))]

##################################################

class AboveTest(TestCondStream):
  def __init__(self):
    x, h = P('x', STATE), P('h', POSITION)
    super(AboveTest, self).__init__([x, h], [
    ], [
      IsAbove(x, h),
    ])
  class StreamFn(TestStream):
    def test(self, (x, h)):
      p, v = x.value
      return p >= h.value

##################################################

def get_problem():
  satellite = C('satellite1', SATELLITE)
  satellite_pos = 50

  initial_atoms = [
    #AtState(Position(0), Velocity(0)),
    AtState(State((0, 0))),
    Carrying(satellite),
    ArePair(State((satellite_pos, 0)), Position(satellite_pos)),
    Landed(),
  ]

  goal_literals = [
    #Above(Position(20)),
    #AtState(State((50, 0))),
    AtOrbit(satellite, Position(satellite_pos)),
    Landed(),
  ]

  operators = [
    Burst(),
    Glide(),
    Deploy(),
    TakeOff(),
    Land(),
    AboveAxiom(),
  ]

  cond_streams = [
    ForwardBurstStream(), # TODO - BackwardBurstStream in order to move back from the goal
    #FixedBurstStream(),
    TargetBurstStream(),
    ForwardGlideStream(),
    AboveTest(),
  ]

  objects = [ # Assorted objects
    #Acceleration(ACCELERATION_RANGE[1]),
    #Acceleration(ACCELERATION_RANGE[0]),
    #Acceleration(0),
  ]

  return STRIPStreamProblem(initial_atoms, goal_literals, operators, cond_streams, objects)
