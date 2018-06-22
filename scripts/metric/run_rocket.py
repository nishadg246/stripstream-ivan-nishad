#!/usr/bin/env python

from time import time
from stripstream.pddl.logic.predicates import EasyPredicate as Pred
from stripstream.pddl.objects import EasyType as Type, EasyParameter as Param
from stripstream.pddl.logic.connectives import Not, And
from stripstream.pddl.logic.quantifiers import Exists
from stripstream.pddl.operators import Action, Axiom
from stripstream.pddl.cond_streams import EasyGenStream as GeneratorStream, EasyTestStream as TestStream
from stripstream.pddl.utils import rename_easy
from stripstream.pddl.problem import STRIPStreamProblem
from math import sqrt

# TODO - mass of the packages and fuel

G = -9.8
ACCELERATION_RANGE = [-10., 20.]
#ACCELERATION_RANGE = [0., 20.]
#TIME_RANGE = [0., 1.]
TIME_RANGE = [0., 5.]

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

def forward_burst((p1, v1)):
  #while True:
  #  a = uniform(*ACCELERATION_RANGE) # NOTE - this should be u because we have gravity
  #  t = uniform(*TIME_RANGE)
  for a in ACCELERATION_RANGE + [-G]:
    t = 1.0 # Small time step for the controlled burst
    v2 = v1 + (a+G)*t
    p2 = .5+(a+G)*t**2 + v1*t + p1
    if not crashes(p1, v1, a+G, t):
      yield [a, t, (p2, v2)]

# def fixed_burst(x1, a):
#   p1, v1 = x1.value
#   t = TIME_RANGE[1]
#   v2 = v1 + (a.value-G)*t
#   p2 = .5+(a.value-G)*t**2 + v1*t + p1
#   return [(p2, v2)]

def target_burst((p1, v1), (p2, v2)):
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
  return [[a, t]]

def forward_glide(x1):
  p1, v1 = x1
  t = 1.0 # Small time step for the controlled glide
  v2 = v1 + G*t
  p2 = .5+G*t**2 + v1*t + p1
  yield [t, (p2, v2)]

##################################################

def create_problem():
  STATE = Type()
  POSITION = Type()
  ACCELERATION = Type()
  TIME = Type()
  SATELLITE = Type()

  #FORCE = Type()
  #MASS = Type('mass')
  #ROCKET = Type('rocket')

  ######

  #AtState = Predicate(POSITION, VELOCITY)
  AtState = Pred(STATE)
  Above = Pred(POSITION)

  AtOrbit = Pred(SATELLITE, POSITION)
  Carrying = Pred(SATELLITE)

  Flying = Pred()
  Landed = Pred()

  ######

  IsBurst = Pred(STATE, ACCELERATION, TIME, STATE)
  IsGlide = Pred(STATE, TIME, STATE)

  #IsCrashed = Pred(POSITION)
  IsAbove = Pred(STATE, POSITION)
  ArePair = Pred(STATE, POSITION)

  ######

  X1, X2 = Param(STATE), Param(STATE)
  A, T = Param(ACCELERATION), Param(TIME)
  S = Param(SATELLITE)
  H = Param(POSITION)

  rename_easy(locals()) # Trick to make debugging easier

  ######

  actions = [
    Action(name='burst', parameters=[X1, A, T, X2],
      condition=And(AtState(X1), IsBurst(X1, A, T, X2), Flying()),
      effect=And(AtState(X2), Not(AtState(X1)))),

    Action(name='glide', parameters=[X1, A, T, X2],
      condition=And(AtState(X1), IsGlide(X1, T, X2), Flying()),
      effect=And(AtState(X2), Not(AtState(X1)))),

    Action(name='deploy', parameters=[S, X1, H],
      condition=And(Carrying(S), AtState(X1), ArePair(X1, H)),
      effect=And(AtOrbit(S, H), Not(Carrying(S)))),

    Action(name='take_off', parameters=[],
      condition=And(Landed(), AtState((0, 0))),
      effect=And(Flying(), Not(Landed()))),

    Action(name='land', parameters=[],
      condition=And(Flying(), AtState((0, 0))),
      effect=And(Landed(), Not(Flying()))),
  ]

  axioms = [
    Axiom(effect=Above(H),
          condition=Exists([X1], And(AtState(X1), IsAbove(X1, H)))),
  ]

  cond_streams = [
    GeneratorStream(inputs=[X1], outputs=[A, T, X2], conditions=[], effects=[IsBurst(X1, A, T, X2)],
                    generator=forward_burst),

    #GeneratorStream(inputs=[X1, A], outputs=[T, X2], conditions=[], effects=[IsBurst(X1, A, T, X2)],
    #                generator=fixed_burst),

    GeneratorStream(inputs=[X1, X2], outputs=[A, T], conditions=[], effects=[IsBurst(X1, A, T, X2)],
                    generator=target_burst),

    GeneratorStream(inputs=[X1], outputs=[T, X2], conditions=[], effects=[IsGlide(X1, T, X2)],
                    generator=forward_glide),

    TestStream(inputs=[X1, H], conditions=[], effects=[IsAbove(X1, H)],
               test=lambda (p, v), h: p >= h, eager=True),
  ]

  ######

  satellite = 'satellite1'
  satellite_pos = 20

  initial_atoms = [
    AtState((0, 0)),
    Carrying(satellite),
    ArePair((satellite_pos, 0), satellite_pos),
    Landed(),
  ]

  goal_literals = [
    #Above(20),
    #AtState((20, 0)),
    AtOrbit(satellite, satellite_pos),
    Landed(),
  ]

  constants = [ # Assorted objects
    #Acceleration(ACCELERATION_RANGE[1]),
    #Acceleration(ACCELERATION_RANGE[0]),
    #Acceleration(0),
  ]

  return STRIPStreamProblem(initial_atoms, goal_literals, actions + axioms, cond_streams, constants)

from stripstream.algorithms.incremental.incremental_planner import incremental_planner
from stripstream.pddl.examples.rocket import get_problem
from stripstream.pddl.utils import convert_plan

def main():
  #planning_problem = get_problem()
  planning_problem = create_problem()
  print planning_problem

  t0 = time()
  plan, _ = incremental_planner(planning_problem, frequency=1) # 20 | 100 | INF
  print
  print 'Plan:', convert_plan(plan)
  print 'Time:', time() - t0

if __name__ == '__main__':
  main()