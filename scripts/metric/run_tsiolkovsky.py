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
from math import log

# TODO - make this executable within the API?

# https://en.wikipedia.org/wiki/Tsiolkovsky_rocket_equation
# https://kclpure.kcl.ac.uk/portal/files/56331945/FAIA285_1185.pdf
# https://www.ijcai.org/Proceedings/16/Papers/455.pdf
# The thrust action duration is flexible and depends on the available propellant mass
# Boolean thrust duration
# Could always assume
# https://en.wikipedia.org/wiki/Specific_impulse
# https://spaceflightsystems.grc.nasa.gov/education/rocket/rktpow.html
# http://web.mit.edu/16.unified/www/SPRING/propulsion/notes/node103.html # NOTE - this is the most helpful one

# Choose time duration and velocity of escape
# This doesn't really say anything about the position?
# I guess I could just do the integral to find the delta change over time (integrate delta t)

# TODO - backward thruster for deceleration
# TODO - drag and variable gravity

G = 9.8
I = 10
MIN_FUEL = .1

def delta_mass(q, t):
  return -q*t

def delta_velocity(m0, m1):
  return I*G*log(m0/m1)

def delta_position(v0, m0, q, t):
  return I*G*(t + (t - m0/q)*log(m0/(m0+delta_mass(q,t))))

def forward_burst((p0, v0, m0), q, t): # TODO - does this take into account for gravity?
  m1 = m0 + delta_mass(q, t)
  #m1 = max(m1, MIN_FUEL)
  if m1 < MIN_FUEL:
    return []
  #v1 = v0
  #p1 = p0 + v0*t
  v1 = v0 - G*t
  p1 = p0 + v0*t - .5*G*t**2
  if q != 0 and t != 0:
    v1 += delta_velocity(m0, m1)
    p1 += delta_position(v0, m0, q, t)
  #print (p1, v1, m1)
  return [(p1, v1, m1)]

# http://www.wolframalpha.com/input/?i=solve+13+%3D+23+ln(m%2F(m-q*t)),+17+%3D+23*(+t+%2B+(-m%2Fq+%2B+t)+log(m%2F(m+-+q*t)))

def inverse():
  raise NotImplementedError()

def create_problem(time_step=.5, fuel_rate=5, start_fuel=10, goal_p=10):
  """
  Creates the Tsiolkovsky rocket problem.

  :return: a :class:`.STRIPStreamProblem`
  """

  # Data types
  STATE, RATE, MASS, TIME  = Type(), Type(), Type(), Type()
  HEIGHT = Type()

  # Fluent predicates
  AtState = Pred(STATE)

  # Derived predicates
  Above = Pred(HEIGHT)

  # Static predicates
  IsBurst = Pred(STATE, RATE, TIME, STATE)
  IsAbove = Pred(STATE, HEIGHT)

  # Free parameters
  X1, X2 = Param(STATE), Param(STATE)
  Q, T = Param(RATE), Param(TIME)
  H = Param(HEIGHT)

  rename_easy(locals()) # Trick to make debugging easier

  ####################

  actions = [
    Action(name='burst', parameters=[X1, Q, T, X2],
      condition=And(AtState(X1), IsBurst(X1, Q, T, X2)),
      effect=And(AtState(X2), Not(AtState(X1)))),
  ]

  axioms = [
    Axiom(effect=Above(H),
          condition=Exists([X1], And(AtState(X1), IsAbove(X1, H)))),
  ]

  ####################

  # Conditional stream declarations
  cond_streams = [
    GeneratorStream(inputs=[X1, Q, T], outputs=[X2], conditions=[], effects=[IsBurst(X1, Q, T, X2)],
                    generator=forward_burst),

    TestStream(inputs=[X1, H], conditions=[], effects=[IsAbove(X1, H)],
               test=lambda (p, v, m), h: p >= h, eager=True),
  ]

  ####################

  constants = [
    TIME(time_step),
    RATE(0),
    RATE(fuel_rate),
  ]

  initial_atoms = [
    AtState((0, 0, start_fuel)),
  ]

  goal_formula = And(
    #Not(initial_atoms[0])
    Above(goal_p)
  )

  problem = STRIPStreamProblem(initial_atoms, goal_formula, actions + axioms, cond_streams, constants)

  return problem

##################################################

from stripstream.algorithms.experimental.simultaneous import simultaneous
from stripstream.algorithms.incremental.incremental_planner import incremental_planner
from stripstream.algorithms.search.fast_downward import get_fast_downward
from stripstream.pddl.utils import convert_plan

def main():
  """
  Creates and solves the Tsiolkovsky rocket problem.
  """
  problem = create_problem(goal_p=100)
  print problem

  #plan, _ = simultaneous(problem, frequency=10, verbose=False, debug=False)
  search = get_fast_downward('eager') # dijkstra | astar | wastar1 | wastar2 | wastar3 | eager | lazy
  plan, _ = incremental_planner(problem, search=search, frequency=1)
  print
  print 'Plan:', convert_plan(plan)

if __name__ == '__main__':
  main()
