from stripstream.pddl.logic.predicates import Type, Predicate
from stripstream.pddl.objects import OBJECT, Parameter, Constant

# Hybrid planning wrapper around my planner
# Declare set of variables

variables = ['R', 'O1', 'H']
aux_variables = ['T']

Var = Type
AuxVar = Type

class StuffVar(Var): # NOTE - this is kind of like a variable domain
  def __init__(self):
    self.dictionary = {}
  def __call__(self, value):
    return Constant(value, self) # TODO - hash the name
  @property
  def p1(self):
    return Parameter('1', self) # NOTE - could also just make an object
  @property
  def p2(self):
    pass
  @property
  def s(self):
    pass
  @property
  def e(self):
    pass

  # TODO - aux param that only has one

R, H = Var('robot'), Var('holding')
T = AuxVar('traj')

hand_empty = H(0)
holding_1 = H(1)

Function = Predicate # TODO - alternatively a constraint

Kin = Function('kin', [R, O])
Motion = Function('motion', [R, R, T])


# TODO - make a shared initial and final variable for all of these
# I could just make a long laundry list of fluents that are constraints

# NOTE - if I list a variable twice, then I have to automatically substitute its values

# Maybe list what changes instead fo what is the same in general


Q = Parameter('q', R)
QE = Parameter('qe', R)

Q1 = Parameter('q1', R)
Q2 = Parameter('q2', R)


def O1(name):
  return 'o' + name + '1'

def B(var):
  return var + 'e'

def E(var):
  return var + 's'

# TODO - maybe I should just directly specify things in the functional form

# So I can specify things in whatever form I want for my experiments
# I just need to specify things in a nice form for a little bit
# I could just assume that any references are to the start value unless specified


# TODO - maybe it wouldn't be do bad to list off all the unchanged variables

# TODO - make a hashable object dictionary for each of these

CONFIG = Type('conf')

def assign(var, value): # TODO - function that produces these atoms?
  return var.name + '_eq_' + value

def equal(var, value): # TODO - function that produces these atoms?
  predicate = Predicate(var.name + '_eq', [var])
  return predicate(value)

Assign = Predicate('assign', [OBJECT, OBJECT])
Equal = Predicate('equal', [OBJECT, OBJECT]) # TODO - make a separate one per variable?

obj_params = {} # NOTE - I can either construct these online or ahead of time

"""
class Move(object):
  #def __init__(self, objects):
  def __init__(self, start, end): # Start and end maps from type to object
    self.constraints = [
      Motion(Q1, Q2, T),
      Equal(H1, H2),
      Equal(H1, 0),
    ] + [
      Equal(o1, o2) for o in objects
    ]
    self.constraints = [
      Motion(R.p1, R.p2, T),
      Equal(H.p1, H.p2),
      Equal(H.p1, H(0)),
    ] + [
      Equal(o1, o2) for o in objects
    ]


class Variable(object):
  def __init__(self, name, args=[]):
  #def __init__(self, *args):
    self.name = name
    self.args = args

V = S = Variable

# TODO - could do a version of this for the initial state and final state


class Pick(object):
  def __init__(self, o):
    self.pre = [
      IsIK(o.p1, r.p1, t.p),
      H(0),
    ]
    self.eff = [
      H(o),
    ]

    o = P('o', BLOCK) # what type should this be?

    # TODO - should I think of this using variables or predicates?
    self.pre = {
      #(R, []): P('q', CONFIG),
      V('r'): P('q', CONFIG),
      (O, [o]): P('p', POSE),
      (G, []): False,
    }
    self.eff = {
      (G, []): True,
    }
    self.aux = { # NOTE - don't need this at all
      (T, []): P('t', TRAJ)
    }
    self.stat = [
      #IsIK(o, self.pre[(O, [o])], self.pre[(O, [o])], t),
      IsIK(V(o), V('q'), t), # NOTE - this has arguments within it
      #IsIK(o, V(o), V('q'), t),
    ]
    self.pre = {

    }

    self.trans = {
      V('r'): (conf1, conf2) # NOTE - could also write like this
    }



class Place(object):
  def __init__(self, o):
    self.pre = [
      IsIK(o.p1, r.p1, t.p),
      H(1),
    ]
    self.eff = [
      H(0),
    ]
    # TODO - maybe just mentioning the value within pre/eff is enough to dictate what is happening
    # NOTE - slightly asymetric because we drop the effects



class Move(object):
  def __init__(self, move):
    self.pre = [Q, H]
    self.eff = [QE]

    self.stat = [
      #Motion(Q1, T, Q2),
      Motion(Q, T, QE),

      Assign(),
      #Equal('H', )
    ]

    # TODO - can I deduce that something is an effect just by using the end value
    # If the end value isn't included, then I need to specify it explicitly

    self.eff = ['R']

# I could just use an array indicate for which variables are involved

class Pick(object):
  def __init__(self, obj, others):
    self.stat = []
    self.der = []

    self.pre = [B('R'), B(obj)] # Maybe I should just directly list the types
    self.eff = [E(obj)]

    # Well we automatically infer the preconditions from the constraints anyways
    # TODO - what about constants?

    self.pre = []

    params = (P('b', BLOCK), P('p', POSE), P('g', GRASP), P('q', CONFIG), P('t', TRAJ))
    b, p, g, q, t = params
    conditions = [
      AtPose(b, p),
      HandEmpty(),
      IsPose(b, p), # NOTE - might not need these
      IsGrasp(b, g),
      IsIK(b, p, g, q, t),
    ]
    if BASE:
      conditions.append(AtConfig(q))
    conditions += collision_conditions(oracle, b, t)
    effects = [
      #HasGrasp(b, g),
      Holding(b),
      HasGrasp(g),
      Not(HandEmpty()),
      Not(AtPose(b, p)),
    ]
    super(Pick, self).__init__(self.__class__.__name__, params, conditions, effects)


# TODO - maybe just make an equality predicate that assigns things


# TODO - for each type make a param vs value thing
#
class Pick(STRIPSAction):
  def __init__(self, oracle):
    b, p, g, q, t = P('b', BLOCK), P('p', POSE), P('g', GRASP), P('q', CONFIG), P('t', TRAJ)
    stat = [KinCon(b, p, g, q, t), PoseCon(b, p)]
    pre = [PoseEq(b, p), HoldVal(C(False, BOOL)), ConfVal(q)]
    eff = [PoseVal(b, p), HoldVal(C(False, BOOL)), ConfVal(q)
           Not(HoldVal(C(False, BOOL)))]
    self.der = []

    #Val(r, q) # TODO - could make the state variable be something here


    #ObjPose(b, p)
    #Robot(q)
"""