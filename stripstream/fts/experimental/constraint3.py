
# Not treating

from stripstream.pddl.logic import And, Or, Exists, ForAll  #, Equal


# parameter != state variable

# NOTE - can only universal quantify over discrete

# NOTE - need to introduce parameter to get rid of old value

# So this really is just like the preconditions and effects thing except that you call them X1 and X2

def X1(*args):
  return args


class Parameter(object):
  def __sub__(self, other):
    pass # TODO - make a parameter of a certain type

class Temporal(object):
  def __init__(self, name):
    self.name = name
  def __getitem__(self, item):
    return item

class TempVar(object):
  def __init__(self, temp, var):
    self.temp = temp
    self.var = var
  def __eq__(self, other):
    return object() # TODO - return a predicate here or something


x1 = Temporal('x1')
x2 = Temporal('x2')

# NOTE - make the initial state only use X2, make the final state only use X1

#####################################


class ConstraintType(object):
  # Each constraint must have an implicit test
  # Should this be specific to particular values? Then can't reuse...
  # Should this be typed?
  # Should this have an explicit length?
  num = 0
  prefix = 'C%s' # 'Con%s'
  #def __init__(self, types=tuple(), test=lambda *args: False, name=None):
  def __init__(self, types=tuple(), test=None, name=None):
    self.types = types
    #self.test = None # None instead of test
    self.test = test
    self.n = ConstraintType.num
    ConstraintType.num += 1
    #self.name = name if name is not None else self.__class__.__name__
    self.name = name if name is not None else self.prefix%self.n
  def __call__(self, *args, **kwargs):
    return Constraint(self, args)
  def __repr__(self):
    return self.name
  # TODO - hash this?

Eq = ConstraintType(types=(OBJECT, OBJECT), test=lambda a, b: a == b, name='Eq') # TODO - should I allow ternary Eq...

class Constraint(object):
  def __init__(self, con_type, items):
    self.con_type = con_type
    self.items = tuple(items) # value or parameter
  def __repr__(self):
    return '%s(%s)'%(self.con_type.name, ','.join(repr(value) for value in self.items))
  def __eq__(self, other):
    return type(self) == type(other) and self.con_type == other.con_type and self.items == other.items
  def __ne__(self, other):
    return not(self == other)
  def __hash__(self):
    return hash((self.__class__, self.con_type, self.items))
  pddl = __repr__

#####################################

Motion = ConstraintType(name='Motion') # NOTE - types are really just for safety more than anything...
Traversable = ConstraintType(name='Traversable')
HoldTraversable = ConstraintType(name='HoldTraversable')
Stable = ConstraintType(name='Stable')
Grasp = ConstraintType(name='Grasp')
Kin = ConstraintType(name='Kin')

q1, t, q2 = 'q1', 't', 'q2'
b, p, g, q = 'b', 'p', 'g', 'q'
ob = 'ob'

# TODO - do I have to introduce intermediate parameters to do equality of uninvolved things


r, h = 'robot', 'hold'

# NOTE - you can query a state to see if it has a value, but you cannot do anything this it directly




transition = Or(
  Exists([q1, t, q2], And(
    ForAll([ob],
      Exists([p], And(
        #Eq(x1[ob], x2[ob]),
        #X1(ob, p),
        x1[ob]: q1 => q2,
        x1[ob]==x2[ob],
        x1[ob]==p, x2[ob]==p,
        #x1[ob] => x2[ob]
        Traversable(t, ob, x1[ob])))),
    #x2[r]<=q2,
    x1[h]==None,x2[h]==None,
    x1[r]==q1, x2[r]==q2)),
    #X1(q1), X2(q2))),
  Exists([b, p, g, q], And(
    Kin(g, q, p),
    Stable(p), Grasp(g),
    a -> b,
    X1(b, p), X2(b, g),
    X1(h, None), X2(h, b),
    X1(r, q)),
    ForAll([ob],
      Exists([p], And(Or(
        ob==o, # NOTE - I end up with the exact same problem
        And(x1[ob]==p, x2[ob]==p),
      ))


  ),

    #X1[r](q),
    #X1(r) = q,
)
# NOTE - does that still mean that if a state variable isn't mentioned, it can be anything?


# NOTE - so this doesn't cleanly define a set then. Rather, it is a test
# I do need to explicitly separate variables and values for setting equality

# This doesn't yet say anything about the values of the other variables!!!!


# So there are constraints on teh values of the variables as well as the parameters themselves

# This serves as a test between two states, control parameters, aux parameters
# Each step is like a constraint satisfaction problem
# NOTE - this fully frames moving between two states as a test
# Are kind of indirectly testing the state, you cannot explicitly put Kin(x[o], x[1], x[2]) in, but in doing so,
# We

# So our this are less like actions but more are like witnesses of a solution
# Any state trajectory sequence that satisfies these
# This has a much more constraint satisfactiony view anyways becuase it direclty exposes teh parameters you acn control
# Rather than making each state a decision variable
# There are a set of logical assertions populated about the states on the boarder of a transition

transition = Or(
  And(Motion(x1[r], u[t], x2[r]), # MoveH
    Eq(x1[h], x2[h]), Eq(x1[h], None),
    #Eq(x1[h], x2[h], None),
    ForAll([ob], And(Eq(x1[ob], x2[ob]),
      Traversable(u[t], ob, x1[ob])))),
  Exists([o], Or( # NOTE - why do I make this a parameter but not the forall? Existentials are naturally parameters
    And(Motion(x1[r], u[t], x2[r]), # MoveH
      Eq(x1[h], x2[h]), Eq(x1[h], o),
      #ForAll([ob], Eq(x1[ob], x2[ob])),
      ForAll([ob], unchanged(ob)),
      ForAll([ob], HoldTraversable(u[t], ob, x1[ob], o, x1[o]))), # Could just make this default to being true
      #ForAll([ob], And(
      #  Not(Eq(ob, o)), HoldTraversable(u[t], ob, x1[ob], o, x1[o]))),
    And(Stable(x1[o]), Grasp(x2[o]), # Pick
      Kin(x2[o], x1[r], x1[o]),
      Eq(x1[r], x2[r]),
      Eq(x1[h], None), Eq(x2[h], o),
        ForAll([ob], Or(Eq(ob, o), Eq(x1[ob], x2[ob])))), # This should be an Or I think...
    And(Grasp(x1[o]), Stable(x2[o]), # Place
      Kin(x1[o], x1[r], x2[o]),
      Eq(x1[r], x2[r]),
      Eq(x1[h], o), Eq(x2[h], None),
        ForAll([ob], Or(Eq(ob, o), Eq(x1[ob], x2[ob])))))))