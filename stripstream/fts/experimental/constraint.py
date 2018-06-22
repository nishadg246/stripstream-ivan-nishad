import sys
sys.path.append('..')
#from stripstream.logic import Not

from operator import add
from collections import defaultdict
from stripstream.pddl.logic import And, Or, Exists, ForAll, Not

# Constraints on types of variables

class Parameter(object):
  def __init__(self, pmap, var):
    self.pmap = pmap
    self.var = var
  def __repr__(self):
    return '%s[%s]'%(self.pmap, self.var)
  def __eq__(self, other):
    return type(self) == type(other) and self.pmap == other.pmap and self.var == other.var
  def __ne__(self, other):
    return not(self == other)
  def __hash__(self):
    return hash((self.__class__, self.pmap, self.var))


class DecisionVariable(Parameter):
  def __init__(self, *names):
    self.names = tuple(names)

class X1(Parameter):
  def __init__(self, var):
    super(X1, self).__init__('x1', var)

class X2(Parameter):
  def __init__(self, var):
    super(X2, self).__init__('x2', var)

class U(Parameter):
  def __init__(self, var):
    super(U, self).__init__('u', var)

class ParameterMap(object):
  def __init__(self, name):
    self.name = name
  def __getitem__(self, item):
    return Parameter(self, item)
  def __repr__(self):
    return self.name
  def __eq__(self, other):
    return type(self) == type(other) and self.name == other.name
  def __ne__(self, other):
    return not(self == other)
  def __hash__(self):
    return hash((self.__class__, self.name))

x1 = ParameterMap('x1')
x2 = ParameterMap('x2')
u = ParameterMap('u')

class Type(object):
  def __init__(self, name=None, parent=None):
    self.name = None
    self.parent = None # TODO - make the parent always be object

OBJECT = Type('object', None)

#####################################

# TODO - should I separate this from the sampler? I don't think I want (or need to require an explicit set)

class Constraint(object):
  # Each constraint must have an implicit test
  # Should this be specific to particular values? Then can't reuse...
  # Should this be typed?
  # Should this have an explicit length?
  num = 0
  prefix = 'C%s' # 'Con%s'
  def __init__(self, types=tuple(), test=lambda *args: False, name=None):
    self.types = types
    #self.test = None # None instead of test
    self.test = test
    self.n = Constraint.num
    Constraint.num += 1
    #self.name = name if name is not None else self.__class__.__name__
    self.name = name if name is not None else self.prefix%self.n
  def __call__(self, *args, **kwargs):
    return Solution(self, args)
  def __repr__(self):
    return self.name
  # TODO - hash this?

Eq = Constraint(types=(OBJECT, OBJECT), test=lambda a, b: a == b, name='Eq') # TODO - should I allow ternary Eq...

class Solution(object):
  def __init__(self, constraint, values):
    self.constraint = constraint
    self.values = values
  def __repr__(self):
    return '%s(%s)'%(self.constraint.name, ','.join(repr(value) for value in self.values))
  def __eq__(self, other):
    return type(self) == type(other) and self.constraint == other.constraint and self.values == other.values
  def __ne__(self, other):
    return not(self == other)
  def __hash__(self):
    return hash((self.__class__, self.constraint, self.values))

def get_param_map(constraints):
  equalities = set() # Form transitive clique and ensure that no conflicts
  for con in constraints:
    if con.constraint == Eq:
      a, b = con.values
      equalities.update({(a,b),(b,a)})

  edges = defaultdict(list)
  for a, b in equalities:
    edges[a].append(b)
  components = []
  reached = set()
  def dfs(a):
    reached.add(a)
    components[-1].append(a)
    for b in edges[a]:
      if b not in reached:
        dfs(b)
  for a in edges:
    if a not in reached:
      components.append([])
      dfs(a)

  reduced_param_map = {}
  for component in components:
    params = sorted(filter(lambda v: isinstance(v, Parameter), component), key=repr) # Sort to ensure same order
    values = filter(lambda v: not isinstance(v, Parameter), component)
    if len(values) != 0:
      assign = values[0]
      if any(val != assign for val in values):
        print 'Warning! Infeasible transition'
        return None
    else:
      assign = params[0]
      #assign = params[0].var # TODO - maybe convert the name nicely or something
    for v in component:
      reduced_param_map[v] = assign
  return reduced_param_map

#####################################

class Test(object):
  def __init__(self, constraint, test=lambda *args: False):
    self.constraint = constraint
    self.test = test
  def __call__(self, *args):
    return self.test(*args)

class Sampler(object):
  def __init__(self, constraint, gen=lambda: iter([])):
    self.constraint = constraint
    self.gen = gen
  def __call__(self, *args):
    return self.gen(*args)

class CondSampler(object):
  # For a solution, just list which indices are cond
  def __init__(self, constraint, inputs=tuple(), domain=tuple(),
               gen=lambda *args: iter([])):
    self.constraint = constraint
    self.inputs = tuple(inputs)
    self.domain = domain # A conjunction (should I instead do DNF) of constraints on inputs
    self.gen = gen
  def __call__(self, *args):
    return self.gen(*args)


class MultiCondSampler(object):
  pass

#####################################

At = Constraint(name='At')

class Not(object):
  def __init__(self, value):
    self.value = value
  def __repr__(self):
    return 'Not(%s)'%self.value

class Transition(object):
  num = 0
  prefix = 'T%s'
  def __init__(self, constraints, name=None):
    self.constraints = constraints
    self.n = Transition.num
    Transition.num += 1
    #self.name = name if name is not None else self.__class__.__name__ # TODO - index these
    self.name = name if name is not None else self.prefix%self.n
  # TODO - detect if conflicting equality constraints so automatically not satisfiable
  # TODO - convert to simplified form
    self.infeasible = False

  def to_strips(self, state_vars): # NOTE - need control variables as well
    reduced_param_map = get_param_map(self.constraints)

    # Need a precondition iff
    # - Variable mentioned in non-eq constraint
    # - In effect (have to delete the old value)

    # Assume all effects unless find out otherwise
    # Only delete if the precondition equals the effect

    # NOTE - can't do any of this without equality constraints
    # 1) Start out with params for everything so all pre and eff
    # 2) For all x1[v]=x2[v], replace tform(x2[v]) with tform(x1[v]) (well these should be the same anyways) Well we are doing this to the name
    # 3) Delete all effects which are the same (these would cancel anyways)
    # 4) Remove preconditions which have variables not used within the constraints
    # 5) Just replace for any control parameters

    # Removing preconditions in general is much less of a big deal

    tform = lambda p: reduced_param_map.get(p, p)

    parameters = {tform(x1[var]) for var in state_vars if isinstance(tform(x1[var]), Parameter)} | \
      {tform(x2[var]) for var in state_vars if isinstance(tform(x2[var]), Parameter)} | \
      {val for val in reduced_param_map.values() if isinstance(val, Parameter)} # Takes care of control

    # TODO - include "self" assignments
    static = set()
    preconditions = set()
    effects = set()

    # NOTE - we only do interesting things for equal vars. If not equal, then cannot remove any pre or eff

    #print reduced_param_map
    equal_vars = {var for var in state_vars if tform(x1[var]) == tform(x2[var])}
    print 'eq:', equal_vars


    constraint_vars = set(reduce(add, [con.values for con in self.constraints if con.constraint != Eq]))
    #print 'con vars:', constraint_vars

    parameters = set()
    for var in equal_vars:
      if x1[var] in constraint_vars or x2[var] in constraint_vars:
        v = tform(x1[var])
        preconditions.add(At(var, v))
        parameters.add(v)
    for var in set(state_vars) - equal_vars:
      v1, v2 = tform(x1[var]), tform(x2[var])
      preconditions.add(At(var, v1))
      effects.update({At(var, v2), Not(At(var, v1))})
      parameters.update({v1, v2})
    for con in self.constraints:
      if con.constraint != Eq:
        vs = map(tform, con.values)
        static.add(con.constraint(*vs))
        parameters.update(vs)
    #parameters = {} # TODO - just update from here anyways
    parameters = filter(lambda p: isinstance(p, Parameter), parameters)

    # NOTE - the point of axioms is to reduce parameters, otherwise they doesn't help

    # For all parameters not mentioned in teh effects
    effect_vars = set()
    for effect in effects:
      if isinstance(effect, Not):
        effect = effect.value
      effect_vars.update(effect.values)
    effect_vars = filter(lambda p: isinstance(p, Parameter), effect_vars)
    print 'eff vars:', effect_vars

    # For all constraints, if ...
    # For all not effect_vars, if ...
    # Yeah I think I just kick out any constraint using a non-effect var
    # Need to kick out all of them using the same var or else no benefit
    # Kick out and replace with some sort of satisfied thing with the effect vars

    # NOTE - what about other static predicates that would reduce the number of instances
    # For instance, if there was only one other variable in which the predicate worked
    # I think we are assuming there are many solutions in general
    # NOTE - definitely include any constraints usable from the arguments

    # What about control variables such as Motion(x1[rob],u[traj],x2[rob])?
    # I think the same story applies (as long as the resulting size is indeed less than others)
    # Well I need the same traj to be shared across testing collisions...
    # Maybe I need an additional requirement that a non-effect is only used within one thing
    # NOTE - If every constraint involving that variable can be put into one axiom (might need putting others as well)

    """
    for var in state_vars:
      v1, v2 = tform(x1[var]), tform(x2[var])
      if v1 != v2:
        preconditions.add(At(var, v1)) # NOTE - maybe only add actions here
        effects.update({At(var, v2), Not(At(var, v1))})
        # What if a state value is set by equality to some other variable, need to include it here
        # But that's okay, the total number of parameters isn't bad
    for con in self.constraints:
      if con.constraint != Eq:
        #preconditions.add(con.constraint(*map(tform, con.values)))
        static.add(con.constraint(*map(tform, con.values)))
        for p in con.values:
          if isinstance(p, Parameter):
            preconditions.add(At(p.var, tform(p)))
            # TODO - what if an x2 is mentioned and it is set equal to an x1 but x1 isn't mentioned?
            # THis is kind of a strange case but it could happen
      if con.constraint == Eq:
        # Add precondition still?
        pass
    """

    """
    for con in self.constraints:
      if con.constraint == Eq:
        for i in range(len(con.values)):
          var = con.values[i]
          if isinstance(var, Parameter):
            assign = con.values[i-1]
            if var.pmap == x1:
              preconditions.add(At(var.var, reduced_param_map.get(assign, assign)))
            if var.pmap == x2:
              effects.add(At(var.var, reduced_param_map.get(assign, assign)))
      else:
        preconditions.add(con.constraint(*[reduced_param_map.get(p, p) for p in con.values]))
      #At
    """

    # Alternatively, can assume that all preconditions and effects active and prune ones that don't matter

    print 'param:', parameters
    print 'static:', static
    print 'pre:', preconditions
    print 'eff:', effects

  def to_sas(self, state_vars):
    values = set(val for con in self.constraints for val in con.values)
    params = filter(lambda v: isinstance(v, Parameter), values)
    pre_params = filter(lambda p: p.pmap == x1, params)
    eff_params = filter(lambda p: p.pmap == x2, params)

    assignments = {}

    # Maybe just make the full STRIPS operator with something
    # Convert parameters
    # Cancel contradicting assignment effects

    reduced_param_map = get_param_map(self.constraints)
    if reduced_param_map is None:
      self.infeasible = True
      return None

    for con in self.constraints:
      if con.constraint == Eq:
        a, b = con.values
        if isinstance(a, Parameter) and isinstance(b, Parameter):
          equalities.add((a, b))
          equalities.add((b, a))
          if a.var == b.var:
            pass
        if isinstance(a, Parameter) and not isinstance(b, Parameter):
          assignments[a] = b
        if not isinstance(a, Parameter) and isinstance(b, Parameter):
          assignments[b] = a
        if not isinstance(a, Parameter) and not isinstance(b, Parameter):
          if a != b:
            self.infeasible = True
            print 'Warning! Infeasible transition'

    print pre_params
    print eff_params

    return params
  def __repr__(self):
    return '%s(%s)'%(self.name, ','.join(repr(value) for value in self.constraints))

#####################################

CONF, POSE, BOOL, TRAJ = Type(), Type(), Type(), Type()

r = 'rob'
h = 'hold'
t = 'traj'
objects = ['a', 'b']

state_vars = dict([(r, CONF), (h, BOOL)] + [(o, POSE) for o in objects])
control_vars = dict([(t, TRAJ)])

# Motion = Constraint() # NOTE - types are really just for safety more than anything...
# Traversable = Constraint()
# HoldTraversable = Constraint()
# Stable = Constraint()
# Grasp = Constraint()
# Kin = Constraint()

Motion = Constraint(name='Motion') # NOTE - types are really just for safety more than anything...
Traversable = Constraint(name='Traversable')
HoldTraversable = Constraint(name='HoldTraversable')
Stable = Constraint(name='Stable')
Grasp = Constraint(name='Grasp')
Kin = Constraint(name='Kin')

# I could make this get and set in order to make this more clear
#def x1(var): return ('x1', var)
#def x2(var): return ('x2', var)
#def u(var): return ('u', var)

# What if I wanted to do a more general CSP formulation?
# I should be able to get this by using no state vars but control vars
# i.e. one step plan but you have to fill in some parameters
# NOTE - just derive set of variables from the mentioned variables
# NOTE - I could automatically deduce a type by seeing if a variable is restricted to be a config in each thing
# What if multiple choices?

class Pick(Transition):
  def __init__(self, o, objects):
    super(Pick, self).__init__([
    Stable(x1[o]), Grasp(x2[o]),
    Kin(x2[o], x1[r], x1[o]),
    Eq(x1[r], x2[r]),
    Eq(x1[h], None), Eq(x2[h], o)] + [
      Eq(x1[ob], x2[ob]) for ob in objects if ob != o],
    name='Pick_%s'%o)

class SparseTransition(Transition):
  def __init__(self, constraints, out_vars, state_vars, name=None):
    # TODO - check out_vars is a superset of the constraint variables
    super(SparseTransition, self).__init__(constraints +
      constraints + [Eq(x1[v], x2[v]) for v in state_vars if v not in out_vars],
      name=name)

# TODO - support explicitly allowing variables to have parameters within them to make the resulting representation more compact

#from api import Eq
constraints = [Motion, Traversable, Kin, Stable, Grasp]
transitions = []
#for o in [None] + objects:
#  transitions.append([ # Move
#    Motion(x1(r), u(t), x2(r)]),
#    Eq(x1(h), x2(h), None)] + reduce([
#      [Traversable(u(t), o, x1[ob]),
#        Eq(x1[ob], x2[ob])] for ob in objects]))
# for o in objects:
#   transitions.append([ # Pick
#     Stable(x1(o)), Grasp(x2(o)),
#     Kin(x2(o), x1(r), x1(o)),
#     Eq(x1(r), x2(r)),
#     Eq(x1(h), None), Eq(x2(h), o)] + [
#       Eq(x1(ob), x2(ob)) for ob in objects if ob != o])
#   transitions.append([ # Place
#     Grasp(x1(o)), Stable(x2(o)),
#     Kin(x1(o), x1(r), x2(o)),
#     Eq(x1(r), x2(r)),
#     Eq(x2(h), o), Eq(x1(h), None)] + [
#       Eq(x1(ob), x2(ob)) for ob in objects if ob != o])


ob = Parameter(None, 'ob')
transitions.append(Transition([ # Move
  Motion(x1[r], u[t], x2[r]),
  Eq(x1[h], x2[h]), Eq(x1[h], None)] + reduce(add, [
    [Traversable(u[t], ob, x1[ob]), # NOTE - I have a non-variable in the constraint here...
      Eq(x1[ob], x2[ob])] for ob in objects]), name='move'))
# NOTE - I need to distinguish between universal and existential quantifiers

# I think I should just represent these as logical statements. That can be combined in some way
# Translate into DNF
# NOTE - make samplers that achieve values that satisfy a literal (Not or regular)


# Quantifiers should be only over discrete things



# NOTE - are variables parameter names then?
# Is there a difference between a parameter and variable?

xi = ParameterMap('xi')
xg = ParameterMap('xg')


# initial = And(
#   Eq(xi() = ),
#   Eq(),
#   Eq(),
# )

# Exists over time itself?
# Maybe I need a forall over time, so this is just a condition on the full plan
# We are making the markov assumption though

transition = Or(
  And(Motion(x1[r], u[t], x2[r]),
    Eq(x1[h], x2[h]), Eq(x1[h], None),
    #Eq(x1[h], x2[h], None),
    ForAll(ob, And(Eq(x1[ob], x2[ob]),
      Traversable(u[t], ob, x1[ob])))),
  Exists(o, Or( # NOTE - why do I make this a parameter but not the forall? Existentials are naturally parameters
    And(Motion(x1[r], u[t], x2[r]),
      Eq(x1[h], x2[h]), Eq(x1[h], o),
      ForAll(ob, Eq(x1[ob], x2[ob])),
      ForAll(ob, And(Not(Eq(ob, o), HoldTraversable(u[t], ob, x1[ob], o, x1[o])))),
    And(Stable(x1[o]), Grasp(x2[o]),
      Kin(x2[o], x1[r], x1[o]),
      Eq(x1[r], x2[r]),
      Eq(x1[h], None), Eq(x2[h], o),
        ForAll(ob, Or(Eq(ob, o), Eq(x1[ob], x2[ob])))) # This should be an Or I think...
    ))))

#transition = Exists(t, Forall(t))


class Parameter(object): # A decision variable or object
  def __init__(self, name, type=None):
    self.name = name
    self.type = type

# TODO - variable vs temporal variable?

# Variable is just the tuple values

# TODO - how do we specify types

class Variable(object): # NOTE - variables implicitly have a parameter in them which they refer to
  def __init__(self, *values): # Values is a list of values/parameters
    self.values = tuple(values)
    # NOTE - include a name for the parameter that is created?
    # This is really just a shorthand for a set of parameters

class X1(Variable):
  def __init__(self, *values):
    #super(X1, self).__init__('x1', *values)
    super(X1, self).__init__(*values)
    # The whole point of X1 anyways is to just signal how it relates in preconditions and effects
  @staticmethod
  def __getitem__(item):
    pass # TODO - could just do this

class XI(Variable): pass
class X1(Variable): pass
class U(Variable): pass
class X2(Variable): pass
class XG(Variable): pass


# NOTE - in another time, I could have constraints that extend temporally involving variables not adjacent on the plan

# NOTE - part of the strangeness is the variables which change over time and the decision variables


# Var('x', i, 'var')


# Decision variable = parameter

# Constraint logic programming
# Are logical objects variable, parameters, or what?
# Maybe I just simply say that they are shorthand for




transitions.append(Transition([ # Move
  Motion(x1[r], u[t], x2[r]),
  Eq(x1[h], x2[h]), Eq(x1[h], None)] + reduce(add, [
    [Traversable(u[t], ob, x1[ob]), # NOTE - I have a non-variable in the constraint here...
      Eq(x1[ob], x2[ob])] for ob in objects]), name='move'))
for o in objects:
  transitions.append(Transition([ # MoveHolding
    Motion(x1[r], u[t], x2[r]),
    Eq(x1[h], x2[h]), Eq(x1[h], o)] + \
    [Eq(x1[ob], x2[ob]) for ob in objects] + \
    [HoldTraversable(u[t], ob, x1[ob], o, x1[o]) for ob in objects if ob != o], name='moveh_%s'%o))
for o in objects:
  transitions.append(Transition([ # Pick
    Stable(x1[o]), Grasp(x2[o]),
    Kin(x2[o], x1[r], x1[o]),
    Eq(x1[r], x2[r]),
    Eq(x1[h], None), Eq(x2[h], o)] + [
      Eq(x1[ob], x2[ob]) for ob in objects if ob != o], name='pick_%s'%o))
  transitions.append(Transition([ # Place
    Grasp(x1[o]), Stable(x2[o]),
    Kin(x1[o], x1[r], x2[o]),
    Eq(x1[r], x2[r]),
    Eq(x2[h], o), Eq(x1[h], None)] + [
      Eq(x1[ob], x2[ob]) for ob in objects if ob != o], name='place_%s'%o))

tests = [
  Test(Traversable),
  Test(HoldTraversable),
]

samplers = [
  Sampler(Grasp),
  Sampler(Stable),
]

cond_samplers = [
  CondSampler(Kin, inputs=(0, 2)),
  CondSampler(Motion, inputs=(0, 2)),
]

# NOTE - should I have samplers for Eq?

# TODO - conversion:
# - automatically combine variables
# - prune dominated constraints that are implied others
# - separate out axioms
# - variable arguments
# - lifted form of operators

print state_vars
print control_vars
print constraints
print
for transition in transitions:
  print transition
  print transition.to_strips(state_vars)
  #print transition.to_sas(state_vars)
  print

#####################################

STATE, CONTROL = Type(), Type()

sv, cv = 0, 0
state_vars = {sv: STATE}
control_vars = {cv: CONTROL}

ValidControl = Constraint(name='dynamics')
ValidTransition = Constraint(name='dynamics')

transitions = [
  Transition([
    #ValidControl(x1[sv],u[cv]),
    ValidTransition(x1[sv],u[cv],x2[sv])
  ])
]

p = ParameterMap('p')

# Forward Model
cond_samplers = [
  CondSampler(ValidTransition, inputs=(0, 1), domain=[ValidControl]),
  #CondSampler(ValidTransition, inputs=(0,)),
  CondSampler(ValidControl, inputs=(0,)),

  CondSampler([ValidTransition(p['x1'], p['u'], p['x2'])],
               inputs=(p['x1'], p['u']), domain=[ValidControl(p['x1'], p['u'])]),

]
# The necessary conditions are the domain of the sampler
# Make the sampler just be a relation rather than an function to sets






# NOTE - in the forward model, the dynamics are written this way






# TODO - marginalized samplers?
# What if you just want to sample value uses of u from x1 but no other factoring

# The marginalized constraint is a necessary condition for the full one


# TODO - why don't we just have samplers that produce random things?
# The certification stuff is kind of interesting in general
# It makes a promise, but allows us to actually do interesting things
# What if you are passed in "invalid" values to a generator? What should it do?
