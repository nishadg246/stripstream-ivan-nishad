import sys
sys.path.append('..')

from stripstream.pddl.logic import And, Or, Exists, ForAll, Not #, Equal

# Maybe pre-specify variables and types together. This can also be how we give the domains for parameters
# Allow types to have a "go-to" name

class Type(object):
  def __init__(self, name=None, domain=None):
    self.name = name
    self.domain = domain # Pre-specify a finite domain for some of these

OBJECT = Type('object', None)

# NOTE - a finite domain is also nice to unify with the constraint satisfaction view

# Why don't I just have a generator for type then?

class Parameter(object): # A decision variable or object
  num = 0
  prefix = '?P%s'
  def __init__(self, type=None, name=None):
    self.type = type
    #self.name = name
    self.n = Parameter.num
    Parameter.num += 1
    self.name = name if name is not None else self.prefix%self.n
  def typed_pddl(self):
    return repr(self)
  def __repr__(self):
    return self.name # TODO - include type here?

  #typed_pddl = __repr__

# TODO - might just want to keep this a tuple
# class Variable(object): # State variable | control variable
#   def __init__(self, *items):
#     self.items = tuple(items)
#   def __repr__(self):
#     #if len(self.items) == 1:
#     #  return repr(self.items[0])
#     return 'V' + repr(self.items)
#   def __eq__(self, other):
#     return type(self) == type(other) and self.items == other.items
#   def __ne__(self, other):
#     return not(self == other)
#   def __hash__(self):
#     return hash((self.__class__, self.items))
#
# V = Variable

class Temporal(object):
  #def __init__(self, name=None):
  def __init__(self, name):
    self.name = name
  #def __getitem__(self, var):
  #  return TemporalVariable(self, var)
  def __getitem__(self, *var):
    return TemporalVariable(self, var)
  __call__ = __getitem__
  #def __getitem__(self, *values):
  #  var = Variable(*values)
  #  return TemporalVariable(self, var)
  def __repr__(self):
    return self.name

class TemporalVariable(object):
  # NOTE - implicitly defines a parameter for its value
  #def __init__(self, *items):
  #  self.var = Variable(items)
  def __init__(self, temp, var):
    self.temp = temp
    self.var = tuple(var)
  def __repr__(self):
    #return '%s[%s]'%(self.temp, self.var)
    #return '%s[%s]'%(self.temp, self.var.items)
    #return '%s[%s]'%(self.temp, ','.join(repr(item) for item in self.var.items))
    return '%s[%s]'%(self.temp, ','.join(repr(item) for item in self.var))
  def __eq__(self, other):
    return type(self) == type(other) and self.temp == other.temp and self.var == other.var
  def __ne__(self, other):
    return not(self == other)
  def __hash__(self):
    return hash((self.__class__, self.temp, self.var))

xs = Temporal('xs')
x1 = Temporal('x1')
u = Temporal('u') # Can think of controls being on a different "time" axis
x2 = Temporal('x2')
xg = Temporal('xg')

# Should I require that states and actions have different vars?

#I()
#X()

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

# Value vs Parameter = item

objects = ['o%s'%i for i in range(3)]

OBJ = Type(domain=objects)
CONF = Type()
POSE = Type()


Motion = ConstraintType(name='Motion') # NOTE - types are really just for safety more than anything...
Traversable = ConstraintType(name='Traversable')
HoldTraversable = ConstraintType(name='HoldTraversable')
Stable = ConstraintType(name='Stable')
Grasp = ConstraintType(name='Grasp')
Kin = ConstraintType(name='Kin')

constraints = [Motion, Traversable, Kin, Stable, Grasp]

r = 'rob'
h = 'hold'
t = 'traj'

#r = V('rob')
#h = V('hold')
#t = V('traj')


o = Parameter(type=OBJ)
#state_vars = ['r', o, ('o', o), 'h'] #+ [('stacked', o, o)]
#state_vars = {V(r): CONF, V(o): POSE, V(h): OBJECT}
state_vars = {(r,): CONF, (o,): POSE, (h,): OBJECT}

# NOTE - only need the variables (not temporal variables) for the initial and final states
initial = And(
  #Equal(XI(r), 1),
  #Equal(X1(o), 1), # Pass string or variable in here?
  #Equal(X1(V(o)), 1),
  Eq(xs[r], 1)
)

# Might be good to pass variable explicitly because we treat o as a value as well

goal = And(
  Eq(xs[r], 1)
)

# TODO - do I want to have quantifiers use multiple variables at once?
# TODO - should I name "actions" at all?
# NOTE - I could just make equality the de

def unchanged(var):
  return Eq(x1[var], x2[var])

ob = Parameter(type=OBJ)
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
  # TODO - how do I handle Eq(ob, o) in an abstract form?
  # We can do Eq(ob, o) because its finite and doesn't involve any variables

# ForAll([ob], ob != o, unchanged(ob)) # Include some kind of test here

# NOTE - maybe I have to factor then combine these in hindsight
# - I could make a SAS+ operator using pre/eff to define the parameters

# TODO - what prevents me from just making a single action using all of this?
# - I could, but then I wouldn't be able to separate preconditions and effects in a meaningful way

# Maybe I should just do SAS+

# What if the forall that had some condition such that the resulting actions where not usable via a common template
# The only reason we can do it is because the != and = are symmetric across the actions
# Could be more interesting equality relations involving multiple variables
# Nice thing about this form is that I don't have to identify the number of parameters


# NOTE - when I compile things back together, I have to put tests on the arguments such that the operator is only used
# by the correct parameters

#print transition


# ForAll([ob], Or(Eq(ob, o), Eq(x1[ob], x2[ob])) is dangerous because then it will split into many copies for whether
# They are equal or not
# TODO - should this function produce several values at a time, a single value, etc...


#####################################

from collections import defaultdict

def get_equality_map(constraints):
  equalities = set() # Form transitive clique and ensure that no conflicts
  for con in constraints:
    if con.con_type == Eq:
      a, b = con.items
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

  eq_map = {}
  for component in components:
    #params = sorted(filter(lambda v: isinstance(v, Parameter), component), key=repr) # Sort to ensure same order
    values = filter(lambda item: not isinstance(item, Parameter) and not isinstance(item, TemporalVariable), component)
    # TODO - assert that the types are the same...
    if len(values) != 0:
      assign = values[0]
      if any(val != assign for val in values):
        print 'Warning! Infeasible transition'
        return None
    else:
      assign = Parameter() # TODO - rename the parameter here
      #assign = params[0]
      #assign = params[0].var # TODO - maybe convert the name nicely or something
    for v in component:
      eq_map[v] = assign
  return eq_map

#####################################

#def At(var, v):
#  return (var, v)

At = ConstraintType(name='At')

def to_strips(constraints, state_vars): # NOTE - need control variables as well
  eq_map = get_equality_map(constraints)
  print eq_map

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

  tform = lambda p: eq_map.get(p, p)

  parameters = {tform(x1[var]) for var in state_vars if isinstance(tform(x1[var]), Parameter)} | \
    {tform(x2[var]) for var in state_vars if isinstance(tform(x2[var]), Parameter)} | \
    {val for val in eq_map.values() if isinstance(val, Parameter)} # Takes care of control

  # TODO - include "self" assignments
  static = set()
  preconditions = set()
  effects = set()

  # NOTE - we only do interesting things for equal vars. If not equal, then cannot remove any pre or eff

  # TODO - need to put the var name in
  equal_vars = {var for var in state_vars if tform(x1(*var)) == tform(x2(*var))}
  print 'eq:', equal_vars
  for var in state_vars:
    print var, x1(*var) in eq_map

  constraint_vars = set(reduce(add, [con.items for con in constraints if con.con_type != Eq]))
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
  for con in constraints:
    if con.con_type != Eq:
      vs = map(tform, con.items)
      static.add(con.con_type(*vs))
      parameters.update(vs)
  #parameters = {} # TODO - just update from here anyways
  parameters = filter(lambda p: isinstance(p, Parameter), parameters)

  # NOTE - the point of axioms is to reduce parameters, otherwise they doesn't help

  # For all parameters not mentioned in teh effects
  effect_vars = set()
  for effect in effects:
    if isinstance(effect, Not):
      effect = effect.formulas[0]
    effect_vars.update(effect.items)
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

  # Alternatively, can assume that all preconditions and effects active and prune ones that don't matter

  print 'param:', parameters
  print 'static:', static
  print 'pre:', preconditions
  print 'eff:', effects

#####################################

from itertools import product
from operator import add

# TODO - what if a temp_var is set with equality to a parameter?

def combine_or(entries, greedy=True):
  actions = []
  for args in entries:
    for cons, assign in to_dnf(*args):
      if greedy and len(cons) == 0:
        return [(cons, assign)] # Terminate early if no constraints!
      actions.append((cons, assign))
  return actions
  # Return the emptyset of parameters if one is always satisfiable

def combine_and(children):
  actions = []
  for combo in product(*[to_dnf(*args) for args in children]):
    con = reduce(add, (c for c, _ in combo))
    assign = dict(reduce(add, (a.items() for _, a in combo)))
    actions.append((con, assign))
  # TODO - I think I only want to remember existential quantifier choices
  return actions

# NOTE - I could first compile away quantifiers then do this

def to_dnf(f1, ext, uni):
  if isinstance(f1, Or):
    return combine_or([(f2, ext, uni) for f2 in f1.formulas])
  elif isinstance(f1, And):
    return combine_and([(f2, ext, uni) for f2 in f1.formulas])
  elif isinstance(f1, Exists):
    children = []
    for param in f1.args:
      assert param.type.domain is not None
      for value in param.type.domain:
        ext2 = ext.copy()
        ext2[param] = value
        children.append((f1.formula, ext2, uni))
    return combine_or(children)
  elif isinstance(f1, ForAll):
    children = []
    for param in f1.args:
      assert param.type.domain is not None
      for value in param.type.domain:
        uni2 = uni.copy()
        uni2[param] = value
        children.append((f1.formula, ext, uni2))
    return combine_and(children)
  elif isinstance(f1, Constraint):
    # TODO - identify cliques at this level?
    parameters = dict(ext.items() + uni.items())
    args = []
    for item in f1.items:
      if isinstance(item, TemporalVariable):
        args = None
        break
      elif isinstance(item, Parameter):
        assert item in parameters
        args.append(parameters[item])
      else:
        args.append(item)

    if args is not None and f1.con_type.test is not None:
      if f1.con_type.test(*args):
        return [([], ext)]
      return []
    #con = f1.con_type(*[uni.get(item, item) for item in f1.items])
    new_args = []
    for item in f1.items:
      if isinstance(item, Parameter) and item in uni:
        new_args.append(uni[item])
      elif isinstance(item, TemporalVariable):
        #var_args = [uni.get(i, i) for i in item.var.items]
        #var = V(*args)
        #new_args.append(TemporalVariable(item.temp, Variable(*var_args)))
        var = [uni.get(i, i) for i in item.var]
        new_args.append(TemporalVariable(item.temp, var))
        #new_args.append(item.temp[var_args])
        #new_args.append(item.temp[var])
      else:
        new_args.append(item)
    con = f1.con_type(*new_args)
    return [([con], ext)] # NOTE - only need ext
  elif isinstance(f1, Not):
    raise NotImplementedError('Negation is not implemented (as a design choice)')
  raise ValueError(f1)

#####################################

# NOTE - only variables mentioned in constraints are meaningful parts of the state
# NOTE - can interpret as infinitely many variables but only choose values for constrained ones
# TODO - how do I define types then?

def ground_variables(vars):
  ground_vars = {}
  for var, ty in vars.iteritems():
    domains = []
    #for item in var.items:
    for item in var:
      if isinstance(item, Parameter):
        assert item.type.domain is not None
        domains.append(item.type.domain)
      else:
        domains.append([item])
    for combo in product(*domains):
      #ground_vars[Variable(*combo)] = ty
      ground_vars[combo] = ty
  return ground_vars

#####################################

ground_state = ground_variables(state_vars)
print ground_state.keys()

stuff = to_dnf(transition, {}, {})
#print stuff
print len(stuff)

for cons, assign in stuff:
  print cons
  print assign
  print to_strips(cons, ground_state)
  print
