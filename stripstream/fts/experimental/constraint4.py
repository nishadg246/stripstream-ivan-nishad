import sys
sys.path.append('..')

from operator import add
from collections import defaultdict

def merge(*dict_args):
  result = {}
  for dictionary in dict_args:
    result.update(dictionary)
  return result

#####################################

# Maybe pre-specify variables and types together. This can also be how we give the domains for parameters
# Allow types to have a "go-to" name

class Type(object):
  num = 0
  prefix = 'T%s'
  def __init__(self, name=None, domain=None):
    self.domain = domain # Pre-specify a finite domain for some of these
    self.n = Type.num
    Type.num += 1
    self.name = name if name is not None else self.prefix%self.n
  def __repr__(self):
    return self.name

OBJECT = Type('object', None)

# NOTE - a finite domain is also nice to unify with the constraint satisfaction view

# Why don't I just have a generator for type then?

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
# V = Variable

class VariableSet(object):
  #def __init__(self, name=None):
  def __init__(self, name):
    self.name = name
  def __getitem__(self, var):
    return Parameter(self, var)
  #def __getitem__(self, *var):
  #  return Parameter(self, var)
  __call__ = __getitem__
  #def __getitem__(self, *values):
  #  var = Variable(*values)
  #  return TemporalVariable(self, var)
  def __repr__(self):
    return self.name

class Parameter(object):
  # NOTE - implicitly defines a parameter for its value
  #def __init__(self, *items):
  #  self.var = Variable(items)
  def __init__(self, temp, var):
    self.temp = temp
    self.var = var
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
  #def __eq__(self, other):
  #  pass # TODO - return a constraint here

xs = VariableSet('xs')
x1 = VariableSet('x1')
u = VariableSet('u') # Can think of controls being on a different "time" axis
x2 = VariableSet('x2')
xg = VariableSet('xg')

#####################################

# TODO - should I separate this from the sampler? I don't think I want (or need to require an explicit set)

class ConstraintForm(object):
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
    self.n = ConstraintForm.num
    ConstraintForm.num += 1
    #self.name = name if name is not None else self.__class__.__name__
    self.name = name if name is not None else self.prefix%self.n
  def __call__(self, *args, **kwargs):
    return Constraint(self, args)
  def __repr__(self):
    return self.name
  # TODO - hash this?

Eq = ConstraintForm(types=(OBJECT, OBJECT), test=lambda a, b: a == b, name='Eq') # TODO - should I allow ternary Eq...
InSet = ConstraintForm(types=(OBJECT, OBJECT), name='InSet')
# TODO - expose this as an explicit constraint form to do more stuff?

class Constraint(object):
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

#####################################

class Action(object):
  pass

#####################################

def get_equality_map(constraints):
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

  eq_map = {}
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
      eq_map[v] = assign
  return eq_map

# NOTE - no point converting to STRIPS here at all

def is_unaffected(con):
  return con.constraint == Eq and \
         isinstance(con.values[0], Parameter) and \
         isinstance(con.values[1], Parameter) and \
         con.values[0].var == con.values[1].var and \
         con.values[0].temp != con.values[1].temp

def is_identity(con):
  return con.constraint == Eq and con.values[0] == con.values[1]

#####################################

class Derived(object):
  def __init__(self, values):
    self.values = values

axioms = {}

def make_axiom(constraint, marginalize):
  # TODO - should I put constants in this
  # I think I should just hash the constraints to avoid repeats
  if constraint in axioms:
    return

  items = filter(lambda it: isinstance(it, Parameter) and it not in marginalize, constraint.values)
  effect = Derived(items)


  # TODO - make sure the naming works well
  print items

  # TODO - I need to make sure the variables match up correctly here...
  params = filter(lambda item: isinstance(item, Parameter), set(constraint.values))
  pre = {p.var: p for p in marginalize if p.temp != u}

  axioms[constraint] = effect # NOTE - the number of these depends on teh number of values...

  #eff = [constraint]

  print params
  print pre
  print effect.values
  print

  return None

#####################################

class Clause(object):
  num = 0
  prefix = 'C%s'
  def __init__(self, constraints, name=None):
    self.constraints = filter(lambda con: not is_identity(con), constraints)
    self.n = Clause.num
    Clause.num += 1
    #self.name = name if name is not None else self.__class__.__name__ # TODO - index these
    self.name = name if name is not None else self.prefix%self.n
  # TODO - detect if conflicting equality constraints so automatically not satisfiable
  # TODO - convert to simplified form
    self.infeasible = False
  def __repr__(self):
    return '%s(%s)'%(self.name, ','.join(repr(value) for value in self.constraints))
  def to_sas(self, state_vars, control_vars):
    eq_map = get_equality_map(self.constraints)
    for var in state_vars:
      for x in [x1, x2]:
        if x[var] not in eq_map:
          eq_map[x[var]] = x[var]
    for var in control_vars:
      if u[var] not in eq_map:
        eq_map[u[var]] = u[var]

    get_eq = lambda item: eq_map.get(item, item)

    print eq_map

    equal_vars = {var for var in state_vars if get_eq(x1[var]) == get_eq(x2[var])}
    print equal_vars
    constraint_vars = set(reduce(add, [con.values for con in self.constraints if con.constraint != Eq]))

    parameters = set()
    static = []
    pre_variables = set()
    for con in self.constraints:
      #if con.constraint != Eq:
      if not is_unaffected(con):
        for item in con.values:
          if isinstance(item, Parameter) and (item.temp == x1 or
              (item.temp == x2 and item.var in equal_vars)):
            pre_variables.add(item.var)
      if con.constraint != Eq:
        new_items = [get_eq(p) for p in con.values]
        parameters.update(new_items)
        static.append(con.constraint(*new_items))

    # TODO - what if the same equality constraint is used in preconditions
    # NOTE - only allow to avoid value if not mentioned in any constraint except its own equality constraint

    preconditions = {var: get_eq(x1[var]) for var in state_vars}
    effects = {var: get_eq(x2[var]) for var in state_vars}
    for var in equal_vars:
      del effects[var]
    for var in (set(state_vars.keys()) - pre_variables):
      del preconditions[var]
    parameters.update(set(preconditions.values() + effects.values()))
    parameters = filter(lambda item: isinstance(item, Parameter), parameters)

    print pre_variables
    print 'param:', parameters
    print 'stat:', static
    print 'pre:', preconditions
    print 'eff:', effects

    """
    # Maybe only parameters used once within a single constraint (value in set)
    # TODO - what about nested constraints?
    single_constraint = {p for p in set(parameters) - set(effects.values())
      if len(filter(lambda s: p in s.values, static)) == 1}
    for p in set(parameters) - set(effects.values()):
      #if p.temp != u: # NOTE - the u doesn't help at all (NOT TRUE!!!)
      involved_static = filter(lambda s: p in s.values, static)
      if len(involved_static) == 1:
        involved_params = filter(lambda item: isinstance(item, Parameter),
          set(reduce(add, (con.values for con in involved_static))))
        if len(involved_params) < len(parameters) and \
            not any(p != o and o in single_constraint for o in involved_params): # TODO - later combine
          print p, involved_static
          print make_axiom(involved_static[0], [p])
    """
    # x1['r'] [Motion(x1['r'],u['t'],x2['r'])] # TODO - what to do here?
    # TODO - only produce action instances that could be applicable under some operator instance

    return None

#####################################

# Maybe I should just make these constraints?
CONF, POSE, BOOL, TRAJ = Type(), Type(), Type(), Type()

r, h, t = 'r', 'h', 't'
objects = ['o%s'%i for i in range(3)]

state_vars = merge({r: CONF, h: BOOL}, {o: POSE for o in objects})
control_vars = {t: TRAJ}

Motion = ConstraintForm(name='Motion') # NOTE - types are really just for safety more than anything...
HoldingMotion = ConstraintForm(name='HoldingMotion')
Traversable = ConstraintForm(name='Traversable')
HoldTraversable = ConstraintForm(name='HoldTraversable')
Stable = ConstraintForm(name='Stable')
Grasp = ConstraintForm(name='Grasp')
Kin = ConstraintForm(name='Kin')

constraint_forms = [Motion, Traversable, HoldTraversable, Stable, Grasp, Kin]

# TODO - don't include the obj within these. This will be automatically "encoded" by the choice of variables

"""
clauses = [
  Clause([ # Move
    Motion(x1[r], u[t], x2[r]),
    Eq(x1[h], x2[h]), Eq(x1[h], None)] + reduce(add, [
      [Traversable(u[t], ob, x1[ob]), # NOTE - I have a non-variable in the constraint here...
        Eq(x1[ob], x2[ob])] for ob in objects]), name='move')]
for o in objects:
  clauses.append(Clause([ # MoveHolding
    #Motion(x1[r], u[t], x2[r]),
    HoldingMotion(x1[r], u[t], x2[r], o, x1[o]),
    Eq(x1[h], x2[h]), Eq(x1[h], o)] + \
    [Eq(x1[ob], x2[ob]) for ob in objects] + \
    #[HoldTraversable(u[t], ob, x1[ob], o, x1[o]) for ob in objects if ob != o], name='moveh_%s'%o))
    [Traversable(u[t], ob, x1[ob]) for ob in objects if ob != o], name='moveh_%s'%o))
for o in objects:
  clauses.append(Clause([ # Pick
    Stable(x1[o]), Grasp(x2[o]),
    Kin(x2[o], x1[r], x1[o]),
    Eq(x1[r], x2[r]),
    Eq(x1[h], None), Eq(x2[h], o)] + [
      Eq(x1[ob], x2[ob]) for ob in objects if ob != o], name='pick_%s'%o))
  clauses.append(Clause([ # Place
    Grasp(x1[o]), Stable(x2[o]),
    Kin(x1[o], x1[r], x2[o]),
    Eq(x1[r], x2[r]),
    Eq(x2[h], o), Eq(x1[h], None)] + [
      Eq(x1[ob], x2[ob]) for ob in objects if ob != o], name='place_%s'%o))
"""

# TODO - so how does equaling a constant work for things? Which variable is the "constant" variable
# NOTE - I think we are assuming that each variable in a constraint has domain it lives in
# Is this thing that we are holding an action then?
# Equal takes two variable of the same type (by default)
# So are x1[r], x2[r] just different parameters within the same variable domain
# What can the constraint infer from constants in it? Maybe they are only allowed in constraints

clauses = [
  Clause([ # Move
    Motion(x1[r], u[t], x2[r]),
    Eq(x1[h], x2[h]), Eq(x1[h], None)] + reduce(add, [
      [Traversable(u[t], x1[ob]), Eq(x1[ob], x2[ob])] for ob in objects]), name='move')]
for o in objects:
  clauses += [
    Clause([ # MoveHolding
      HoldingMotion(x1[r], u[t], x2[r], x1[o]),
      Eq(x1[h], x2[h]), Eq(x1[h], o)] + \
      [Eq(x1[ob], x2[ob]) for ob in objects] + \
      [Traversable(u[t], x1[ob]) for ob in objects if ob != o], name='moveh_%s'%o),
    Clause([ # Pick
      Stable(x1[o]), Grasp(x2[o]),
      Kin(x2[o], x1[r], x1[o]),
      Eq(x1[r], x2[r]),
      Eq(x1[h], None), Eq(x2[h], o)] + [
        Eq(x1[ob], x2[ob]) for ob in objects if ob != o], name='pick_%s'%o),
    Clause([ # Place
      Grasp(x1[o]), Stable(x2[o]),
      Kin(x1[o], x1[r], x2[o]),
      Eq(x1[r], x2[r]),
      Eq(x2[h], o), Eq(x1[h], None)] + [
        Eq(x1[ob], x2[ob]) for ob in objects if ob != o], name='place_%s'%o)]


print state_vars
print control_vars
print constraint_forms
print len(clauses)
print
for clause in clauses:
  print clause
  #print transition.to_strips(state_vars)
  print clause.to_sas(state_vars, control_vars)
  print
print axioms

#####################################

# NOTE - condition sampler for one type of constraint (no objects anywhere)

class CondSampler(object):
  # For a solution, just list which indices are cond
  def __init__(self, constraints, inputs=[], domain=[],
               gen=lambda *args: iter([])):
    self.constraints = constraints
    self.inputs = tuple(inputs)
    self.domain = domain # A conjunction (should I instead do DNF) of constraints on inputs
    self.gen = gen
  def __call__(self, *args):
    return self.gen(*args)

CS = CondSampler

# Do I want to do the types stuff here then?
cond_samplers = [
  CS([Motion(x1[r], u[t], x2[r])], inputs=[x1[r], x2[r]])
]
for o in objects:
  # I could instead just put variables here
  # This all depends on the variable as well...
  # NOTE - maybe I shouldn't have constraint forms? Maybe that's also lifted
  # TODO - is it strange if there are objects in constraints?
  cond_samplers += [
    CS([Stable(x1[o])]),
    CS([Grasp(x1[o])]), # TODO - do I need one for the reverse. What about the initial and final states?
    CS([Stable(x2[o])]),
    CS([Grasp(x2[o])]),

    CS([Kin(x1[o], x1[r], x2[o])],
       inputs=[x1[o], x2[o]],
       domain=[Grasp(x1[o]), Stable(x2[o])]),
    CS([Kin(x2[o], x1[r], x1[o])], inputs=[x2[o], x1[o]]), # Do I need one for the reverse then?
  ]

# A single type per each variable or do I want to share underlying data types
# NOTE - if I share underlying types, then I will have to explicitly give the name of the object again...

# NOTE - definitely don't want to make one per temporal thing or
# else need a new constraint and things per each xs, x1, x2, xg