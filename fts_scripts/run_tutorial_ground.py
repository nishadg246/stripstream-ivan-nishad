#!/usr/bin/env python2

from itertools import product
from collections import deque

import copy

# https://stackoverflow.com/questions/25456448/is-it-possible-to-override-the-assignment-operator-in-python
# https://www.python-course.eu/python3_magic_methods.php
# https://docs.python.org/3/reference/datamodel.html#special-method-names

def get_dict(d, k):
  return d.get(k, k)

class Argument(object):
  def __init__(self, var, num=1):
    self.var = var
    self.num = num
  def __repr__(self):
    if self.num == 1:
      return '?{}'.format(self.var)
    return '?{}{}'.format(self.var, self.num)

class Parameter(object):
  def __init__(self, s, var):
    self.set = s
    self.var = var
  #def __eq__(self, parameter):
  #  # if Parameter is not a Paramter, make it a constant
  #  return Equal(self, parameter)
  #def __sub__(self):
  #  pass
  def __eq__(self, other):
    return type(self) == type(other) and \
           (self.set == other.set) and \
           (self.var == other.var)
  def __ne__(self, other):
    return not (self == other)
  def __hash__(self):
    return hash((self.set, self.var))
  def __repr__(self):
    return '{}[{}]'.format(self.set, self.var)


class FixedParameter(Parameter):
  def __init__(self, s, var, value):
    super(FixedParameter, self).__init__(s, var)
    self.value = value
  def __repr__(self):
    #return '{}[{}]={}'.format(self.set, self.var, self.value)
    return '{}'.format(self.value)

class VariableSet(object):
  def __init__(self, name):
    self.name = name
  def __getitem__(self, var):
    return Parameter(self, var)
  #def __setitem__(self, var, value):
  #  return Equal(self[var], value)
  #def __call__(self, *args):
  #  return self.__getitem__(args)
  def __repr__(self):
    return self.name

class ConstantSet(VariableSet):
  def __getitem__(self, (var, value)):
    return FixedParameter(self, var, value)

I = VariableSet('inp')
O = VariableSet('out')
X = VariableSet('X')
nX = VariableSet('nX')
U = VariableSet('U')
C = ConstantSet('C')

class Generator(object):
  def __init__(self, sampler, input_values, generator):
    self.sampler = sampler
    assert len(self.sampler.inputs) == len(input_values)
    self.input_values = input_values
    self.generator = generator
    self.enumerated = False
  # def __next__(self, **kwargs):
  #   assert not self.enumerated
  #   try:
  #     output_list = next(self.generator)
  #     assert type(output_list) in (list, tuple)
  #     return output_list
  #   except StopIteration:
  #     self.enumerated = True
  #     return []
  @property
  def greedy(self):
    return self.sampler.greedy
  def next(self, **kwargs):
    output_list = []
    for output_values in next(self.generator):
      assert len(self.sampler.outputs) == len(output_values)
      for i, val in enumerate(output_values):
        var = self.sampler.outputs[i]
        output_list.append(Value(C[var, val]))
      for atom in self.sampler.constraints:
        variables = []
        arguments = []
        for param in atom.parameters:
          if param.set == I:
            variables.append(self.sampler.inputs[param.var])
            arguments.append(self.input_values[param.var])
          elif param.set == O:
            variables.append(self.sampler.outputs[param.var])
            arguments.append(output_values[param.var])
          else:
            raise ValueError()
        constraint = Constraint(atom.constraint.constraint_type, variables)
        output_list.append(constraint(*arguments))
    return output_list
  def __eq__(self, other):
    return type(self) == type(other) and \
           (self.sampler == other.sampler) and \
           (self.input_values == other.input_values)
  def __ne__(self, other):
    return not (self == other)
  def __hash__(self):
    return hash((type(self), self.sampler, self.input_values))
  def __repr__(self):
    return 'g({})=>{}'.format(','.join(map(repr, self.input_values)),
                             self.sampler.outputs)

# TODO: sampler could potentially not have state
class Sampler(object):
  names = set()
  def __init__(self, inputs, domain, outputs, constraints, generator_fn, greedy=False):
    self.inputs = tuple(inputs)
    self.domain = tuple(domain)
    # TODO: add smaller constraints if desired
    self.outputs = tuple(outputs)
    self.constraints = tuple(constraints)
    self.generator_fn = generator_fn
    self.greedy = greedy
  def __call__(self, input_values, **kwargs):
    return Generator(self, input_values, self.generator_fn(*input_values))
  def __repr__(self):
    return 'f({})=>{}'.format(','.join(map(repr, self.inputs)),
                             self.outputs)

class Constraint(object):
  def __init__(self, constraint_type, variables):
    self.constraint_type = constraint_type
    self.variables = tuple(variables)
  def __call__(self, *parameters):
    return Atom(self, parameters)
  def __eq__(self, other):
    return type(self) == type(other) and \
           (self.constraint_type == other.constraint_type) and \
           (self.variables == other.variables)
  def __ne__(self, other):
    return not (self == other)
  def __hash__(self):
    return hash((self.constraint_type, self.variables))
  def __repr__(self):
    return '{}({})'.format(self.constraint_type, ','.join(map(repr, self.variables)))

class Atom(object):
  def __init__(self, constraint, parameters):
    self.constraint = constraint
    self.parameters = tuple(parameters)
  def items(self):
    return zip(self.constraint.variables, self.parameters)
  def replace(self, new_from_old):
    return self.constraint(*[get_dict(new_from_old, p) for p in self.parameters])
  def __eq__(self, other):
    return type(self) == type(other) and \
           (self.constraint == other.constraint) and \
           (self.parameters == other.parameters)
  def __ne__(self, other):
    return not (self == other)
  def __hash__(self):
    return hash((self.constraint, self.parameters))
  def __repr__(self):
    return '{}({})'.format(self.constraint.constraint_type, ','.join(map(repr, self.parameters)))

class ConstraintType(object):
  names = {}
  def __init__(self, name=None):
    self.name = name
  def __call__(self, *parameters):
    variables = [p.var for p in parameters]
    constraint = Constraint(self, variables)
    values =  [p.value if isinstance(p, FixedParameter) else p for p in parameters]
    return constraint(*values)
  def __repr__(self):
    return self.name

#def Equal(a, b):
#  assert isinstance(a, Parameter)
#  if not isinstance(b, Parameter):
#    b = Parameter(C, a.var)
#  return ('equal', a, b)

class ConstraintTypeConstants(ConstraintType):
  def __call__(self, *values):
    parameters = filter(lambda v: isinstance(v, Parameter), values)
    assert parameters
    variable = parameters[0].var
    # TODO: assert only one

    new_parameters = []
    for value in values:
      if isinstance(value, Parameter):
        new_parameters.append(value)
      else:
        new_parameters.append(C[variable, value]) # TODO: include the value
    return super(ConstraintTypeConstants, self).__call__(*new_parameters)

Equal = ConstraintTypeConstants(name='Equal')
Value = ConstraintTypeConstants(name='Value')

class ActionInstance(object):
  def __init__(self, lifted, arguments, conditions, effects):
    self.lifted = lifted
    self.arguments = arguments
    self.conditions = conditions
    self.effects = effects
    self.cost = 1
  def __repr__(self):
    return '<{},{}>'.format(self.conditions, self.effects)

class Action(object):
  def __init__(self, state_vars, atoms):
    # Get instances from problem
    # TODO: different versions of instantiation and count number of actions

    equalities = filter(lambda a: a.constraint.constraint_type is Equal, atoms)
    constraints = filter(lambda a: a not in equalities, atoms)
    parameters = set()
    for atom in constraints:
      parameters |= set(atom.parameters)
    self.control_vars = set()
    for param in parameters:
      if param.set == U:
        self.control_vars.add(param.var)

    self.value_from_param = {}
    fixed_vars = set()
    for equal in equalities:
      p1, p2 = equal.parameters
      if (not isinstance(p1, Parameter)) and (not isinstance(p2, Parameter)):
        raise RuntimeError()
      elif not isinstance(p1, Parameter):
        self.value_from_param[p2] = p1
      elif not isinstance(p2, Parameter):
        self.value_from_param[p1] = p2
      elif ((p1.set == X) and (p2.set == nX)) or \
           ((p1.set == nX) and (p2.set == X)):
        assert p1.var == p2.var
        fixed_vars.add(p1.var)
      else:
        raise ValueError(equal)

    self.conditions = {}
    self.effects = {}
    for var in sorted(state_vars):
      p1 = X[var]
      if p1 in self.value_from_param:
        self.conditions[var] = self.value_from_param[p1]
      elif p1 in parameters:
        arg = Argument(var)
        self.value_from_param[p1] = arg
        self.conditions[p1.var] = arg

      p2 = nX[var]
      if p2 in self.value_from_param:
        self.effects[var] = self.value_from_param[p2]
      elif var in fixed_vars:
        if p1 in parameters:
          self.value_from_param[p2] = self.value_from_param[X[var]]
      else:
        arg = Argument(var, 2)
        self.value_from_param[p2] = arg
        self.effects[p2.var] = arg

    for var in sorted(self.control_vars):
      p = U[var]
      assert p not in self.value_from_param
      #if p not in self.value_from_param:
      self.value_from_param[p] = Argument(var)

    self.arguments = tuple(arg for arg in set(self.value_from_param.values())
                      if isinstance(arg, Argument))
    self.constraints = tuple(a.replace(self.value_from_param) for a in constraints)

    # TODO: clean this up later

  def print_self(self):
    print
    print self.name
    print self.control_vars
    print self.value_from_param
    print self.arguments
    print self.conditions
    print self.effects
    print

  def get_instances(self, variables, constraints):
    domains = [variables.get(arg.var, []) for arg in self.arguments]
    for values in product(*domains):
      value_from_arg = dict(zip(self.arguments, values))
      atoms = [a.replace(value_from_arg) for a in self.constraints]
      if any(a not in constraints[a.constraint] for a in atoms):
        continue
      conditions = {var: get_dict(value_from_arg, arg) for var, arg in self.conditions.items()}
      effects = {var: get_dict(value_from_arg, arg) for var, arg in self.effects.items()}
      yield ActionInstance(self, values, conditions, effects)
  def __repr__(self):
    return '<{}, {}, {}, {}>'.format(self.arguments, self.constraints,
                                     self.conditions, self.effects)

class FTSProblem(object):
  def __init__(self, initial_state, goal_clause, transitions, samplers):
    self.initial_state = initial_state
    self.goal_clause = goal_clause
    self.transitions = transitions
    self.samplers = samplers


#from sas.downward2 import write_file
from sas.downward2 import solve_sas
from sas.utils import Problem

# The third line specifies the axiom layer of the variable.

class State(object):
  def __init__(self, values):
    self.values = values

class Goal(object):
  def __init__(self, conditions):
    self.conditions = conditions

# TODO: how quickly should I convert to an index?

GOAL_VAR = 'goal'
GOAL_CLAUSE = 'goal'

def transform_goal(problem):
  initial = copy.copy(problem.initial_state)
  initial[GOAL_VAR] = False
  goal = {GOAL_VAR: True}

  for atom in problem.goal_clause:
    for param in atom.parameters:
      assert (not isinstance(param, Parameter)) or (param.set == X, C)
  clauses = [list(problem.goal_clause) +
                 [Equal(X[var], nX[var]) for var in problem.initial_state] +
                 [Equal(nX[GOAL_VAR], True)]]
  for clause in problem.transitions:
    clauses.append(list(clause) + [Equal(X[GOAL_VAR], nX[GOAL_VAR])])
  return initial, goal, clauses

def incremental(problem):
  initial, goal, clauses = transform_goal(problem)
  print 'Initial:', initial
  print 'Clauses:', clauses

  # TODO: could also include a default value

  generators = set()

  domain_from_sampler = {}
  for sampler in problem.samplers:
    domain_from_sampler[sampler] = list(sampler.domain)
    parameters = {p for a in sampler.domain for p in a.parameters}
    for index in xrange(len(sampler.inputs)):
      param = I[index]
      if param not in parameters:
        domain_from_sampler[sampler].append(Value(param))

  variable_values = {}
  constraint_values = {}
  greedy_queue = deque()
  lazy_queue = deque()

  def add_generator(generator):
    if generator not in generators:
      generators.add(generator)
      if generator.greedy:
        greedy_queue.append(generator)
        sample_greedy()
      else:
        lazy_queue.append(generator)

  def update_generators(new_atom):
    for sampler in problem.samplers:
      # TODO: do this incrementally to prune samples
      constraints = [Constraint(a.constraint.constraint_type,
                                [sampler.inputs[p.var] for p in a.parameters])
                     for a in domain_from_sampler[sampler]]
      for i, constraint in enumerate(constraints):
        if constraint != new_atom.constraint:
          continue
        values = [constraint_values[c] if i != j else {new_atom}
                  for j, c in enumerate(constraints)]
        for combo in product(*values):
          assignment = {}
          for t, a in zip(domain_from_sampler[sampler], combo):
            for p, v in zip(t.parameters, a.parameters):
              if assignment.get(p.var, v) == v:
                assignment[p.var] = v
              else:
                break
            else:
              continue
            break
          else:
            input_values = tuple(assignment[i] for i in xrange(len(assignment)))
            add_generator(sampler(input_values))

  def add_atom(new_atom):
    ground = True
    for var, value in new_atom.items():
      if isinstance(value, Parameter):
        ground = False
      else:
        if var not in variable_values:
          variable_values[var] = set()
        if value not in variable_values[var]:
          variable_values[var].add(value)
          add_atom(Value(C[var, value]))
    if (new_atom.constraint.constraint_type == Equal) or (not ground):
      return
    if new_atom.constraint not in constraint_values:
      constraint_values[new_atom.constraint] = set()
    if new_atom in constraint_values[new_atom.constraint]:
      return
    constraint_values[new_atom.constraint].add(new_atom)
    update_generators(new_atom)

  for var, val in (initial.items() + goal.items()):
    add_atom(Value(C[var, val]))
  for clause in clauses:
    for atom in clause:
      add_atom(atom)

  state_vars = tuple(sorted(initial.keys()))
  control_vars = tuple(sorted(filter(lambda v: v not in state_vars, variable_values)))
  actions = [Action(state_vars, clause) for clause in clauses]
  print actions
  # TODO: name the trajectory variable_values using the type of movement

  #for sampler in problem.samplers:
  #  input_values = tuple(variable_values[var] for var in sampler.inputs)
  #  for combo in product(*input_values):
  #    for atom in sampler(combo):
  #      add_atom(atom)

  print 'State variable_values:', state_vars
  print 'Control variable_values:', control_vars
  print 'Variables:', variable_values
  print 'Constraints:', constraint_values
  print

  def sample_greedy():
    while greedy_queue:
      generator = greedy_queue.popleft()
      try:
        for atom in next(generator):
          add_atom(atom)
        greedy_queue.append(generator)
      except StopIteration:
        pass

  frequency = 10
  while True:
    for i in xrange(frequency):
      if not lazy_queue:
        break
      generator = lazy_queue.popleft()
      try:
        for atom in next(generator):
          add_atom(atom)
        lazy_queue.append(generator)
      except StopIteration:
        pass

    action_instances = []
    axioms_instances = []
    for action in actions:
      action_instances += list(action.get_instances(variable_values, constraint_values))
    print action_instances

    solution = solve_sas(Problem(State(initial), Goal(goal), action_instances, axioms_instances))
    if solution is None:
      continue

      return None
    plan = solution[:-1]
    print
    print plan
    for i, instance in enumerate(plan):
      lifted = instance.lifted
      value_from_arg = dict(zip(lifted.arguments, instance.arguments))
      print (i+1), {var: get_dict(value_from_arg, lifted.value_from_param[U[var]])
                    for var in lifted.control_vars}

    return plan
  return None

def create_problem():
  """
  Creates the 1D task and motion planning FTSProblem problem.

  :return: a :class:`.FTSProblem`
  """

  blocks = ['other%i'%i for i in range(2)]
  num_poses = pow(10, 10)

  initial_config = 0 # the initial robot configuration is 0
  initial_poses = {block: i for i, block in enumerate(blocks)} # the initial pose for other i is i

  goal_poses = {block: i+1 for i, block in enumerate(blocks)} # the goal pose for other i is i+1

  ####################

  # TODO: could just use integers

  Q, T = 'Q', 'T'
  P = lambda b: '{}_P'.format(b)
  G = lambda b: '{}_G'.format(b)
  # TODO: could make the state variables integers and just use this to make to them
  #R = 0

  print 'block''p-' # Automatically concatenate these?
  # Could also just use plus


  #Motion = lambda q1, t, q2: ('motion', q1, t, q2)
  Motion = ConstraintType(name='Motion')

  # Still want to list some satisfied constraints upfront

  # TODO: do I want to use tests here?

  ##########

  state_vars = [Q]
  control_vars = [T]

  transitions = [[Motion(X[Q], U[T], nX[Q])]]

  def motion_plan(q1, q2):
    path = (q1, q2)
    yield [(path,)]

  #samplers = [Sampler([Motion(Q, T, Q)], gen=lambda q1, q2: [[((q1, q2),)]], inputs=[Q, Q2])]
  # Include the trajectories such that the end result is just a sequence of motion plans (include gripper)

  # TODO: input and output variables
  # TODO: could provide a mapping from names to input value types
  samplers = [Sampler(inputs=(Q, Q), domain=[],
                      #inputs={'q1': Q, 'q2': Q}, domain=[],
                      #outputs={'t', T}, # Would have to pass and return these dictionaries
                      outputs=(T,), constraints=[Motion(I[0], O[0], I[1])],
                      generator_fn=motion_plan, greedy=False)]
  # Only likely to have two values at a time anyways...

  #initial_state = [Eq(X[Q], initial_config)]
  #initial_state = [X[Q] == initial_config]
  initial_state = {Q: initial_config}

  # TODO: should I just make this a map
  #print X[Q]

  goal_constraints = [Equal(X[Q], 10)]

  print initial_state


  problem = FTSProblem(initial_state, goal_constraints, transitions, samplers)

  incremental(problem)

  return None




  # Samplers need output variables
  # Can infer input variables (but not necessarily order)
  # Constraints just need to mention which number input/output they are using
  # Raw constraints themselves don't
  # Do I want to construct objects that use

  # TODO: just do initial values with greedy streams

  ##########

  # TODO: should be able to do this all programmatically
  # TODO: explain the flexibility when you move to PDDL
  # - lifting
  # - state size not fixed
  # - shared objects

  # TODO: explicitly separate into base and robot
  # TODO: make motion plans just sequences of actions
  # TODO: relate to a factoring into parts

  # TODO: sample several constraints for this paper
  # http://aima.cs.berkeley.edu/python/csp.py

  # TODO: can get the type from the name of objects

  # Automatically include a Eq for each types

  for b in blocks:
    state_vars += [P(b), G(b)]

    #LegalKin = ConType([b_p, Q])
    # TODO: need to do pairs of these
    #CollisionFree = ConType([b_p, POSE], test=lambda p1, p2: None in (p1, p2) or p1 != p2)
    # TODO: could also just make these tuples but what's the point of that

    #CollisionFree = Constraint((vars), )
    # TODO: need to handle constants within the inputs/outputs. Need to know the variable types then

    #none = C[P(b)]
    #true, false = C[G(b)]
    transition += [
      [LegalKin(X[P(b)], X[Q]), Equal(nX[P(b)], None), Equal(X[G(b)], False), Equal(nX[G(b)], True)] +
        [Equal(X[G(b2)], False) for b2 in blocks],
      [LegalKin(nX[P(b)], X[Q]), Equal(X[P(b)], None), Equal(X[G(b)], True), Equal(nX[G(b)], False)] +
        [CollisionFree(X[P(b2)], nX[P(b)]) for b2 in blocks]]

    # TODO: do I want to have separate variable entities or do I want to assume they are unique
    # TODO: I can just index the inputs and output variables and use integer indices for constraints

    samplers += [
      #Sampler([b_p], gen=lambda: ([(p,)] for p in xrange(num_poses)), inputs=[]),
      #Sampler([LegalKin(b_p, Q)], gen=lambda p: [[(p,)]] if p is not None else [], inputs=[b_p])
    ]

    # TODO: how do I want to handle Eq in the scheme of this?

    initial_state += [
      X[G(b)] == False,

      Equal(X[G(b)], False),
      Equal(X[P(b)], initial_poses[b])]
    if b in goal_poses:
      goal_constraints.append(Equal(X[P(b)], goal_poses[b]))

  #rename_variables(locals()) # Trick to make debugging easier

  return FTSProblem(state_vars, control_vars, transition, samplers,
                           initial_state, goal_constraints)

##################################################

def main():
  """
  Creates and solves the 1D task and motion planning FTSProblem problem.
  """

  constraint_problem = create_problem()
  print
  print constraint_problem

if __name__ == '__main__':
  main()