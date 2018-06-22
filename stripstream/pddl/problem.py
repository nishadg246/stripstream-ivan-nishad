from collections import Iterable, defaultdict
#from stripstream.algorithms.hierarchy.utils import AbsCondition
from stripstream.pddl.logic.connectives import And, Not, When
from stripstream.pddl.operators import STRIPSAction, STRIPSAxiom, Action, Axiom
from stripstream.pddl.logic.formulas import Formula
from stripstream.pddl.logic.quantifiers import ForAll
from stripstream.pddl.objects import Constant, Parameter
from stripstream.pddl.logic.atoms import Atom
from stripstream.utils import flatten

# TODO - could just pass locals to STRIPStreamProblem

class STRIPStreamProblem(object):
  def __init__(self, initial_atoms, goal_literals, operators, cond_streams, objects=[]):
    # TODO - warnings for improperly passed types here
    self.initial_atoms = list(initial_atoms)
    if isinstance(goal_literals, Atom):
      self.goal_literals = And(goal_literals)
    elif isinstance(goal_literals, Iterable):
      self.goal_literals = And(*goal_literals)
    elif isinstance(goal_literals, Formula): # or isinstance(goal_literals, AbsCondition):
      self.goal_literals = goal_literals
    else:
      raise ValueError(goal_literals)
    # TODO - assert goals aren't empty
    self.operators = operators
    self.cond_streams = cond_streams
    self.objects = objects
  def get_constants(self):
    objects = set(self.objects)
    for atom in self.initial_atoms:
      objects.update(atom.args)
    if isinstance(self.goal_literals, Formula):
      for atom in self.goal_literals.get_atoms():
        objects.update(atom.args)
    #elif isinstance(self.goal_literals, AbsCondition):
    #  for formula in self.goal_literals.conditions:
    #    for atom in formula.get_atoms():
    #      objects.update(atom.args)
    for operator in self.operators:
      for atom in operator.condition.get_atoms() | operator.effect.get_atoms():
        objects.update(atom.args)
    for cs in self.cond_streams:
      for atom in cs.conditions + cs.effects:
        objects.update(atom.args)
    return filter(lambda o: isinstance(o, Constant), objects)
  def get_functions(self):
    from stripstream.pddl.operators import get_function_atoms
    functions = set()
    for operator in self.operators:
      if isinstance(operator, Action):
        functions.update({atom.predicate for atom in get_function_atoms(operator)})
    return functions
  def get_fluent_predicates(self): # TODO - enforce consistency for these and make them a property
    from stripstream.pddl.operators import Action
    fluents = set()
    for operator in self.operators:
      if isinstance(operator, Action):
        fluents.update({atom.predicate for atom in operator.effect.get_atoms()})
    return fluents
  def get_derived_predicates(self):
    from stripstream.pddl.operators import Axiom
    return set(operator.effect.predicate for operator in self.operators if isinstance(operator, Axiom))
  def get_stream_predicates(self):
    return {atom.predicate for cs in self.cond_streams for atom in cs.conditions + cs.effects}
  def get_predicates(self):
    return {atom.predicate for op in self.operators for atom in op.condition.get_atoms() | op.effect.get_atoms()} | \
      {atom.predicate for cs in self.cond_streams for atom in cs.conditions+cs.effects} | \
      {atom.predicate for atom in self.initial_atoms} | \
      {atom.predicate for atom in self.goal_literals.get_atoms()}
  def get_static_predicates(self):
    return self.get_predicates() - self.get_fluent_predicates() - \
           self.get_derived_predicates() - self.get_stream_predicates()
  def __repr__(self):
    return 'STRIPStream Problem\n' \
           'Initial: %s\n' \
           'Goal: %s\n' \
           'Operators: %s\n' \
           'Streams: %s\n' \
           'Constants: %s'%(self.initial_atoms, self.goal_literals,
                            self.operators, self.cond_streams, self.objects)

##################################################

def is_strips(problem):
  return all(isinstance(op, STRIPSAction) or
             isinstance(op, STRIPSAxiom) for op in problem.operators) #and isinstance(self.goal_literals, list)

def convert_goal(problem, axiom=True):
  from stripstream.pddl.logic.predicates import NamedPredicate
  from stripstream.pddl.operators import Action, Axiom
  Goal = NamedPredicate('_goal')
  if axiom:
    operator = Axiom(effect=Goal(), condition=problem.goal_literals)
  else:
    operator = Action(name='_achieve_goal', parameters=[],
                      condition=problem.goal_literals, effect=Goal(), cost=None)
  operator.is_internal = True
  problem.operators.append(operator)
  problem.goal_literals = Goal()

def to_strips(problem): # NOTE - does not affect the goal
  constants = defaultdict(set)
  for const in problem.get_constants():
    constants[const.type].add(const)
  new_operators = []
  for operator in problem.operators:
    for strips in operator.to_strips(constants):
      new_operators.append(strips)
  problem.operators = new_operators

def new_problem(problem, initial_atoms):
  return STRIPStreamProblem(initial_atoms, problem.goal_literals,
                            problem.operators, problem.cond_streams, problem.objects)

##################################################

def substitute_axioms(condition, derived): # TODO - clean this up
  augmented = False
  for atom in condition.get_atoms():
    if atom.predicate in derived:
      axiom = derived[atom.predicate]
      condition.substitute(axiom.effect, axiom.condition)
      augmented = True
  if augmented:
    substitute_axioms(condition, derived) # Recursively applies

def replace_axioms(problem): # TODO - compile out axioms by making them effects
  derived = {}
  for op in problem.operators:
    if isinstance(op, Axiom):
      derived[op.effect.predicate] = op
  actions = []
  for action in problem.operators:
    if isinstance(action, Action):
      substitute_axioms(action.condition, derived)
      actions.append(action)
  problem.operators = actions
  substitute_axioms(problem.goal_literals, derived)

def convert_axioms_to_effects(problem): # NOTE - can always use the previous compilation process for some problematic axioms
  derived_predicates = problem.get_derived_predicates()
  fluent_predicates = problem.get_fluent_predicates()
  constants = defaultdict(set)
  for const in problem.get_constants():
    constants[const.type].add(const)

  mapping = []
  for op in problem.operators: # NOTE - can always just convert one
    if isinstance(op, Axiom):
      [conditions] = op.condition.dequantify(constants).get_literals() # TODO - more complicated if multiple ways to achieve
      assert not filter(lambda a: a.predicate in derived_predicates, conditions) # TODO - difficulty when levels of axioms
      print conditions
      [fluent] = filter(lambda a: a.predicate in fluent_predicates, conditions) # NOTE - could easily expand to conjunctive case
      others = filter(lambda a: a != fluent, conditions)
      mapping.append((fluent, others, op.effect))

  new_operators = []
  for op in problem.operators:
    if isinstance(op, Action):
      new_op = op.clone()
      new_operators.append(new_op)
      [literals] = new_op.effect.get_literals() # TODO
      for literal in literals:
        effect = literal.formula if isinstance(literal, Not) else literal
        for fluent, conditions, derived in mapping:
          if effect.predicate == fluent.predicate:
            free_params = set(flatten(map(lambda a: a.args, [derived] + conditions))) - set(fluent.args)
            param_map = dict(zip(fluent.args, effect.args))
            for i, param in enumerate(free_params): # Prevents conflicting names
              #param_map[param] = Parameter('_%s'%i, param.type) # NOTE - FF can't parse this
              param_map[param] = Parameter('l%s'%i, param.type)

            new_params = list(set(param_map.values()) - set(effect.args))
            new_derived = derived.instantiate(param_map)
            new_conditions = [cond.instantiate(param_map) for cond in conditions] # TODO - could put in the negated version of new_derived
            if isinstance(literal, Not):
              new_op.add_effects(ForAll(new_params, When(And(*new_conditions), new_derived)))
            else:
              new_op.add_effects(ForAll(new_params, When(And(*new_conditions), Not(new_derived)))) # Not necessary, but it could reduce the number of instances
              #op.add_effects(ForAll(new_params, Not(new_derived))) # One failure can break
  return new_operators