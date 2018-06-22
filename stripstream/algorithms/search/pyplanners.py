from stripstream.pddl.logic.connectives import Not, And
from stripstream.pddl.logic.atoms import Atom
from stripstream.pddl.logic.operations import Initialize
from stripstream.algorithms.instantiation import smart_instantiate_operator
from stripstream.utils import INF
from stripstream.pddl.problem import STRIPStreamProblem
from stripstream.pddl.operators import Action, Axiom
from strips.utils import default_derived_plan
from strips.states import State, PartialState
from itertools import ifilterfalse

COST_TRANSFORM = 1

class PyLiteral(object):
  def __init__(self, literal, sign=True):
    if isinstance(literal, Not):
      self.atom, self.sign = literal.formulas[0], False
    else:
      self.atom, self.sign = literal, sign
  def __abs__(self):
    return self.__class__(self.atom)
  #def __neg__(self): # TODO - cannot override not?
  def __invert__(self):
    return self.__class__(self.atom, not self.sign)
  def __eq__(self, other):
    return type(self) == type(other) and self.atom == other.atom and self.sign == other.sign
  def __ne__(self, other):
    return not(self == other)
  def __hash__(self):
    return hash((self.__class__, self.atom, self.sign))
  def __repr__(self):
    return repr(self.atom) if self.sign else '(not ' + repr(self.atom) + ')'
  __str__ = __repr__

class PyOperator(object):
  def __init__(self, lifted, args, conditions, effects):
    self.lifted = lifted
    self.args = args
    self.conditions = map(PyLiteral, conditions) # TODO - maybe filter before expansion
    self.effects = map(PyLiteral, effects)
    self._hash = None
  def __contains__(self, state):
    return all(literal in state for literal in self.conditions)
  def apply(self, state):
    return State({atom for atom in state.atoms if ~atom not in self.effects} |
                 {literal for literal in self.effects if literal.sign})
  def __call__(self, state): return self.apply(state) if state in self else None
  def __iter__(self): yield self
  def __len__(self): return 1
  # NOTE - the previous methods are the same as in Operator
  # NOTE - cannot hash because each operator is no longer unique
  # def __eq__(self, other):
  #   return type(self) == type(other) and self.name == other.name and self.args == other.args
  # def __ne__(self, other): return not self == other
  # def __hash__(self):
  #   if self._hash is None: self._hash = hash((self.__class__, self.name, self.args))
  #   return self._hash
  def __repr__(self): return self.name + repr(self.args)
  __str__ = __repr__

class PyAction(PyOperator):
  def __init__(self, lifted, *args):
    self.name = lifted.name
    self.cost = (lifted.cost if lifted.cost is not None else 0) + COST_TRANSFORM
    super(PyAction, self).__init__(lifted, *args)
  def is_axiom(self): return False

class PyAxiom(PyOperator):
  cost = 0
  def __init__(self, lifted, *args):
    self.name = lifted.effect.predicate.name
    super(PyAxiom, self).__init__(lifted, *args)
  def is_axiom(self): return True

##################################################

# TODO - bug when static predicates are not solely used conjunctively
def get_operators(operators, static_map, objects, Class):
  for operator in operators:
    for instance in smart_instantiate_operator(operator, static_map, objects):
      #for instance in smart_instantiate_operators(operators, universe): # TODO - do this for lifted all at once before constructing instances?
      #print instance, instance.args # TODO - bug in which it doesn't treat the condition quantifier as a parameter
      dnf_conditions = list(instance.condition.propositional(objects).get_literals())
      [effects] = list(instance.effect.propositional(objects).get_literals())
      for conj in dnf_conditions:
        fluent_conditions = filter(lambda c: not (isinstance(c, Atom) and c.predicate in static_map), conj)
        #print conj, fluent_conditions
        #print [c for c in conj if c in fluent_conditions or c in static_map[c.predicate]]
        if all(c in fluent_conditions or c in static_map[c.predicate] for c in conj): # TODO - not sure why I need this...
        #if True:
          assert all(c in fluent_conditions or c in static_map[c.predicate] for c in conj)
          yield Class(instance.lifted, instance.args, fluent_conditions, effects)

# NOTE - fixes the previous problems with get_operators by first converting to DNF
def get_dnf_operators(operators, objects):
  for operator in operators:
    dnf_conditions = list(operator.condition.propositional(objects).get_literals())
    dnf_effects = list(operator.effect.propositional(objects).get_literals())
    assert len(dnf_effects) == 1 # NOTE - no non-determinstic effects
    effect = dnf_effects[0]
    for conj in dnf_conditions:
      yield operator, conj, effect

def get_actions(operators, static_map, objects):
  actions = []
  for action, conditions, effects in get_dnf_operators(operators, objects):
    actions.append(Action(action.name, action.parameters, And(*conditions), And(*effects)))
  return get_operators(actions, static_map, objects, PyAction)

def get_axioms(operators, static_map, objects):
  axioms = []
  for _, conditions, effects in get_dnf_operators(operators, objects):
    [effect] = effects
    axioms.append(Axiom(effect, And(*conditions)))
  return get_operators(axioms, static_map, objects, PyAxiom)

##################################################

# TODO - use search to encode the type of search

def strips_planner(universe, search, max_time=INF, max_cost=INF, verbose=False):
  initial_atoms = filter(lambda atom: not isinstance(atom, Initialize), universe.get_initial_atoms())
  objects = universe.type_to_objects
  problem = STRIPStreamProblem(initial_atoms, universe.goal_formula, universe.actions + universe.axioms, []) # TODO - constants?
  static_map = {pr: [] for pr in problem.get_static_predicates()}
  for atom in initial_atoms:
    if atom.predicate in static_map:
      static_map[atom.predicate].append(atom)

  initial_state = State(PyLiteral(atom) for atom in initial_atoms if atom.predicate not in static_map)
  dnf_goal = list(universe.goal_formula.propositional(objects).get_literals())
  if not dnf_goal:
    return []
  if len(dnf_goal) != 1:
    raise NotImplementedError('Currently only support a conjunctive goal: %s'%universe.goal_formula)
  goal_state = PartialState(map(PyLiteral, dnf_goal[0]))
  strips_actions = list(get_actions(universe.actions, static_map, objects))
  strips_axioms = list(get_axioms(universe.axioms, static_map, objects))
  if verbose:
    print 'Actions:', len(strips_actions)
    print 'Axioms:', len(strips_axioms)

  plan, state_space = default_derived_plan(initial_state, goal_state, strips_actions, strips_axioms)
  if verbose:
    print plan
    print state_space
  if plan is None:
    return None
  return [(universe.name_to_action[action.lifted.name], action.args) for action in plan.operators]

def get_pyplanners(max_time=30, max_cost=INF, verbose=False):
  """
  Returns a pyplanners search function configured using the arguments.

  :param max_time: a numeric constant for the maximum BFS search time.
  :param max_cost: a numeric constant for the maximum BFS solution cost.
  :param verbose: a boolean flag toggling the amount of terminal output
  :return: function wrapper around :class:`.strips_planner`
  """
  return lambda p, mt, mc: strips_planner(p, None, max_time=min(mt, max_time),
                                          max_cost=min(mc, max_cost), verbose=verbose)
