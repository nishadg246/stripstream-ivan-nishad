import stripstream.pddl.logic.connectives
from stripstream.pddl.logic.formulas import Formula, Condition, Effect
from stripstream.pddl.logic.predicates import Function
from stripstream.pddl.objects import Object, OBJECT, Parameter, Constant


#class Literal(object):
#  def __init__(self, atom, sign):
#    self.atom = atom
#    self.sign = sign
#  def invert(self):
#    return Literal(self.atom, not self.sign)

class Atom(Formula, Condition, Effect):
  def __init__(self, predicate, args):
    """
    Atomic formula (P O1 O2 ... ON).

    :param predicate: the :class:`.Predicate` form of the atom
    :param args: a list of :class:`.Object` arguments to ``predicate``
    :raises ValueError: if ``predicate.types`` does not match the types of ``args``
    .. automethod:: __invert__
    """
    if not all(isinstance(arg, Object) for arg in args):
      raise ValueError('Expected atom arguments %s to extend class Object'%args)
    if len(predicate.types) != len(args):
      raise ValueError('Expected atom arguments %s to have the same length as predicate types %s'%(args, predicate.types))
    arg_types = tuple(arg.type for arg in args)
    if not all(ty == arg_ty or ty == OBJECT for ty, arg_ty in zip(predicate.types, arg_types)):
      raise ValueError('Expected atom argument types %s to be the same as predicate types %s'%(arg_types, predicate.types))
    self.predicate = predicate
    self.args = tuple(args)
    self._hash = None
  @property
  def name(self): return self.predicate.name
  @property
  def types(self): return self.predicate.types
  @property
  def function(self): return self.predicate
  def get_atoms(self): return {self}
  def get_literals(self): return [[self]]
  def get_formulas(self): return []
  def get_quantified(self): return set()
  def substitute(self, atom, subformula): pass
  def holds(self, atoms, constants): return self in atoms
  def positive_supporters(self, atoms, constants): return {self} if self.holds(atoms, constants) else None
  def negative_supporters(self, atoms, constants): return None if self.holds(atoms, constants) else {self}
  def add(self, atoms, constants): return [self]
  def delete(self, atoms, constants): return []
  def is_function(self): return isinstance(self.function, Function)
  def get_parameters(self):
    return set(arg for arg in self.args if isinstance(arg, Parameter))
  def propositional(self, constants): return self
  def dequantify(self, constants): return self
  def instantiate(self, parameter_map):
    return Atom(self.predicate, [parameter_map[arg] if arg in parameter_map else arg for arg in self.args])
  def de_morgan(self, sign=True):
    return self if sign else stripstream.pddl.logic.connectives.Not(self)
  def __invert__(self): # __not__, __neg__,
    """
    Negates the atom.

    :return: :class:`.Not` of the atom
    """
    return stripstream.pddl.logic.connectives.Not(self)
  def __eq__(self, other):
    return type(self) == type(other) and self.predicate == other.predicate and self.args == other.args
  def __ne__(self, other):
    return not(self == other)
  def __hash__(self):
    if self._hash is None:
      self._hash = hash((self.__class__, self.predicate, self.args))
    return self._hash
    #return hash((self.__class__, self.predicate, self.args))
  def __repr__(self):
    return self.predicate.name + '(' + ','.join(repr(arg) for arg in self.args) + ')'
  def pddl(self):
    return '(' + ' '.join([self.name] + [arg.pddl() for arg in self.args]) + ')'
  def typed_pddl(self):
    return '(' + self.name + ''.join(' ' + arg.typed_pddl() for arg in self.args) + ')'

##################################################

class Equal(Formula, Condition):
  """
  Equality atom (= O1 O2 ... ON).

  .. code:: python

    BLOCK = EasyType()
    B1, B2 = EasyParameter(BLOCK), EasyParameter(BLOCK)

    f = Equal(B1, B2)
  """
  def __init__(self, arg1, arg2, *args):
    """
    :param arg1: the first :class:`.Object` argument
    :param arg2: the second :class:`.Object` argument
    :param *args: a list of any remaining :class:`.Object` arguments
    """
    self.args = [arg1, arg2] + list(args)
    if not all(isinstance(arg, Object) for arg in self.args):
      raise ValueError('Expected atom arguments %s to extend class Object'%self.args)
  def get_atoms(self): return set()
  def get_literals(self):
    constants = filter(lambda a: isinstance(a, Constant), self.args)
    if constants and not all(c == constants[0] for c in constants):
      return []
    params = filter(lambda a: not isinstance(a, Constant), self.args)
    if not params:
      return [[]]
    if constants:
      return [[Equal(constants[0], *params)]]
    return [[Equal(*params)]]
  def get_formulas(self): return []
  def get_quantified(self): return set()
  def substitute(self, atom, subformula): pass
  def propositional(self, constants): return self
  def dequantify(self, constants): return self
  def instantiate(self, parameter_map):
    return self.__class__(*[parameter_map.get(arg, arg) for arg in self.args])
  def de_morgan(self, sign=True):
    return self if sign else stripstream.pddl.logic.connectives.Not(self)
  def holds(self, atoms, constants):
    #assert all(isinstance(arg, Constant) for arg in self.args)
    return all(self.args[0] == arg for arg in self.args)
  def positive_supporters(self, atoms, constants):
    return set() if self.holds(atoms, constants) else None
  def negative_supporters(self, atoms, constants):
    raise None if self.holds(atoms, constants) else set()
  def pddl(self):
    return '(= %s)'%' '.join(arg.pddl() for arg in self.args)
  __repr__ = pddl
