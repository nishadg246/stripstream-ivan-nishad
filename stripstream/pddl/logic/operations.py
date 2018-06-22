from stripstream.pddl.logic.formulas import Formula, Effect
from stripstream.pddl.logic.predicates import TotalCost


class Operation(Formula, Effect): # TODO - what should I do about this?
  _pddl_name = None
  def __init__(self, function, value):
    """
    Increase function atom (increase F1 F2).

    :param function: the :class:`.Function` that is increased
    :param value: the numeric or :class:`.Function` amount that the ``function`` increases
    """
    self.function = function
    self.value = value
    self.function_value = hasattr(self.value, 'pddl')
  def get_atoms(self): return {self.value} if self.function_value else set()
  def get_literals(self): return [[self]]
  def get_formulas(self): return []
  def get_quantified(self): return set()
  def substitute(self, atom, subformula): pass
  def propositional(self, constants): return self
  def dequantify(self, constants): return self
  def instantiate(self, param_map):
    if not self.function_value:
      return self.__class__(self.function.instantiate(param_map), self.value)
    return self.__class__(self.function.instantiate(param_map), self.value.instantiate(param_map))
  def de_morgan(self, sign=True):
    if sign: return self
    raise ValueError('Cannot negate %s'%self)
  def add(self, atoms, constants): return []
  def delete(self, atoms, constants): return []
  def pddl(self):
    return '(%s %s %s)'%(self._pddl_name, self.function.pddl(), self.value.pddl() if self.function_value else self.value)
  __repr__ = pddl



class Increase(Operation):
  _pddl_name = 'increase'


class Decrease(Operation):
  _pddl_name = 'decrease'


class Multiply(Operation):
  _pddl_name = '*'

##################################################


class Initialize(Formula):
  def __init__(self, atom, value):
    """
    Formula atom (= F N) which initializes the value of an atom of :class:`.Function`.

    :param atom: an :class:`.Atom` of :class:`.Function`
    :param value: the numeric value of ``value``
    """
    self.atom = atom
    self.value = value
  def get_atoms(self): return set()
  def get_formulas(self): return []
  def get_quantified(self): return set()
  #def substitute(self, atom, subformula): pass
  #def instantiate(self, parameter_map): pass
  def pddl(self):
    return '(= %s %s)'%(self.atom.pddl(), self.value)
  __repr__ = pddl


def Cost(cost): # This is a shortcut
  """
  Increase cost atom (increase total-cost F) which extends :class:`.Function` by
  setting ``function`` to be :class:`.TotalCost()`.

  :param cost: the numeric or :class:`.Function` amount that :class:`.TotalCost()` increases

  """
  return Increase(TotalCost(), cost)