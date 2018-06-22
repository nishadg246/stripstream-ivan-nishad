from connectives import Connective
from stripstream.pddl.logic.formulas import Condition, Effect

# TODO: applications
# - Function is time (always want to do things in less time)
# - Function is energy/fuel (always want to recharge)
# -

# TODO: function class (that combines)

class LessThanEqual(Connective, Condition):
  _pddl_name = '<='
  def __init__(self, f1, f2): # TODO: should these be functions
    super(LessThanEqual, self).__init__(f1, f2)
  def de_morgan(self, sign=True):
    formulas = [f.de_morgan(sign=sign) for f in self.formulas]
    return LessThanEqual(*formulas) if sign else GreaterThanEqual(*formulas)
  #def get_literals(self):
  #  return flatten(f.get_literals() for f in self.formulas)
  def holds(self, atoms, constants): # TODO: rename to evaluate?
    return self.formulas[0].evaluate() <= self.formulas[1].evaluate()
  def positive_supporters(self, atoms, constants):
    # TODO: find a value for self.formula[0] and formula[1] such that the inequality holds
    raise NotImplementedError()
    #for formula in self.formulas:
    #  child_literals = formula.positive_supporters(atoms, constants)
    #  if child_literals is not None:
    #    return child_literals
    #return None
  def negative_supporters(self, atoms, constants):
    literals = set()
    for formula in self.formulas:
      child_literals = formula.negative_supporters(atoms, constants)
      if child_literals is None:
        return None
      literals |= child_literals
    return literals

class GreaterThanEqual(Connective, Condition):
  _pddl_name = '>='
  def __init__(self, f1, f2):
    super(GreaterThanEqual, self).__init__(f1, f2)

class LessThan(Connective, Condition):
  _pddl_name = '<'
  def __init__(self, f1, f2):
    super(LessThan, self).__init__(f1, f2)

class GreaterThan(Connective, Condition):
  _pddl_name = '>'
  def __init__(self, f1, f2):
    super(GreaterThan, self).__init__(f1, f2)

#class Equal(Connective, Condition):
#  _pddl_name = '='