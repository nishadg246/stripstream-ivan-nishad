import operator
import atoms
import connectives
#import quantifiers
import stripstream.pddl.logic.operations


def invert(literal):
  if isinstance(literal, connectives.Not):
    return literal.formulas[0]
  return connectives.Not(literal)

def is_literal(f):
  return isinstance(f, atoms.Atom) or (isinstance(f, connectives.Not) and is_literal(f.formulas[0])) or \
         isinstance(f, stripstream.pddl.logic.operations.Increase) or isinstance(f, atoms.Equal) # Should equal be a literal?

def is_conjunction(f):
  return is_literal(f) or (isinstance(f, connectives.And) and all(is_conjunction(formula)) for formula in f.formulas)

def get_literals(f):
  if is_literal(f):
    return [f]
  return reduce(operator.add, [get_literals(formula) for formula in f.formulas])

def get_increases(f):
  if isinstance(f, stripstream.pddl.logic.operations.Increase):
    return [f]
  if isinstance(f, atoms.Atom):
    return []
  if isinstance(f, connectives.Connective):
    return reduce(operator.add, [get_increases(formula) for formula in f.formulas])
  return []
  #if isinstance(f, connectives.Connective):
  #  return reduce(operator.add, [get_increases(formula) for formula in f.formulas])
  #if isinstance(f, quantifiers.Quantifier):
  #  return get_increases(f.formula)
  #raise ValueError('Unexpected formula {}'.format(f))