from stripstream.pddl.objects import PARAM_PREFIX, TYPE_PAIR, Object, EasyType, Type
from stripstream.utils import INF

class Head(object):
  def __init__(self, name, types=tuple()):
    """
    Head abstract class.

    :param name: the string name of the head
    :param types: a list of :class:`.Type` inputs to the head
    .. automethod:: __call__
    """
    if not all(isinstance(t, Type) for t in types):
      raise ValueError('{} must all be of class Type'.format(types))
    self.name = name.lower()
    self.types = tuple(types)
    self._hash = None
  def __call__(self, *args):
    """
    Creates an :class:`.Atom` instance from ``args``

    :param args: a list of hashable arguments to ``self``
    :returns: :class:`.Atom` with predicate ``self`` and wrapped :class:`.Object` arguments ``args``
    :raises ValueError: if ``self.types`` does not match the length of ``args``
    """
    return convert_args(self, args)
  def __eq__(self, other):
    return type(self) == type(other) and self.name == other.name and self.types == other.types
  def __ne__(self, other):
    return not(self == other)
  def __hash__(self):
    if self._hash is None:
      self._hash = hash((self.__class__, self.name, self.types))
    return self._hash
    #return hash((self.__class__, self.name, self.types))
  def pddl(self): # NOTE - FF cannot parse parameters with names starting with digits
    return '(' + ' '.join([self.name] + [TYPE_PAIR%('%sx%s'%(PARAM_PREFIX, i), ty)
                                         for i, ty in enumerate(self.types)]) + ')'
  def __repr__(self):
    return '(' + ' '.join([self.name] + [repr(ty) for ty in self.types]) + ')'

##################################################

class Predicate(Head):
  pass
  """
  Predicate P which extends :class:`.NamedHead`.
  """
NamedPredicate = Predicate # NOTE - for backwards support

class EasyPredicate(Predicate):
  """
  Anonymous predicate P which extends :class:`.Predicate`.

  .. code:: python

    CONF = EasyType()
    AtConf = EasyPredicate(CONF)
  """
  num = 0
  template = '_pr%s'
  def __init__(self, *types):
    """
    :param types: a list of :class:`.Type` inputs to the predicate
    .. automethod:: __call__
    """
    name = self.template%EasyPredicate.num
    EasyPredicate.num += 1
    super(EasyPredicate, self).__init__(name, types)

class StaticPredicate(EasyPredicate):
  def __init__(self, *types, **kwargs):
    super(StaticPredicate, self).__init__(*types)
    self.default = kwargs.get('default', False)

##################################################


class Function(Head):
  pass
  """
  Function F which extends :class:`.Head`.
  """

TotalCost = Function('total-cost')
#: total-cost :class:`.Function`.
# NOTE - should move this before TotalCost

TotalTime = Function('total-time')

Time = Function('#t')
#Duration = Function('?duration')

class EasyFunction(Function):
  """
  Anonymous function F which extends :class:`.Function`.

  .. code:: python

    CONF = EasyType()
    Distance = EasyFunction(CONF, CONF)
  """
  num = 0
  template = '_fn%s'
  def __init__(self, *types):
    """
    :param types: a list of :class:`.Type` inputs to the predicate
    .. automethod:: __call__
    """
    name = self.template%EasyFunction.num
    EasyFunction.num += 1
    super(EasyFunction, self).__init__(name, types)

class StaticFunction(EasyFunction):
  def __init__(self, *types, **kwargs):
    super(StaticFunction, self).__init__(*types)
    self.default = kwargs.get('default', INF)

##################################################

import atoms # NOTE - trouble with cyclic imports

def convert_args(self, args):
  if len(args) != len(self.types):
    raise ValueError('%s must have the same length as %s'%(args, self.types))
  new_args = []
  for ty, arg in zip(self.types, args):
    if isinstance(ty, EasyType) and not isinstance(arg, Object):
      new_args.append(ty.get_const(arg))
    else:
      new_args.append(arg)
  return atoms.Atom(self, new_args)