from operators import Operator, OPERATOR_PDDL

# TODO: is it going to be strange if I can't actually simulate these things...

class TemporalOperator(Operator):
  _pddl_name = None
  def __init__(self, name, parameters, condition, effect):
    """
    :param name: the name of the action
    :param parameters: a list of :class:`.Parameter`
    :param condition: a :class:`.Condition`
    :param effect: an :class:`.Effect`
    """
    super(TemporalOperator, self).__init__(parameters, condition, effect)
    self.name = name.lower()
  #def instantiate(self, args): return ActionInstance(self, args)
  def pddl(self, costs):
    parameters_pddl = ' '.join(param.typed_pddl() for param in self.parameters)
    return OPERATOR_PDDL.format(self._pddl_name, self.name,
                                parameters_pddl,
                                self.condition.pddl(),
                                self.effect.pddl())
  def clone(self):
    return self.__class__(self.name[:], self.parameters[:], self.condition.clone(), self.effect.clone())
  def __repr__(self):
    return self.name
  __str__ = __repr__

class Process(TemporalOperator):
  _pddl_name = 'process'

class Event(Operator):
  _pddl_name = 'event'

##################################################

DURATIVE_PDDL = '(:durative-action {}\n' \
                '\t:parameters ({})\n' \
                '\t:duration (= ?duration {})\n' \
                '\t:condition {}\n' \
                '\t:effect {})'

class DurativeAction(object):
  def __init__(self, name, parameters, duration, condition, effect):
    """
    :param name: the name of the action
    :param parameters: a list of :class:`.Parameter`
    :param condition: a :class:`.Condition`
    :param effect: an :class:`.Effect`
    """
    self.name = name.lower()
    self.parameters = parameters
    self.duration = duration
    self.condition = condition
    self.effect = effect
  #def instantiate(self, args): return ActionInstance(self, args)
  def pddl(self, costs):
    parameters_pddl = ' '.join(param.typed_pddl() for param in self.parameters)
    return DURATIVE_PDDL.format(self.name,
                                parameters_pddl,
                                self.duration.pddl(),
                                self.condition.pddl(),
                                self.effect.pddl())
