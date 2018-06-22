from collections import namedtuple
from stripstream.pddl.logic.predicates import Predicate

Head = namedtuple('Head', ['name', 'parameters']) # TODO - use this?

class Task(Predicate): pass # NOTE - a task is very similar to a predicate

# NOTE - a method potentially has multiple conditions/subtasks although we will restrict to one

class Method(object):
  def __init__(self, name, parameters, task, conditions, subtasks, partial_orders):
    self.name = name
    self.parameters = parameters
    self.task = task # Could be called controllable effect
    self.conditions = conditions
    self.subtasks = subtasks
    self.partial_orders = partial_orders

class UnorderedMethod(Method):
  def __init__(self, name, parameters, task, conditions, subtasks):
    super(UnorderedMethod, self).__init__(name, parameters, task, conditions, subtasks, set())

class TotallyOrderedMethod(Method):
  def __init__(self, name, parameters, task, conditions, subtasks):
    super(TotallyOrderedMethod, self).__init__(name, parameters, task, conditions, subtasks,
                                               set(zip(subtasks[:-1], subtasks[1:])))

# TODO - compile to/from HTN
# - can make axioms be the high level tasks
# - can make abstract action that achieves preconditions of axioms
# - this abstract action is decomposed

def compile_to_htn(*args):
  raise NotImplementedError()

def compile_from_htn(*args):
  raise NotImplementedError()
