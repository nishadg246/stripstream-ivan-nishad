from stripstream.pddl.logic.predicates import Predicate
from stripstream.pddl.operators import STRIPSAction
from stripstream.algorithms.hierarchy.utils import apply_image, apply_preimage
from stripstream.pddl.logic.connectives import Not, And
from stripstream.pddl.operators import Action

# TODO - I could implement hierarchy in several ways
# - HTN
# - ABSTRIPS
# - Operator decomposition
# - Subgoal decomposition
# - Sequence decomposition

class Refinable(object):
  valid_prefix = '_valid'
  refined_prefix = '_refined'
  def __init__(self, refinement, partial_orders=set(), eager=False): # TODO - be careful about named arguments and which ones match
    self.refinement = refinement # NOTE - only allowing a single refinement. Can make multiple operator copies if there is another
    self.partial_orders = partial_orders # TODO - partial order constraints for sequence refinement?
    self.eager = eager
    assert not isinstance(self, STRIPSAction) or self.is_consistent()
    self.RefinedPredicate = Predicate('_'.join([self.refined_prefix, self.name]), [arg.type for arg in self.parameters])
  def add_refined(self):
    self.add_conditions(Not(self.RefinedPredicate(*self.parameters)))
    for operator in self.refinement:
      operator.add_conditions(self.get_valid_atom(operator, self.parameters))
  def get_valid_predicate(self, operator):
    name = '_'.join([self.valid_prefix, self.name, operator.name])
    parameters = [param for param in self.parameters if param in operator.parameters]
    return Predicate(name, [param.type for param in parameters]) # TODO - save this?
  def get_valid_atom(self, operator, args):
    param_map = dict(zip(self.parameters, args))
    args = [param_map[param] for param in self.parameters if param in operator.parameters]
    ValidPredicate = self.get_valid_predicate(operator)
    return ValidPredicate(*args)
  @property
  def level(self):
    if not self.refinement:
      return 0
    return min(child.level if isinstance(child, Refinable) else 0 for child in self.refinement) + 1
  @property
  def refinement_parameters(self):
    return {param for operator in self.refinement for param in operator.parameters}
  @property
  def refinement_preimage(self):
    literals = set()
    for operator in reversed(self.refinement):
      literals = apply_preimage(operator, literals=literals)
    return literals
  @property
  def refinement_image(self):
    literals = set()
    for operator in self.refinement:
      literals = apply_image(operator, literals=literals)
    return literals
  def is_consistent(self):
    #print self.refinement_parameters, \
    #  self.refinement_preimage, \
    #  self.refinement_image
    return set(self.parameters) <= self.refinement_parameters and \
      set(self.conditions) <= self.refinement_preimage and \
      set(self.effects) <= self.refinement_image # Consider the projected conditions here?
  #def refine(self, args):
  #  pass # TODO - can make a refine function which dynamically produces refinements (similar to before)

# TODO - support hierarchical axioms?
class RefinableAction(Action, Refinable):
  #def __init__(self, action, children=[]):
  def __init__(self, name, parameters, condition, effect, refinement=tuple(), cost=None):
    Action.__init__(self, name, parameters, condition, effect, cost=cost)
    Refinable.__init__(self, refinement)

class RefinableSTRIPSAction(STRIPSAction, Refinable):
  #def __init__(self, action, children=[]):
  def __init__(self, name, parameters, conditions, effects, refinement=tuple(), cost=None):
    STRIPSAction.__init__(self, name, parameters, conditions, effects, cost=cost)
    Refinable.__init__(self, refinement)

##################################################

ABS_COST = 100

# TODO - associate different costs to each level

def abs_action(name, parameters, conditions, effect, cost=None): # NOTE - using And() to indicate no condition
  assert len(conditions) >= 1
  refinement = Action(name, parameters, And(*conditions), effect, cost=cost)
  for i in reversed(range(1, len(conditions))):
    abs_name = '_%s_%s'%(name, i)
    abs_condition = And(*conditions[:1])
    parameter_pool = abs_condition.get_parameters() | effect.get_parameters()
    abs_parameters = filter(lambda p: p in parameter_pool, parameters)
    abs_cost = (cost if cost is not None else 0) + i*ABS_COST
    refinement = RefinableAction(abs_name, abs_parameters, abs_condition, effect,
                                 refinement=[refinement], cost=abs_cost)
  return refinement

class InheritAction(STRIPSAction):
  def __init__(self, name, parent, new_parameters, new_conditions, new_effects):
    assert isinstance(parent, STRIPSAction)
    self.parent = parent
    super(InheritAction, self).__init__(name, # TODO - obtain the name from the parent
                                        list(parent.parameters) + list(new_parameters),
                                        list(parent.conditions) + list(new_conditions),
                                        list(parent.effects) + list(new_effects))

# TODO - hierarchical goal specification
# TODO - just make this a list of goal conditions or an explicit goal class
# TODO - make each goal achiever be an action itself
class GoalOperator(STRIPSAction):
  def __init__(self, conditions):
    super(GoalOperator, self).__init__('goal', [], conditions, [])
