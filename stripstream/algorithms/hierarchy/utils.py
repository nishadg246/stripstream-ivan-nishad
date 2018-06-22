from stripstream.algorithms.plan import get_states

def apply_image(operator, literals=set()):
  from stripstream.pddl.logic.utils import invert
  new_literals = literals.copy()
  for condition in operator.conditions:
    assert not invert(condition) in literals
    new_literals.add(condition)
  for effect in operator.effects:
    new_literals.discard(invert(effect))
    new_literals.add(effect)
  return new_literals

# NOTE - remember that preconditions and effects can be seen more symmetrically if a precondition must be an effect when not overwritten
def apply_preimage(operator, literals=set()):
  from stripstream.pddl.logic.utils import invert
  new_literals = literals.copy()
  for effect in operator.effects:
    assert not invert(effect) in literals
    new_literals.discard(effect)
  for condition in operator.conditions:
    assert not invert(condition) in new_literals
    new_literals.add(condition)
  return new_literals

##################################################

# TODO - unify this with InheritAction
class AbsCondition(object): # StagedCondition
  def __init__(self, conditions):
    from stripstream.pddl.logic.formulas import Formula
    assert all(isinstance(cond, Formula) for cond in conditions)
    self.conditions = conditions


# def preimage_sequence(problem, plan):
#   [goal_conditions] = problem.goal_literals.get_literals()
#   # assert all(isinstance(c, Atom) for c in goal_conditions)
#   subgoals = set(goal_conditions)  # Plan preimage
#   goal_sequence = [list(subgoals)]
#   for action, args in reversed(plan[1:]):
#     instance = action.instantiate(args)
#     [effects] = instance.effect.get_literals()
#     for effect in effects:
#       if effect in subgoals:
#         subgoals.remove(effect)
#     [conditions] = instance.condition.get_literals()
#     # assert all(isinstance(c, Atom) for c in goal_conditions)
#     subgoals.update(conditions)
#     goal_sequence.append(list(subgoals))
#   # TODO: include initial state?
#   return goal_sequence[::-1]

def preimage_sequence(universe, plan):
  from stripstream.pddl.logic.connectives import And
  assert not universe.axioms
  states = get_states(universe, plan)
  instances = [action.instantiate(args) for action, args in plan]
  operators = [(i.condition, i.effect) for i in instances] + [(universe.problem.goal_literals, None)]

  subgoals = set()
  goal_sequence = []
  for state, (pre, eff) in reversed(zip(states, operators)[1:]):
    if eff is not None:
      [effects] = eff.get_literals()
      for effect in effects:
        subgoals.discard(effect)
    subgoals |= pre.positive_supporters(state, universe.type_to_objects)
    goal_sequence.append(And(*subgoals))
  return goal_sequence[::-1]