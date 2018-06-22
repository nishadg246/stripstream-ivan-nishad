from collections import defaultdict
from stripstream.algorithms.instantiation import instantiate_axioms
from stripstream.pddl.logic.atoms import Atom
from stripstream.pddl.logic.predicates import TotalCost
from stripstream.pddl.logic.utils import get_increases
from stripstream.utils import INF, SEPARATOR

def plan_length(universe, plan):
  if plan is None:
    return INF
  return len(plan)

def plan_cost(universe, plan):
  if plan is None:
    return INF
  cost = universe.initial_cost
  for action, args in plan:
    instance = action.instantiate(args)
    increases = get_increases(instance.effect)
    # TODO - ensure that the last effect is used
    new_cost = 0 # 0 | INF
    if action.cost is not None:
      new_cost += action.cost
    for increase in increases:
      if increase.function == TotalCost():
        if isinstance(increase.value, Atom):
          atom = increase.value
          if atom in universe.perm_functions:
            value = universe.evaluate_func(atom)
          elif atom in universe.temp_functions:
            value = universe.temp_functions[atom]
          else:
            raise ValueError(increase)
        else:
          value = increase.value
        new_cost += value
        #new_cost = min(new_cost, value)
        #new_cost = max(new_cost, value)
        #new_cost = value # TODO: several ways to handle repeated costs
    cost += new_cost
  return cost

##################################################

# TODO - for use in picking out which predicates where used
#def min_cost_supporting_static_predicates(plan, universe):
#  pass

# TODO - include the conditions used here
# TODO - this doesn't work when the condition is a single atom... recursion forever...

def substitute_axioms(condition, state, universe):
  # TODO: this assumes all axioms are positive which is not necessarily true
  #if condition.holds(state, universe.type_to_objects):
  #  return
  augmented = False
  for literal in condition.get_atoms():
  #for literal in condition.get_literals(): # TODO: this returns a list of lists...
    if isinstance(literal, Atom) and universe.is_derived(literal):
      axiom = universe.derived_predicates[literal.predicate]
      condition.substitute(axiom.effect, axiom.condition)
      augmented = True
  if augmented:
    substitute_axioms(condition, state, universe) # Recursively applies

def apply_axioms(state, constants, axioms):
  #for axiom in axioms:
  #  if axiom.is_applicable(state, constants):
  #    state = axiom.apply(state, constants)
  unused_axioms = set(axioms)
  while True:
    terminate = True
    for axiom in list(unused_axioms):
      if axiom.is_applicable(state, constants):
        state = axiom.apply(state, constants)
        terminate = False
        unused_axioms.remove(axiom)
    if terminate:
      break
  return state

##################################################

def get_states(universe, plan):
  instances = [operator.instantiate(parameters) for operator, parameters in plan]
  constants = universe.type_to_objects
  axioms = list(instantiate_axioms(universe))
  state = universe.initial_atoms
  yield apply_axioms(state, constants, axioms)
  for i, instance in enumerate(instances):
    #assert instance.is_applicable(states[-1], constants)
    state = instance.apply(state, constants)
    yield apply_axioms(state, constants, axioms)
  #assert universe.goal_formula.holds(states[-1], constants)

def feasible_subplan(universe, plan):
  instances = [action.instantiate(args) for action, args in plan]
  conditions = [instance.condition for instance in instances]
  state = universe.get_initial_atoms()
  subplan = []
  for i, (condition, instance) in enumerate(zip(conditions, instances) + [(universe.goal_formula, None)]):
    substitute_axioms(condition, state, universe) # TODO - I could also just add an instantiated action to the plan
    if not condition.holds(state, universe.type_to_objects):
      return subplan, False
    if instance is not None:
      state = instance.apply(state, universe.type_to_objects)
      subplan.append(plan[i])
  return subplan, True

def is_solution(universe, plan):
  instances = [action.instantiate(args) for action, args in plan]
  conditions = [instance.condition for instance in instances]
  state = universe.get_initial_atoms()
  for condition, instance in zip(conditions, instances) + [(universe.goal_formula, None)]:
    substitute_axioms(condition, state, universe) # TODO - I could also just add an instantiated action to the plan
    if not condition.holds(state, universe.type_to_objects):
      return False
    if instance is not None:
      state = instance.apply(state, universe.type_to_objects)
  return True

##################################################

def supporting_static_predicates(plan, universe):
  supporting_atoms = set()
  instances = [action.instantiate(args) for action, args in plan]
  conditions = [instance.condition for instance in instances]
  state = universe.get_initial_atoms()
  for condition, instance in zip(conditions, instances) + [(universe.goal_formula, None)]:
    substitute_axioms(condition, state, universe) # TODO - I could also just add an instantiated action to the plan
    assert condition.holds(state, universe.type_to_objects)
    for condition in condition.positive_supporters(state, universe.type_to_objects):
      if condition.predicate in universe.stream_predicates: # NOTE - this used to be static_predicates?
        supporting_atoms.add(condition)
    if instance is not None:
      state = instance.apply(state, universe.type_to_objects)
  return supporting_atoms

# TODO - keep track of a map from object to stream
# NOTE - this greedily selects the amount of time and calls to generate the plan. Likely not optimal
# NOTE - the calls and time are treated independently. The minimum values may correspond to different streams
def print_plan_stats(plan, universe):
  print 'Plan statistics'
  print plan
  stream_calls = defaultdict(float)
  stream_time = defaultdict(float)
  values = {obj for _, args in plan for obj in args} | supporting_static_predicates(plan, universe)
  for value in values:
    call_stream, calls = universe.get_min_attr(value, 'prior_calls')
    if call_stream is not None:
      stream_calls[call_stream] = max(stream_calls[call_stream], calls)
    time_stream, time = universe.get_min_attr(value, 'prior_time')
    if time_stream is not None:
      stream_time[time_stream] = max(stream_time[time_stream], time)

  cs_calls = defaultdict(float)
  cs_time = defaultdict(float)
  for stream in stream_calls:
    cs_calls[stream.cond_stream] += stream_calls[stream]
    cs_time[stream.cond_stream] += stream_time[stream]
    #print stream, stream_calls[stream], stream_time[stream]
  #print
  for cs in cs_calls:
    print cs, cs_calls[cs], cs_time[cs]
  print SEPARATOR
