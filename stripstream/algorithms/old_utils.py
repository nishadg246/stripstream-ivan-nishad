from stripstream.algorithms.instantiation import instantiate_axioms
from stripstream.algorithms.plan import apply_axioms

"""
# TODO - could do this without grounding all the axioms by matching them
def identify_supporting_axioms(formula, state, axioms, objects):
  if formula.holds(state, objects):
    return []
  assert is_conjunction(formula) # TODO - assuming STRIPS for now
  supporters = []
  # TODO - only positive atoms for now (while neglecting nested Not(Not(...)))
  for atom in [atom for atom in get_literals(formula) if isinstance(atom, Atom) and atom not in state]:
    for axiom in axioms:
      if axiom.effect == atom:
        # TODO - may need two layers of axioms to do something
        print axiom.lifted, axiom.args
        print axiom.condition.positive_supporters(state, objects)
        raw_input('Paused')
        #assert axiom.condition.holds(state, objects) # TODO - support multiple layers in the future
        supporters.append((axiom.lifted, axiom.args))
        # TODO - I guess I could pick out who supports it in the pool of other axioms
        # NOTE - The axiom property guarantees this is not cyclic
        # TODO - could also just do a relaxed plan graph type thing to determine the ordering
  return supporters

# TODO - could also substitute the axiom for the use of the condition for formulas
# NOTE - we should be able to map each derived predicate to one axiom
# NOTE - the axiom order is a DAG inferred from which axioms use which derived predicates
# TODO - what if an action/axiom has multiple possible supporting atoms?

def expand_plan(plan, universe):
  expanded_plan = []
  axioms = list(instantiate_axioms(universe))
  objects = universe.type_to_objects
  state = universe.get_initial_atoms()
  for operator, parameters in plan:
    instance = operator.instantiate(parameters)
    expanded_plan += identify_supporting_axioms(instance.condition, state, axioms, objects)
    expanded_plan.append((operator, parameters))
    state = instance.apply(state, objects)
  return expanded_plan + identify_supporting_axioms(universe.goal_formula, state, axioms, objects)
"""

##################################################

"""
def is_solution(universe, plan): # TODO - this doesn't seem to be right
  instances = [operator.instantiate(parameters) for operator, parameters in plan]
  constants = universe.type_to_objects
  axioms = list(instantiate_axioms(universe))
  state = apply_axioms(universe.get_initial_atoms(), constants, axioms)
  for i, instance in enumerate(instances):
    if not instance.is_applicable(state, constants):
      return False
    state = apply_axioms(instance.apply(state, constants), constants, axioms)
  return universe.goal_formula.holds(state, constants)
"""

def get_raw_states(universe, plan): # NOTE - doesn't apply axioms because axioms are mostly used for inferences
  instances = [operator.instantiate(parameters) for operator, parameters in plan]
  constants = universe.type_to_objects
  #initial_atoms = universe.initial_atoms
  initial_atoms = universe.fixed_initial_atoms # Doesn't include deferred literals
  states = [initial_atoms]
  for i, instance in enumerate(instances):
    states.append(instance.apply(states[-1], constants))
  return states

def trace_plan(universe, plan):
  instances = [operator.instantiate(parameters) for operator, parameters in plan]
  print instances
  #state = universe.get_initial_state()
  constants = universe.type_to_objects
  axioms = list(instantiate_axioms(universe))
  print 'Axioms:', len(axioms)
  state = apply_axioms(universe.initial_atoms, constants, axioms) # Avoids the refined literals
  print
  print 0, state
  for i, instance in enumerate(instances):
    assert instance.is_applicable(state, constants)
    state = apply_axioms(instance.apply(state, constants), constants, axioms)
    print i+1, state
  assert universe.goal_formula.holds(state, constants)

def get_state_objects(initial_atoms):
  objects = {}
  for atom in initial_atoms:
    for arg in atom:
      if arg.type not in objects:
        objects[arg.type] = []
      objects[arg.type].append(arg)
  return objects