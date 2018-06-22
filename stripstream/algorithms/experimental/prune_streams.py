from collections import defaultdict
from stripstream.algorithms.instantiation import parameter_product
from stripstream.pddl.logic.atoms import Atom
from stripstream.pddl.logic.connectives import Not
from stripstream.pddl.objects import Parameter
from stripstream.pddl.operators import STRIPSAction

# NOTE - need to call this in conjunction with get_streams
# TODO - could always prune operators not useful/reachable on initial relaxed plan like in fastdownward

def useful_overall(action, universe): # No bindings
  fluents = filter(lambda c: c.predicate in universe.fluent_predicates, action.conditions)
  fluent_params = {a for con in fluents for a in con.args}
  useful_types = {a.type for a in action.parameters if a not in fluent_params}
  useful_atoms = {con for con in action.conditions if con.predicate in universe.stream_predicates}
  return useful_atoms, useful_types

def useful_universe(universe): # TODO - include goal?
  useful_atoms = set()
  useful_types = set()
  for operator in universe.actions + universe.axioms:
    static_atoms = filter(lambda a: universe.is_stream(a), operator.condition.get_atoms())
    useful_atoms.update(static_atoms)
    static_parameters = {a for atom in static_atoms for a in atom.args}
    print static_parameters
    useful_types.update({p.type for p in operator.parameters if p not in static_parameters})
  return useful_atoms, useful_types

##################################################

def useful_at_state_for_action_old(action, state_fluents, universe):
  assert isinstance(action, STRIPSAction)
  fluents = filter(lambda c: isinstance(c, Atom) and universe.is_fluent(c), action.conditions)
  initial_atoms = defaultdict(list)
  for f in state_fluents:
    initial_atoms[f.predicate].append(f)
  fluent_values = [initial_atoms[f.predicate] for f in fluents]
  #if any(len(values) == 0 for values in fluent_values): return set(), set() # Comes out implicitly through parameter_product

  useful_atoms = set()
  for binding in parameter_product(fluents, fluent_values, [], {}):
    if not any(isinstance(con, Not) and con.instantiate(binding) in state_fluents
               for con in action.conditions): # Prunes some negative effects. # TODO - prune more by keeping track of them
      useful_atoms |= {con.instantiate(binding) for con in action.conditions if universe.is_stream(con)}

  bound_params = {arg for atom in useful_atoms for arg in atom.args if isinstance(arg, Parameter)} # NOTE - only consider 'free parameters'
  fluent_params = {a for con in fluents for a in con.args}
  useful_types = {a.type for a in action.parameters if a not in fluent_params and a not in bound_params}

  return useful_atoms, useful_types

# TODO - we can treat axioms like conditional streams
# NOTE - may still overestimate teh set ot useful streams because of negative effects

# TODO - to support axioms, could always do this in a backward way, identifying the possible axioms needed for an action
# TODO - or could do this in a forward way, by "adding" possible axiom effects

def state_fluent_bindings(state, action, universe):
  assert isinstance(action, STRIPSAction)
  fluents = filter(lambda c: isinstance(c, Atom) and universe.is_fluent(c), action.conditions)
  initial_atoms = defaultdict(list)
  for atom in state:
    initial_atoms[atom.predicate].append(atom)
  fluent_values = [initial_atoms[f.predicate] for f in fluents]
  return parameter_product(fluents, fluent_values, [], {})

def useful_at_state_for_action(action, state, universe):
  assert isinstance(action, STRIPSAction)
  streams = filter(universe.is_stream, action.conditions)
  #fluent_params = {a for con in fluents for a in con.args}
  #useful_types = {a.type for a in action.parameters if a not in fluent_params}
  stream_params = {a for con in streams for a in con.args}
  useful_types = {a.type for a in action.parameters if a not in stream_params}
  for binding in state_fluent_bindings(state, action, universe):
    if not any(isinstance(con, Not) and con.instantiate(binding) in state
               for con in action.conditions): # Prunes some negative effects. # TODO - prune more by keeping track of them
      yield {con.instantiate(binding) for con in action.conditions if universe.is_stream(con)}, useful_types

def useful_at_state(state_fluents, universe): # Bound to current state
  useful_atoms, useful_types = set(), set()
  #for action in universe.actions:
  #  new_useful_atoms, new_useful_types = useful_at_state_for_action(action, state_fluents, universe)
  #  useful_atoms |= new_useful_atoms; useful_types |= new_useful_types
  for action in universe.actions:
    for new_useful_atoms, new_useful_types in useful_at_state_for_action(action, state_fluents, universe):
      useful_atoms |= new_useful_atoms; useful_types |= new_useful_types
  return useful_atoms, useful_types

def useful_at_partial_state(partial_state, universe):
  # TODO - conditions useful in the bound partial-state
  raise NotImplementedError()

##################################################

def useful_at_state_for_relaxed_plan(vertex, relaxed_plan, universe):
  raise NotImplementedError()

def useful_at_state_for_relaxed_planning(vertex, universe):
  raise NotImplementedError()

##################################################

# TODO - over-approximation because do not account for merging of predicates?
# NOTE - we are actually reversing the generation order

def get_streams(useful_atoms, useful_types, universe, allow_intersect=False):
  goal_atoms, goal_types = set(useful_atoms), set(useful_types) # Processed goals
  streams = set()
  while True: # TODO - should I iterate over goals or actions
    new_useful_atoms, new_useful_types = set(), set()
    for cs in universe.problem.cond_streams:
      if useful_types & {p.type for p in cs.outputs} != set():
        key = (cs, frozenset())
        if key not in streams:
          streams.add(key)
          new_useful_types |= {p.type for p in cs.free_params}
          new_useful_atoms |= set(cs.conditions)
      for eff in cs.effects:
        for atom in useful_atoms:
          if atom.predicate == eff.predicate:
            mapping = dict(zip(eff.args, atom.args))
            if allow_intersect or all(isinstance(mapping[a], Parameter) or a in cs.inputs for a in mapping):
              #binding = {i: mapping[i] for i in cs.inputs}
              binding = {e: a for e, a in mapping.iteritems() if not isinstance(a, Parameter)}
              key = (cs, frozenset(binding.iteritems()))
              if key is not streams:
                streams.add(key)
                new_useful_types |= {p.type for p in cs.free_params if p not in binding}
                new_useful_atoms |= {con.instantiate(binding) for con in cs.conditions
                                     if con.predicate in universe.stream_predicates}
    if len(new_useful_atoms) == 0 and len(new_useful_types) == 0:
      return streams
    useful_atoms, useful_types = new_useful_atoms - goal_atoms, new_useful_types - goal_types
    goal_atoms |= new_useful_atoms; goal_types |= new_useful_types
    # TODO - account for some streams implying others