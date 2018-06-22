from collections import deque
from collections import defaultdict
from time import time

from stripstream.algorithms.universe import Universe
from stripstream.utils import irange, INF, argmin
from stripstream.algorithms.instantiation import parameter_product
from stripstream.algorithms.plan import print_plan_stats
from stripstream.algorithms.incremental.incremental_planner import add_streams, call_queue
from stripstream.pddl.operators import Action, Axiom, STRIPSAction, STRIPSAxiom, STRIPS
from stripstream.pddl.utils import get_value
from stripstream.pddl.logic.atoms import Atom

# NOTE - simultaneous is related to PRM/Hauser's algorithm in that it dynamically update the state-space

# TODO
# - only update nodes that beat a heuristic value (or bias towards them)
# - only call streams which enable an action that reaches a new state (i.e. maintain a tree)
# - integrate this with the incremental algorithm?
# - alternatively, choose an action which is to generate a sample with some frequency in planning
# - this would balance calls and search in the event where the search is expensive
# - use state-space lookup to efficiently determine which vertices to update
# - the state-space is currently really tiny but the overhead is incredible

def retrace(v):
  if v.parent is None:
    return []
  op, pv = v.parent
  return retrace(pv) + [op]

def get_plan(v):
  return map(lambda op: (op.lifted, op.args), retrace(v))

def str_atom(atom):
  #return str(atom.predicate.name) + str(tuple(map(get_value, atom.args)))
  return atom.predicate.name + '(' + ' '.join(repr(get_value(arg)) for arg in atom.args) + ')'

class Vertex(object):
  def __init__(self, fluents):
    self.fluents = frozenset(fluents)
    self.derived = set()
    self.axioms = set()
    self.edges = {}
    self.parent = None
    self.cost = 0
    self.distance = 0
    self.op_index = 0 # TODO - maybe store the number of ops applied
  def atoms(self, stream_atoms):
    return self.fluents | self.derived | stream_atoms # TODO - this is the slowest part
  def relax(self, op, pv): # TODO - dynamically update cost
    self.cost = pv.cost + (op.cost if op.cost is not None else 0)
    self.distance = pv.distance + 1
    self.parent = (op, pv)
  def __eq__(self, other):
    return type(self) == type(other) and self.fluents == other.fluents
  def __ne__(self, other):
    return not self == other
  def __hash__(self):
    return hash((self.__class__, self.fluents))
  def __repr__(self):
    #return self.__class__.__name__ + repr(tuple(self.fluents))
    return self.__class__.__name__ + '(' + ', '.join(map(str_atom, self.fluents)) + ')'
  __str__ = __repr__

##################################################

# TODO - map of conditions corresponding to each action for each state
# TODO - could just remove the stream conditions and only keep track of fluents
# TODO - when I create a state, cache based on subset of fluents it has
# TODO - just handle STRIPS subset

def apply_axioms(vertex, axioms, stream_atoms, universe):
  new_derived = False
  while True:
    terminate = True
    for axiom in list(axioms):
      assert isinstance(axiom.lifted, Axiom)
      if axiom not in vertex.axioms and axiom.is_applicable(vertex.atoms(stream_atoms), universe.type_to_objects):
        vertex.derived = axiom.apply(vertex.derived, universe.type_to_objects)
        vertex.axioms.add(axiom)
        new_derived, terminate = True, False
        axioms.remove(axiom)
    if terminate: break
  return new_derived

# NOTE - could also do this with a DFS

def apply_actions(vertex, actions, stream_atoms, universe, optimal):
  state_atoms = vertex.atoms(stream_atoms)
  if universe.goal_formula.holds(state_atoms, universe.type_to_objects): # TODO - do I want to assert that only fluents are in the goal?
    universe.goal_vertices.add(vertex)
    if not optimal: return True
  for act in actions:
    assert isinstance(act.lifted, Action)
    if act not in vertex.edges and act.is_applicable(state_atoms, universe.type_to_objects):
      nv = Vertex(act.apply(vertex.fluents, universe.type_to_objects))
      if universe.verbose: print nv
      if nv not in universe.vertices:
        nv.relax(act, vertex)
        universe.vertices.add(nv)
        universe.vertex_queue.append(nv)
        if hasattr(universe, 'state_map'): # TODO - cleanup
          for a in universe.name_to_action.values():
            for key in get_vertex_keys(nv, a, universe):
              universe.state_map[key].append(nv)
      vertex.edges[act] = nv
  return False

##################################################

def print_helper(universe):
  print 'Iteration: %d | Time: %s | Calls: %s | Search Time: %s | State Space: %s | Goal Vertices: %s'%(universe.iterations,
      round(time() - universe.start_time, 3), universe.calls, round(universe.search_time, 3),
      len(universe.vertices), len(universe.goal_vertices))

def debug_helper(universe):
  for type in universe.type_to_objects:
    print type, len(universe.type_to_objects[type]), map(get_value, universe.type_to_objects[type])[:10]
  raw_input('Continue?\n')

# Fixes the set of actions and updates the state-space
def simultaneous(problem, max_time=INF, max_iterations=INF, frequency=100,
                   verbose=False, debug=False, optimal=False):
  # TODO - prune any unnecessary cond_streams
  universe = Universe(problem, use_ground=False, make_action_instances=True, verbose=verbose)
  stream_queue = deque()
  add_streams(universe, stream_queue)
  universe.root = Vertex(universe.initial_fluents())
  universe.vertices = {universe.root}
  universe.goal_vertices = set()
  universe.vertex_queue = deque()

  for _ in irange(0, max_iterations):
    print_helper(universe)
    if debug: debug_helper(universe)

    universe.iterations += 1

    stream_atoms = universe.initial_stream_atoms() # TODO - could always augment the state with the static atoms or just ignore them
    new_axioms = filter(lambda op: isinstance(op.lifted, Axiom), universe.new_instances)
    new_actions = filter(lambda op: isinstance(op.lifted, Action), universe.new_instances)
    axioms = filter(lambda op: isinstance(op.lifted, Axiom), universe.action_instances)
    actions = filter(lambda op: isinstance(op.lifted, Action), universe.action_instances)
    #if verbose:
    print 'New Actions: %d | New Axioms: %s | Total Actions: %s | Total Axioms: %s\n'%(len(new_actions), len(new_axioms),
                                                                                       len(actions), len(axioms))
    t0 = time()
    for vertex in list(universe.vertices): # Process new_axioms and actions
      if apply_axioms(vertex, new_axioms, stream_atoms, universe) and \
          apply_actions(vertex, actions, stream_atoms, universe, optimal):
        break
      elif apply_actions(vertex, new_actions, stream_atoms, universe, optimal):
        break

    universe.new_instances = [] # NOTE - didn't have this before

    while universe.vertex_queue: # Process new_vertices
      vertex = universe.vertex_queue.popleft()
      apply_axioms(vertex, axioms, stream_atoms, universe)
      if apply_actions(vertex, actions, stream_atoms, universe, optimal):
        break
    universe.search_time += time() - t0

    if (not optimal and universe.goal_vertices) or not call_queue(stream_queue, universe, frequency, max_time):
      break # TODO - call queue waves

  dijkstra(universe)
  best_v, best_plan = None, None
  if universe.goal_vertices:
    best_v = argmin(lambda v: v.cost, universe.goal_vertices)
    best_plan = get_plan(best_v)
  if verbose:
    universe.print_statistics()
    if best_plan is not None:
      print_plan_stats(best_plan, universe)
      print 'Cost:', best_v.cost
  return best_plan, universe

##################################################

def get_actions(vertex, universe):
  for action in universe.name_to_action.values():
    for key in get_vertex_keys(vertex, action, universe):
      for instance in universe.action_map[key]:
        yield instance

def get_vertices(instance, universe):
  return universe.state_map[get_operator_key(instance, universe)]

# NOTE - alternatively could just return the precondition param assignment for the keys

def get_vertex_keys(vertex, lifted, universe): # TODO - similar to
  assert isinstance(lifted, STRIPS) and not lifted.neg_conditions() # NOTE - only positive conditions for now
  fluents = filter(universe.is_fluent, lifted.conditions) # TODO - include derived
  predicates = defaultdict(list)
  for fluent in vertex.fluents:
    predicates[fluent.predicate].append(fluent)
  condition_values = [predicates[c.predicate] for c in fluents]
  for param_map in parameter_product(fluents, condition_values, [], []):
    instantiated = tuple(condition.instantiate(param_map) for condition in fluents)
    yield lifted, instantiated
  #return map(lambda x: None,   )

def get_operator_key(instance, universe):
  lifted = instance.lifted
  assert isinstance(lifted, STRIPS) and not lifted.neg_conditions() # NOTE - only positive conditions for now
  param_map = dict(zip(lifted.parameters, instance.args))
  fluents = filter(universe.is_fluent, lifted.conditions)
  instantiated = tuple(condition.instantiate(param_map) for condition in fluents)
  #instantiated = op_instance.conditions.get_atoms()
  return lifted, instantiated # TODO - include derived

##################################################

# TODO - make the goal condition an axiom?

def apply_actions_strips(vertex, stream_atoms, universe, optimal):
  if universe.goal_formula.holds(vertex.atoms(stream_atoms), universe.type_to_objects): # TODO - do I want to assert that only fluents are in the goal?
    universe.goal_vertices.add(vertex)
    if not optimal: return True
  for act in get_actions(vertex, universe):
    assert isinstance(act.lifted, Action)
    if act not in vertex.edges: # and act.is_applicable(vertex.atoms(stream_atoms), universe.type_to_objects):
      nv = Vertex(act.apply(vertex.fluents, universe.type_to_objects))
      if universe.verbose: print nv
      if nv not in universe.vertices:
        nv.relax(act, vertex)
        universe.vertices.add(nv)
        universe.vertex_queue.append(nv)
        for a in universe.name_to_action.values():
          for key in get_vertex_keys(nv, a, universe):
            universe.state_map[key].append(nv)
      vertex.edges[act] = nv
  return False

# NOTE - is slightly faster than simultaneous because it looks up the states

def simultaneous_strips(problem, max_time=INF, max_iterations=INF, frequency=100,
                   verbose=False, debug=False, optimal=False):
  # TODO - prune any unnecessary cond_streams
  universe = Universe(problem, use_ground=False, make_action_instances=True, verbose=verbose)
  stream_queue = deque()
  add_streams(universe, stream_queue)
  universe.root = Vertex(universe.initial_fluents())
  universe.vertices = {universe.root}
  universe.goal_vertices = set()
  universe.vertex_queue = deque()

  universe.state_map = defaultdict(list)
  universe.action_map = defaultdict(list)
  for vertex in universe.vertices:
    for a in universe.name_to_action.values():
      for key in get_vertex_keys(vertex, a, universe):
        universe.state_map[key].append(vertex)

  for _ in irange(0, max_iterations):
    print_helper(universe)
    if debug: debug_helper(universe)
    universe.iterations += 1

    stream_atoms = universe.initial_stream_atoms() # TODO - could always augment the state with the static atoms or just ignore them
    new_axioms = filter(lambda op: isinstance(op.lifted, Axiom), universe.new_instances)
    new_actions = filter(lambda op: isinstance(op.lifted, Action), universe.new_instances)
    axioms = filter(lambda op: isinstance(op.lifted, Axiom), universe.action_instances)
    actions = filter(lambda op: isinstance(op.lifted, Action), universe.action_instances)
    #if verbose:
    print 'New Actions: %d | New Axioms: %s | Total Actions: %s | Total Axioms: %s\n'%(len(new_actions), len(new_axioms),
                                                                                       len(actions), len(axioms))

    # TODO - recombine with simultaneous

    for instance in universe.new_instances:
      universe.action_map[get_operator_key(instance, universe)].append(instance)

    t0 = time()
    success = False
    for instance in new_actions:
      for vertex in get_vertices(instance, universe):
        #print action.lifted.name, map(get_value, action.args), vertex
        #raw_input('Continue?')
        if apply_actions(vertex, [instance], stream_atoms, universe, optimal):
          success = True
          break
      if success: break
    if success: break

    for vertex in list(universe.vertices): # Process new_axioms and actions
      if apply_axioms(vertex, new_axioms, stream_atoms, universe) and \
          apply_actions(vertex, actions, stream_atoms, universe, optimal):
        break
      #elif apply_actions(vertex, new_actions, stream_atoms, universe, optimal):
      #  break

    universe.new_instances = [] # NOTE - didn't have this before

    while universe.vertex_queue: # Process new_vertices
      vertex = universe.vertex_queue.popleft()
      apply_axioms(vertex, axioms, stream_atoms, universe)
      #if apply_actions(vertex, actions, stream_atoms, universe, optimal):
      if apply_actions_strips(vertex, stream_atoms, universe, optimal):
      #if apply_actions(vertex, get_actions(vertex, universe), stream_atoms, universe, optimal):
        break
    universe.search_time += time() - t0

    if (not optimal and universe.goal_vertices) or not call_queue(stream_queue, universe, frequency, max_time):
      break # TODO - call queue waves

  dijkstra(universe)
  best_v, best_plan = None, None
  if universe.goal_vertices:
    best_v = argmin(lambda v: v.cost, universe.goal_vertices)
    best_plan = get_plan(best_v)
  if verbose:
    universe.print_statistics()
    if best_plan is not None:
      print_plan_stats(best_plan, universe)
      print 'Cost:', best_v.cost
  return best_plan, universe

##################################################

from heapq import heappush, heappop

# TODO - consider variable costs

def dijkstra(universe): # Ensures correct costs
  universe.root.cost, universe.root.distance = 0, 0
  for vertex in universe.vertices:
    vertex.cost, vertex.distance = INF, INF
  queue = [(universe.root.cost, universe.root)]
  processed = set()
  while not queue:
    _, vertex = heappop(queue)
    if vertex in processed:
      continue
    processed.add(vertex)
    if vertex in universe.goal_vertices:
      return get_plan(vertex)
    for edge, next_vertex in vertex.edges.iteritems():
      cost = vertex.cost + (edge.cost if edge.cost is not None else 0)
      if cost < next_vertex.cost:
        assert next_vertex not in processed
        next_vertex.relax(edge, vertex)
        heappush(queue, (next_vertex.cost, next_vertex))
  return None
