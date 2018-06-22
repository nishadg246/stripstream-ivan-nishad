from time import time
from heapq import heappush, heappop

from stripstream.utils import INF
from stripstream.algorithms.instantiation import instantiate_operators
from stripstream.algorithms.focused.stream_search import compute_costs, StreamN, AtomN
from stripstream.algorithms.focused.utils import AbstractConstant
from stripstream.algorithms.focused.bind_abstract import call_stream

def process_eager_streams(universe):
  assert universe.make_stream_instances
  new_streams = list(universe.new_streams)
  universe.new_streams = [] # Prevents this from possibly being infinite
  t0 = time()
  num_eager = 0
  for stream in new_streams:
    if stream.cond_stream.eager and not stream.called and \
        not any(isinstance(arg, AbstractConstant) for arg in stream.inputs): # NOTE - might be called by the DFS
      num_eager += 1
      call_stream(stream.cond_stream, stream.inputs, universe) # Kind of silly
      assert stream.enumerated # TODO - later add to a queue or something
  if universe.verbose:
    print 'eager_streams | num: %s, time: %s'%(num_eager, time() - t0)

def get_stream_instances(stream_actions, universe):
  t0 = time()
  stream_instances = set(instantiate_operators(stream_actions, universe.type_to_objects))
  num = len(stream_instances)
  filtered_instances = (stream_instances - universe.temp_blocked) - universe.perm_blocked
  if universe.verbose: print 'stream_instances | pre: %s, filtered: %s, time: %s'%(num, len(stream_instances), time() - t0)
  l_costs, s_costs = compute_costs(universe.initial_atoms, filtered_instances)
  static_atoms = set(l_costs.keys()) - set(universe.initial_atoms) - {None}
  universe.temporary_atoms = set()
  for atom in static_atoms:
    universe.add_atom(atom)
    universe.temporary_atoms.add(atom)
  return l_costs, s_costs, static_atoms

##################################################

# TODO - can either dynamically recreate from scratch or can use pddl_problem to track everything that could be valid
# NOTE - right now this is slower than get_stream_instances for some reason (although it creates fewer instances)

def dynamic_compute_costs(stream_actions, universe, fn=sum):
  t0 = time()
  universe.reset_temporary(stream_actions)
  atom_costs = {}
  stream_costs = {}
  queue = []
  def process_new_actions():
    for act in universe.new_stream_actions:
      #cs, args = act.lifted.cond_stream, act.inp_args
      #if cs.eager and not any(isinstance(arg, AbstractConstant) for arg in args):
      #  call_stream(cs, args, problem)
      #  continue # TODO - should I dynamically add the new objects? Yes I need to do this
      stream_costs[act] = StreamN(fn(atom_costs[con].c for con in act.conditions), max(atom_costs[con].l
                            for con in act.conditions)) if len(act.conditions) != 0 else StreamN(0, 0)
      l_cost, l_level = stream_costs[act].c + act.cost, stream_costs[act].l + 1
      for eff in act.effects:
        if eff not in universe.temporary_atoms and l_cost < atom_costs.get(eff, INF):
          atom_costs[eff] = AtomN(l_cost, l_level, act)
          heappush(queue, (l_cost, eff))
    universe.new_stream_actions = []

  for atom in universe.initial_atoms:
    atom_costs[atom] = AtomN(0, 0, None)
    heappush(queue, (atom_costs[atom], atom))
  process_new_actions()
  while len(queue) != 0:
    _, lit = heappop(queue)
    if lit in universe.temporary_atoms: continue
    universe.add_temporary_atom(lit)
    process_new_actions()
  static_atoms = set(atom_costs.keys()) - set(universe.initial_atoms)
  if universe.verbose:
    print 'stream_instances | instances: %s, time: %s'%(len(stream_costs), time() - t0)
  return atom_costs, stream_costs, static_atoms