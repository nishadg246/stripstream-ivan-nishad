from collections import defaultdict
from itertools import product

from stripstream.priority_queue import PriorityQueue
from stripstream.pddl.logic.atoms import Atom
from stripstream.pddl.logic.operations import Initialize
from stripstream.pddl.objects import Object
from stripstream.algorithms.instantiation import parameter_product
from stripstream.pddl.cond_streams import TestCondStream
from stripstream.algorithms.focused.utils import AbstractConstant, Concrete
from stripstream.utils import INF

# TODO - use the full history here to get outputs_list
# TODO - should I make the user explicitly give the binding
def recover_bindings(cs, values):
  predicate_map = defaultdict(list)
  for eff in values:
    if isinstance(eff, Atom):
      predicate_map[eff.predicate].append(eff)
  effect_values = [predicate_map[eff.predicate] for eff in cs.effects]
  type_map = defaultdict(list)
  for obj in values: # TODO - do I want to consider predicate values?
    if isinstance(obj, Object):
      type_map[obj.type].append(obj)
  type_values = [type_map[param.type] for param in cs.free_outputs]
  for grounding in parameter_product(cs.effects, effect_values, cs.free_outputs, type_values): # TODO - initialize?
    yield [grounding[param] for param in cs.outputs]

# TODO - maybe infer what is useful from which effects we are trying to achieve
def call_stream(cs, input_values, universe, verbose=False):
  assert not any(isinstance(arg, AbstractConstant) for arg in input_values)
  stream = cs(input_values)
  universe.calls += 1
  if isinstance(cs, TestCondStream):
    universe.test_calls += 1
  # TODO - some streams can be still enumerated if it samples composed values already achieved. Has to do with eager tests
  # TODO - But the stream isn't called before the processing?
  # NOTE - maybe it's not saving the outputs because they are empty? No it would still save them
  # TODO - maybe it's the axiom search which chooses something illegal
  # NOTE
  # - This can happen if the action uses abstract as inputs. The bound values turn out to be a previous action.
  # - What should the protocol for previously used values?
  #   - Include them all? They may be used in future actions
  #   - Quit? They may result in the same calls of future things

  #if stream.enumerated and not cs.eager:
  #  print
  #  print cs.eager
  #  print cs
  #  print input_values
  #  print stream
  #  print stream.call_history
  if stream.enumerated and cs.eager: # TODO - this fixes the above problem, but there still is something wrong...
    values = stream.call_history[-1]
    #print cs.action.instantiate(input_values) in universe.perm_blocked
    #values = stream.call()
  elif not stream.enumerated:
    values = stream.call()
  else:
    values = [] # Quits if already exhausted
  if verbose: print stream, input_values, values

  new_atoms = set()
  for value in values:
    universe.add_producer(value, stream)
    if isinstance(value, Object):
      universe.add_object(value)
      new_atoms.add(Concrete(value))
    elif isinstance(value, Atom):
      universe.add_initial_atom(value)
      new_atoms.add(value)
    elif isinstance(value, Initialize):
      universe.add_perm_initialize(value)
    else:
      raise ValueError(value)

  ground_instance = cs.action.instantiate(input_values) # NOTE - need to use the grounded input
  if stream.enumerated:
    universe.perm_blocked.add(ground_instance)
    universe.enumerated += 1
  else:
    universe.temp_blocked.add(ground_instance)
  return list(recover_bindings(cs, values)), new_atoms

##################################################

def get_priority_fn(goal_sequence, plan, achievers, sort_order):
  goal_position = {}
  for i, level in enumerate(goal_sequence):
    for goal in level:
      goal_position[goal] = min(i, goal_position.get(goal, INF))

  incoming_edges, outgoing_edges = defaultdict(set), defaultdict(set)
  for level in plan:
    for instance in level:
      for condition in instance.conditions:
        if condition in achievers:
          incoming_edges[instance].add(achievers[condition])
          outgoing_edges[achievers[condition]].add(instance)

  stream_position = {}
  def recur(instance1, i):
    stream_position[instance1] = min(i, stream_position.get(instance1, INF))
    for instance2 in incoming_edges[instance1]: # TODO - RuntimeError: maximum recursion depth exceeded?
      recur(instance2, i)
  for goal, i in goal_position.items():
    recur(achievers[goal], i)

  order_priority_fn = lambda s: s.lifted.cond_stream.order
  position_priority_fn = lambda s: stream_position[s]
  priority_fn = order_priority_fn if sort_order else position_priority_fn

  return stream_position, incoming_edges, outgoing_edges, priority_fn

def get_ordering(goal_sequence, plan, achievers, sort_order, stream_limit):
  stream_position, incoming_edges, outgoing_edges, priority_fn = \
    get_priority_fn(goal_sequence, plan, achievers, sort_order)

  min_priority = INF
  ordering = []
  queue = PriorityQueue()
  for instance in plan[0]:
    if instance in stream_position: # Otherwise, the instance isn't needed
      queue.push(priority_fn(instance), instance)
  while not queue.empty():
    instance1 = queue.pop()
    if priority_fn(instance1) > min_priority:
      continue
    min_priority = instance1.lifted.cond_stream.order
    ordering.append(instance1)
    for instance2 in outgoing_edges[instance1]:
      incoming_edges[instance2].remove(instance1)
      if len(incoming_edges[instance2]) == 0 and instance2 in stream_position:
        queue.push(priority_fn(instance2), instance2)

  return ordering[:min(stream_limit, len(ordering))]

##################################################

def single_groundings(goal_sequence, relaxed_plan, achievers, sort_order, stream_limit, universe, greedy=True, verbose=False):
  #stream_plan = [instance for level in relaxed_plan for instance in level]
  stream_plan = get_ordering(goal_sequence, relaxed_plan, achievers, sort_order, stream_limit)

  stream_outputs = {}
  #groundings = {arg: arg for cs in stream_plan for arg in cs.args} # NOTE - previously only this worked...
  groundings = {arg: arg for cs in stream_plan for arg in cs.args if not isinstance(arg, AbstractConstant)}
  failure = False
  for instance in stream_plan: # TODO - could just pass the cs and args then if not using the instance
    cs = instance.lifted.cond_stream
    if not all(name in groundings for name in instance.inp_args):
      continue # greedy = False and a previous stream was empty
    input_values = tuple(groundings[name] for name in instance.inp_args)

    key = (cs, input_values)
    if key not in stream_outputs:
      stream_outputs[key], _ = call_stream(cs, input_values, universe)
      if len(stream_outputs[key]) == 0:
        universe.empty += 1
    outputs_list = stream_outputs[key]
    if len(outputs_list) == 0:
      failure = True
      if greedy:
        break
    else:
      output_values = outputs_list[0]
      for name, value in zip(instance.out_consts, output_values):
        if groundings.get(name, value) != value:
          failure = True
          universe.intersect += 1
          if greedy:
            break # TODO - should I try all conflicting bindings?
        else:
          groundings[name] = value
      if greedy and failure:
        break
  return groundings if not failure else None

##################################################

# TODO - Figure out a more efficient way to undoing when a failed assignment
# - Only undo the children that depend on the sample
# - Only backtrack on the affected parents
# - Try all groundings automatically
# - Revisit connected components

# TODO - separate into different parameters and determine success when all constraints are achieved

# TODO - map a precondition to the object which satisfies it
# TODO - can add implicit preconditions based on which streams support the action
# NOTE - would the pick and place task work without Pose adn Grasp preconditions?
# TODO - would it be too expensive for FastDownward to create an object for each possible object and grasp etc
# TODO - would the relaxed planning also potentially be unbounded

def backtrack_bindings(stream_plan, universe, stream_outputs, groundings, i, greedy=True, verbose=False):
  if i >= len(stream_plan):
    return groundings
  instance = stream_plan[i]
  cs = instance.lifted.cond_stream
  input_values = tuple(groundings[name] for name in instance.inp_args)
  key = (cs, input_values)
  #print key
  #print stream_outputs
  if key not in stream_outputs:
    stream_outputs[key], _ = call_stream(cs, input_values, universe) # TODO - error where a stream is already called here!
    if len(stream_outputs[key]) == 0:
      universe.empty += 1
  outputs_list = stream_outputs[key]
  if len(outputs_list) == 0:
    if greedy:
      return None
  else:
    for output_values in outputs_list:
      new_groundings = groundings.copy()
      for name, value in zip(instance.out_consts, output_values):
        if new_groundings.get(name, value) != value: # TODO - flag to avoid ever resampling if already assigned
          universe.intersect += 1
          if greedy:
            break
        else:
          new_groundings[name] = value
      else:
        goal_groundings = backtrack_bindings(stream_plan, universe, stream_outputs, new_groundings, i+1,
                                             greedy=greedy, verbose=verbose)
        if goal_groundings is not None:
          return goal_groundings
  return None

def dfs_groundings(goal_sequence, relaxed_plan, achievers, sort_order, stream_limit, problem, greedy=True, verbose=False):
  #stream_plan = [instance for level in relaxed_plan for instance in level]
  stream_plan = get_ordering(goal_sequence, relaxed_plan, achievers, sort_order, stream_limit)

  groundings = {arg: arg for cs in stream_plan for arg in cs.args if not isinstance(arg, AbstractConstant)}
  stream_outputs = {}
  x = backtrack_bindings(stream_plan, problem, stream_outputs, groundings, 0, greedy=greedy, verbose=verbose)
  return x

##################################################

def queue_bindings(goal_sequence, relaxed_plan, achievers, sort_order, universe):
  goals = {goal for level in goal_sequence for goal in level}
  stream_position, incoming_edges, outgoing_edges, priority_fn = \
    get_priority_fn(goal_sequence, relaxed_plan, achievers, sort_order)

  input_params = defaultdict(dict)
  #output_params = set() # TODO - store the set of parameters here
  for level in relaxed_plan:
    for instance in level:
      for param in instance.inp_args:
        if isinstance(param, AbstractConstant):
          input_params[instance][param] = set()

  # TODO - should I assert that no repeated values for things
  min_priority = INF
  queue = PriorityQueue()
  for instance in relaxed_plan[0]:
    if instance in stream_position: # Otherwise, the instance isn't needed
      queue.push((0, priority_fn(instance)), (instance, instance.inp_args))

  # TODO - terminate if all satisfied
  # Should I just allow each to be independently achieved?
  # I could add a goal node that requires them to be achieved together
  # TODO - need to check whether all the domain conditions are satisfied
  # TODO - check if each parameter has a single assignment and return success
  # TODO - could return success only if each action has consistent samples
  # NOTE - I guess I could just mix the full pile of samples
  # How does it make sure that collision conditions are actually satisfied?

  greedy = True

  for condition, instance in achievers.iteritems():
    min_priority = min(min_priority, priority_fn(instance))
  unsatisfied_goals = {cond for cond in goals if priority_fn(achievers[cond]) <= min_priority}

  while not queue.empty() and (not greedy or unsatisfied_goals):
    instance1, args = queue.pop()
    if priority_fn(instance1) > min_priority:
      continue
    min_priority = instance1.lifted.cond_stream.order
    assert not any(isinstance(name, AbstractConstant) for name in args)
    outputs_list, atoms = call_stream(instance1.lifted.cond_stream, args, universe)
    unsatisfied_goals -= set(instance1.effects) # TODO - what if it doesn't quite produce what it needs?

    print
    print unsatisfied_goals
    print instance1, args, outputs_list, atoms
    raw_input('awefwefa')

    for instance2 in outgoing_edges[instance1]:
      i = 0
      for output_values in outputs_list:
        for name, value in zip(instance1.out_consts, output_values):
          # TODO - I should do the thing where I try combinations that satisfy static predicates
          if name in input_params[instance2] and value not in input_params[instance2][name]:
            input_params[instance2][name].add(value)
            values = []
            for param in instance2.inp_args:
              if not isinstance(param, AbstractConstant):
                values.append([param])
              elif param == name:
                values.append([value])
              else:
                values.append(input_params[instance2][param])
            for combo in product(*values):
              queue.push((i, priority_fn(instance2)), (instance2, combo))
              i += 1
  return None