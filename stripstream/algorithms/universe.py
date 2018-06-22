from collections import defaultdict, namedtuple
from time import time
from operator import attrgetter

from stripstream.algorithms.focused.utils import AbstractConstant, Concrete, partition_values
from stripstream.utils import SEPARATOR
from stripstream.pddl.logic.atoms import Atom
from stripstream.pddl.logic.operations import Initialize
from stripstream.pddl.logic.predicates import Predicate, TotalCost, Function
from stripstream.pddl.objects import Parameter, Constant
from stripstream.pddl.operators import Action, Axiom
#from stripstream.pddl.logic.connectives import And
#from stripstream.algorithms.hierarchy.operators import Refinable
#from stripstream.algorithms.hierarchy.utils import AbsCondition
from stripstream.algorithms.instantiation import parameter_product, instantiate_axioms, parameter_product2
#from stripstream.algorithms.plan import apply_axioms
from stripstream.utils import INT_INF

ValueStats = namedtuple('ValueStats', ['stream', 'call_time', 'prior_time', 'prior_calls'])

DEFAULT_COST = INT_INF
DEFAULT_TRUTH = False

def get_key(atom): # NOTE - Ground has the object type so it automatically gets type without this...
  return atom.predicate, tuple(arg.type for arg in atom.args)

# TODO - given fixed problem, compile out STRIPS axioms by using conditional effects
# TODO - can even consider the special case where they only involve one positive fluent
# NOTE - need to update the initial state as well
# TODO - adl2strips to compile out ADL (http://fai.cs.uni-saarland.de/hoffmann/)

####################

class Universe(object):
  domain_name = 'stripstream'
  problem_name = domain_name
  def __init__(self, problem, use_ground, make_stream_instances=False,
               make_action_instances=False, hierarchical=False, verbose=False):
    self.start_time = time()
    self.problem = problem
    self.use_ground = use_ground
    self.make_stream_instances = make_stream_instances or not use_ground
    self.make_action_instances = make_action_instances
    #self.hierarchical = hierarchical
    self.verbose = verbose

    self.predicates = {}
    self.functions = []
    self.name_to_object = {}
    self.type_to_objects = {}
    self.object_to_type = {}
    self.name_to_action = {}
    self.axioms = []
    self.initial_cost = 0

    self.initial_atoms = set()
    self.temporary_atoms = set()
    self.perm_functions = {}
    self.temp_functions = {}
    self.stream_atoms = set() # TODO - separate fluents and stream atoms and combine them in initial_atoms

    self.goal_formula = problem.goal_literals
    #if isinstance(problem.goal_literals, AbsCondition):
    #  if self.hierarchical:
    #    self.goal_formula = problem.goal_literals.conditions[0]
    #  else:
    #    self.goal_formula = And(*problem.goal_literals.conditions)

    self.fluent_predicates = set()
    #self.fluent_predicates = problem.get_fluent_predicates()
    self.derived_predicates = {}
    #self.derived_predicates = problem.get_derived_predicates()
    self.stream_predicates = set()
    #self.stream_predicates = problem.get_stream_predicates()
    self.predicate_to_atoms = defaultdict(set)
    self.value_producers = defaultdict(list) # Values are objects and atoms
    #assert not ((self.fluent_predicates & self.derived_predicates) or
    #            (self.fluent_predicates & self.stream_predicates) or
    #            (self.derived_predicates & self.stream_predicates))

    self.calls = 0
    self.empty = 0
    self.intersect = 0
    self.search_time = 0
    self.resets = 0 # NOTE - could also local and global statistics
    self.enumerated = 0
    self.test_calls = 0
    self.iterations = 0

    # TODO: make this a queue
    self.new_streams = []
    self.new_problem = False
    self.temp_blocked = set() # TODO - should I reset this when doing hierarchy?
    self.perm_blocked = set()
    self.new_instances = []
    self.action_instances = set()

    self.cond_streams = []
    self.cond_conditions = defaultdict(list)
    self.cond_params = defaultdict(list)

    ##########

    # TODO - move this to the creation of a new action?
    self.act_conditions = defaultdict(list)
    self.act_params = defaultdict(list)
    self.act_stream_conditions = {}
    self.act_free_params = {}
    if self.make_action_instances:
      #assert not self.hierarchical # TODO: doesn't work for hierarchical
      # TODO: this assumes static_predicates are used conjunctively
      print 'Warning: make_action_instances only currently supports conjunctive static predicates'
      #stream_predicates = self.stream_predicates # NOTE - this is only populated later... I made a change that affected this
      stream_predicates = problem.get_stream_predicates() # TODO - also include regular static predicates?
      for act in problem.operators:
        self.act_stream_conditions[act] = filter(lambda c: c.predicate in stream_predicates, act.condition.get_atoms())
        for con in self.act_stream_conditions[act]:
          self.act_conditions[con.predicate].append((act, con))
        con_params = {a for con in self.act_stream_conditions[act] for a in con.args}
        self.act_free_params[act] = [p for p in act.parameters if p not in con_params]
        for param in self.act_free_params[act]:
          self.act_params[param.type].append((act, param))

    ##########

    for cs in problem.cond_streams:
      self.add_cond_stream(cs)
    for atom in problem.initial_atoms:
      if isinstance(atom, Initialize):
        self.add_perm_initialize(atom)
      else:
        self.add_initial_atom(atom)
    self.add_formula(self.goal_formula)
    for obj in problem.objects:
      self.add_object(obj)
    for operator in problem.operators:
      if isinstance(operator, Action):
        self.add_action(operator)
      elif isinstance(operator, Axiom):
        self.add_axiom(operator)

    if self.verbose:
      print 'Fluents:', self.fluent_predicates
      print 'Derived:', set(self.derived_predicates)
      print 'Stream:', self.stream_predicates
      print 'Static:', self.static_predicates

    self.stream_atoms.update(self.initial_stream_atoms()) # Hack to ensure all initial stream atoms are here

  ##################################################

  # TODO - store the initial objects so I don't have to return to them
  # NOTE - I could also just make a new problem entirely
  # NOTE - carry around statistics
  #def reset(self):
  #  assert not self.make_stream_instances # Don't support the incremental algorithm yet
  #  for cs in self.problem.cond_streams:
  #    cs.reset()

  ##################################################

  @property
  def fixed_predicates(self):
    return set(self.predicates.values()) - self.fluent_predicates - set(self.derived_predicates)

  @property
  def static_predicates(self):
    return self.fixed_predicates - self.stream_predicates

  @property
  def actions(self):
    return self.name_to_action.values()

  #@property
  #def objects(self):
  #  return self.type_to_objects

  def get_object(self, name):
    return self.name_to_object[name]

  def get_action(self, name):
    return self.name_to_action[name]

  ##################################################

  def is_internal(self, atom):
    return atom.predicate in [Concrete]

  def is_static(self, atom):
    return atom.predicate in self.static_predicates

  def is_fluent(self, atom):
    return atom.predicate in self.fluent_predicates

  def is_derived(self, atom):
    return atom.predicate in self.derived_predicates

  def is_stream(self, atom):
    return atom.predicate in self.stream_predicates

  ##################################################

  def add_stream(self, cond_stream, combo):
    # TODO - could always eagerly evaluate streams that satisfy current conditions while using the operator in search
    stream = cond_stream(combo)
    self.new_streams.append(stream)
    # NOTE - is_free_test was here

  def add_cond_stream(self, cs):
    for atom in cs.conditions + cs.effects:
      #self.add_atom(atom) # TODO - this could add an object which would make stream instances...
      if isinstance(atom.predicate, Predicate):
        self.add_predicate(atom.predicate)
        if self.is_derived(atom):
          raise ValueError('%s is a stream and derived predicate'%atom.predicate)
        if self.is_fluent(atom):
          raise ValueError('%s is a stream and fluent predicate'%atom.predicate)
        self.stream_predicates.add(atom.predicate)
    if cs.make_instances and self.make_stream_instances: # TODO - if I ever do this online, I need to add_streams for all existing
      for con in cs.conditions:
        self.cond_conditions[con.predicate].append((cs, con))
      for param in cs.free_params:
        self.cond_params[param.type].append((cs, param)) # TODO - could just add one of each type now
      if not cs.inputs:
        self.add_stream(cs, tuple())

  ##################################################

  def add_type(self, ty):
    if ty not in self.type_to_objects:
      self.type_to_objects[ty] = set()

  # TODO - Right now, eager evaluation is causing this to be computed immediately
  def add_object(self, obj):
    assert isinstance(obj, Constant)
    #assert obj.type in self.type_to_objects
    self.add_type(obj.type)
    if obj in self.type_to_objects[obj.type]:
      return False

    self.new_problem = True
    self.name_to_object[obj.name] = obj
    self.type_to_objects[obj.type].add(obj)
    if obj in self.object_to_type:
      raise RuntimeError('Cannot share objects across types: %s vs %s'%(obj.type, self.object_to_type[obj]))
    assert obj not in self.object_to_type #
    self.object_to_type[obj] = obj.type
    if self.use_ground and not isinstance(obj, AbstractConstant):
      self.add_initial_atom(Concrete(obj))

    if self.make_stream_instances and obj.type in self.cond_params:
      for cs, param in self.cond_params[obj.type]:
        con_values = [self.predicate_to_atoms[cond.predicate] for cond in cs.conditions]
        param_values = [self.type_to_objects.get(other.type, []) if param != other else [obj] for other in cs.free_params]
        for param_map in parameter_product(cs.conditions, con_values, cs.free_params, param_values):
          combo = tuple(param_map[p] for p in cs.inputs)
          if combo not in cs.streams: # Happens when an object is used twice
            self.add_stream(cs, combo)

    if self.make_action_instances and obj.type in self.act_params:
      for act, param in self.act_params[obj.type]:
        con_values = [self.predicate_to_atoms[cond.predicate] for cond in self.act_stream_conditions[act]]
        param_values = [self.type_to_objects.get(other.type, []) if param != other else [obj] for other in self.act_free_params[act]]
        for param_map in parameter_product(self.act_stream_conditions[act], con_values, self.act_free_params[act], param_values):
          instance = act.instantiate(tuple(param_map[p] for p in act.parameters))
          if instance not in self.action_instances:
            self.action_instances.add(instance)
            self.new_instances.append(instance)

    return True

  ##################################################

  # NOTE - the atom time must be above the object time
  def add_producer(self, value, stream):
    self.value_producers[value].append(ValueStats(stream, stream.call_times[-1], stream.calls, stream.call_time))

  def get_min_attr(self, value, attr):
    streams = map(attrgetter('stream'), self.value_producers[value])
    attrs = map(attrgetter(attr), self.value_producers[value])
    if len(streams) == 0:
      return None, 0
    index = attrs.index(min(attrs))
    return streams[index], attrs[index]

  ##################################################

  def add_predicate(self, predicate):
    assert isinstance(predicate, Predicate)
    if predicate.name in self.predicates: return
    self.predicates[predicate.name] = predicate
    #for ty in predicate.types:
    #  self.type_to_objects[ty] = set()

  def add_atom(self, atom):
    assert isinstance(atom, Atom) and not atom.is_function()
    self.add_predicate(atom.predicate)
    for arg in atom.args:
      #assert arg.type in self.type_to_objects
      if not isinstance(arg, Parameter):
        self.add_object(arg)

  def add_initial_atom(self, atom):
    if atom in self.initial_atoms:
      return False
    self.new_problem = True
    self.add_atom(atom)
    self.initial_atoms.add(atom)
    if self.is_stream(atom):
      self.stream_atoms.add(atom)
    self.predicate_to_atoms[atom.predicate].add(atom)

    # NOTE - this can be thought of as just the smart instantiate where you limit the values of one thing
    if self.make_stream_instances and atom.predicate in self.cond_conditions:
      for cs, con in self.cond_conditions[atom.predicate]:
        con_values = [self.predicate_to_atoms[other.predicate] if other != con else [atom] for other in cs.conditions]
        param_values = [self.type_to_objects.get(other.type, []) for other in cs.free_params]
        for param_map in parameter_product(cs.conditions, con_values, cs.free_params, param_values):
          combo = tuple(param_map[p] for p in cs.inputs)
          if combo not in cs.streams: # Happens when an object is used twice
            self.add_stream(cs, combo)

    if self.make_action_instances and atom.predicate in self.act_conditions:
      for act, con in self.act_conditions[atom.predicate]:
        con_values = [self.predicate_to_atoms[other.predicate] if other != con else [atom] for other in self.act_stream_conditions[act]]
        param_values = [self.type_to_objects.get(other.type, []) for other in self.act_free_params[act]]
        for param_map in parameter_product(self.act_stream_conditions[act], con_values, self.act_free_params[act], param_values):
          instance = act.instantiate(tuple(param_map[p] for p in act.parameters))
          if instance not in self.action_instances:
            self.action_instances.add(instance)
            self.new_instances.append(instance)
    return True

  ##################################################

  def reset_temporary(self, stream_actions):
    assert self.use_ground # NOTE - don't need add_temporary_object because we use Ground(?)
    self.temporary_atoms = set()
    self.predicate_to_temp_atoms = defaultdict(set)
    self.sa_conditions = defaultdict(list)
    self.stream_action_instances = set()
    self.new_stream_actions = []
    for act in stream_actions:
      for con in act.conditions:
        self.sa_conditions[get_key(con)].append((act, con))
      if len(act.parameters) == 0:
        instance = act.instantiate(tuple())
        self.stream_action_instances.add(instance)
        self.new_stream_actions.append(instance)
    #for atom in self.initial_atoms: # TODO - add all the initial atoms?
    #  self.add_temporary_atom(atom)

  def add_temporary_atom(self, atom):
    key = get_key(atom)
    self.add_atom(atom)
    self.temporary_atoms.add(atom)
    self.predicate_to_temp_atoms[key].add(atom)
    for act, con in self.sa_conditions.get(key, []):
      con_values = [self.predicate_to_temp_atoms[get_key(other)] if other != con else [atom] for other in act.conditions]
      for param_map in parameter_product(act.conditions, con_values, [], {}): # TODO - assert no free parameters?
        instance = act.instantiate(tuple(param_map[p] for p in act.parameters))
        if instance not in self.stream_action_instances and \
                instance not in self.temp_blocked and instance not in self.perm_blocked:
          self.stream_action_instances.add(instance)
          self.new_stream_actions.append(instance)

  ##################################################

  def add_function(self, function):
    assert isinstance(function, Function)
    if function not in self.functions:
      self.functions.append(function)

  def add_func_atom(self, atom):
    assert isinstance(atom, Atom) and atom.is_function()
    function = atom.predicate # TODO - rename this...
    self.add_function(function)
    for arg in atom.args:
      #assert arg.type in self.type_to_objects
      if not isinstance(arg, Parameter):
        self.add_object(arg)

  def evaluate_func(self, atom):
    # TODO: could record all possible values of predicates
    return min(self.perm_functions.get(atom, [DEFAULT_COST]))

  def add_perm_initialize(self, initialize):
    # TODO: could treat predicates as binary valued functions (INF / 0)
    assert isinstance(initialize, Initialize)
    self.add_func_atom(initialize.atom)
    #self.initial_atoms.add(initialize)
    if initialize.atom not in self.perm_functions:
      self.perm_functions[initialize.atom] = []
    assert initialize.value < DEFAULT_COST
    self.perm_functions[initialize.atom].append(initialize.value)

  def add_temp_initialize(self, initialize):
    assert isinstance(initialize, Initialize)
    self.add_func_atom(initialize.atom)
    #self.temporary_atoms.add(initialize)
    self.temp_functions[initialize.atom] = min(self.temp_functions.get(initialize.atom, DEFAULT_COST), initialize.value)
    #self.temp_functions[initialize.atom] = initialize.value

  @property
  def perm_initialize(self):
    return {Initialize(atom, self.evaluate_func(atom)) for atom in self.perm_functions}
  @property
  def temp_initialize(self):
    return {Initialize(atom, value) for atom, value in self.temp_functions.iteritems()}

  def add_formula(self, formula):
    for atom in formula.get_atoms():
      self.add_atom(atom)

  ##################################################

  def add_action(self, operator):
    assert isinstance(operator, Action)
    if operator.name in self.name_to_action:
      raise ValueError('Repeated action name: %s'%operator.name)
    #if isinstance(operator, Refinable):
    #  if self.hierarchical:
    #    operator.add_refined()
    #  for suboperator in operator.refinement:
    #    self.add_action(suboperator)
    #  if not self.hierarchical:
    #    return
    self.name_to_action[operator.name] = operator
    for atom in (operator.condition.get_atoms() | operator.effect.get_atoms()):
      if atom.is_function():
        self.add_func_atom(atom)
      else:
        self.add_atom(atom)
    for atom in operator.effect.get_atoms():
      if self.is_derived(atom):
        raise ValueError('%s is a derived and fluent predicate'%atom.predicate)
      if self.is_stream(atom):
        raise ValueError('%s is a stream and fluent predicate'%atom.predicate)
      if not atom.is_function():
        self.fluent_predicates.add(atom.predicate)

  def add_axiom(self, axiom):
    assert isinstance(axiom, Axiom)
    for atom in (axiom.condition.get_atoms() | axiom.effect.get_atoms()):
      self.add_atom(atom)
    if self.is_fluent(axiom.effect):
      raise ValueError('%s is a fluent and derived predicate'%axiom.effect.predicate)
    if self.is_stream(axiom.effect):
      raise ValueError('%s is a stream and derived predicate'%axiom.effect.predicate)
    if self.is_derived(axiom.effect):
      raise ValueError('Derived predicate %s used in two axioms'%axiom.effect.predicate)
    self.derived_predicates[axiom.effect.predicate] = axiom
    self.axioms.append(axiom)

  ##################################################

  def constants_pddl(self):
    s = ''
    for ty in self.type_to_objects:
      if len(self.type_to_objects[ty]) != 0:
        s += '\n\t' + ' '.join(sorted(obj.pddl() for obj in self.type_to_objects[ty]))
        #s += '\n\t' + ' '.join(obj.typed_pddl() for obj in self.type_to_objects[ty])
        if ty is not None:
          s += ' - ' + ty.pddl() # TODO - kind of ugly
    return s

  def domain_pddl(self, costs, derived, durative=False): # Need initial and goal in case strange predicates
    s = '(define (domain %s)\n'%self.domain_name
    requirements = [':strips', ':typing', ':equality', ':disjunctive-preconditions', ':derived-predicates']
    if costs:
      requirements += [':action-costs']
    if derived:
      requirements += [':derived-predicates', ':adl']
    s += '(:requirements %s)\n'%' '.join(requirements)
    #s += '(:types ' + ' '.join(types) + ' - object)\n' # TODO - nested types (i.e. poses, grasps, object properties)
    s += '(:types ' + ' '.join(ty.pddl() for ty in self.type_to_objects if ty is not None) + ')\n'
    s += '(:constants' + self.constants_pddl() + ')\n'
    s += '(:predicates\n\t' + '\n\t'.join(predicate.pddl() for predicate in self.predicates.values()) + ')\n'
    if costs:
      #s += '(:functions (total-cost) - number)\n'
      s += '(:functions %s)\n'%'\n\t'.join(function.pddl() for function in [TotalCost] + self.functions)
    if len(self.name_to_action) != 0:
      actions = self.name_to_action.values()
      #actions = self.problem.convert_axioms_to_effects() # NOTE - converts easy axioms
      s += '\n' + '\n\n'.join(action.pddl(costs, durative) for action in actions) + '\n'
    if len(self.axioms) != 0:
      s += '\n' + '\n\n'.join(axiom.pddl() for axiom in self.axioms)
    return s + ')\n'

  def get_initial_atoms(self):
    return self.initial_atoms | self.temporary_atoms | \
           self.perm_initialize | self.temp_initialize

  def initial_fluents(self):
    return {atom for atom in self.get_initial_atoms() if isinstance(atom, Atom) and
            not self.is_stream(atom) and atom.predicate is not Concrete} # TODO - check static predicates?

  def initial_stream_atoms(self):
    return {atom for atom in self.get_initial_atoms() if isinstance(atom, Atom) and self.is_stream(atom)}

  def initial_pddl(self, costs):
    atoms = list(self.get_initial_atoms())
    #atoms = list(apply_axioms(set(atoms), self.type_to_objects, instantiate_axioms(self)))
    if costs: atoms.append(Initialize(TotalCost(), self.initial_cost))
    return '(' + '\n\t'.join([':init'] + sorted([atom.pddl() for atom in atoms])) + ')'

  def goal_pddl(self):
    return '(:goal %s)'%self.goal_formula.pddl()

  def problem_pddl(self, costs):
    s = '(define (problem %s)\n'%self.problem_name
    s += '(:domain %s)\n'%self.domain_name
    #s += '(:objects' + self.constants_pddl() + ')\n' # NOTE - don't need this
    s += self.initial_pddl(costs) + '\n'
    s += self.goal_pddl() + '\n'
    if costs:
      s += '(:metric minimize %s)'%TotalCost().pddl()
    return s + ')\n'

  ##################################################

  def print_domain_statistics(self):
    print SEPARATOR
    print 'Types'
    for type in self.type_to_objects:
      print type, len(self.type_to_objects[type]) #, map(get_value, self.type_to_objects[type])[:10]

    predicate_map = defaultdict(list)
    for atom in self.initial_atoms:
      predicate_map[atom.predicate].append(atom)
    print
    print 'Predicates'
    for predicate in predicate_map:
      print predicate, len(predicate_map[predicate])

  def print_statistics(self):
    print SEPARATOR
    print 'Total time:', time() - self.start_time
    print 'Iterations:', self.iterations
    print 'Search time:', self.search_time
    print 'Average search time:', self.search_time/self.iterations
    print 'Empty:', self.empty
    print 'Intersect:', self.intersect
    print 'Resets:', self.resets
    print
    print 'Calls:', self.calls
    print 'Gen calls:', self.calls - self.test_calls
    print 'Test calls:', self.test_calls
    print 'Enumerated:', self.enumerated
    print
    print 'Problem statistics'
    print 'Cond Stream | Calls | Total Time | Average Time | Success %'
    for cs in self.problem.cond_streams:
      successes = 0.
      for stream in cs.streams.values():
        for values in stream.call_history:
          successes += len(list(bind_stream_outputs2(stream, values))) != 0
      print cs, cs.calls, round(cs.call_time, 5),
      if cs.calls:
        print round(cs.call_time/cs.calls, 5), round(successes/cs.calls, 5)
      else:
        print
    #print SEPARATOR

def bind_stream_outputs2(stream, values, history=True): # TODO - consider the history here?
  target_parameters = stream.cond_stream.outputs
  target_atoms = set(stream.instantiate_effects({}))
  produced_atoms, produced_objects = partition_values(values)
  if not target_parameters:
    return iter([{}]) if target_atoms <= produced_atoms else iter([])
  return parameter_product2(target_parameters, produced_objects, target_atoms, produced_atoms)