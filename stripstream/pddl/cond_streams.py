from stripstream.pddl.logic.atoms import Atom
from stripstream.pddl.streams import Stream, TestStream, GeneratorStream, FunctionStream, unwrap_inputs
from stripstream.pddl.logic.operations import Initialize
from stripstream.pddl.logic.predicates import Function
from stripstream.pddl.objects import OBJECT,  Parameter
from stripstream.pddl.logic.predicates import NamedPredicate

import math
import numbers
import collections
# isinstance({}, collections.Hashable)

DEFAULT_COST = 1 # None | 1 | 100
TEST_COST = 0 # None | 0 # TODO - why did I make these None before?
PLAN_TIME = 1.

# TODO
# - function that tests whether something could help generate a particular value
# - the cond stream is supposed to decide whether the properties hold for some input values
# - combine inputs/conditions and outputs/effects

def log_cost(p, min_p=1e-6):
  return -math.log(max(p, min_p))

def geom_cost(cost, p, min_p=1e-6):
  return cost/max(p, min_p)

# Cost for MDP where failures remain in the same state
def mdp_cost(success_cost, failure_cost, p, min_p=1e-6):
  # Equals geom_cost(cost, p) == mdp_cost(cost, cost, p)
  return success_cost + failure_cost*(1./max(p, min_p) - 1.)

def expected_cost(success_cost, failure_cost, p):
  return p*success_cost + (1-p)*failure_cost

class ConditionalStream(object):
  _num = 0
  def __init__(self, inputs, outputs, conditions, effects,
               eager=False, cost=DEFAULT_COST, order=0, plannable=True,
               prob=1., avg_time=1., max_level=None, sign=True):
    """
    Conditional stream abstract class.

    :param inputs: a list of :class:`.Parameter` which are the inputs to ``StreamFn``
    :param outputs: a list of :class:`.Parameter` which are the outputs from ``StreamFn``
    :param conditions: a list of :class:`.Atom` forming a conjunctive condition on inputs
    :param effects: a list of :class:`.Atom` forming a conjunctive effect on inputs
    :param eager: boolean which informs a planner to apply ``self`` immediately
    :param cost: numeric which informs a planner of the difficultly (ex. runtime, likelihood of success) in applying ``self``
    :param order: numeric which informs a planner of the order to apply ``self``
    :param plannable: boolean which informs a planner if ``self`` can be treated as an action
    """
    if not all(isinstance(p, Parameter) for p in inputs):
      raise ValueError('All inputs must be class Parameter: %s'%inputs) # TODO - list offending parameter?
    if not all(isinstance(p, Parameter) for p in outputs):
      raise ValueError('All outputs must be of class Parameter: %s'%outputs)
    if set(inputs) & set(outputs):
      raise ValueError('The input and output parameters must be distinct: %s'%set(inputs) & set(outputs)) # TODO - make sure no repeated
    if not all(isinstance(atom, Atom) for atom in conditions):
      raise ValueError('All conditions must be of class Atom: %s'%conditions)
    if not all(isinstance(atom, Atom) for atom in effects):
      raise ValueError('All effects must be of class Atom: %s'%effects)
    con_params = {p for con in conditions for p in con.args if isinstance(p, Parameter)}
    if not (con_params <= set(inputs)):
      raise ValueError('Parameters %s within the conditions must be specified within the inputs %s'%(con_params, set(inputs)))
    eff_params = {p for eff in effects for p in eff.args if isinstance(p, Parameter)}
    if not (eff_params <= set(list(inputs) + list(outputs))):
      raise ValueError('Parameters %s within the effects must be specified within the inputs %s or outputs %s'%(eff_params, set(inputs), set(outputs)))

    self.name = self.__class__.__name__ # TODO - need unique name. Can append inputs and outputs
    self.inputs = tuple(inputs)
    self.outputs = tuple(outputs)
    self.conditions = conditions
    self.effects = effects
    self.free_params = tuple(set(inputs) - con_params)
    self.free_outputs = tuple(set(outputs) - eff_params)
    self.streams = {}
    self.eager = eager
    self.cost = cost # TODO - cost function that varies with time
    self.order = order
    self.plannable = plannable # TODO - unify eager and plannable
    self.prob = prob
    self.avg_time = avg_time
    self.mdp_cost = mdp_cost(self.avg_time, self.avg_time + PLAN_TIME, self.prob)
    self.max_level = max_level
    self.sign = sign # TODO: automatically infer based on usage
    self.make_instances = True
    self.n = CondStream._num
    CondStream._num += 1
  #: subclasses of :class:`.ConditionalStream` must override StreamFn with a subclass of :class:`.Stream`
  StreamFn = Stream
  def instantiate_conditions(self, inputs):
    param_map = dict(zip(self.inputs, inputs))
    return [condition.instantiate(param_map) for condition in self.conditions]
  def instantiate_effects(self, inputs, outputs):
    param_map = dict(zip(self.inputs, inputs) + zip(self.outputs, outputs))
    return [effect.instantiate(param_map) for effect in self.effects]
  def __call__(self, inputs):
    inputs = tuple(inputs)
    if inputs not in self.streams:
      self.streams[inputs] = self.StreamFn(self, inputs)
    return self.streams[inputs]
  def reset(self):
    for stream in self.streams.values():
      stream.reset()
  def all_values(self):
    for stream in self.streams.values():
      for values in stream.call_history():
        for value in values:
          yield value
  def get_cost(self, inputs):
    # TODO: handle case where inputs are abstract
    return self.cost
  @property
  def calls(self):
    return sum(stream.calls for stream in self.streams.values())
  @property
  def call_time(self):
    return sum(stream.call_time for stream in self.streams.values())
  def __repr__(self):
    #return '%s(%s | %s)'%(self.name, self.outputs, self.inputs)
    return '%s->%s'%(self.inputs, self.outputs)

CondStream = ConditionalStream

##################################################

class ConstCondStream(CondStream):
  def __init__(self, outputs, effects, **kwargs):
    super(ConstCondStream, self).__init__([], outputs, [], effects, **kwargs)

class TestCondStream(CondStream):
  def __init__(self, inputs, conditions, effects, **kwargs):
    super(TestCondStream, self).__init__(inputs, [], conditions, effects, cost=TEST_COST, **kwargs)

class ProducerCondStream(CondStream):
  def __init__(self, inputs, outputs, **kwargs):
    super(ProducerCondStream, self).__init__(inputs, outputs, [], [], **kwargs)

class CostCondStream(CondStream): # Only a single output
  def __init__(self, inputs, conditions, effects, **kwargs):
    super(CostCondStream, self).__init__(inputs, [], conditions, effects, cost=TEST_COST, **kwargs)

##################################################

# TODO - combine these?

class ExternalTestCondStream(TestCondStream):
  def __init__(self, test, name=None, *args, **kwargs):
    self.test = test
    super(ExternalTestCondStream, self).__init__(*args, **kwargs)
    if name is not None:
      self.name = name # NOTE - must be after constructor
  class StreamFn(TestStream):
    def __init__(self, cs, inputs, **kwargs):
      self.test = cs.test
      TestStream.__init__(self, cs, inputs, **kwargs)

class ExternalGenCondStream(CondStream):
  def __init__(self, generator, name=None, *args, **kwargs):
    self.generator = generator
    super(ExternalGenCondStream, self).__init__(*args, **kwargs)
    if name is not None:
      self.name = name # NOTE - must be after constructor
  class StreamFn(GeneratorStream):
    def __init__(self, cs, inputs, **kwargs):
      self.get_generator = cs.generator # NOTE - should probably rename
      GeneratorStream.__init__(self, cs, inputs, **kwargs)

##################################################

class EasyGenStream(CondStream):
  """
  Conditional stream given by a generator of tuple outputs.

  Example for inverse kinematics:

  .. code:: python

    CONF, POSE = EasyType(), EasyType()
    LegalKin = EasyPredicate(POSE, CONF)
    Q, P = EasyParameter(CONF), EasyParameter(POSE)

    cs = EasyGenStream(inputs=[P], outputs=[Q], conditions=[], effects=[LegalKin(P, Q)],
                       generator=lambda p: iter([p]))
  """
  def __init__(self, inputs, outputs, conditions, effects, generator, **kwargs):
    """
    :param inputs: a list of :class:`.Parameter` which are the inputs to ``generator``
    :param outputs: a list of :class:`.Parameter` which are the outputs from ``generator``
    :param conditions: a list of :class:`.Atom` forming a conjunctive condition on inputs
    :param effects: a list of :class:`.Atom` forming a conjunctive effect on inputs and outputs
    :param generator: a function from values for ``inputs`` to a generator for values of ``outputs``
    :param kwargs: keyword arguments for :class:`.ConditionalStream`
    """
    super(EasyGenStream, self).__init__(inputs, outputs, conditions, effects, **kwargs)
    if not callable(generator):
      raise ValueError('EasyGenStream expects generator to be a function: %s'%generator)
    self.generator = generator
  class StreamFn(GeneratorStream):
    def get_generator(self, inputs):
      for outputs in self.cond_stream.generator(*unwrap_inputs(inputs)):
        yield [] if outputs is None else [self.wrap_outputs(outputs)]

class EasyListGenStream(EasyGenStream):
  """
  Conditional stream given by a generator that returns a list of tuple outputs.
  """
  class StreamFn(GeneratorStream):
    def get_generator(self, inputs):
      for output_list in self.cond_stream.generator(*unwrap_inputs(inputs)):
        assert type(output_list) in (list, tuple)
        yield [self.wrap_outputs(outputs) for outputs in output_list]

##################################################

class EasyTestStream(TestCondStream):
  """
  Conditional stream given by a test.

  Example for collision checking:

  .. code:: python

    POSE = EasyType()
    CollisionFree = EasyPredicate(POSE, POSE)
    P1, P1 = EasyParameter(POSE), EasyParameter(POSE)

    cs = EasyTestStream(inputs=[P1, P2], conditions=[], effects=[CollisionFree(P1, P2)],
                        test=lambda p1, p2: p1 != p2)
  """
  def __init__(self, inputs, conditions, effects, test, **kwargs):
    """
    :param inputs: a list of :class:`.Parameter` which are the inputs to ``test``
    :param conditions: a list of :class:`.Atom` forming a conjunctive condition on inputs
    :param effects: a list of :class:`.Atom` forming a conjunctive effect on inputs
    :param test: a function from values for ``inputs`` to ``{False, True}``
    :param kwargs: keyword arguments for :class:`.ConditionalStream`
    """
    super(EasyTestStream, self).__init__(inputs, conditions, effects, **kwargs)
    if not callable(test):
      raise ValueError('EasyTestStream expects test to be a function: %s'%test)
    self.test = test
  class StreamFn(TestStream):
    def test(self, inputs):
      truth = self.cond_stream.test(*unwrap_inputs(inputs))
      if truth not in (True, False):
        raise ValueError('Expected boolean test output but received %s'%truth)
      return truth

##################################################

class EasyFnStream(CondStream):
  """
  Conditional stream given by a function to tuple outputs.
  """
  def __init__(self, inputs, outputs, conditions, effects, function, **kwargs):
    """
    :param inputs: a list of :class:`.Parameter` which are the inputs to ``generator``
    :param outputs: a list of :class:`.Parameter` which are the outputs from ``generator``
    :param conditions: a list of :class:`.Atom` forming a conjunctive condition on inputs
    :param effects: a list of :class:`.Atom` forming a conjunctive effect on inputs and outputs
    :param function: a function from values for ``inputs`` to values of ``outputs``
    :param kwargs: keyword arguments for :class:`.ConditionalStream`
    """
    super(EasyFnStream, self).__init__(inputs, outputs, conditions, effects, **kwargs)
    if not callable(function):
      raise ValueError('EasyFnStream expects function to be a function: %s'%function)
    self.function = function
  class StreamFn(FunctionStream):
    def function(self, inputs):
      outputs = self.cond_stream.function(*unwrap_inputs(inputs))
      if outputs is None:
        return []
      return [self.wrap_outputs(outputs)]

class EasyListFnStream(EasyFnStream):
  """
  Conditional stream given by a function to a list of tuple outputs.
  """
  class StreamFn(FunctionStream):
    def function(self, inputs):
      output_list = self.cond_stream.function(*unwrap_inputs(inputs))
      assert type(output_list) in (list, tuple)
      return [self.wrap_outputs(outputs) for outputs in output_list]

##################################################

class RawListGenStream(CondStream):
  """
  Conditional stream given by a generator of raw objects and atoms.
  """
  def __init__(self, inputs, outputs, conditions, effects, generator, **kwargs):
    """
    :param inputs: a list of :class:`.Parameter` which are the inputs to ``generator``
    :param outputs: a list of :class:`.Parameter` which are the outputs from ``generator``
    :param conditions: a list of :class:`.Atom` forming a conjunctive condition on inputs
    :param effects: a list of :class:`.Atom` forming a conjunctive effect on inputs and outputs
    :param generator: a function from values for ``inputs`` to a generator of raw objects and atoms
    :param kwargs: keyword arguments for :class:`.ConditionalStream`
    """
    super(RawListGenStream, self).__init__(inputs, outputs, conditions, effects, **kwargs)
    if not callable(generator):
      raise ValueError('{} expects generator to be a function: {}'.format(self.__class__.__name__, generator))
    self.generator_fn = generator
  class StreamFn(Stream):
    def __init__(self, cond_stream, inputs):
      Stream.__init__(self, cond_stream, inputs)
      self.generator = self.cond_stream.generator_fn(*unwrap_inputs(self.inputs))
    def get_values(self, **kwargs):
      try:
        raw_list = next(self.generator)
        assert type(raw_list) in (list, tuple)
        return raw_list
      except StopIteration:
        self.enumerated = True
        return []

class ClassStream(CondStream):
  def __init__(self, inputs, outputs, conditions, effects, StreamClass, **kwargs):
    super(ClassStream, self).__init__(inputs, outputs, conditions, effects, **kwargs)
    if not issubclass(StreamClass, Stream):
      raise ValueError('ClassStream expects StreamClass to extend %s: %s'%(Stream, StreamClass))
    self.StreamFn = StreamClass

class EasyCostStream(CostCondStream):
  def __init__(self, inputs, conditions, effects, function, scale=100, eager=True, **kwargs):
    """
    :param inputs: a list of :class:`.Parameter` which are the inputs to ``test``
    :param conditions: a list of :class:`.Atom` forming a conjunctive condition on inputs
    :param effects: a list of :class:`.Atom` forming a conjunctive effect on inputs
    :param function: a function from values for ``inputs`` to a nonnegative number
    :param kwargs: keyword arguments for :class:`.ConditionalStream`
    """
    super(EasyCostStream, self).__init__(inputs, conditions, effects, eager=eager, **kwargs)
    if not callable(function):
      raise ValueError('EasyCostStream expects function to be a function: %s'%function)
    assert all(isinstance(effect.predicate, Function) for effect in effects)
    self.function = function
    self.scale = scale # NOTE - many solvers only deal with integral costs
  class StreamFn(Stream):
    def get_values(self, **kwargs):
      self.enumerated = True
      cost = self.cond_stream.function(*unwrap_inputs(self.inputs))
      assert isinstance(cost, numbers.Number) and (0 <= cost)
      cost = int(self.cond_stream.scale*cost)
      return map(lambda f: Initialize(f, cost), self.cond_stream.instantiate_effects(self.inputs, []))

# TODO: constants stream

##################################################

def equal_stream(ty1=OBJECT, ty2=OBJECT):
  name = '_eq_%s_%s'%(ty1.name, ty2.name)
  AreEqual = NamedPredicate(name, [ty1, ty2])
  X1, X2 = Parameter('x1', ty1), Parameter('x2', ty2)
  cs = EasyTestStream([X1, X2], [], [AreEqual(X1, X2)], lambda x1, x2: x1 == x2, eager=True)
  return AreEqual, cs

def not_equal_stream(ty1=OBJECT, ty2=OBJECT):
  name = '_not_eq_%s_%s'%(ty1.name, ty2.name)
  AreNotEqual = NamedPredicate(name, [ty1, ty2])
  X1, X2 = Parameter('x1', ty1), Parameter('x2', ty2)
  cs = EasyTestStream([X1, X2], [], [AreNotEqual(X1, X2)], lambda x1, x2: x1 != x2, eager=True)
  return AreNotEqual, cs

def make_test_stream(cond_stream):
  if not cond_stream.outputs:
    return cond_stream
  # TODO - assign very high cost (infinite in some cases) for doing this
  raise NotImplementedError()

# TODO - subset test?