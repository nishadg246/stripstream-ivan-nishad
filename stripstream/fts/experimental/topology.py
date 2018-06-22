
class Constraint(object):
  # Each constraint must have an implicit test
  def __init__(self, types=tuple(), test=lambda *args: False):
    self.types = types
    #self.test = None # None instead of test
    self.test = test

# TODO - separate the test or keep it here? We could try to plan without the test

class ConstraintType(Constraint):
  # For use when combining in ExplicitSet...
  # Has extra variables about what makes this constraint interesting
  pass

#####################################

class ExplicitSet(object):
  def __init__(self, in_vars, out_vars, constraint_pairs):
    self.in_vars = in_vars # Conditioned variables
    self.out_vars = out_vars # The order matters here
    self.constraints = {con: vars for con, vars in constraint_pairs.iteritems()}


class FiniteSet(ExplicitSet):
  def __init__(self, in_vars, out_vars, constraint_pairs):
    super(FiniteSet, self).__init__(in_vars, out_vars, constraint_pairs)
  def construct_set(self, in_values):
    assert len(in_values) == len(self.in_vars) # And types match
    raise NotImplementedError()
    # Returns a set (list) on the fly

class CountableSet(ExplicitSet):
  def __init__(self, in_vars, out_vars, constraint_pairs):
    super(CountableSet, self).__init__(in_vars, out_vars, constraint_pairs)
  def construct_set(self, in_values):
    assert len(in_values) == len(self.in_vars) # And types match
    #raise NotImplementedError()
    for a in range(4):
      yield a
    # Returns a generator on the fly

# NOTE - assume chart contained within zero and one
# NOTE - maybe manifold means something more abstract now
class Manifold(ExplicitSet):
  def __init__(self, in_vars, out_vars, constraint_pairs):
    self.latent_d = [] # Latent upon being conditioned by in_vars
    # Don't really need to know the exterior variables
    self.chart_fns = []

    # TODO - what if you only want to produce some values from this intersection
    # Need to marginalize out the rest

    # NOTE - supplied just a

    # x in X, y in Y = (0, 1)^d - open interval
    # phi(X) subset Y
    # Assume phi^-1
    # Define the domain and codomain of the chart
    # Codomain is more appropriate
    # The domain is more reasonable
    # How could I even call the function for something not in its domain?
    # Maybe its codomain is the full (0, 1) but we consider a submanifold
    # So we consider the codomain of the inclusion map
    # I actually mean range instead of codomain

    super(Manifold, self).__init__(in_vars, out_vars, constraint_pairs)
  def get_chart(self, in_values):



    return

# Certify that the produced trajectory meets the test

class MetricSpace(ExplicitSet):
  pass

class FiniteUnionSet(ExplicitSet):
  pass

class CountableUnionSet(ExplicitSet):
  pass

#####################################

class Atlas(object):
  def __init__(self, m, n, charts):
    assert n <= m
    self.n = n
    self.m = m
    self.charts = charts

# Inclusion map
# InverseChart
# NOTE - assume domain is [0,1]
class Chart(object):
  def __init__(self, codomain_fn, inverse_fn):
    self.codomain_fn = codomain_fn
    self.inverse_fn = inverse_fn


class Foliation(object):
  pass
