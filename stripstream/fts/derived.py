from stripstream.pddl.logic.predicates import Predicate
from stripstream.pddl.objects import Constant
from stripstream.pddl.operators import STRIPSAxiom
from stripstream.fts.variable import FreeParameter, VarMember, Par

def create_axiom(con_form, var_names, params, inputs, var_map, axiom_map):
  key = (con_form, tuple(var_names), tuple(params), tuple(inputs))
  if key not in axiom_map:
    conditions = []
    for name, nested in zip(var_names, params):
      if name is not None:
        conditions.append(var_map[name].predicate(*nested))
    con_params = [nested[-1] for nested in params]
    conditions.append(con_form.predicate(*con_params))

    DerPred = Predicate('_der_%s'%len(axiom_map), [par.type for par in inputs])
    effect = DerPred(*inputs)
    axiom_map[key] = STRIPSAxiom(conditions, [effect])
  return axiom_map[key].effects[0]

def get_derived(con, var_map, axiom_map, constants):
  # TODO - worry about repeats
  # NOTE - difference between variable and dtype causes some issues here
  # TODO - maybe it would just be easier to decompose into parameters first?

  param_map = {}
  num_params = [0]
  def get_item(item, dtype):
    if isinstance(item, FreeParameter) or isinstance(item, VarMember):
      if item not in param_map:
        num_params[0] += 1
        param_map[item] = Par('%s'%num_params[0], dtype)
      return param_map[item]
    num_params[0] += 1
    return Par('%s'%num_params[0], dtype)

  var_names = []
  params = []
  inputs = []
  constant_map = {}
  for i, item in enumerate(con.values):
    if isinstance(item, VarMember):
      name, args = item.var[0], item.var[1:]
      var_names.append(name)
      assert len(var_map[name].args) == len(args)
      nested = []
      for j, arg in enumerate(args):
        param = get_item(arg, var_map[name].args[j])
        #if not isinstance(arg, Param):
        if arg in constants:
          inputs.append(param)
          constant_map[param] = arg
        nested.append(param)
      nested.append(get_item(item, var_map[name].dtype))
      params.append(tuple(nested))
    else:
      var_names.append(None)
      param = get_item(item, con.constraint.types[i])
      #if not isinstance(item, Param):
      if item in constants:
        inputs.append(param) # Not possible for param to be inputs already
        constant_map[param] = item
      params.append((param,))
  # Do this uniquely given the vars and parameter order
  # Parameter order will be unique
  # NOTE - I could just cache on the parameter objects used. This is crude but it would work
  # TODO - make an axiom for the form without a constant
  # NOTE - the object and region should be a control parameter?
  # NOTE - this would have been different if they used free parameters as well (i.e. for any object)

  #assert len(params) == len(constants)

  effect = create_axiom(con.constraint, var_names, params, inputs, var_map, axiom_map)
  #new_args = [ty(constant_map[arg]) for ty, arg in zip(effect.predicate.types, effect.args)]
  new_args = []
  for ty, arg in zip(effect.predicate.types, effect.args):
    value = constant_map[arg]
    if not isinstance(value, FreeParameter) and not isinstance(value, Constant): # TODO - probably shouldn't include this here
      value = ty(value)
    new_args.append(value)
  return effect.predicate(*new_args)
