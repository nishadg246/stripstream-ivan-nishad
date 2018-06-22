from stripstream.algorithms.hierarchy.operators import Refinable
from stripstream.algorithms.hierarchy.utils import AbsCondition
from stripstream.algorithms.focused.focused_planner import focused_subroutine
from stripstream.algorithms.universe import Universe
from stripstream.utils import INF, irange, separator, header
from stripstream.pddl.logic.connectives import And
from stripstream.pddl.utils import convert_plan

# TODO - allow abstract objects when replanning

# NOTE - this is not quite like the lazy shortest path problem because each refinement will always work

# TODO - replan which forces the search to use a partially ordered version of the current plan
# - use a heuristic that counts the number of actions used along it
# - add effect that requires that each action along the plan be used
# - could also add partial order constraints as well to constrain solution
# - the goal can be seen as having partial order constraints

def refine_action(action, args, universe):
  assert isinstance(action, Refinable)
  for subaction in action.refinement:
    universe.add_initial_atom(action.get_valid_atom(subaction, args))
    universe.add_initial_atom(action.RefinedPredicate(*args))

def first_selector(plan, universe):
  for action, args in plan:
    if isinstance(action, Refinable):
      refine_action(action, args, universe)
      return

def all_selector(plan, universe):
  for action, args in plan:
    if isinstance(action, Refinable):
      refine_action(action, args, universe)

# TODO - Sort expansions by expected time. Factors in expected overhead from time and success percentage
def sorted_selector(plan, universe):
  levels = {(action, args): action.level for action, args in plan if isinstance(action, Refinable)}
  for action, args in sorted(levels, key=lambda pair: levels[pair]):
    refine_action(action, args, universe)
    return # TODO - return just first or all that meet the minimum

##################################################

def backtracking_hierarchy(problem, local_planner):
  # TODO - considers the partially ordered plan and globally plans. Upon failure discards the current branch
  # NOTE - this is very similar to replan except that it uses failure instead of costs to bias reusing
  raise NotImplementedError()

# TODO - automatically refine eager actions
# TODO - is there a way of merging this with the focused algorithm in general to allow plans that don't have bound args
# TODO - do the stream pruning to only consider streams that could achieve a condition on the set of actions. But do this for the current hierarchical instances
# TODO - remove objects or reset the problem after you have committed to moving a certain distance along the plan to keep the space small
# TODO - explicitly make achieve goals actions and defer their preconditions
# TODO - apply to eager algorithm (although it would not benefit from saving stream calls)

# TODO - add an effect to all level zero and a precondition to all abstract that forces executable action applied first

def replan_hierarchy(problem, selector=first_selector, max_time=INF, max_iterations=INF,
                     first_only=False, execute=True, verbose=False):
  abs_goal = problem.goal_literals
  goal_level = 1
  if not isinstance(problem.goal_literals, AbsCondition):
    abs_goal = AbsCondition([problem.goal_literals])
  universe = Universe(problem, use_ground=True, make_stream_instances=False, hierarchical=True) # Previously removed for hierarchy
  solution = []
  for i in irange(0, max_iterations):
    #if i != 0: print separator(10)
    print header('Replan iteration: %s'%i)
    #if verbose: print universe.initial_fluents() # TODO - this includes ground and the refinement stuff...
    if verbose: print solution
    plan, _ = focused_subroutine(universe, max_time=INF, verbose=False) # Maybe just return this an update it?
    # TODO - arguments for this?
    if verbose: print convert_plan(plan)
    #raw_input('Continue?')
    if plan is None:
      break
    n = 1 if first_only else len(plan)
    if not any(isinstance(action, Refinable) for action, _ in plan[:n]): # NOTE - don't need this anymore
      if goal_level != len(abs_goal.conditions):
        goal_level += 1
        universe.goal_formula = And(*abs_goal.conditions[:goal_level])
        universe.add_formula(universe.goal_formula)
        continue
      static_atoms = filter(lambda a: a.predicate not in universe.fluent_predicates, universe.initial_atoms)
      universe.initial_atoms = set(universe.problem.initial_atoms) | set(static_atoms) # Resets the initial state
      return solution + plan, universe
    selector(plan, universe)

    if execute: # TODO - no incentive for actions to be in an order which allows executable first
      subplan = []
      for action, args in plan:
        if isinstance(action, Refinable): break
        subplan.append((action, args))
      if subplan:
        # TODO - when resetting the state, save the information on the later plan
        state = universe.initial_fluents()
        for action, args in subplan:
          state = action.instantiate(args).apply(state, universe.type_to_objects)
        fluent_atoms = filter(lambda a: a.predicate in universe.fluent_predicates, state) # NOTE - might be unnecessary
        static_atoms = filter(lambda a: a.predicate not in universe.fluent_predicates, universe.initial_atoms)
        universe.initial_atoms = set(fluent_atoms) | set(static_atoms)
        solution += subplan
  return None, universe
