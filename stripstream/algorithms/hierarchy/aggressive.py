from stripstream.algorithms.hierarchy.operators import Refinable
from stripstream.algorithms.hierarchy.utils import AbsCondition
from stripstream.algorithms.focused.focused_planner import focused_subroutine
from stripstream.algorithms.universe import Universe
from stripstream.utils import INF, irange, separator, header
from stripstream.pddl.logic.connectives import And
from stripstream.pddl.utils import convert_plan
from stripstream.algorithms.plan import substitute_axioms
from stripstream.pddl.logic.atoms import Atom

# NOTE - hierarchical planning methods
# - Aggressive hierarchy that finds a plan and commits to it
#   - Can either rigidly enforce total order or flexibly do partial order
# - Backtracking search that is able to undo decisions when it fails
# - Replanning search that considers everything at once

def aggressive_total_hierarchy(problem, local_planner):
  # TODO - considers the totally ordered plan and plans between goals
  raise NotImplementedError()

def aggressive_partial_hierarchy(problem, local_planner):
  # TODO - considers the partially ordered plan and globally plans
  # NOTE - what if you need the operator on a plan several times in order to do something
  # - I can add an extra argument that adds the special effects and partial order preconditions only
  #   when the extra argument is passed in as a conditional effect
  # - I could also just make a copy of the effect or something as long as the planner doesn't slow down
  raise NotImplementedError()

def goal_serialization(problem, max_time=INF, reset=True, verbose=False, debug=False):
  # TODO
  # - achieves goals sequentially
  # - make the goals be a disjunction so it can choose which one to achieve
  # - similar to serialization in SIW
  # - must achieve the previous goals and a new one
  # - soft preferences (cost preferences)
  abs_goal = problem.goal_literals
  if not isinstance(problem.goal_literals, AbsCondition):
    abs_goal = AbsCondition([problem.goal_literals])
  universe = Universe(problem, use_ground=True, make_stream_instances=True, hierarchical=False, verbose=verbose)
  solution = []
  for goal_level in range(1, len(abs_goal.conditions)+1):
    universe.goal_formula = And(*abs_goal.conditions[:goal_level])
    universe.add_formula(universe.goal_formula)
    print header('Serialization iteration: %s'%goal_level)
    if verbose: print universe.goal_formula
    if verbose: print solution
    plan, _ = focused_subroutine(universe, max_time=INF, verbose=verbose, debug=debug)
    if verbose: print '\n', 'Plan:', convert_plan(plan)
    #raw_input('Continue?')
    if plan is None:
      break
    solution += plan
    if goal_level == len(abs_goal.conditions):
      universe.initial_atoms = set(universe.problem.initial_atoms) | \
                               set(filter(universe.is_stream, universe.initial_atoms)) # Resets the initial state
      return solution, universe
    #state = universe.initial_fluents()
    state = universe.get_initial_atoms()
    for action, args in plan:
      state = action.instantiate(args).apply(state, universe.type_to_objects)
    fluent_atoms = filter(lambda a: isinstance(a, Atom) and universe.is_fluent(a), state) # NOTE - might be unnecessary (if using initial_fluents)
    if reset:
      #goal_formula = universe.goal_formula.clone() # TODO - clone in focused?
      substitute_axioms(universe.goal_formula, state, universe)
      assert universe.goal_formula.holds(state, universe.type_to_objects)
      print universe.goal_formula.positive_supporters(state, universe.type_to_objects)
      supporters = filter(universe.is_stream, universe.goal_formula.positive_supporters(state, universe.type_to_objects))
      # TODO - advance all the satisfied goals (i.e. don't repeat useless plans)

      supporter_objects = set()
      for atom in supporters:
        supporter_objects.update(atom.args)
      supporter_objects -= set(problem.get_constants()) # TODO - only include predicates generated at the same time as these objects
      for atom in state:
        if isinstance(atom, Atom) and universe.is_stream(atom) and \
            any(arg in supporter_objects for arg in atom.args):
          supporters.append(atom)

      static_atoms = filter(lambda a: not universe.is_fluent(a), universe.problem.initial_atoms)
      for cs in problem.cond_streams:
        cs.reset() # Move this?
      new_problem = problem.new_problem(set(fluent_atoms + static_atoms + supporters))

      old_goal = universe.goal_formula # Maybe the goal itself is not mutated
      universe = Universe(new_problem, use_ground=True, make_stream_instances=True, hierarchical=False, verbose=verbose)
      universe.goal_formula = old_goal
      assert universe.goal_formula.holds(universe.get_initial_atoms(), universe.type_to_objects)
    else:
      static_atoms = filter(lambda a: not universe.is_fluent(a), universe.initial_atoms) # Includes Concrete
      universe.initial_atoms = set(fluent_atoms) | set(static_atoms)
  return None, universe
