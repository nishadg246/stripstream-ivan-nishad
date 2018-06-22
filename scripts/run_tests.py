import argparse

from scripts.run_blocksworld import create_problem
from stripstream.algorithms.focused.focused_planner import focused_planner
from stripstream.algorithms.focused.plan_focused import plan_focused
from stripstream.algorithms.focused.simple_focused import simple_focused
from stripstream.algorithms.incremental.incremental_planner import incremental_planner
from stripstream.pddl.examples.countable_tamp.countable_tamp import compile_problem
from stripstream.pddl.examples.countable_tamp.countable_tamp_utils import get_grasp_problem, get_distract_problem, \
  get_invert_problem, get_shift_problem
from stripstream.utils import SEPARATOR

FINITE_PROBLEMS = [
  ('blocksworld', create_problem)
]

P = 3
COUNTABLE_PROBLEMS = [
  ('grasp', lambda: compile_problem(get_grasp_problem(p=P))),
  ('distract', lambda: compile_problem(get_distract_problem(p=P))),
  ('shift', lambda: compile_problem(get_shift_problem(p=P))),
  ('invert', lambda: compile_problem(get_invert_problem(p=P))),
]
PROBLEMS = FINITE_PROBLEMS #+ COUNTABLE_PROBLEMS

SEARCHES = [
  #('BFS', get_bfs()),
  #('FastDownward-A*', get_fast_downward('astar', verbose=False)),
  #('FastDownward-Eager', get_fast_downward('eager', verbose=False)),
  #('FastForward', get_fast_forward(verbose=False)),
  #('LAPTK-BFS-F', get_lakpt(search='bfs_f', verbose=False)),
  #('LAPTK-SIW', get_lakpt(search='siw', verbose=False)),
  #('YAHSP', get_yahsp(verbose=False)),
  #('pyplanners', get_pyplanners(verbose=False)),
]

GREEDY, DFS = True, True
PLANNERS = [
  ('incremental', lambda p, s: incremental_planner(p, search=s, frequency=1, verbose=False)),
  # Focused planners
  ('focused', lambda p, s: focused_planner(p, search=s, check_feasible=False, greedy=GREEDY, dfs=DFS, verbose=True)),
  ('simple_focused', lambda p, s: simple_focused(p, search=s, check_feasible=False, greedy=GREEDY, dfs=DFS, verbose=False)),
  ('plan_focused', lambda p, s: plan_focused(p, search=s, check_feasible=False, greedy=GREEDY, dfs=DFS, verbose=False)),
]

def main():
  parser = argparse.ArgumentParser() # Automatically includes help
  parser.add_argument('--planner', help='planner name.', default=None)
  parser.add_argument('--search', help='search name.', default=None)
  parser.add_argument('--problem', help='problem name.', default=None)
  #parser.add_argument('-verbose', action='store_true', help='verbose.')
  args = parser.parse_args()

  #planners = [args.planner] if args.planner else map(itemgetter(0), PLANNERS)
  #searches = [args.search] if args.search else map(itemgetter(0), SEARCHES)
  #problems = [args.problem] if args.problem else map(itemgetter(0), PROBLEMS)

  planner_map, search_map, problem_map = dict(PLANNERS), dict(SEARCHES), dict(PROBLEMS)
  planners = [(args.planner, planner_map[args.planner])] if args.planner else PLANNERS
  searches = [(args.search, search_map[args.search])] if args.search else SEARCHES
  problems = [(args.problem, problem_map[args.problem])] if args.problem else PROBLEMS

  # TODO - merge this with experiments
  for planner_name, planner_fn in planners:
    for search_name, search_fn in searches:
      for problem_name, problem_fn in problems:
        print SEPARATOR
        print planner_name, search_name, problem_name
        problem = problem_fn()
        #try:
        if True:
          plan, _ = planner_fn(problem, search_fn)
          if plan is not None:
            print 'SUCCESS'
          else:
            print 'FAILURE'
            return
        #except Exception as e:
        #  print e
        #  return
if __name__ == '__main__':
  main()