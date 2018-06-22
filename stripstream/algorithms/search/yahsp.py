from os.path import expanduser
import os

from stripstream.utils import write, read, ensure_dir, safe_remove, remove_dir
from stripstream.algorithms.search.utils import TEMP_DIRECTORY, DOMAIN_PATH, PROBLEM_PATH
from stripstream.algorithms.search.fast_downward import lookup_action

# TODO - compile out derived predicates

YAHSP_ROOT = expanduser('~/Programs/LIS/planners/yahsp2/plan')
YAHSP_ROOT = expanduser('~/Programs/LIS/planners/descarwin/plan')

#COMMAND = ' --domain %s --problem %s --output %s'
#PLANNER_OUTPUT = 'sas_plan'

##################################################

def yahsp(universe, verbose):
  raise NotImplementedError()
  """
  ensure_dir(TEMP_DIRECTORY)
  write(DOMAIN_PATH, universe.domain_pddl(True, False))
  write(PROBLEM_PATH, universe.problem_pddl(True))

  command = LAPKT_ROOT + COMMAND%(DOMAIN_PATH, PROBLEM_PATH, PLANNER_OUTPUT)
  p = os.popen(command) # NOTE - cannot pipe input easily with subprocess
  output = p.read()
  if verbose:
    print output
    print command

  if not os.path.exists(PLANNER_OUTPUT):
    return None
  plan = []
  for line in read(PLANNER_OUTPUT).split('\n')[:-1]:
    entries = line.strip('()').lower().split(' ')
    plan.append(lookup_action(universe, entries[0], entries[1:]))
  if not verbose:
    remove_dir(TEMP_DIRECTORY)
    safe_remove(PLANNER_OUTPUT)
    safe_remove(PLANNER_DETAILS)
  return plan
  """

##################################################

def get_yahsp(verbose=False):
  """
  Returns a FastDownward (FD) search function configured using the arguments.

  :param search: name of a LAPKT search algorithm
  :param verbose: a boolean flag toggling the amount of terminal output
  :return: function wrapper around :func:`.fast_downward`
  """
  return lambda p, mt, mc: yahsp(p, verbose)
