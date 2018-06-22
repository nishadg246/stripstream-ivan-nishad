import os

from stripstream.utils import write, ensure_dir, remove_dir
from stripstream.algorithms.search.utils import TEMP_DIRECTORY, DOMAIN_PATH, PROBLEM_PATH
from stripstream.algorithms.search.fast_downward import lookup_action

ENV_VAR = 'SMTPLAN_PATH'

#COMMAND = 'SMTPlan %s %s -l 1 -u -1 -c 2 -s 1' # default settings
COMMAND = 'SMTPlan %s %s -l 1 -u %s -c 2 -s 1' # u can't be unlimited to catch infeasibility
# -d displays intermediate fluents
FAILURE = 'No plan found'

# No plan found in 1 happenings

##################################################

def has_smtplan():
  return ENV_VAR in os.environ

def get_smtplan_root():
  if not has_smtplan():
    raise RuntimeError('Environment variable %s is not defined.'%ENV_VAR)
  return os.environ[ENV_VAR]

def smtplan(universe, max_length, verbose):
  ensure_dir(TEMP_DIRECTORY)
  write(DOMAIN_PATH, universe.domain_pddl(True, False))
  write(PROBLEM_PATH, universe.problem_pddl(True))

  command = os.path.join(get_smtplan_root(), COMMAND%(DOMAIN_PATH, PROBLEM_PATH, max_length))
  if verbose: 
    print command
  p = os.popen(command) # NOTE - cannot pipe input easily with subprocess
  output = p.read()
  if verbose: 
    print output

  if FAILURE in output:
    return None
  plan = []
  for line in output.split('\n')[:-2]:
    #if 'sta' not in line:
    #  continue
    entries = line[line.find('(')+1:line.find(')')].split(' ')
    plan.append(lookup_action(universe, entries[0], entries[1:]))
  if not verbose:
    remove_dir(TEMP_DIRECTORY)
  return plan

##################################################

def get_smtplan(max_length=10, verbose=False):
  """
  Returns a SMTPlan search function configured using the arguments.

  :param max_length: non-negative int for max number of plan steps
  :param verbose: a boolean flag toggling the amount of terminal output
  :return: function wrapper around :func:`.smtplan`
  """
  return lambda p, mt, mc: smtplan(p, max_length, verbose)

# TODO: neither SMTPlan or Temporal FD handle axioms well (they both technically support them though)
# SMTPlan does support quantified preconditions well though

# without -d
"""
0.0:  |(pickup block_1)0_sta| [0.0]
2.0:  |(stack block_1 block_2)1_sta| [0.0]
4.0:  |(pickup block_0)2_sta| [0.0]
6.0:  |(stack block_0 block_1)3_sta| [0.0]
Goal at [6.0]
"""

# with -d
"""
0.0:  |(pickup block_1)0_sta| [0.0]
0.0:  |(armempty)0_0|
0.0:  |(clear block_0)0_0|
0.0:  |(clear block_0)0_1|
0.0:  |(clear block_1)0_0|
0.0:  |(clear block_2)0_0|
0.0:  |(clear block_2)0_1|
0.0:  |(ontable block_0)0_0|
0.0:  |(ontable block_0)0_1|
0.0:  |(ontable block_1)0_0|
0.0:  |(ontable block_2)0_0|
0.0:  |(ontable block_2)0_1|
0.0:  |(holding block_1)0_1|
0.0:  |(total-cost)0_0| == |(total-cost)0_0|
0.0:  |(total-cost)0_1| == |(total-cost)0_1|
2.0:  |(stack block_1 block_2)1_sta| [0.0]
2.0:  |(armempty)1_1|
2.0:  |(clear block_0)1_0|
2.0:  |(clear block_0)1_1|
2.0:  |(clear block_1)1_1|
2.0:  |(clear block_2)1_0|
2.0:  |(ontable block_0)1_0|
2.0:  |(ontable block_0)1_1|
2.0:  |(ontable block_2)1_0|
2.0:  |(ontable block_2)1_1|
2.0:  |(holding block_1)1_0|
2.0:  |(on block_1 block_2)1_1|
2.0:  |(total-cost)1_0| == |(total-cost)1_0|
2.0:  |(total-cost)1_1| == |(total-cost)1_1|
4.0:  |(pickup block_0)2_sta| [0.0]
4.0:  |(armempty)2_0|
4.0:  |(clear block_0)2_0|
4.0:  |(clear block_1)2_0|
4.0:  |(clear block_1)2_1|
4.0:  |(ontable block_0)2_0|
4.0:  |(ontable block_2)2_0|
4.0:  |(ontable block_2)2_1|
4.0:  |(holding block_0)2_1|
4.0:  |(on block_1 block_2)2_0|
4.0:  |(on block_1 block_2)2_1|
4.0:  |(total-cost)2_0| == |(total-cost)2_0|
4.0:  |(total-cost)2_1| == |(total-cost)2_1|
6.0:  |(stack block_0 block_1)3_sta| [0.0]
6.0:  |(armempty)3_1|
6.0:  |(clear block_0)3_1|
6.0:  |(clear block_1)3_0|
6.0:  |(ontable block_2)3_0|
6.0:  |(ontable block_2)3_1|
6.0:  |(holding block_0)3_0|
6.0:  |(on block_0 block_1)3_1|
6.0:  |(on block_1 block_2)3_0|
6.0:  |(on block_1 block_2)3_1|
6.0:  |(total-cost)3_0| == |(total-cost)3_0|
6.0:  |(total-cost)3_1| == |(total-cost)3_1|
Goal at [6.0]
"""
