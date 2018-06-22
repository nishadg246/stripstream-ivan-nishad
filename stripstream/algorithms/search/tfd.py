#from os.path import expanduser
import os
import subprocess

from stripstream.utils import write, read, ensure_dir, safe_remove, remove_dir
from stripstream.algorithms.search.utils import TEMP_DIRECTORY, DOMAIN_PATH, PROBLEM_PATH
from stripstream.algorithms.search.fast_downward import lookup_action

ENV_VAR = 'TFD_PATH'

COMMAND = 'plan.py y+Y+a+e+r+O+1+C+1+b {} {} {}'
PLAN_FILE = 'plan'

# Finds a plan and then retimes it

##################################################

def has_tfd():
  return True
  #return ENV_VAR in os.environ

def get_tfd_root():
  return '/home/caelan/Programs/Planners/tfd-src-0.4/downward'

  if not has_tfd():
    raise RuntimeError('Environment variable %s is not defined.'%ENV_VAR)
  return os.environ[ENV_VAR]
  #return expanduser(os.environ[ENV_VAR])

# TODO: can only really move one action at a time

def tfd(universe, verbose): 
  remove_dir(TEMP_DIRECTORY) # Ensures not using old plan
  ensure_dir(TEMP_DIRECTORY)
  write(DOMAIN_PATH, universe.domain_pddl(costs=True, derived=True, durative=True))
  write(PROBLEM_PATH, universe.problem_pddl(costs=True))

  plan_path = os.path.join(TEMP_DIRECTORY, PLAN_FILE)
  #assert not actions, "There shouldn't be any actions - just temporal actions"

  command = os.path.join(get_tfd_root(), COMMAND.format(DOMAIN_PATH, PROBLEM_PATH, plan_path))
  if verbose: print command

  paths = [os.path.join(os.getcwd(), p) for p in (DOMAIN_PATH, PROBLEM_PATH, plan_path)]
  args = os.path.join('.', COMMAND.format(*paths)).split(' ')
  stdout = None if verbose else open(os.devnull, 'w')
  #stdout = open(os.devnull, 'w')
  #stderr = open(os.devnull, 'w')
  stderr = None
  try:
    proc = subprocess.Popen(args, cwd=get_tfd_root(), stdout=stdout, stderr=stderr)
    proc.wait()
    #proc.terminate()
  except subprocess.CalledProcessError, e:
    print "Subprocess error", e.output
    raw_input("Continue?")

  temp_path = os.path.join(os.getcwd(), TEMP_DIRECTORY)
  plan_files = sorted([f for f in os.listdir(temp_path) if f.startswith(PLAN_FILE)])
  if verbose:
    print plan_files

  if not plan_files:
    return None
  output = None
  with open(os.path.join(temp_path, plan_files[-1]), 'r') as f:
    output = f.read()
  plan = []
  for line in output.split('\n')[:-1]:
    entries = line[line.find('(')+1:line.find(')')].split(' ')
    plan.append(lookup_action(universe, entries[0], entries[1:]))
  if not verbose:
    remove_dir(TEMP_DIRECTORY)
  return plan

##################################################

def get_tfd(verbose=True):
  """
  Returns a Temporal FastDownward search function configured using the arguments.

  :param verbose: a boolean flag toggling the amount of terminal output
  :return: function wrapper around :func:`.tfd`
  """
  return lambda p, mt, mc: tfd(p, verbose)
