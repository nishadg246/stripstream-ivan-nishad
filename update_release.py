#!/usr/bin/env python2

import os
import sys
import shutil, errno
import types

CURRENT_PATH = os.getcwd()
RELEASE_PATH = '/Users/caelan/Programs/LIS/git/stripstream-release'

#cwd = os.getcwd()
#print __file__
#os.walk(directory)

def local_imports():
  for name, val in globals().items():
    if isinstance(val, types.ModuleType):
      yield val

#print sys.modules.keys()
#print list(imports())

def copy_anything(src, dst):
  try:
    shutil.copytree(src, dst)
  except OSError as exc:
    if exc.errno == errno.ENOTDIR:
      dir_path = os.path.dirname(dst)
      if not os.path.exists(dir_path):
        os.makedirs(dir_path)
      shutil.copy(src, dst)
    else: raise

def remove_anything(path):
  if os.path.isdir(path):
    shutil.rmtree(path)
  elif os.path.isfile(path):
    os.remove(path)
  else:
    raise ValueError(path)

print 'Current path:', CURRENT_PATH
print 'Release path:', RELEASE_PATH
print

#################################################################

ignore_paths = [
  '.git',
  '.gitignore',
  'README.md',
  'docs',
]

delete_paths = []
for rel_path in os.listdir(RELEASE_PATH):
  if rel_path not in ignore_paths:
    abs_path = os.path.join(RELEASE_PATH, rel_path)
    delete_paths.append(abs_path)
    print abs_path

response = raw_input('Delete the previous paths? [yes]: ')
if response.lower() in ['', 'y', 'yes']:
  print 'Removed paths'
  for abs_path in delete_paths:
    remove_anything(abs_path)
    #print 'Removed %s'%abs_path
else:
  print 'Skipped removal'

#################################################################

"""
existing_imports = set(local_imports())

#import scripts.run_tutorial
from scripts import run_tutorial
from scripts import __init__

from stripstream import pddl
from stripstream.algorithms import incremental
from stripstream.algorithms.search import bfs
from stripstream.algorithms import universe
from stripstream.algorithms import utils
from stripstream import utils

modules = set(local_imports()) - existing_imports
for module in modules:
  path = os.path.relpath(module.__file__)
  directory = os.path.dirname(path)
  filename = os.path.basename(path)
  if filename.endswith('.pyc'):
    filename = filename[:-1]
  if filename == '__init__.py':
    filename = ''
  new_path = os.path.join(directory, filename)
  rel_paths.append(new_path)
"""

#################################################################

rel_paths = [
  #'.gitignore',
  'LICENSE',
  'sphinx',
]

module_names = [
 'run_all_scripts.py',

 'scripts/run_tutorial.py',
 'scripts/run_pddl_tutorial.py',
 'scripts/run_blocksworld.py',
 'scripts/run_continuous_tamp.py',
 'scripts/run_countable_tamp.py',
 'scripts/openrave/run_fixed_base.py',
 'scripts/pyr2/run_fixed_base.py',
 #'scripts/run_prm.py',

 'fts_scripts/run_openrave_tamp_fixed_base.py',
 'fts_scripts/run_prm.py',
 'fts_scripts/run_tutorial.py',

 'robotics/pyr2',
 'robotics/openrave',
 'robotics/motion2D.py',

 'stripstream/algorithms/incremental',
 'stripstream/algorithms/focused/simple_focused.py',
 'stripstream/algorithms/focused/utils.py',

 'stripstream/algorithms/search/search.py',
 'stripstream/algorithms/search/fast_downward.py',
 'stripstream/algorithms/search/fast_forward.py',
 'stripstream/algorithms/search/lapkt.py',
 'stripstream/algorithms/search/utils.py',

 'stripstream/algorithms/instantiation.py',
 'stripstream/algorithms/plan.py',
 'stripstream/algorithms/universe.py',
 'stripstream/algorithms/utils.py',

 'stripstream/fts/clause.py',
 'stripstream/fts/constraint.py',
 'stripstream/fts/derived.py',
 'stripstream/fts/problem.py',
 'stripstream/fts/sampler.py',
 'stripstream/fts/stripstream_conversion.py',
 'stripstream/fts/utils.py',
 'stripstream/fts/variable.py',

 'stripstream/pddl/logic',
 'stripstream/pddl/cond_streams.py',
 'stripstream/pddl/objects.py',
 'stripstream/pddl/operators.py',
 'stripstream/pddl/problem.py',
 'stripstream/pddl/streams.py',
 'stripstream/pddl/utils.py',

 'stripstream/pddl/examples/continuous_tamp/continuous_tamp.py',
 'stripstream/pddl/examples/continuous_tamp/continuous_tamp_utils.py',
 'stripstream/pddl/examples/continuous_tamp/continuous_tamp_viewer.py',

 'stripstream/pddl/examples/countable_tamp/countable_tamp.py',
 'stripstream/pddl/examples/countable_tamp/countable_tamp_utils.py',
 'stripstream/pddl/examples/countable_tamp/countable_tamp_viewer.py',

 'stripstream/priority_queue.py',
 'stripstream/utils.py',

 'tests',
]
for name in module_names:
  modules = name.split('/')
  for i in range(1, len(modules)):
    init_path = os.path.join(*(modules[:i] + ['__init__.py']))
    if init_path not in rel_paths:
      rel_paths.append(init_path)
  if name not in rel_paths:
    rel_paths.append(name)

print
response = raw_input('%s\nCopy the previous paths? [yes]: '%'\n'.join(rel_paths))
if response.lower() in ['', 'y', 'yes']:
  print 'Copied paths'
  for rel_path in rel_paths:
    src_path = os.path.abspath(rel_path)
    dst_path = os.path.join(RELEASE_PATH, rel_path)
    copy_anything(src_path, dst_path)
else:
  print 'Skipped copy'

# Don't want to directly copy this. Want to rebuild this instead.
#copy_anything(os.path.join(CURRENT_PATH, 'sphinx/_build/html/'),
#              os.path.join(RELEASE_PATH, 'docs/'))

# autopep8 py_file --in-place
# autopep8 . --recursive --in-place --pep8-passes 200 --verbose
# /opt/local/Library/Frameworks/Python.framework/Versions/2.7/bin/pyminifier

#################################################################

from remove_comments import remove_comments

clean_paths = [
  'stripstream',
  'robotics',
  #'fts_scripts', # Don't want to remove comments from these
]

for clean_path in clean_paths:
  for path, dirs, files in os.walk(os.path.join(RELEASE_PATH, clean_path)):
    for rel_path in files:
      if rel_path.endswith('.py'):
        abs_path = os.path.join(path, rel_path)
        print abs_path
        with open(abs_path, 'r') as f:
          s = remove_comments(f.read())
        with open(abs_path, 'w') as f:
          f.write(s)

#################################################################

from subprocess import call

COMMAND = 'autopep8 %s --recursive --in-place --pep8-passes 200 --verbose'%RELEASE_PATH

print
call(COMMAND.split())

#################################################################

#COMMAND = '%s/doc/make html'%RELEASE_PATH
#
#print
#call(COMMAND.split())

# In doc/
# make html
# copy _builds/html/
# push to gh-pages
