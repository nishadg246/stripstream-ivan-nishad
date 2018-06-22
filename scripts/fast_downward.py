from stripstream.algorithms.search.fast_downward import search_options, get_fd_root
from subprocess import call
import argparse
import os

COMMAND = '../../fast-downward.py'
MAX_TIME = 'infinity'
MAX_COST = 'infinity'

def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('domain', help='Domain PDDL file.')
  parser.add_argument('problem', help='Problem PDDL file.')
  parser.add_argument('--search', default='eager', help='FastDownward.')
  args = parser.parse_args()

  assert args.search in search_options
  command = os.path.join(get_fd_root(), COMMAND)
  search = [s.strip('"') for s in (search_options[args.search]%(MAX_TIME, MAX_COST)).split(' ')]
  call([command, args.domain, args.problem] + search)

if __name__ == '__main__':
  main()