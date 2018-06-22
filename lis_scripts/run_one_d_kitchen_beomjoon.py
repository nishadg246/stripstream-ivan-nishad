#!/usr/bin/env python

from time import time
from stripstream.utils import set_deterministic
from stripstream.algorithms.plan import get_states
from stripstream.algorithms.incremental.incremental_planner import incremental_planner
from stripstream.algorithms.focused.focused_planner import focused_planner
from stripstream.algorithms.experimental.state_space import progression
from stripstream.algorithms.universe import Universe
from stripstream.algorithms.search.fast_downward import get_fast_downward
from stripstream.algorithms.search.bfs import get_bfs

from stripstream.pddl.utils import convert_plan
#from stripstream.pddl.examples.toy_tamp.continuous_tamp import sample_tamp_problem, compile_problem, \
#  visualize_atoms, visualize_initial, visualize_goal
#from stripstream.pddl.examples.toy_tamp.easy_continuous_tamp import compile_problem, \
#  visualize_atoms, visualize_initial, visualize_goal

from one_d_cooking import in_region, sample_region_pose, \
  sample_block_poses,compile_problem,sample_one_d_kitchen_problem

import argparse
import pickle

def solve_one_d_kitchen(visualize, incremental, focused, deterministic=False):
  if deterministic:
    set_deterministic()
  precond_traj_list = open('precond_traj_list.txt','w'); precond_traj_list.close()
  eff_traj_list = open('eff_traj_list.txt','w');eff_traj_list.close()

  for episode_n in range(400,10000):
    precond_traj_list = open('precond_traj_list.txt','a')
    eff_traj_list = open('eff_traj_list.txt','a')
    tamp_problem = sample_one_d_kitchen_problem() 
    stream_problem,static_pred_names = compile_problem(tamp_problem)
    print stream_problem

    state_viewer = None
    if visualize:
      state_viewer = visualize_initial(tamp_problem, stream_problem)
      raw_input('Continue?')

    search = get_fast_downward('eager') # 'dijkstra | astar | wastar1 | wastar2 | wastar3 | eager | lazy
    plan, universe = incremental_planner(stream_problem, search=search, frequency=1, verbose=True, max_time=200)
    plan = convert_plan(plan)
    if plan is None:
      continue
    precond_traj = [[] for i in range(len(plan))]
    i=0
    for action in plan:
      action_name = action[0].name
      action_args = action[1]
      if action_name == 'pickplace':
        O   = action_args[0].name
        L_s = action_args[1]
        L_g = action_args[2]
        precond_traj[i].append([('emptysweptvolume',(O,L_s,L_g)),('atpose',(O,L_s))])
      elif action_name == 'wash':
        O   = action_args[0].name
        precond_traj[i].append([(('insink'),(O))])
      elif action_name == 'cook':
        O   = action_args[0].name
        precond_traj[i].append([(('instove'),(O))])
      i+=1
    
    eff_traj = [[] for i in range(len(plan))]
    i=0
    for action in plan:
      action_name = action[0].name
      action_args = action[1]
      if action_name == 'pickplace':
        O   = action_args[0].name
        L_s = action_args[1]
        L_g = action_args[2]
        eff_traj[i].append([('atpose',(O,L_g))])
      elif action_name == 'wash':
        O   = action_args[0].name
        eff_traj[i].append([('clean',(O))])
      elif action_name == 'cook':
        O   = action_args[0].name
        eff_traj[i].append([(('cooked'),(O))])
      i+=1

    pickle.dump(precond_traj,open('./data/precond_traj_'+str(episode_n)+'.p','w'))
    pickle.dump(eff_traj,open('./data/eff_traj_'+str(episode_n)+'.p','w'))
    precond_traj_list.write(str(tamp_problem.initial_poses)+'\n')
    eff_traj_list.write(str(tamp_problem.initial_poses)+'\n')
    for p in precond_traj:
      precond_traj_list.write(str(p)+'\n')
    for p in eff_traj:
      eff_traj_list.write(str(p)+'\n')
    precond_traj_list.write('********************\n')
    eff_traj_list.write('********************\n')
    precond_traj_list.close()
    eff_traj_list.close()
    

##################################################

def main():
  parser = argparse.ArgumentParser() # Automatically includes help
  parser.add_argument('-v', action='store_true', help='enable viewer.')
  parser.add_argument('-d', action='store_true', help='display solution.')
  parser.add_argument('-i', action='store_true', help='incremental.')
  parser.add_argument('-f', action='store_true', help='focused.')
  args = parser.parse_args()

  solve_one_d_kitchen(args.v, incremental=args.i, focused=args.f)

if __name__ == '__main__':
  main()
