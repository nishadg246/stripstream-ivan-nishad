from collections import namedtuple

from stripstream.utils import random_sequence
from stripstream.pddl.logic.connectives import Not, Or, And
from stripstream.pddl.logic.quantifiers import Exists, ForAll
from stripstream.pddl.logic.atoms import Equal
from stripstream.pddl.operators import Action, Axiom
from stripstream.utils import irange, INF
from stripstream.pddl.problem import STRIPStreamProblem
from stripstream.pddl.cond_streams import EasyGenStream, EasyTestStream
from stripstream.pddl.objects import EasyType as Type, EasyParameter as Param
from stripstream.pddl.logic.predicates import NamedPredicate as Pred # EasyPredicate as Pred

#from stripstream.pddl.examples.toy_tamp.continuous_tamp import sample_region_pose, in_region, are_colliding, inverse_kinematics


# planning related
from stripstream.algorithms.incremental.incremental_planner import incremental_planner
from stripstream.algorithms.search.fast_downward import get_fast_downward
from stripstream.pddl.utils import convert_plan

import numpy as np

Block = namedtuple('Block', ['name', 'w', 'h', 'color'])
REGION_TIME = 0
EAGER_TESTS = True


def are_colliding(b1,x,b2,x2):
  return (x2 >= x and x2 <= (b1.w+x)) or ( (x2+b2.w)>=x and (x2+b2.w)<(b1.w+x)  )

def in_region(ls, lg, block, x):
#  print ls,lg,x, ls <= x  and lg >= x + block.w

  return ls <= x  and \
         lg >= x + block.w

def sample_region_pose(block, ls, lg):
#  print 'sample_region = %d,%d'%(ls,lg)
  return ((lg-block.w)-ls)*np.random.random_sample() + ls

def is_smaller(ls,lg):
  return ls<lg

def sample_block_poses(blocks, ls, lg, timeout=100):
  block_poses = {}
  for block in random_sequence(blocks):
    for _ in irange(0, timeout):
      x = sample_region_pose(block,ls,lg)
      if not any(are_colliding(block, x, block2, x2) for block2, x2 in block_poses.iteritems()):
        block_poses[block] = x
        break
    #else:
    #  import pdb;pdb.set_trace()
    #  return sample_block_poses(blocks)
  return block_poses

def not_in_region(target_block,block,l,ls,lg):
  # checks if block is outside the region ls to lg+target_block.w
  if ls==lg:
    truth_val = False
  if ls<lg:
    truth_val =  (l+block.w<=ls) or (l>=lg+target_block.w)
  else:
    truth_val =  (l+block.w<=lg) or (l>=ls+target_block.w)
  return truth_val

# these are types of parameters of the predicates? Or variables?
OBJECT, LOCATION, STOVE_L_S, STOVE_L_G, SINK_L_S, SINK_L_G = Type(), Type(), Type(), Type(), Type(), Type()

# define predicates
AtPose           = Pred('AtPose',[OBJECT,LOCATION])
InStove          = Pred('InStove',[OBJECT])
InSink           = Pred('InSink',[OBJECT])
Clean            = Pred('Clean',[OBJECT])
Cooked           = Pred('Cooked',[OBJECT])
EmptySweptVolume = Pred('EmptySweptVolume',[OBJECT,LOCATION,LOCATION])


# define static predicates
Contained     = Pred('Contained',[OBJECT,LOCATION,LOCATION,LOCATION])
OutsideRegion = Pred('OutsideRegion',[OBJECT,OBJECT,LOCATION,LOCATION,LOCATION])
IsStove       = Pred('IsStove',[LOCATION,LOCATION])
IsSink        = Pred('IsSink',[LOCATION,LOCATION])
IsSmaller     = Pred('IsSmaller',[LOCATION,LOCATION])
static_pred_names = ['contained','outsideregion','isstove','issink','issmaller']

def solve_incrementally():

  O = Param(OBJECT)
  O1,O2 = Param(OBJECT),Param(OBJECT)
  L = Param(LOCATION)
  L_s,L_g = Param(LOCATION),Param(LOCATION)                 # generic location
  Stove_l_s,Stove_l_g  = Param(STOVE_L_S), Param(STOVE_L_G) # locations for stove and sink
  Sink_l_s,Sink_l_g  = Param(SINK_L_S), Param(SINK_L_G)

  actions = [
    Action(name='wash',
           parameters=[O],
           condition=And( InSink(O) ),
           effect=And( Clean(O) ) ),
    Action(name='cook',
           parameters=[O],
           condition=And( InStove(O), Clean(O) ),
           effect=And( Cooked(O) )),
    Action(name='pickplace',
           parameters=[O,L_s,L_g],
           condition=And(EmptySweptVolume(O,L_s,L_g),AtPose(O,L_s)),
           effect=And( AtPose(O,L_g),Not(AtPose(O,L_s)) )) # You should delete!
   ]

  axioms = [
   # For all objects in the world, either object is O1 or if not, then it is not in the region
   Axiom(effect=EmptySweptVolume(O,L_s,L_g),condition=ForAll([O2],\
                                                  Or(Equal(O,O2),\
                                                      Exists([L],(And(AtPose(O2,L),OutsideRegion(O,O2,L,L_s,L_g))))))),
   # Object is in the stove if it is at pose L for which Ls and Lg define stove
   Axiom(effect=InStove(O),condition=Exists([L,L_s,L_g],And(AtPose(O,L), Contained(O,L,L_s,L_g), IsStove(L_s,L_g)))),
   Axiom(effect=InSink(O),condition=Exists([L,L_s,L_g],And(AtPose(O,L), Contained(O,L,L_s,L_g), IsSink(L_s,L_g)))),
  ]

  cond_streams = [
    EasyGenStream(inputs=[O,L_s,L_g], outputs=[L], conditions=[IsSmaller(L_s,L_g)], effects=[Contained(O,L,L_s,L_g)],\
                  generator=lambda b, ls, lg: (sample_region_pose(b, ls, lg ) for _ in irange(0, INF))),
    EasyTestStream(inputs=[L_s,L_g],conditions=[],effects=[IsSmaller(L_s,L_g)],test=is_smaller,eager=EAGER_TESTS),

    # Generate static predicates that object is contained in sink for which Ls and Lg define the sink. If L was not continuous value,
    # then we would define this in the intial condition and would not be changed by any of the actions (hence static predicate)
    EasyTestStream(inputs=[L_s,L_g,O,L],conditions=[IsSink(L_s,L_g)],effects=[Contained(O,L,L_s,L_g)],test=in_region,eager=EAGER_TESTS),
    EasyTestStream(inputs=[L_s,L_g,O,L],conditions=[IsStove(L_s,L_g)],effects=[Contained(O,L,L_s,L_g)],test=in_region,eager=EAGER_TESTS),

    # OutsideRegion tests if O2 is is outside of the region (Ls,Lg)
    EasyTestStream(inputs=[O,O2,L,L_s,L_g],conditions=[],effects=[OutsideRegion(O,O2,L,L_s,L_g)],test=not_in_region,eager=EAGER_TESTS),
  ]

  ####################
  tamp_problem = sample_one_d_kitchen_problem()

  # instantiate the environment region?
  constants = [
    STOVE_L_S(tamp_problem.stove_region_s),
    STOVE_L_G(tamp_problem.stove_region_g),
    SINK_L_S(tamp_problem.sink_region_s),
    SINK_L_G(tamp_problem.sink_region_g),
  ]

  # define initial state using initial poses of objects
  initial_atoms = [
    AtPose(block, pose) for block, pose in tamp_problem.initial_poses.iteritems()
  ] + [
    IsSink(tamp_problem.sink_region_s,tamp_problem.sink_region_g)
  ] + [
    IsStove(tamp_problem.stove_region_s,tamp_problem.stove_region_g)
  ]  # initial_atoms = static predicates, but on steroid

  goal_literals = []
  subgoal_list = [ InSink(tamp_problem.target_obj), \
                   InStove(tamp_problem.target_obj), Cooked(tamp_problem.target_obj) ]
  for i in range(len(subgoal_list)):
    goal_literals = [subgoal_list[0]]

    stream_problem = STRIPStreamProblem(initial_atoms, goal_literals, \
                                          actions+axioms, cond_streams, constants)
    search = get_fast_downward('eager')
    plan, universe = incremental_planner(stream_problem, search=search,\
                                         frequency=1, verbose=True, max_time=200)
    plan = convert_plan(plan)
    # move the object to the new locations; todo: add new predicates
    if len(plan) > 0: # if no action needed, keep last initial atoms
      initial_atoms = []
    for action in plan:
      action_name = action[0].name
      action_args = action[1]
      if action_name == 'pickplace':
        O   = action_args[0].name
        pick_l = action_args[1]
        place_l = action_args[2]
        block = [b for b in tamp_problem.blocks if b.name==O][0]
        initial_atoms += [AtPose(block,place_l)]
    if len(initial_atoms) == 1:
      block = [b for b in tamp_problem.blocks if b.name!=O][0]
      initial_atoms += [AtPose(block,tamp_problem.initial_poses[block])]
    initial_atoms += [IsSink(tamp_problem.sink_region_s,tamp_problem.sink_region_g)]
    initial_atoms += [IsStove(tamp_problem.stove_region_s,tamp_problem.stove_region_g)]


class KitchenProblem(object):
  pass


def sample_one_d_kitchen_problem(block_w=1., block_h=1.):
  env_region_s = 0.
  env_region_g = 20.
  stove_region_s = 3.
  stove_region_g = 5.
  sink_region_s = 8.
  sink_region_g = 10.

  blocks = [
   # Block('red_obj', block_w, block_h, 'red'),
    Block('blue_obj', block_w, block_h, 'blue'),
    Block('target_block', block_w, block_h, 'green'),
  ]

  initial_regions = [
    env_region_s,
    env_region_g
  ]

  initial_config = 0
  initial_poses = sample_block_poses(blocks, env_region_s, env_region_g)
#  initial_poses[blocks[-1]] =  8.077543580382441
#  initial_poses[blocks[-2]] =  6.738301299306249
  print initial_poses

  tamp_problem = KitchenProblem()
  tamp_problem.initial_poses = initial_poses
  tamp_problem.stove_region_s = stove_region_s
  tamp_problem.stove_region_g = stove_region_g
  tamp_problem.sink_region_s = sink_region_s
  tamp_problem.sink_region_g = sink_region_g
  tamp_problem.target_obj = blocks[-1]
  tamp_problem.blue_obj = blocks[-2]
  tamp_problem.blocks = blocks
  return tamp_problem


if __name__ == '__main__':
  solve_incrementally()
