#!/usr/bin/env python

from stripstream.pddl.logic.predicates import EasyPredicate as Pred
from stripstream.pddl.objects import EasyType as Type, EasyParameter as Param
from stripstream.pddl.logic.connectives import Not, Or, And
from stripstream.pddl.logic.quantifiers import Exists, ForAll
from stripstream.pddl.logic.atoms import Equal
from stripstream.pddl.operators import Action, Axiom
from stripstream.pddl.cond_streams import EasyGenStream as GeneratorStream, EasyTestStream as TestStream
from stripstream.pddl.utils import rename_easy
from stripstream.pddl.problem import STRIPStreamProblem
from stripstream.algorithms.search.fast_downward import get_fast_downward
import cProfile
import pstats

# TODO - make this executable within the API?

# TODO: debug mode that prints the value of things instead of the index

def create_problem(n=50):
  """
  Creates the 1D task and motion planning STRIPStream problem.

  :return: a :class:`.STRIPStreamProblem`
  """

  blocks = ['block%i'%i for i in xrange(n)]
  num_poses = pow(10, 10)

  initial_config = 0 # the initial robot configuration is 0
  initial_poses = {block: i for i, block in enumerate(blocks)} # the initial pose for block i is i

  #goal_poses = {block: i+1 for i, block in enumerate(blocks)} # the goal pose for block i is i+1
  goal_poses = {blocks[0]: 1} # the goal pose for block i is i+1
  #goal_poses = {blocks[0]: 100} # the goal pose for block i is i+1

  ####################

  # Data types
  CONF, BLOCK, POSE = Type(), Type(), Type()

  # Fluent predicates
  AtConf = Pred(CONF)
  AtPose = Pred(BLOCK, POSE)
  IsPose = Pred(BLOCK, POSE)
  HandEmpty = Pred()
  Holding = Pred(BLOCK)
  Moved = Pred() # Prevents double movements

  # Derived predicates
  Safe = Pred(BLOCK, POSE)
  #Unsafe = Pred(BLOCK, BLOCK, POSE)
  Unsafe = Pred(BLOCK, POSE)
  #Unsafe = Pred(POSE)

  # Static predicates
  Kin = Pred(POSE, CONF)
  CFree = Pred(POSE, POSE)
  Collision = Pred(POSE, POSE)

  # Free parameters
  B1, B2 = Param(BLOCK), Param(BLOCK)
  P1, P2 = Param(POSE), Param(POSE)
  Q1, Q2 = Param(CONF), Param(CONF)

  rename_easy(locals()) # Trick to make debugging easier

  ####################

  # TODO: drp_pddl_adl/domains/tmp.py has conditional effects When(Colliding(pose, trajectory), Not(Safe(obj, trajectory))))
  # TODO: maybe this would be okay if the effects really are sparse (i.e. not many collide)

  # http://www.fast-downward.org/TranslatorOutputFormat
  # FastDownward will always make an axiom for the quantified expressions
  # I don't really understand why FastDownward does this... It doesn't seem to help
  # It creates n "large" axioms that have n-1 conditions (removing the Equal)
  # universal conditions: Universal conditions in preconditions, effect conditions and the goal are internally compiled into axioms by the planner.
  # Therefore, heuristics that do not support axioms (see previous point) do not support universal conditions either.
  # http://www.fast-downward.org/PddlSupport

  # TODO: the compilation process actually seems to still make positive axioms for things.
  # The default value is unsafe and it creates positive axioms...
  # A heuristic cost of 4 is because it does actually move something out the way
  # drp_pddl/domains/tmp_separate.py:class CollisionAxiom(Operator, Refinable, Axiom):
  # TODO: maybe I didn't actually try negative axioms like I thought?
  # See also 8/24/16 and 8/26/16 notes
  # Maybe the translator changed sometime making it actually invert these kinds of axioms
  # TODO: maybe this would be better if I did a non-boolean version that declared success if at any pose other than this one
  # It looks like temporal fast downward inverts axioms as well

  actions = [
    Action(name='pick', parameters=[B1, P1, Q1],
      condition=And(AtPose(B1, P1), HandEmpty(), IsPose(B1, P1), Kin(P1, Q1)), # AtConf(Q1),
      effect=And(Holding(B1), Not(AtPose(B1, P1)), Not(HandEmpty()), Not(Moved()))),

    Action(name='place', parameters=[B1, P1, Q1],
      condition=And(Holding(B1), IsPose(B1, P1), Kin(P1, Q1), # AtConf(Q1),
                    #*[Safe(b, P1) for b in blocks]),
                    *[Not(Unsafe(b, P1)) for b in blocks]),
           #*[Not(Unsafe(b, B1, P1)) for b in blocks]),
                    #*[Or(Equal(b, B1), Not(Unsafe(b, B1, P1))) for b in blocks]),
        #ForAll([B2], Or(Equal(B1, B2), Not(Unsafe(B2, P1))))),
        #ForAll([B2], Or(Equal(B1, B2), Safe(B2, P1)))),
        #ForAll([B2], Not(Unsafe(B2, B1, P1)))),
      effect=And(AtPose(B1, P1), HandEmpty(), Not(Holding(B1)), Not(Moved()))),

    # Action(name='place', parameters=[B1, P1, Q1],
    #        condition=And(Holding(B1), AtConf(Q1), IsPose(B1, P1), Kin(P1, Q1),
    #                      #ForAll([B2], Or(Equal(B1, B2),
    #                      #                Exists([P2], And(AtPose(B2, P2), CFree(P1, P2)))))),
    #                      ForAll([B2], Or(Equal(B1, B2), # I think this compiles to the forward axioms that achieve things...
    #                                      Exists([P2], And(AtPose(B2, P2), IsPose(B2, P2), Not(Collision(P1, P2))))))),
    #                      #ForAll([B2], Or(Equal(B1, B2),
    #                      #                Not(Exists([P2], And(AtPose(B2, P2), Not(CFree(P1, P2)))))))),
    #                      #ForAll([B2], Or(Equal(B1, B2), # Generates a ton of axioms...
    #                      #                Not(Exists([P2], And(AtPose(B2, P2), IsPose(B2, P2), Collision(P1, P2))))))),
    #        effect=And(AtPose(B1, P1), HandEmpty(), Not(Holding(B1)), Not(Moved()))),

    #Action(name='place', parameters=[B1, P1, Q1],
    #       condition=And(Holding(B1), AtConf(Q1), IsPose(B1, P1), Kin(P1, Q1), Not(Unsafe(P1))),
    #       effect=And(AtPose(B1, P1), HandEmpty(), Not(Holding(B1)), Not(Moved()))),

    #Action(name='move', parameters=[Q1, Q2],
    #  condition=And(AtConf(Q1), Not(Moved())),
    #  effect=And(AtConf(Q2), Moved(), Not(AtConf(Q1)))),

    # TODO: a lot of the slowdown is because of the large number of move axioms

    # Inferred Safe
    #Translator operators: 1843
    #Translator axioms: 3281
    #Search Time: 10.98

    # Explicit Safe
    #Translator operators: 1843
    #Translator axioms: 3281
    #Search Time: 9.926
  ]

  # TODO: translate_strips_axiom in translate.py

  # TODO: maybe this is bad because of shared poses...
  # 15*15*15*15 = 50625

  # Takeaways: using the implicit collision is good because it results in fewer facts
  # The negated axiom does better than the normal axiom by a little bit for some reason...
  axioms = [
    # For some reason, the unsafe version of this is much better than the safe version in terms of making axioms?
    # Even with one collision recorded, it makes a ton of axioms
    #Axiom(effect=Safe(B2, P1),
    #      condition=Or(Holding(B2), Exists([P2], And(AtPose(B2, P2), IsPose(B2, P2), Not(Collision(P1, P2)))))),

    #Axiom(effect=Unsafe(B2, B1, P1),
    #      condition=And(Not(Equal(B1, B2)),
    #           # Exists([P2], And(AtPose(B2, P2), Not(CFree(P1, P2)))))),
    #            Exists([P2], And(AtPose(B2, P2), Collision(P1, P2))))),

    #Axiom(effect=Unsafe(B2, B1, P1),
    #      condition=Exists([P2], And(AtPose(B2, P2), Not(CFree(P1, P2))))),

    # TODO: I think the inverting is implicitly doing the same thing I do where I don't bother making an axiom if always true
    Axiom(effect=Unsafe(B2, P1),
                condition=Exists([P2], And(AtPose(B2, P2), IsPose(B2, P2), Collision(P1, P2)))), # Don't even need IsPose?
    # This is the best config. I think it is able to work well because it can prune the number of instances when inverting
    # It starts to take up a little time when there are many possible placements for things though
    # TODO: the difference is that it first instantiates axioms and then inverts!

    #Axiom(effect=Unsafe(B2, P1),
    #        condition=Exists([P2], And(AtPose(B2, P2), IsPose(B2, P2), Not(CFree(P1, P2)))))
    # This doesn't result in too many axioms but takes a while to instantiate...

    #Axiom(effect=Unsafe(P1), # Need to include the not equal thing
    #      condition=Exists([B2, P2], And(AtPose(B2, P2), IsPose(B2, P2), Collision(P1, P2)))),

    # TODO: Can turn off options.filter_unreachable_facts
  ]

  ####################

  # Conditional stream declarations
  cond_streams = [
    #GeneratorStream(inputs=[], outputs=[P1], conditions=[], effects=[],
    #                generator=lambda: ((p,) for p in xrange(n, num_poses))),
    GeneratorStream(inputs=[B1], outputs=[P1], conditions=[], effects=[IsPose(B1, P1)],
                    #generator=lambda b: ((p,) for p in xrange(n, num_poses))),
                    generator = lambda b: iter([(n + blocks.index(b),)])), # Unique placements

    GeneratorStream(inputs=[P1], outputs=[Q1], conditions=[], effects=[Kin(P1, Q1)],
                    generator=lambda p: [(p,)]), # Inverse kinematics

    #TestStream(inputs=[P1, P2], conditions=[], effects=[CFree(P1, P2)],
    #           test=lambda p1, p2: p1 != p2, eager=True),
    #           #test = lambda p1, p2: True, eager = True),

    TestStream(inputs=[P1, P2], conditions=[], effects=[Collision(P1, P2)],
               test=lambda p1, p2: p1 == p2, eager=False, sign=False),
  ]

  ####################

  constants = [
    CONF(initial_config) # Any additional objects
  ]

  initial_atoms = [
    AtConf(initial_config),
    HandEmpty(),
  ] + [
    AtPose(block, pose) for block, pose in initial_poses.items()
  ]  + [
    IsPose(block, pose) for block, pose in (initial_poses.items() + goal_poses.items())
  ]

  goal_literals = [AtPose(block, pose) for block, pose in goal_poses.iteritems()]

  problem = STRIPStreamProblem(initial_atoms, goal_literals, actions + axioms, cond_streams, constants)

  return problem

##################################################

from stripstream.algorithms.focused.signed_focused import signed_focused
from stripstream.algorithms.focused.simple_focused import simple_focused
from stripstream.algorithms.incremental.incremental_planner import incremental_planner
from stripstream.pddl.utils import convert_plan

def main():
  """
  Creates and solves the 1D task and motion planning STRIPStream problem.
  """

  pr = cProfile.Profile()
  pr.enable()
  problem = create_problem()
  print problem
  search = get_fast_downward('eager', max_time=10, verbose=True) # dijkstra | astar
  #plan, _ = incremental_planner(problem, search=search, max_time=30, optimal=False)
  #plan, _ = simple_focused(problem, search=search)
  plan, _ = signed_focused(problem, search=search, verbose=True)
  print
  print 'Plan:', convert_plan(plan)
  pr.disable()
  pstats.Stats(pr).sort_stats('cumtime').print_stats(10) # tottime

# TODO - visualize by applying actions to env state

if __name__ == '__main__':
  main()
