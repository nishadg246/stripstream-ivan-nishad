from toyTest import Belief, glob, BBhAddBackBSetNew, UniformDist, DD, State, Bd, ObjLoc, makeOperators, objDictCopy, World, hpn, ObjState, Clear

# adapted from toyTest.py

def test0():
  objects0 = {'a' : ['dirty', 'i2']} # NOTE - this is the underlying state
  locations = ('i0', 'i1', 'i2')

  #glob.failProbs = {'Move' : .5, 'Paint' : 0, 'Dry' : 0, 'Wash' : 0, 'Look': .1} # If an observation fails, it might have to repeat
  glob.failProbs = {'Move' : .1, 'Paint' : 0, 'Dry' : 0, 'Wash' : 0, 'Look': .1}
  env = World(objects0)

  start = State([], Belief({'a' : UniformDist(locations)},
                    {'a' : DD({'dirty' : 1.0})},
                    locations))
  goal = State([Bd([ObjLoc(['a']), 'i0', 0.95], True)])
  return env, start, goal

def test1():
  objects1 = {'a' : ['dirty', 'i1'],
              'b' : ['clean', 'i2']}
  locations = ['washer', 'painter', 'dryer', 'i1', 'i2', 'i3', 'i4']

  glob.failProbs = {'Move' : .1, 'Paint' : .2, 'Dry' : .3, 'Wash' : .1, 'Look': .2}
  env = World(objects1)

  op = lambda l: 0.99 if l in ('i1', 'i2') else 0.01
  occDists = dict([(l, DD({True : op(l), False: 1 - op(l)})) \
                      for l in locations])
  start = State([], Belief({'a' : DD({'i1' : 1.0}),
                     'b' : DD({'i2' : 1.0})},
                    {'a' : DD({'dirty' : 1.0}),
                     'b' : DD({'dirty' : 0.5, 'clean' : 0.5})},
                    locations,
                    occDists))
  goal = State([Bd([ObjState(['a']), 'clean', 0.7], True)])
  return env, start, goal

def test2():
    # Do a bunch of steps.  Only uncertainty is in moving and looking
    objects1 = {'a' : ['dirty', 'i1'],
                'b' : ['clean', 'i2']}
    locations = ['washer', 'painter', 'dryer', 'i1', 'i2', 'i3', 'i4']

    glob.failProbs = {'Move' : .2, 'Paint' : 0, 'Dry' : 0, 'Wash' : 0, 'Look': 0.2}
    env = World(objects1)

    start = State([], Belief({'a' : DD({'i1' : 1.0}),
                       'b' : DD({'i2' : 1.0})},
                      {'a' : DD({'dirty' : 1.0}),
                       'b' : DD({'dirty' : 0.5, 'clean' : 0.5})},
                      locations))

    goal = State([Bd([ObjState(['a']), 'dryPaint', 0.8], True)])
    #goal = State([Bd([ObjState(['a']), 'wetPaint', 0.8], True)])
    #goal = State([Bd([ObjState(['a']), 'clean', 0.8], True)])
    #goal = State([Bd([ObjLoc(['a']), 'painter', 0.8], True)])
    return env, start, goal

def test3(useSkeleton = False, hierarchical = True,
          heuristic = BBhAddBackBSetNew):
    # Object in the way; initial poses known
    objects2 = {'a': ['dirty', 'dryer'], 'b': ['dirty', 'washer']}
    locations = ['washer', 'painter', 'dryer', 'i1', 'i2', 'i3', 'i4']

    glob.failProbs = {'Move' : .2, 'Paint' : 0, 'Dry' : 0, 'Wash' : 0, 'Look': 0.2}
    env = World(objects2)

    start = State([], Belief({'a' : DD({'dryer' : 1.0}),
                       'b' : DD({'washer' : 1.0})},
                      {'a' : DD({'dirty' : 1.0}),
                       'b' : DD({'dirty' : 0.5, 'clean' : 0.5})},
                      locations))

    goal = State([Bd([Clear(['washer', []]), True, 0.8], True)])
    return env, start, goal

def test8():
  someLocations = ['washer', 'painter', 'dryer', 'i1', 'i2', 'i3', 'i4']
  allLocations = someLocations + ['i5', 'i6', 'i7', 'i8', 'i9', 'i10',
                                  'i11', 'i12', 'i13', 'i14', 'i15']
  objects2 = {'a': ['dirty', 'dryer'], 'b': ['dirty', 'washer']}

  #glob.failProbs = {'Move' : .2, 'Paint' : 0, 'Dry' : 0, 'Wash' : 0, 'Look': 0.2}
  glob.failProbs = {'Move' : .05, 'Paint' : .02, 'Dry' : .02, 'Wash' : .02,
               'Look': 0.05}
  env = World(objects2)

  start = State([], Belief({'a' : UniformDist(someLocations),
                     'b' : DD({'washer' : 1.0})},
                    {'a' : DD({'dirty' : 1.0}),
                     'b' : DD({'dirty' : 0.5, 'clean' : 0.5})},
                    allLocations))

  goal = State([Bd([ObjState(['a']), 'dryPaint', 0.95], True)])
  return env, start, goal
