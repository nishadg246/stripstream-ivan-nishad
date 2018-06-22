# stripstream

STRIPStream is a python library for modeling and solving discrete-time planning problems invovling parameters with infinite domains. Example applications include robot task and motion planning, a rocket deploying satellites, a car, mars rover which must recharge during the daytime, UberPool driver that must make a certain amount of money, etc... 

## Documentation

STRIPStream Documentation: https://github.mit.edu/pages/caelan/stripstream/

```stripstream/pddl/``` is modeled after Planning Domain Definition Language (PDDL).
The following is a short tutorial about the elements of a PDDL problem:

http://users.cecs.anu.edu.au/~patrik/pddlman/writing.html

## Installation

Clone the repository using:

```
git clone git@github.mit.edu:caelan/stripstream.git
```

Then, optionally follow the subsequent instructions to install FastDownward.
If you do not wish to install FastDownward, STRIPStream currently supports a simple breadth-first search (BFS) ```search``` subroutine.
However, this is considerably less efficient than FastDownward and will generally only solve small problem instances.

```
plan, _ = incremental_planner(stream_problem, search=get_bfs(), ...)
plan, _ = focused_planner(stream_problem, search=get_bfs(), ...)
```

## FastDownward

STRIPStream supports using the FastDownward planning system to implement the ```search``` subroutine.
In the future, it will support integration with other PDDL planners.
Follow these instructions to install FastDownward: 

http://www.fast-downward.org/ObtainingAndRunningFastDownward

Once installed, set the environment variable FD_PATH to be the build of the FastDownward installation.
It should look similar to ```.../FastDownward/builds/release32/``` depending on directory and chosen release.

Then, you can use ```get_fast_downward()``` in order to produce a FastDownward search subroutine for both the incremental and focused planners.

```
plan, _ = incremental_planner(stream_problem, search=get_fast_downward(), ...)
plan, _ = focused_planner(stream_problem, search=get_fast_downward(), ...)
```

See the FastDownward Google Group if you need help with the installation:

https://groups.google.com/forum/#!forum/fast-downward

## Tutorial

The following tutorial implements a countable task and motion planning (TAMP) problem:

https://github.mit.edu/pages/caelan/stripstream/tutorial.html

You can run this tutorial using the following command:

```
python -m scripts.run_tutorial
```

You can also run the following to-be-documented tutorials:

```
python -m scripts.run_pddl_tutorial
python -m scripts.run_blocksworld
python -m scripts.run_car
```

## Examples

The following two files are example STRIPStream formulations which do not require any additional dependencies:

```
stripstream/pddl/examples/toy_tamp/easy_countable_tamp.py
stripstream/pddl/examples/toy_tamp/easy_continuous_tamp.py
```

These examples can solved using the following scripts:

```
python -m scripts.run_countable_tamp [-fd] [-focus] [-display]
python -m scripts.run_continuous_tamp [-fd] [-focus] [-display]
```

The flag [-fd] uses FastDownward instead of a breadth-first search (BFS) to solve the problem.

The flag [-focus] uses the focused algorithm instead of the incremental algorithm to solve the problem.

The flag [-display] uses a visualizer to display the solution. The visualizer uses Tkinter. You can install Tkinter using ```sudo pip install python-tk```.

## OpenRAVE Examples

The following examples requires OpenRAVE, a robotics simulator, as well as my OpenRAVE utility functions. Install both by following these instructions https://github.mit.edu/caelan/lis-openrave and adding lis-openrave to your PYTHONPATH. You can run the following examples:

```
python -m scripts.run_tamp [--problem] [-focus] [-display]
python -m scripts.run_constraint [--problem] [-focus] [-display]
```

The argument [--problem] specifies the TAMP problem to solve. The default is ```--p simple```.

The flag [-focus] uses the focused algorithm to solve the problem.

The flag [-display] displays the plan if it is found.

## Testing

The following command executes tests that do not require FastDownward:

```
python -m unittest tests.test_countable_bfs
```

The following command executes tests that require FastDownward:

```python -m unittest tests.test_countable_fd```

The following command executes all tests:

```python -m unittest discover -s tests```

Additional tests will be added in the future.

## Publications

STRIPStream: Planning In Infinite Domains
https://arxiv.org/abs/1701.00287

## Citation

Caelan R. Garrett, Tomas Lozano-Perez, Leslie P. Kaelbling. STRIPS Planning in Infinite Domains, 2016.

## Contact

Email Caelan Garrett at caelan@mit.edu if you have any issues.
STRIPStream is still in early development, so it certainly has bugs.
