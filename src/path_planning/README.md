✨ Path planning in robotics
----------------------------
Path planning problems can be divided into two parts, trajectory generation
and smoothness.

✨ Trajectory generation
------------------------
### Covariant Hamiltonian Optimization for Motion Planning (CHOMP)


✨ Trajectory smoothness
------------------------
### Non-uniform rational B-spline (NURBS)

### Discrete trajectory decomposition
In most cases, industrial robot only execute primitive instruct like moveL,
moveC, and etc. While they are expected to perform more complicated motion
with irregular trajectory in complex situation. One solution would be
decomposing the irregular trajectory into sequential arcs and line segments.

#### Example
`example/discrete_trajectory_decomposition.cpp` shows how to use class
`DiscreteTrajectory` to perform trajectory decomposition. After saving the
result, we offer `example/discrete_trajectory_decomposition.py` to visualize
the result of decomposition.
