# Python Guide

The Python binding provides an object-oriented interface over the native dynibo
library. Install it with `python -m pip install dynibo` and import public types
from `dynibo`.

## Lifetime and errors

Use `Robot` as a context manager so native resources are released
deterministically:

```python
from dynibo import DyniboError, Robot

try:
    with Robot.from_urdf("robot.urdf") as robot:
        print(robot.name)
except DyniboError as error:
    print(f"dynibo: {error}")
```

Invalid model input raises `ModelError`, numerical solver failure raises
`SolverError`, and a panic caught at the native boundary raises `PanicError`.
All inherit from `DyniboError`.

## Arrays and results

Joint inputs accept NumPy arrays or Python sequences of numbers. Contiguous
`float64` arrays use the zero-copy path. Poses and twists are immutable value
objects. Vector and matrix methods return `float64` NumPy arrays; matrices stay
flat and column-major. See
[Frames and Spatial Vectors](../user-guide/frames-and-spatial-vectors.md).

Reuse caller-owned storage in control loops with `out=`:

```python
import numpy as np

q = np.zeros(robot.joint_count)
gravity = np.empty(robot.generalized_count)
robot.gravity(q, out=gravity)
```

Each `Robot` owns one native workspace. Calls on the same instance are
serialized. Use separate robot instances when calculations must run in parallel.

## Floating bases

`FloatingRobot` has its own workspace and never stores a mutable base state.
Supply `BaseState` as the first argument to every calculation:

```python
from dynibo import BaseState, FloatingRobot, Pose

with FloatingRobot.from_urdf("robot.urdf") as robot:
    target = robot.link_id("tool")
    q = [0.0] * robot.joint_count
    base = BaseState(frame=Pose(translation=(0.1, 0.0, 0.0)))
    pose = robot.forward_kinematics(base, q, target)
    mass = robot.mass_matrix(base, q)
```

For floating robots, `generalized_count == joint_count + 6`; generalized
outputs begin with world-frame angular then linear base components. Only fixed
`Robot` exposes `set_base_frame()`.

[Open the Python API reference](../reference/python.md){ .md-button }
