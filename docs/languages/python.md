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

Joint inputs accept Python sequences of numbers. Poses and twists are immutable
value objects. Matrix methods return flat tuples in column-major order; see
[Frames and Spatial Vectors](../user-guide/frames-and-spatial-vectors.md).

Each `Robot` owns one native workspace. Calls on the same instance are
serialized. Use separate robot instances when calculations must run in parallel.

[Open the Python API reference](../reference/python.md){ .md-button }
