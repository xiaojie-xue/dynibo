<!-- markdownlint-disable MD033 MD041 -->

<div align="center">

<h1>dynibo</h1>

<p><strong>Dynamics for the Loop</strong></p>

<p>
  <a href="https://dynibo.readthedocs.io/">Documentation</a> &nbsp;&middot;&nbsp;
  <a href="https://github.com/xiaojie-xue/dynibo">GitHub</a>
</p>

<p>
  <a href="https://github.com/xiaojie-xue/dynibo/actions/workflows/package-ci.yml"><img alt="CI" src="https://github.com/xiaojie-xue/dynibo/actions/workflows/package-ci.yml/badge.svg?branch=main"></a>
  <a href="https://codecov.io/gh/xiaojie-xue/dynibo"><img alt="codecov" src="https://codecov.io/gh/xiaojie-xue/dynibo/branch/main/graph/badge.svg"></a>
  <a href="https://pypi.org/project/dynibo/"><img alt="PyPI" src="https://img.shields.io/pypi/v/dynibo.svg?color=3776AB&amp;logo=python&amp;logoColor=white"></a>
  <a href="https://github.com/xiaojie-xue/dynibo/blob/main/LICENSE"><img alt="License: MIT" src="https://img.shields.io/badge/license-MIT-blue.svg"></a>
</p>

</div>

`dynibo` provides robot kinematics and dynamics for controller development
through a Python API backed by a Rust core. It loads robot models from URDF
and supports manipulators, humanoids, and other robots with fixed or floating
bases.

## Features

### Fast

Each robot owns reusable storage for native calculations. Contiguous `float64`
NumPy inputs avoid copies, and array-returning methods accept reusable `out=`
buffers to avoid allocating a new output array on every call.

To put computation speed in context, we benchmark Dynibo against
[Pinocchio](https://github.com/stack-of-tasks/pinocchio), an open-source library
for robot kinematics and dynamics. The benchmarks use Franka, a fixed-base
manipulator with 7 joints, and unitree G1, a floating-base humanoid with 29 joints.
The table below shows Dynibo's speedup using the Python interfaces of both libraries.

| Operation | Franka | unitree G1 |
|---|---:|---:|
| Jacobian | 1.28× | 1.38× |
| RNEA | 1.17× | 1.54× |
| ABA | 1.81× | 1.89× |

Source code to reproduce these results is available in
[`benches/`](https://github.com/xiaojie-xue/dynibo/tree/main/benches).

### Reliable

Tests cover fixed and floating bases, mixed joint types, external loads,
invalid inputs, and repeated use of calculation storage. Numerical results
are checked against finite differences, consistency relations between
algorithms, and an independent Pinocchio reference. The installed Python
package is tested separately. See the
[test architecture](https://github.com/xiaojie-xue/dynibo/blob/main/tests/TESTING.md)
for details.

### Easy to Use

Load a URDF with `Robot` or `FloatingRobot`, then call kinematics and dynamics
methods directly. Each object manages its internal calculation storage, so
there are no separate `Model` and `Data` objects to maintain. Inputs accept
NumPy arrays or Python sequences, and errors are reported as Python exceptions.

## Dependencies

Requires Python 3.9 or newer and NumPy 1.23 or newer. Prebuilt wheels bundle
the native library.

## Quick start

Install the Python package from PyPI:

```bash
python -m pip install dynibo
```

Load a URDF, compute a target-link pose, and reuse an output buffer for its Jacobian:

```python
import numpy as np

from dynibo import Robot

with Robot.from_urdf("robot.urdf") as robot:
    tool = robot.link_id("tool")
    q = np.zeros(robot.joint_count)
    pose = robot.forward_kinematics(q, tool)
    jacobian = np.empty(6 * robot.generalized_count)
    robot.jacobian(q, tool, out=jacobian)
    print(pose.translation)
```

Replace `robot.urdf` and `tool` with your model path and target-link name.
Vector and matrix results are one-dimensional `float64` NumPy arrays;
matrices use column-major order.

## Floating bases

Use `FloatingRobot` for floating-base models and pass a `BaseState` explicitly
to each calculation:

```python
import numpy as np

from dynibo import BaseState, FloatingRobot, Pose

with FloatingRobot.from_urdf("robot.urdf") as robot:
    tool = robot.link_id("tool")
    q = np.zeros(robot.joint_count)
    base = BaseState(frame=Pose(translation=(0.2, 0.0, 0.0)))
    pose = robot.forward_kinematics(base, q, tool)
    mass = robot.mass_matrix(base, q)
```

For floating robots, `generalized_count == joint_count + 6`. Generalized
velocities and forces begin with world-frame angular then linear base components.

## Examples and documentation

Complete Python examples are available in
[`examples/python/`](https://github.com/xiaojie-xue/dynibo/tree/main/examples/python).
See the [Python guide](https://dynibo.readthedocs.io/en/latest/languages/python/)
for array layouts, output reuse, and thread safety, and the
[Python API reference](https://dynibo.readthedocs.io/en/latest/reference/python/)
for complete method documentation.

## License

Dynibo code is licensed under
[MIT](https://github.com/xiaojie-xue/dynibo/blob/main/LICENSE).
