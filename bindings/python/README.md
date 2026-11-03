<!-- markdownlint-disable MD033 MD041 -->

<div align="center">

<h1>dynibo</h1>

<p><strong>Robot kinematics and rigid-body dynamics with URDF, Jacobians, inverse kinematics, RNEA and ABA</strong></p>

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

## Algorithms and task recipes

Dynibo provides forward kinematics (FK), world geometric Jacobians and their
time derivatives, damped least-squares inverse kinematics (IK), mass matrices,
gravity compensation, recursive Newton–Euler inverse dynamics (RNEA), and
articulated-body forward dynamics (ABA). Fixed and floating bases use the same
Rust core; IK is available for fixed bases.

| Task | Complete, verified example |
|---|---|
| Load URDF and calculate a tool pose | [Forward kinematics](https://dynibo.readthedocs.io/en/latest/recipes/forward-kinematics/) |
| Compute a Jacobian and reshape for NumPy | [Jacobian and array layout](https://dynibo.readthedocs.io/en/latest/recipes/jacobian/) |
| Calculate holding torque | [Gravity compensation](https://dynibo.readthedocs.io/en/latest/recipes/gravity-compensation/) |
| Calculate torque or recover acceleration | [RNEA and ABA](https://dynibo.readthedocs.io/en/latest/recipes/inverse-forward-dynamics/) |
| Evaluate a floating-base robot | [Floating-base dynamics](https://dynibo.readthedocs.io/en/latest/recipes/floating-base/) |
| Reuse arrays in repeated calculations | [Output buffers](https://dynibo.readthedocs.io/en/latest/recipes/reuse-buffers/) |

Each Python recipe embeds a small URDF and checks its numerical output, with
no ROS or mesh downloads.
See [Choosing Dynibo and migrating from Pinocchio](https://dynibo.readthedocs.io/en/latest/getting-started/choosing-dynibo/)
for supported workloads, API mappings, and frame/array conversion rules.

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
| Jacobian | 1.26× | 1.40× |
| RNEA | 1.19× | 1.56× |
| ABA | 1.76× | 1.94× |

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

The wheel build targets are Linux x86-64 and aarch64 (glibc 2.28+), macOS
Apple Silicon arm64, and Windows x86-64, for CPython 3.9–3.14. Check the
[files for your release](https://pypi.org/project/dynibo/#files); older releases
may not include every target. Alpine/musl wheels are not built.

## Quick start

Install the Python package from PyPI:

```bash
python -m pip install dynibo
```

Load a URDF, compute a target-link pose, and reuse an output buffer for its Jacobian:

```python
"""URDF forward kinematics: run with an installed dynibo package; no model download needed."""

from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np

from dynibo import Robot

URDF = """<robot name="pendulum">
  <link name="base">
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
  <link name="arm">
    <inertial>
      <origin xyz="0.5 0 0"/>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
  <joint name="hinge" type="revolute">
    <parent link="base"/><child link="arm"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="10"/>
  </joint>
  <link name="tool"/>
  <joint name="tip" type="fixed">
    <parent link="arm"/><child link="tool"/>
    <origin xyz="1 0 0"/>
  </joint>
</robot>"""

with TemporaryDirectory() as directory:
    path = Path(directory) / "pendulum.urdf"
    path.write_text(URDF, encoding="utf-8")
    with Robot.from_urdf(path) as robot:
        target = robot.link_id("tool")
        q = np.array([0.3], dtype=np.float64)
        pose = robot.forward_kinematics(q, target)
        expected = (np.cos(q[0]), 0.0, -np.sin(q[0]))
        np.testing.assert_allclose(pose.translation, expected, atol=1e-12)
        assert len(pose.rotation_xyzw) == 4
        print("world tool translation:", pose.translation)
```

This example writes its own model into a temporary directory.
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
