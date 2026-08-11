# dynibo Python guide

`dynibo` provides Python bindings for fast, runtime-sized robot kinematics and
dynamics. It loads a tree-structured robot from URDF and bundles the Rust native
library in its Python wheels, so it has no runtime Python dependencies.

## Installation

Install the package from PyPI:

```bash
python -m pip install dynibo
```

Python 3.9 or newer is required.

## Quick start

```python
from dynibo import Robot

with Robot("robot.urdf") as robot:
    tool = robot.link_id("tool")
    q = [0.0] * robot.joint_count

    pose = robot.forward_kinematics(q, tool)
    jacobian = robot.jacobian(q, tool)
    gravity = robot.gravity(q)
```

The context manager releases native resources when the block exits. Calling
`Robot.close()` explicitly is equivalent and is safe to do more than once.

## Data conventions

- Joint vectors contain one value per joint in the model's topological order.
- Positions and translations use metres; angular quantities use radians.
- Quaternions use `(x, y, z, w)` order.
- Spatial vectors are angular-first: `(angular, linear)`.
- External loads are torque-first: `(torque, force)` and expressed in the
  selected link's coordinate frame.
- Jacobians are returned as flat, column-major `6 x joint_count` tuples. Each
  column is `(angular_x, angular_y, angular_z, linear_x, linear_y, linear_z)`.

To convert a Jacobian to a NumPy array:

```python
import numpy as np

matrix = np.asarray(jacobian).reshape((6, robot.joint_count), order="F")
```

NumPy is optional and is not installed with `dynibo`.

## Mathematical notation

For a robot with $N$ joints, the joint position, velocity, and acceleration
vectors are $q, \dot{q}, \ddot{q} \in \mathbb{R}^N$. The geometric Jacobian
maps joint velocity to an angular-first spatial velocity:

$$
\mathcal{V} = J(q)\dot{q}.
$$

The inverse-kinematics solver uses the damped least-squares update

$$
\Delta q = J^{\mathsf{T}}
\left(JJ^{\mathsf{T}} + \lambda^2 I\right)^{-1} e,
$$

where $e$ is the target-pose error and $\lambda$ is `IkOptions.damping`.

## Errors

Invalid Python arguments and invalid native inputs raise `ValueError`. Model
loading failures, iterative solver failures, and unexpected native failures use
the exceptions derived from `DyniboError`:

```python
from dynibo import ModelError, Robot, SolverError

try:
    robot = Robot("robot.urdf")
except ModelError as error:
    print(f"could not load robot: {error}")
```

## Concurrency

Each `Robot` owns one reusable native calculation workspace. That workspace is
not thread-safe, so do not call calculation methods concurrently on the same
instance. Create a separate `Robot` instance for each concurrent worker.

See the [Python API](python-api.md) for complete signatures and method details.
The Rust interface is documented separately on
[docs.rs](https://docs.rs/dynibo).
