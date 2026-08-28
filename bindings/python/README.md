# dynibo

PyO3 bindings for the `dynibo` tree-structured robot kinematics and dynamics
library. The extension calls the Rust core directly and uses NumPy arrays for
zero-copy inputs and efficient outputs.

```python
import numpy as np

from dynibo import Robot

with Robot.from_urdf("robot.urdf") as robot:
    tool = robot.link_id("tool")
    q = np.zeros(robot.joint_count)
    pose = robot.forward_kinematics(q, tool)
    jacobian = robot.jacobian(q, tool)
    gravity = robot.gravity(q)
    velocity_forces = robot.velocity_product_forces(q, [0.0] * robot.joint_count)
```

Vector and matrix results are one-dimensional `float64` NumPy arrays. Matrices
remain flat and column-major for compatibility; reshape a Jacobian with
`jacobian.reshape((6, robot.generalized_count), order="F")`.

Pass a contiguous `float64` array to avoid input copies. Calculation methods
that return arrays also accept a reusable `out=` array:

```python
jacobian = np.empty(6 * robot.generalized_count)
robot.jacobian(q, tool, out=jacobian)
```

Only fixed `Robot` persists a root frame through `set_base_frame()`. Floating
models use a separate type and an explicit, immutable state for every call:

```python
from dynibo import BaseState, FloatingRobot, Pose

with FloatingRobot.from_urdf("robot.urdf") as robot:
    tool = robot.link_id("tool")
    q = np.zeros(robot.joint_count)
    base = BaseState(frame=Pose(translation=(0.2, 0.0, 0.0)))
    pose = robot.forward_kinematics(base, q, tool)
    mass = robot.mass_matrix(base, q)
```

For a floating model, `generalized_count == joint_count + 6`; generalized
outputs begin with world-frame angular then linear base components. Calls on a
single robot object are serialized, so use separate instances for true parallel
execution.

See the [Python API documentation](https://dynibo.readthedocs.io/) for complete
method documentation. The native Rust API is available on
[docs.rs](https://docs.rs/dynibo).
