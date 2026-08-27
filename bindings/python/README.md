# dynibo

Python bindings for the `dynibo` tree-structured robot kinematics and dynamics
library. The wheel bundles the Rust native library and has no runtime Python
dependencies.

```python
from dynibo import Robot

with Robot.from_urdf("robot.urdf") as robot:
    tool = robot.link_id("tool")
    q = [0.0] * robot.joint_count
    pose = robot.forward_kinematics(q, tool)
    jacobian = robot.jacobian(q, tool)
    gravity = robot.gravity(q)
    velocity_forces = robot.velocity_product_forces(q, [0.0] * robot.joint_count)
```

The Jacobian is a flat column-major `6 x N` tuple. With NumPy, convert it using
`np.asarray(jacobian).reshape((6, robot.generalized_count), order="F")`.

Only fixed `Robot` persists a root frame through `set_base_frame()`. Floating
models use a separate type and an explicit, immutable state for every call:

```python
from dynibo import BaseState, FloatingRobot, Pose

with FloatingRobot.from_urdf("robot.urdf") as robot:
    tool = robot.link_id("tool")
    q = [0.0] * robot.joint_count
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
