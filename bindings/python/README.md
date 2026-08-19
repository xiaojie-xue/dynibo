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

The root pose and floating-base motion are stored on `Robot` and are used
consistently by every calculation. Use `set_base_frame()` for either base mode,
or `set_floating_base_state()` to replace the complete floating-base state. Calls on one
`Robot` are serialized so its native workspace can be shared safely between
Python threads; use separate instances for true parallel execution.

See the [Python API documentation](https://dynibo.readthedocs.io/) for complete
method documentation. The native Rust API is available on
[docs.rs](https://docs.rs/dynibo).
