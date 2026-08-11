# dynibo

Python bindings for the `dynibo` tree-structured robot kinematics and dynamics
library. The wheel bundles the Rust native library and has no runtime Python
dependencies.

```python
from dynibo import Robot

with Robot("robot.urdf") as robot:
    tool = robot.link_id("tool")
    q = [0.0] * robot.joint_count
    pose = robot.forward_kinematics(q, tool)
    jacobian = robot.jacobian(q, tool)
    gravity = robot.gravity(q)
```

The Jacobian is a flat column-major `6 x N` tuple. With NumPy, convert it using
`np.asarray(jacobian).reshape((6, robot.joint_count), order="F")`.

See the [Python API documentation](https://dynibo.readthedocs.io/) for complete
method documentation. The native Rust API is available on
[docs.rs](https://docs.rs/dynibo).
