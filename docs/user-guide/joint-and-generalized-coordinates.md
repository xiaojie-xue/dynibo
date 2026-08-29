# Joint and Generalized Coordinates

Dynibo distinguishes joint-state inputs from generalized outputs. This matters
most for floating-base models.

## Dimensions

Let:

- `J = joint_count`: the number of non-fixed URDF joints.
- `G = generalized_count`: the dimension of Jacobian columns, mass matrices,
  and generalized-force vectors.

For a fixed base, `G = J`. For a floating base, `G = J + 6`.

Inputs `q`, `qd`, and `qdd` always contain exactly `J` values in non-fixed URDF
joint order. Floating-base coordinates are not prepended to these arrays. A
fixed `Robot` stores its root frame; a `FloatingRobot` receives pose, velocity,
and acceleration through an explicit `BaseState` on every calculation.

## Generalized ordering

Fixed-base generalized vectors use joint order directly. Floating-base vectors
use this order:

```text
[base angular xyz, base linear xyz, non-fixed URDF joints...]
```

For generalized forces, the first six values are the world-frame root wrench:

```text
[base torque xyz, base force xyz, scalar joint forces...]
```

The same ordering defines Jacobian columns and both axes of the mass matrix.

## Buffer sizes

| Quantity | Number of values |
|---|---:|
| `q`, `qd`, `qdd` | `J` |
| Pose | translation 3 + quaternion 4 |
| Twist | 6 |
| Jacobian or its derivative | `6 * G` |
| Mass matrix | `G * G` |
| Generalized force | `G` |

C and Rust require caller-provided output buffers for matrix and generalized
force operations. Python returns NumPy arrays or writes into `out=`, while C++
allocates a vector; the result has the same dimensions.

## Joint limits

URDF limits belong to the model. Inverse kinematics applies supported joint
limits while updating its candidate state. Other calculations validate array
length and finite values but do not silently clamp the supplied joint state.
