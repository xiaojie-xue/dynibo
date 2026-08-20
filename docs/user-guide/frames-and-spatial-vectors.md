# Frames and Spatial Vectors

Frame conventions are part of the API contract. A numerically correct vector
used in the wrong frame or at the wrong point is still the wrong result.

## Poses

A pose contains translation in metres and a unit quaternion. C, C++, and Python
store quaternion coefficients as `(x, y, z, w)`. Rust uses nalgebra's
`Isometry3<f64>` representation through `Frame`.

Forward kinematics returns the target-link pose in the world frame:

$$
{}^W T_{target}(q) = {}^W T_{base}
\prod_{i \in path} {}^{i-1}T_i(q_i).
$$

## Twists and accelerations

Spatial motion vectors use angular-first order:

```text
[angular_x, angular_y, angular_z, linear_x, linear_y, linear_z]
```

Kinematic velocity and acceleration results are expressed in the world frame
at the documented target origin or tool point. A tool pose is relative to the
target-link frame and selects another rigidly attached point.

## Wrenches and loads

Wrenches use torque-first order:

```text
[torque_x, torque_y, torque_z, force_x, force_y, force_z]
```

External loads are expressed in the selected link's local frame at its origin.
See [External Loads](external-loads.md) for sign and ownership rules.

## Matrix layout

Jacobians are `6 x G`; mass matrices are `G x G`. Flat matrix buffers in every
binding use column-major storage. For a matrix with `rows` rows:

```text
values[column * rows + row]
```

For a Jacobian, each six-element contiguous column is one angular-first spatial
motion response. Convert explicitly when passing a result to a row-major
library.
