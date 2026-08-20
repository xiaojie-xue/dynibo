# API Mapping

Dynibo exposes one numerical model through four idiomatic interfaces. Operation
names remain recognizable; resource ownership, buffers, and errors follow each
language's conventions.

Rust uses crate and type namespaces, Python uses modules and classes, and C++
uses `namespace dynibo`. C has no namespace, so every stable ABI symbol carries
the `dynibo_` library prefix. The prefix is the C spelling of a namespace, not a
different numerical API.

| Concept | Rust | Python | C++ | C |
|---|---|---|---|---|
| Load fixed-base model | `Robot::from_urdf` | `Robot.from_urdf` | `dynibo::Robot(path)` | `dynibo_robot_from_urdf` |
| Load with base mode | `Robot::from_urdf_with_base` | `Robot.from_urdf_with_base` | `dynibo::Robot(path, mode)` | `dynibo_robot_from_urdf_with_base` |
| Resolve link | `robot.link_id` | `robot.link_id` | `robot.link_id` | `dynibo_robot_link_id` |
| Forward kinematics | `robot.forward_kinematics` | `robot.forward_kinematics` | `robot.forward_kinematics` | `dynibo_forward_kinematics` |
| Jacobian | `robot.jacobian` | `robot.jacobian` | `robot.jacobian` | `dynibo_jacobian` |
| Mass matrix | `robot.mass_matrix` | `robot.mass_matrix` | `robot.mass_matrix` | `dynibo_mass_matrix` |
| Inverse dynamics | `robot.inverse_dynamics` | `robot.inverse_dynamics` | `robot.inverse_dynamics` | `dynibo_inverse_dynamics` |
| Workspace | Explicit argument | Owned by `Robot` | Owned by `Robot` | Explicit handle |
| Matrix output | Caller buffer | Flat tuple | Flat `std::vector` | Caller buffer |
| Errors | `Result<T>` | Exceptions | `dynibo::Error` | `DyniboStatus` |

## Value types

| Meaning | Rust | Python | C++ | C |
|---|---|---|---|---|
| Pose | `Frame` | `Pose` | `DyniboPose` | `DyniboPose` |
| Spatial motion | `Twist` | `Twist` | `DyniboTwist` | `DyniboTwist` |
| External load | `IndexedLoad` | `Load` | `DyniboLoad` | `DyniboLoad` |
| IK options | `InverseKinematicsOptions` | `IkOptions` | `DyniboIkOptions` | `DyniboIkOptions` |
| Link identifier | `LinkId` | `int` | `std::size_t` | `size_t` |

The C++ wrapper deliberately reuses ABI-compatible C value structs while
adding RAII, methods, move semantics, and exceptions around object handles.

See the [User Guide](../user-guide/index.md) for shared semantics and the
individual language pages for integration details.
