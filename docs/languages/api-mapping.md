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
| Load floating-base model | `FloatingRobot::from_urdf` | `FloatingRobot.from_urdf` | `dynibo::FloatingRobot(path)` | `dynibo_floating_robot_from_urdf` |
| Resolve fixed/floating link | `robot.link_id` | `robot.link_id` | `robot.link_id` | `dynibo_robot_link_id` / `dynibo_floating_robot_link_id` |
| Fixed forward kinematics | `Robot::forward_kinematics` | `Robot.forward_kinematics` | `Robot.forward_kinematics` | `dynibo_forward_kinematics` |
| Floating forward kinematics | `FloatingRobot::forward_kinematics(base, …)` | `FloatingRobot.forward_kinematics(base, …)` | `FloatingRobot.forward_kinematics(base, …)` | `dynibo_floating_forward_kinematics` |
| Fixed/floating Jacobian | `Robot` / `FloatingRobot` methods | `Robot` / `FloatingRobot` methods | `Robot` / `FloatingRobot` methods | `dynibo_jacobian` / `dynibo_floating_jacobian` |
| Fixed mass/dynamics | `Robot` methods | `Robot` methods | `Robot` methods | `dynibo_mass_matrix`, `dynibo_inverse_dynamics`, `dynibo_forward_dynamics` |
| Floating mass/dynamics | `FloatingRobot` methods with `base` | `FloatingRobot` methods with `base` | `FloatingRobot` methods with `base` | `dynibo_floating_mass_matrix`, `dynibo_floating_inverse_dynamics`, `dynibo_floating_forward_dynamics` |
| Fixed/floating workspace | Owned by typed robot | Owned by typed robot | Owned by typed robot | `DyniboWorkspace` / `DyniboFloatingWorkspace` |
| Matrix output | Caller buffer | Flat NumPy array or reusable `out=` | Flat `std::vector` | Caller buffer |
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
