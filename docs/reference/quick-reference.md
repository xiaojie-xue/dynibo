# Conventions and API Quick Reference

Single-page reference for all shared numerical conventions, Pinocchio
differences, mathematical symbols, and public operations across Rust, Python,
C++, and C.

## Core conventions

| Topic | Rule |
|---|---|
| Units | SI: metres (m), radians (rad), seconds (s), kilograms (kg), newtons (N), newton-metres (N·m) |
| Joint count `J` | Number of non-fixed URDF joints (`revolute`, `continuous`, `prismatic`) |
| Joint order | Breadth-first topological order from the root link (children of the same parent preserve URDF joint declaration order); inspect with `joint_name(i)` in Rust |
| Generalized count `G` | Fixed base: `G == J`; floating base: `G == J + 6` |
| Joint inputs (`q`, `qd`, `qdd`) | Always length `J`; never prepend base pose or velocity |
| Floating generalized order | `[base_angular_xyz, base_linear_xyz, J joints]` for velocities/accelerations; `[base_torque_xyz, base_force_xyz, J joint forces]` for forces (world frame, at root origin) |
| Quaternion order | `(x, y, z, w)` unit quaternion in Python, C++, and C; `nalgebra::Isometry3<f64>` (`Frame`) in Rust |
| Twists (`Twist` / `DyniboTwist`) | Angular-first `[wx, wy, wz, vx, vy, vz]`; **classical** velocity and **classical** acceleration in the world frame (not Featherstone spatial acceleration, which differs by $\omega \times v$) |
| External loads (`Wrench` / `Load` / `DyniboLoad`) | Torque-first `[tx, ty, tz, fx, fy, fz]`, expressed in the **target link's local frame at its origin**, as a **resisting** wrench (`+` in `gravity` / `inverse_dynamics`, `-` in `forward_dynamics`; opposite sign to a physical force applied to the robot) |
| Matrix layout | Flat 1-D **column-major**: `values[col * rows + row]`. Jacobian / derivative: `6 * G`; mass matrix: `G * G`. In Python/NumPy: `arr.reshape((rows, cols), order="F")` |
| `forward_kinematics_all` layout | All `link_count` world poses in link-ID order (index `0` is root). Python returns flat `7 * link_count` (`reshape((link_count, 7))`, rows `[x, y, z, qx, qy, qz, qw]`) |
| Default gravity | `[0.0, 0.0, -9.80665]` m/s² in the world frame; configurable per robot/workspace via `set_gravity` |
| `forward_dynamics` base state | Uses `BaseState.frame` and `BaseState.velocity`; **ignores** `BaseState.acceleration`. Set the first 6 generalized forces to zero for an unactuated floating base |
| Default IK options (fixed-base only) | `max_iterations = 100`, `translation_tolerance = 1e-6` m, `rotation_tolerance = 1e-6` rad, `damping = 1e-3`, `max_step_norm = 0.5` |
| Concurrency | One calculation at a time per `Robot`/`FloatingRobot`/C workspace. Use `fork()` (Rust/Python), separate instances (C++), or separate workspaces (C) for parallel execution |

## Differences from Pinocchio

If you are migrating code from Pinocchio, note these convention differences:

| Convention | dynibo | Pinocchio (default) |
|---|---|---|
| Spatial / Jacobian row order | **Angular first**: `[angular_xyz, linear_xyz]` | **Linear first**: `[linear_xyz, angular_xyz]` |
| Reference frame of `jacobian` & kinematics | World-aligned at target origin (equivalent to `LOCAL_WORLD_ALIGNED`), **classical** acceleration | Selectable (`LOCAL`, `WORLD`, `LOCAL_WORLD_ALIGNED`); spatial vs classical acceleration |
| Floating-base representation | Explicit `BaseState` argument; `q` has length `J`; generalized base coords are world-frame `[ang, lin]` | Free-flyer joint inside `q` (`7 + J`, `[xyz, xyzw]`) and `v` (`6 + J`, local-frame `[lin, ang]`) |
| External wrench sign | **Resisting** wrench in link-local frame (`+` in RNEA, `-` in ABA) | Applied external force `fext` (`-` in RNEA, `+` in ABA) |
| Branched-tree joint order | **Breadth-first** from root | **Depth-first** from root |
| Calculation storage | Owned per `Robot` / `FloatingRobot` (`state()` snapshot in Rust; explicit workspace in C) | Separate `Model` and `Data` objects |

## Mathematical symbol mapping

| Symbol | Meaning | Corresponding API / layout |
|---|---|---|
| $q, \dot q, \ddot q \in \mathbb{R}^J$ | Active joint positions, velocities, accelerations | `q`, `qd`, `qdd` (length `J`, BFS topological order) |
| $\nu, \dot\nu \in \mathbb{R}^G$ | Generalized velocity and classical acceleration | Fixed: $\dot q, \ddot q$; Floating: `[base_ang, base_lin, qd/qdd]` |
| ${}^W T_{\text{target}}(q)$ | Target-link pose in world frame | `forward_kinematics` (`Frame` / `Pose` / `DyniboPose`) |
| ${}^W V_{\text{target}} = J(q)\nu$ | World-frame classical velocity twist at target origin | `forward_velocity_kinematics` (at `tool = identity`) |
| ${}^W A_{\text{target}} = J(q)\dot\nu + \dot J(q,\nu)\nu$ | World-frame classical acceleration twist at target origin | `forward_acceleration_kinematics` |
| $J(q), \dot J(q,\nu) \in \mathbb{R}^{6 \times G}$ | World-frame target-origin geometric Jacobian and time derivative | `jacobian`, `jacobian_derivative` (flat column-major `6 * G`) |
| $M(q) \in \mathbb{R}^{G \times G}$ | Symmetric generalized inertia matrix | `mass_matrix` (flat column-major `G * G`) |
| $C(q,\nu)\nu \in \mathbb{R}^G$ | Coriolis and centrifugal generalized forces | `velocity_product_forces` (length `G`) |
| $g(q) \in \mathbb{R}^G$ | Gravity generalized forces (optionally plus $\tau_{\mathrm{load}}$) | `gravity` (length `G`) |
| $\tau = M\dot\nu + C\nu + g + \tau_{\mathrm{load}}$ | RNEA required generalized effort | `inverse_dynamics` (length `G`) |
| $\dot\nu = M^{-1}(\tau - C\nu - g - \tau_{\mathrm{load}})$ | ABA generalized acceleration | `forward_dynamics` (length `G`) |

## Complete four-language API table

For floating-base operations, prepend `base` (`&BaseState` in Rust, `BaseState` in Python/C++, `const DyniboBaseState *` in C) and use `FloatingRobot` / `dynibo_floating_*`.

| Operation | Output size | Rust (`Robot` / `FloatingRobot`) | Python (`Robot` / `FloatingRobot`) | C++ (`dynibo::Robot` / `FloatingRobot`) | C (`dynibo.h`) |
|---|---|---|---|---|---|
| Load fixed / floating | Handle | `Robot::from_urdf` / `FloatingRobot::from_urdf` | `Robot.from_urdf` / `FloatingRobot.from_urdf` | `Robot(path)` / `FloatingRobot(path)` | `dynibo_[floating_]robot_from_urdf` |
| Immutable shared model | Handle | `RobotModel::from_urdf`, `robot.model()` | — | — | `Dynibo[Floating]Robot` |
| Independent parallel copy | Handle | `fork()` | `fork()` | Separate instance | `dynibo_[floating_]workspace_create` |
| Counts & name | Scalar | `name`, `joint_count`, `generalized_count`, `link_count` | `.name`, `.joint_count`, `.generalized_count`, `.link_count` | `name()`, `joint_count()`, `generalized_count()`, `link_count()` | `dynibo_[floating_]robot_{name,joint_count,generalized_count,link_count}` |
| Link lookup | `LinkId` / `size_t` | `link_id`, `link_id_at`, `root_link_id` | `link_id`, `link_id_at` | `link_id`, `link_id_at` | `dynibo_[floating_]robot_link_id` |
| Joint & link metadata | Scalar / Vec / Mat | `joint_{name,type,lower_limit,upper_limit,velocity_limit}`, `link_{name,mass,center_of_mass,inertia}` | — | — | — |
| Fixed base frame | — | `set_base_frame`, `base_frame` | `set_base_frame` | `set_base_frame` | `dynibo_workspace_set_base_frame` |
| Gravity vector | 3 | `set_gravity`, `gravity_vector` | `set_gravity`, `.gravity_vector` | `set_gravity`, `gravity_vector` | `dynibo_[floating_]workspace_{set_gravity,gravity}` |
| Single-link FK | 1 pose | `forward_kinematics` | `forward_kinematics` | `forward_kinematics` | `dynibo_[floating_]forward_kinematics` |
| All-link FK | `link_count` poses (`7*L` in Py) | `forward_kinematics_all` | `forward_kinematics_all(..., out=)` | `forward_kinematics_all[_into]` | `dynibo_[floating_]forward_kinematics_all` |
| Velocity kinematics | 1 twist | `forward_velocity_kinematics` | `forward_velocity_kinematics` | `forward_velocity_kinematics` | `dynibo_[floating_]forward_velocity_kinematics` |
| Acceleration kinematics | 1 twist | `forward_acceleration_kinematics` | `forward_acceleration_kinematics` | `forward_acceleration_kinematics` | `dynibo_[floating_]forward_acceleration_kinematics` |
| Jacobian | `6 * G` | `jacobian` | `jacobian(..., out=)` | `jacobian[_into]` | `dynibo_[floating_]jacobian` |
| Jacobian derivative | `6 * G` | `jacobian_derivative` | `jacobian_derivative(..., out=)` | `jacobian_derivative[_into]` | `dynibo_[floating_]jacobian_derivative` |
| Inverse kinematics (fixed only) | `J` | `inverse_kinematics` | `inverse_kinematics(..., out=)` | `inverse_kinematics[_into]` | `dynibo_inverse_kinematics` |
| Mass matrix (CRBA) | `G * G` | `mass_matrix` | `mass_matrix(..., out=)` | `mass_matrix[_into]` | `dynibo_[floating_]mass_matrix` |
| Velocity-product forces | `G` | `velocity_product_forces` | `velocity_product_forces(..., out=)` | `velocity_product_forces[_into]` | `dynibo_[floating_]velocity_product_forces` |
| Gravity (+ loads) | `G` | `gravity` | `gravity(..., loads=, out=)` | `gravity[_into]` | `dynibo_[floating_]gravity` |
| Inverse dynamics (RNEA) | `G` | `inverse_dynamics` | `inverse_dynamics(..., loads=, out=)` | `inverse_dynamics[_into]` | `dynibo_[floating_]inverse_dynamics` |
| Forward dynamics (ABA) | `G` | `forward_dynamics` | `forward_dynamics(..., loads=, out=)` | `forward_dynamics[_into]` | `dynibo_[floating_]forward_dynamics` |
| Reusable load buffer | Buffer | `load_buffer()` -> `LoadBuffer` | `load_buffer()` -> `LoadBuffer` | `make_load(...)` -> `DyniboLoad` | `DyniboLoad[]` array |
| State snapshot reuse | Borrowed view | `state(&q, &qd)` / `state(&base, &q, &qd)` | — | — | — |
