# dyno

English | [简体中文](README.zh.md)

`dyno` is a lightweight and reliable, Rust-based library for fixed-size serial
robot kinematics and dynamics. It uses [`nalgebra`](https://nalgebra.rs/) for
fixed-size numerical types and [`urdf-rs`](https://github.com/openrr/urdf-rs)
for URDF parsing.

## Design goals

- **Lightweight runtime:** the compute path uses fixed-size, stack-backed
  vectors, matrices, and work arrays. It performs no heap allocation during FK,
  Jacobian, Jacobian-derivative, gravity, or inverse-dynamics evaluation.
- **Reliable behavior:** the runtime library contains no project-owned `unsafe`
  code, rejects invalid models explicitly, and verifies analytical kinematics
  against finite differences as well as numerical regression cases. The
  optional Pinocchio benchmark isolates its required C ABI in the benchmark
  harness.
- **Rust-based:** const generics make the joint count part of the type, while
  ownership and borrowing keep model data and calculation inputs explicit.

URDF parsing and name lookup allocate memory only while constructing a model;
they do not enter the real-time compute path. "Reliable" describes the tested,
safe-Rust implementation and is not a functional-safety certification.

## Public API

### Core types

| Type | Purpose |
|---|---|
| `RobotArm<const N: usize>` | Fixed-size serial robot model and algorithms |
| `RobotLink` | Joint transform, axis, limits, mass, center of mass, and inertia |
| `JointVector<N>` | Fixed-size joint vector |
| `Jacobian<N>` | Angular-first `6 x N` geometric Jacobian |
| `Frame` | Rigid transform backed by `nalgebra::Isometry3<f64>` |
| `Motion` | Angular-first spatial velocity or acceleration |
| `Wrench` | Torque-first spatial force |

### Model construction and access

| Interface | Result |
|---|---|
| `RobotArm::from_links(name, links)` | Construct from `[RobotLink; N]` |
| `RobotArm::from_urdf_str(source)` | Parse a URDF string |
| `RobotArm::from_urdf_file(path)` | Parse a URDF file |
| `name()`, `links()`, `link_mut()` | Inspect or update model data |
| `replace_link(index, link)` | Replace one link and refresh the home pose |
| `home_end_frame()` | Return the zero-position end frame |

### Kinematics

| Interface | Result |
|---|---|
| `forward_kinematics(q)` | End-effector frame |
| `jacobian(q)` | Base-frame geometric Jacobian |
| `jacobian_with_base(q, base)` | Jacobian rotated into a base frame |
| `jacobian_with_tool(q, tool)` | Jacobian shifted to a tool point |
| `forward_velocity_kinematics(q, qd, base, tool)` | End-effector spatial velocity |
| `jacobian_dot(q, qd)` | Analytical time derivative of the Jacobian |
| `jacobian_dot_times_velocity(q, qd)` | Convective acceleration `J_dot * qd` |
| `forward_acceleration_kinematics(q, qd, qdd)` | Acceleration `J * qdd + J_dot * qd` |

### Dynamics and joint utilities

| Interface | Result |
|---|---|
| `gravity_torque(q, base, end_load)` | Joint gravity forces and base wrench |
| `inverse_dynamics(...)` | Joint forces and base wrench from Newton-Euler recursion |
| `joint_position_limits()` | Lower and upper joint-limit vectors |
| `saturate_joint_position(lower, upper, q)` | Element-wise position clamping |
| `PassiveJointMap` | Map active coordinates to all joints and forces back |
| `RobotWithPassiveJoints` | Kinematics and dynamics adapter for passive joints |

```rust
use dyno::{JointVector, RobotArm};

let arm = RobotArm::<4>::from_urdf_file("test_arm.urdf")?;
let q = JointVector::<4>::zeros();
let end = arm.forward_kinematics(&q);
let jacobian = arm.jacobian(&q);
# Ok::<(), dyno::Error>(())
```

## Compatibility scope

This crate currently mirrors the serial-chain scope of the original
`RobotArm.h`. A branched URDF is rejected during construction instead of being
silently flattened. Branched-tree support needs a parent-indexed model and is a
separate extension.

The compatibility dynamics intentionally preserve legacy numerical conventions,
including positive-Z gravity and the original product-of-inertia signs, so the
C++ regression values remain reproducible. Consequently, the gravity and RNEA
benchmarks below compare execution cost, not numerical equivalence with
Pinocchio's standard rigid-body dynamics conventions.

## Pinocchio benchmark

The optional Criterion benchmark compares Dyno and Pinocchio at `N=4` and
`N=40`, using the same URDF and joint inputs for each implementation. It covers
forward kinematics, end-joint Jacobian, gravity, and RNEA. Model construction
and URDF parsing are outside the timed region; both implementations reuse their
model and calculation workspaces. A separate no-op measurement reports the
fixed Rust-to-C ABI call overhead.

Pinocchio is only needed when the `pinocchio-bench` feature is selected. The
bridge, `cc`, `pkg-config`, and Criterion do not become runtime dependencies of
normal Dyno builds. For a ROS installation such as Humble on x86-64 Linux:

```bash
export PKG_CONFIG_PATH=/opt/ros/humble/lib/x86_64-linux-gnu/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/opt/ros/humble/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
cargo bench --features pinocchio-bench --bench pinocchio
```

Adjust the ROS distribution and architecture paths for the local installation.
Use `-- --quick` for a short smoke run; omit it for measurements intended for
comparison.

## Verification

```text
cargo fmt --all -- --check
cargo clippy --all-targets -- -D warnings
cargo test --all-targets
# With Pinocchio installed:
cargo clippy --features pinocchio-bench --bench pinocchio -- -D warnings
```

The integration tests cover a generic four-axis test URDF, Jacobian derivatives,
acceleration and inverse-dynamics reference values, finite-difference Jacobian
and Jacobian-derivative validation, revolute/prismatic links, gravity, limits
and passive joints.
