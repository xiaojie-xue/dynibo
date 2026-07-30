# dyno

English | [简体中文](README.zh.md)

`dyno` is a lightweight and reliable, Rust-based library for serial robot
kinematics and dynamics. Robot size is discovered while parsing the model,
while calculation inputs and outputs remain fixed-size. It uses
[`nalgebra`](https://nalgebra.rs/) for numerical types and
[`urdf-rs`](https://github.com/openrr/urdf-rs) for URDF parsing.

## Design goals

- **Lightweight runtime:** the compute path uses fixed-size, stack-backed
  vectors, matrices, and work arrays. It performs no heap allocation during
  kinematics, gravity, or inverse-dynamics evaluation.
- **Reliable behavior:** the runtime library contains no project-owned `unsafe`
  code, rejects invalid models explicitly, and verifies analytical kinematics
  against finite differences as well as numerical regression cases. The
  optional Pinocchio benchmark isolates its required C ABI in the benchmark
  harness.
- **Rust-based:** const generics preserve fixed-size calculation inputs and
  outputs, while the model's joint count is discovered at construction.

URDF parsing and name lookup allocate memory only while constructing a model;
they do not enter the real-time compute path. "Reliable" describes the tested,
safe-Rust implementation and is not a functional-safety certification.

## Public API

### Core types

| Type | Purpose |
|---|---|
| `RobotArm` | Runtime-sized serial model with fixed-size calculation APIs |
| `RobotJoint` | Joint transform, axis, limits, and stored joint state |
| `RobotLink` | Link mass, center of mass, and inertia |
| `JointVector<N>` | Fixed-size joint vector |
| `Jacobian<N>` | Angular-first `6 x N` geometric Jacobian |
| `Frame` | Rigid transform backed by `nalgebra::Isometry3<f64>` |
| `Motion` | Angular-first spatial velocity or acceleration |
| `Wrench` | Torque-first spatial force |

### Model construction and access

| Interface | Result |
|---|---|
| `RobotArm::from_urdf(path)` | Construct a model from a URDF file path |
| `name()`, `joints()`, `links()` | Inspect the model, with the root included in `links()` |
| `link_count()` | Return the number of URDF links, including the root link |
| `joint_count()` | Return the number of joints parsed from the model |

### Calculation API

To keep the library focused and lightweight, its calculation API is limited to
the following operations. The two inverse-kinematics entries define the planned
scope but are not implemented yet.

| Interface | Status and result |
|---|---|
| `forward_kinematics(q)` | End-effector frame |
| `jacobian(q)` | Base-frame geometric Jacobian |
| `inverse_kinematics(...)` | Planned; not implemented yet |
| `inverse_kinematics_with_boundary(...)` | Planned; not implemented yet |
| `forward_velocity_kinematics(q, qd, base, tool)` | End-effector spatial velocity |
| `forward_acceleration_kinematics(q, qd, qdd)` | Direct-recursive acceleration `J * qdd + J_dot * qd` |
| `gravity(q, base, end_load)` | Joint gravity forces and base wrench |
| `inverse_dynamics(...)` | Joint forces and base wrench from Newton-Euler recursion |

```rust
use dyno::{JointVector, RobotArm};

let arm = RobotArm::from_urdf("test_arm.urdf")?;
let q = JointVector::<4>::zeros();
let end = arm.forward_kinematics(&q)?;
let jacobian = arm.jacobian(&q)?;
# Ok::<(), dyno::Error>(())
```

The calculation size `N` is inferred from each `JointVector<N>` input; it is
not part of the `RobotArm` type. A mismatch returns `Error::WrongJointCount`
before calculation begins.

## Compatibility scope

This crate currently supports serial chains. A branched URDF is rejected during
construction instead of being silently flattened. Branched-tree support needs
a parent-indexed model and is a separate extension.

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
parsed models, and Pinocchio reuses its `Data` object. A separate no-op
measurement is used to correct Pinocchio timings for the fixed Rust-to-C ABI
call overhead.

The following smoke-test results were measured with `--quick` on an Intel Core
i9-14900K, using rustc 1.97.1 and Pinocchio 3.9.0. Lower latency is better.
They show the local trend rather than serving as a portable or statistically
rigorous performance claim.

| Operation | DoF | Dyno | Pinocchio | Dyno speedup |
|---|---:|---:|---:|---:|
| Forward kinematics | 4 | 65.5 ns | 79.0 ns | 1.21x |
| End Jacobian | 4 | 81.4 ns | 135.6 ns | 1.67x |
| Gravity | 4 | 91.5 ns | 187.6 ns | 2.05x |
| RNEA | 4 | 148.4 ns | 298.8 ns | 2.01x |
| Forward kinematics | 40 | 646.4 ns | 819.2 ns | 1.27x |
| End Jacobian | 40 | 786.1 ns | 1.351 µs | 1.72x |
| Gravity | 40 | 950.0 ns | 1.850 µs | 1.95x |
| RNEA | 40 | 1.462 µs | 3.209 µs | 2.19x |

The Pinocchio values above already account for the measured C ABI overhead. As
noted above, the gravity and RNEA rows compare runtime only because the
compatibility kernel and Pinocchio use different numerical conventions.

Pinocchio is only needed when the `pinocchio-bench` feature is selected. The
bridge, `cc`, `pkg-config`, and Criterion do not become runtime dependencies of
normal Dyno builds. For a ROS installation such as Humble on x86-64 Linux:

```bash
export PKG_CONFIG_PATH=/opt/ros/humble/lib/x86_64-linux-gnu/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/opt/ros/humble/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
cargo bench --features pinocchio-bench --bench pinocchio
```

The Dyno-only benchmark does not require Pinocchio:

```bash
cargo bench --features core-bench --bench core
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
