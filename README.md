# dyno

[![Package CI](https://github.com/xiaojie-xue/dyno/actions/workflows/package-ci.yml/badge.svg?branch=main)](https://github.com/xiaojie-xue/dyno/actions/workflows/package-ci.yml)
[![codecov](https://codecov.io/gh/xiaojie-xue/dyno/branch/main/graph/badge.svg)](https://codecov.io/gh/xiaojie-xue/dyno)

English | [简体中文](README.zh.md)

Installation and releases: [English guide](docs/RELEASING.md)

`dyno` is a lightweight Rust library for tree-structured robot kinematics and
dynamics. It discovers link, joint, and parent-child topology from URDF at
runtime and performs calculations through slices and an explicit reusable
`Workspace`.

Numerical types come from [`nalgebra`](https://nalgebra.rs/), and URDF parsing
uses [`urdf-rs`](https://github.com/openrr/urdf-rs).

## Design goals

- **Runtime-sized:** one binary can load valid URDFs with any joint count.
- **Allocation-free calculation:** after creating a workspace and output
  buffers, kinematics, dynamics, and IK calls do not allocate or resize.
- **Explicit errors:** invalid lengths and model-mismatched workspaces, link
  identifiers, and loads return errors before calculation.
- **FFI-ready boundary:** the included stable C ABI, C++17 RAII wrapper, and
  Python package reuse the same Rust algorithms.

The core library contains no project-owned `unsafe` code.

## Public types

| Type | Purpose |
|---|---|
| `Robot` | Read-only runtime-topology tree model |
| `Workspace` | Reusable, model-bound calculation scratch buffers |
| `LinkId` | Opaque, model-bound link identifier |
| `IndexedLoad` | External wrench associated with a `LinkId` |
| `InverseKinematicsOptions` | IK tolerances, damping, step, and iteration settings |
| `Joint`, `JointType`, `Link` | URDF model information |
| `Frame` | Rigid transform backed by `nalgebra::Isometry3<f64>` |
| `Twist` | Angular-first spatial velocity or acceleration |
| `Wrench` | Torque-first spatial force |

## Basic usage

Install the Rust crate with `cargo add dyno`. Python users install
`dyno-robotics`; C and C++ users can build the CMake package. See the
[installation and release guide](docs/RELEASING.md) for complete commands.

```rust
use dyno::{Frame, Robot};

let robot = Robot::from_urdf("robot.urdf")?;
let target = robot.link_id("tool")?;
let mut workspace = robot.workspace();

let q = vec![0.0; robot.joint_count()];
let mut jacobian = vec![0.0; 6 * robot.joint_count()];
let mut gravity = vec![0.0; robot.joint_count()];

let frame = robot.forward_kinematics(&q, target, &mut workspace)?;
robot.jacobian(&q, target, &mut workspace, &mut jacobian)?;
robot.gravity(
    &q,
    &Frame::identity(),
    &[],
    &mut workspace,
    &mut gravity,
)?;
# Ok::<(), dyno::Error>(())
```

Workspace construction allocates every internal buffer once. Reuse it in the
calculation loop. Use a separate workspace for each concurrent calculation;
`Robot` remains read-only and may be shared.

## Calculation API

| Interface | Result |
|---|---|
| `forward_kinematics(q, target, workspace)` | Target-link frame relative to the root |
| `jacobian(q, target, workspace, output)` | Write the target's `6 x N` Jacobian |
| `forward_velocity_kinematics(...)` | Spatial velocity at a target link/tool |
| `forward_acceleration_kinematics(...)` | Spatial acceleration at a link origin |
| `gravity(q, base, loads, workspace, output)` | Write gravity and load joint forces |
| `inverse_dynamics(...)` | Write Newton–Euler inverse dynamics forces |
| `inverse_kinematics(..., options, workspace, output)` | Write an IK solution using the supplied options |

Pass `InverseKinematicsOptions::default()` when the default solver settings are
appropriate.

## Errors

Fallible Rust APIs return the non-exhaustive `dyno::Error` enum. Match the
structured variants when details matter, and use `Error::category()` for the
stable `InvalidInput`, `Model`, or `Solver` classification used by language
bindings. Downstream exhaustive matches should include a wildcard arm so new
specific errors can be added without breaking callers.

Joint inputs and ordinary outputs must contain `robot.joint_count()` elements.
Jacobian output must contain `6 * robot.joint_count()` elements. A mismatch
returns `Error::WrongSliceLength`; methods never resize caller buffers.

## Jacobian layout

Jacobians are flat, column-major `6 x N` arrays. Each joint owns six contiguous
elements:

```text
[angular_x, angular_y, angular_z, linear_x, linear_y, linear_z]
```

Column `joint` begins at `jacobian[6 * joint]`. This matches the default column
layout used by nalgebra and Eigen.

## Model ownership

`LinkId`, `Workspace`, and every `IndexedLoad` are bound to the model that
created them. Passing values from Robot A to Robot B returns
`Error::InvalidLinkId` or `Error::InvalidWorkspace`. Cloned robots represent the
same model and accept the original IDs and workspace.

`LinkId` is a process-local handle and is not a persistent, serialized, or
cross-process identifier.

## Model and dynamics conventions

`Robot` supports valid tree URDFs with arbitrary branch count and depth.
Construction rejects multiple roots, duplicate names, cycles, disconnected
components, missing links, and links reached by multiple joints. Revolute,
continuous, prismatic, and fixed joints are supported.

Joint slices contain every URDF joint. Fixed joints still occupy an element but
contribute no motion or active joint force. Dynamics use positive-Z gravity.
URDF inertia tensors, including a non-zero inertial-origin rotation, are
converted into the corresponding link frame when the model is loaded.

IK uses the damped inverse `J^T (J J^T + lambda^2 I)^-1`. Iterations are
unconstrained; a converged solution is checked against URDF joint limits.

## Performance against Pinocchio

These results compare Dyno with Pinocchio on an Intel Core i9-14900K using rustc
1.97.1 and Pinocchio 3.9.0. Robot and workspace creation, Pinocchio `Data`, and
output-buffer allocation are outside the timed region and reused. Both
implementations receive the same URDF and joint inputs.

Values are Criterion quick-mode interval medians. The measured 0.882 ns fixed C
ABI overhead has been subtracted from Pinocchio times. All times are in ns.

| Model | Operation | Dyno | Pinocchio | Dyno speedup |
|---|---|---:|---:|---:|
| 4-joint chain | FK | 66.414 ns | 77.627 ns | 1.17x |
| 4-joint chain | Jacobian | 81.475 ns | 128.778 ns | 1.58x |
| 4-joint chain | Gravity | 117.420 ns | 187.158 ns | 1.59x |
| 4-joint chain | RNEA | 181.030 ns | 300.268 ns | 1.66x |
| 40-joint chain | FK | 669.530 ns | 833.328 ns | 1.24x |
| 40-joint chain | Jacobian | 784.060 ns | 1347.518 ns | 1.72x |
| 40-joint chain | Gravity | 1120.500 ns | 1824.718 ns | 1.63x |
| 40-joint chain | RNEA | 1643.800 ns | 3133.918 ns | 1.91x |
| 7-joint two-leaf tree | FK | 66.105 ns | 137.298 ns | 2.08x |
| 7-joint two-leaf tree | Jacobian | 80.325 ns | 217.188 ns | 2.70x |
| 7-joint two-leaf tree | Gravity | 170.080 ns | 322.128 ns | 1.89x |
| 7-joint two-leaf tree | RNEA | 284.220 ns | 543.928 ns | 1.91x |

Run the comparison with:

```bash
export PKG_CONFIG_PATH=/opt/ros/humble/lib/x86_64-linux-gnu/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/opt/ros/humble/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
cargo bench --features pinocchio-bench --bench pinocchio -- --quick
```

Quick mode uses few samples, so these values show this machine's trend rather
than a cross-platform performance guarantee. Separate Pinocchio tests compare
complete FK, Jacobian, gravity, and RNEA outputs over 32 deterministic states;
the benchmark itself measures execution cost only.

## Example and verification

```bash
cargo run --example franka
cargo fmt --all -- --check
cargo clippy --all-targets -- -D warnings
cargo test --workspace --all-targets
cargo +nightly llvm-cov --branch --workspace --all-targets
cargo bench --features core-bench --bench core
```

Run formatting, lints, the default and optional Pinocchio suites, and all Rust,
C, C++, and Python package tests through one entry point:

```bash
bash ci/test-all.sh
```

When Pinocchio is available through `pkg-config`, run the independent numerical
reference suite with:

```bash
cargo test -p dyno --features pinocchio-tests --tests
```

Local unit tests cover only the Rust source workspace. GitHub Package CI tests
the extracted Rust `.crate`, installed Python packages, and extracted C/C++
CPack artifacts. See the [release guide](docs/RELEASING.md#pre-release-checks) for the CI
behavior. Tests cover finite-difference kinematics, dynamics regressions,
branched loads, workspace residue and ownership, invalid lengths, IK, and
allocation-free calculation.
The coverage CI job publishes an LLVM JSON report and requires at least 85%
line coverage and 75% branch coverage across the Rust workspace.
