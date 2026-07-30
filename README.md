# dyno

English | [简体中文](README.zh.md)

`dyno` is a lightweight and reliable, Rust-based library for tree-structured
robot kinematics and dynamics. Within the currently supported joint types, it
loads valid tree URDFs with arbitrary branch count and depth. Link, joint, and
parent-child topology are discovered automatically while calculation inputs and
outputs remain fixed-size. It uses
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

URDF parsing and topology construction allocate memory only while constructing
a model; kinematics and dynamics evaluation do not allocate on the heap.
"Reliable" describes the tested, safe-Rust implementation and is not a
functional-safety certification.

## Public API

### Core types

| Type | Purpose |
|---|---|
| `RobotArm` | Runtime-topology tree model with fixed-size calculation APIs |
| `RobotJoint` | Joint transform, axis, limits, and stored joint state |
| `RobotLink` | Link mass, center of mass, and inertia |
| `LinkId` | Stable numeric link identifier used to select calculation targets |
| `ExternalWrench` | Wrench applied at a link origin and expressed in that link frame |
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
| `root_link()`, `leaf_links()` | Inspect the root and all leaf links |
| `link_id(name)` | Resolve a reusable `LinkId` by name |
| `link_count()` | Return the number of URDF links, including the root link |
| `joint_count()` | Return the number of joints parsed from the model |

### Calculation API

To keep the library focused and lightweight, its calculation API is limited to
the following operations. The two inverse-kinematics entries define the planned
scope but are not implemented yet.

| Interface | Status and result |
|---|---|
| `forward_kinematics(q, target)` | Frame of a selected link |
| `jacobian(q, target)` | Base-frame Jacobian of a selected link; non-ancestor columns are zero |
| `inverse_kinematics(...)` | Planned; not implemented yet |
| `inverse_kinematics_with_boundary(...)` | Planned; not implemented yet |
| `forward_velocity_kinematics(q, qd, target, base, tool)` | Spatial velocity of a selected link/tool |
| `forward_acceleration_kinematics(q, qd, qdd, target)` | Direct-recursive acceleration of a selected link |
| `gravity(q, base, external_wrenches)` | Tree gravity recursion with loads on multiple links |
| `inverse_dynamics(..., external_wrenches)` | Tree RNEA with multi-link loads and branch accumulation |

```rust
use dyno::{JointVector, RobotArm};

let arm = RobotArm::from_urdf("test_arm.urdf")?;
let q = JointVector::<4>::zeros();
let target = arm.link_id("test_link_4").expect("target link must exist");
let end = arm.forward_kinematics(&q, target)?;
let jacobian = arm.jacobian(&q, target)?;
# Ok::<(), dyno::Error>(())
```

The calculation size `N` is inferred from each `JointVector<N>` input; it is
not part of the `RobotArm` type. A mismatch returns `Error::WrongJointCount`
before calculation begins.

## Tree-model conventions and compatibility scope

`RobotArm` supports valid tree URDFs with arbitrary branch count and depth. A
model has one root and exactly one parent joint for every non-root link.
Construction creates a parent-before-child topological order and rejects
multiple roots, duplicate names, cycles, disconnected components, missing
links, and links reached by multiple joints. Revolute, continuous, prismatic,
and fixed joints are supported; other URDF joint types still return
`UnsupportedJoint`.

Kinematics functions accept a target `LinkId` directly, so one model can
evaluate any branched endpoint. Gravity and inverse dynamics accept an
`&[ExternalWrench]`, allowing loads on any number of links; an empty slice means
no external load. `JointVector<N>` currently contains every URDF joint; fixed
joints occupy an element but contribute no motion or active joint force.

The root is retained in `links()`, but fixed-base compatibility dynamics do not
include the root link's own inertia in joint forces or the base wrench. An
`ExternalWrench` acts at a link origin and is expressed in that link's frame.

The compatibility dynamics intentionally preserve legacy numerical conventions,
including positive-Z gravity and the original product-of-inertia signs, so the
C++ regression values remain reproducible. Consequently, the gravity and RNEA
benchmarks below compare execution cost, not numerical equivalence with
Pinocchio's standard rigid-body dynamics conventions.

## Performance benchmarks

The tree benchmark model has seven movable joints: one shared trunk and two
three-joint branches ending in two leaves. The results use the same tree URDF
and joint inputs for Dyno and Pinocchio 3.9.0. They were collected
with `cargo bench --features pinocchio-bench --bench pinocchio -- --quick` on an
Intel Core i9-14900K. The measured 0.70 ns C ABI overhead has been subtracted
from the Pinocchio values.

| Function | Dyno | Pinocchio | Dyno speedup |
|---|---:|---:|---:|
| `forward_kinematics` | 111.27 ns | 134.07 ns | 1.20x |
| `jacobian` | 138.88 ns | 214.00 ns | 1.54x |
| `gravity` | 163.51 ns | 321.75 ns | 1.97x |
| `inverse_dynamics` | 256.50 ns | 513.81 ns | 2.00x |

Model construction and URDF parsing are outside the timed region. Both
implementations reuse their parsed model, and Pinocchio also reuses its `Data`
object. Dyno uses fixed-size stack arrays for per-node intermediates and does
not allocate on the heap during evaluation. Criterion quick mode uses few
samples, so these values show the current machine's trend rather than portable
or rigorous statistical results.

The bridge normalizes joint ordering, spatial-vector row ordering, and gravity
direction. A separate integration test compares full FK, Jacobian, gravity, and
RNEA outputs against Pinocchio; the benchmark itself measures execution time.

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
cargo test --features pinocchio-bench --test tree_pinocchio
```

Integration tests cover Jacobian derivatives, acceleration and inverse-dynamics
reference values, finite-difference Jacobian and Jacobian-derivative validation,
revolute/prismatic links, gravity, limits, and passive joints. The Pinocchio
cross-check evaluates both branches over 32 deterministic configurations and
compares FK, Jacobian, gravity, and RNEA element by element. The performance
benchmark uses the same seven-joint tree URDF with a shared trunk, two branches,
and two leaves.
