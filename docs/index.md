# dynibo

**Fast · Lightweight · Reliable**

Dynibo is a robot kinematics and dynamics library. It loads robot topology from
URDF at runtime, keeps allocation outside repeated calculations through reusable
workspaces, and exposes Rust, Python, C++, and C interfaces backed by the same
Rust implementation.

[Get started](getting-started/quick-start.md){ .md-button .md-button--primary }
[Install dynibo](getting-started/installation.md){ .md-button }

## Why dynibo

### Fast

The core algorithms are written in Rust and designed to reuse memory. Once a
`Workspace` and the required output buffers have been created, the main
kinematics and dynamics routines do not allocate or resize memory inside the
calculation loop. In the benchmark set published with the project, the measured
core operations run 1.19–2.51× as fast as Pinocchio.

### Lightweight

Dynibo focuses on a compact set of commonly used operations instead of a large
framework. Its API is built around a small vocabulary—`Robot`, `Workspace`,
`LinkId`, `Frame`, `Twist`, and `Wrench`—shared by all four language
interfaces.

### Reliable

Tests cover finite-difference kinematics, dynamics regressions, branched robots,
external loads, inverse kinematics, invalid input, workspace reuse, and
allocation behavior. Core numerical results are also checked against an
independent Pinocchio oracle.

## What you can compute

| Area | Operations | Read next |
|---|---|---|
| Model | Load URDF models, look up links, configure fixed or floating bases | [Robot model and URDF](user-guide/robot-model-and-urdf.md) |
| Kinematics | Poses, Jacobians, Jacobian derivatives, spatial velocities and accelerations | [Kinematics](user-guide/kinematics.md) |
| Inverse kinematics | Damped least-squares target-pose solving | [Kinematics](user-guide/kinematics.md#inverse-kinematics) |
| Dynamics | Mass matrix, velocity-product forces, gravity and inverse dynamics | [Dynamics](user-guide/dynamics.md) |
| Loads | Apply external wrenches to links | [External loads](user-guide/external-loads.md) |

Before interpreting numerical results, read [Frames and spatial
vectors](user-guide/frames-and-spatial-vectors.md) for the shared conventions on
ordering, matrix layout, units, and reference frames.

## Choose an interface

| Interface | Best for | API style |
|---|---|---|
| [Rust](languages/rust.md) | Native Rust applications and explicit memory control | `Robot` methods with a reusable `Workspace` |
| [Python](languages/python.md) | Research, scripting, and rapid prototyping | `Robot` methods with an internally owned workspace |
| [C++](languages/cpp.md) | C++ applications that want RAII and exceptions | Move-only `dynibo::Robot` wrapper |
| [C](languages/c.md) | Stable ABI and integration with other languages | Opaque handles and `dynibo_*` functions |

The interfaces follow the conventions of their languages while preserving the
same concepts and operations. [API mapping](languages/api-mapping.md) shows the
corresponding names, ownership rules, and error models side by side.

## Explore the documentation

- [Installation](getting-started/installation.md) — choose and install a package.
- [Quick start](getting-started/quick-start.md) — run the same first calculation
  in Rust, Python, C++, or C.
- [User Guide](user-guide/index.md) — learn the model, coordinate, frame,
  workspace, kinematics, and dynamics semantics shared by every binding.
- [API mapping](languages/api-mapping.md) — translate an operation between
  languages.
- API reference — browse [Python](reference/python.md),
  [C++](cpp-api/dynibo_8hpp.md), [C](c-api/dynibo_8h.md), or
  [Rust](https://docs.rs/dynibo).
- [GitHub source code](https://github.com/xiaojie-xue/dynibo) — examples, issue
  tracking, benchmarks, and development information.
