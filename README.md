<!-- markdownlint-disable MD033 MD041 -->

<div align="center">

<h1>dynibo</h1>

<p><strong>Fast &middot; Lightweight &middot; Reliable</strong></p>

<p>
  <a href="https://dynibo.readthedocs.io/">Documentation</a>
</p>

<p>
  <a href="https://github.com/xiaojie-xue/dynibo/actions/workflows/package-ci.yml"><img alt="CI" src="https://github.com/xiaojie-xue/dynibo/actions/workflows/package-ci.yml/badge.svg?branch=main"></a>
  <a href="https://codecov.io/gh/xiaojie-xue/dynibo"><img alt="codecov" src="https://codecov.io/gh/xiaojie-xue/dynibo/branch/main/graph/badge.svg"></a>
  <a href="https://crates.io/crates/dynibo"><img alt="crates.io" src="https://img.shields.io/crates/v/dynibo.svg?color=CE422B&amp;logo=rust&amp;logoColor=white"></a>
  <a href="https://pypi.org/project/dynibo/"><img alt="PyPI" src="https://img.shields.io/pypi/v/dynibo.svg?color=3776AB&amp;logo=python&amp;logoColor=white"></a>
  <a href="LICENSE"><img alt="License: MIT" src="https://img.shields.io/badge/license-MIT-blue.svg"></a>
</p>

</div>

English | [简体中文](README.zh.md)

`dynibo` is a fast, lightweight, and reliable library for
robot kinematics and dynamics. It loads robot topology from URDF at runtime and
provides allocation-free calculations through reusable workspaces. Python and
C/C++ interfaces are available on top of the same Rust core.

## Features

### Fast

Across the benchmarks below, Dynibo runs 1.19–2.51× as fast as Pinocchio for the
measured core operations. It is written in Rust and keeps allocation outside the
calculation loop. After a `Workspace` and output buffers are created, the main
kinematics and dynamics routines reuse that memory without allocating or
resizing.

The table below shows Dynibo's speedup over Pinocchio for core kinematics and
dynamics operations.

| Model | FK | Jacobian | Gravity | RNEA |
|---|---:|---:|---:|---:|
| Two-leaf tree (7 joints, fixed base) | 1.90× | 2.05× | 1.89× | 1.94× |
| Two-leaf tree (7 joints, floating base) | 2.16× | 2.51× | 2.15× | 2.20× |
| Serial chain (40 joints, fixed base) | 1.19× | 1.49× | 1.78× | 1.99× |
| Serial chain (40 joints, floating base) | 1.21× | 1.56× | 1.79× | 2.09× |

These Criterion quick-mode results use the same URDF models and joint states on
an Intel Core i9-14900K with rustc 1.97.1 and Pinocchio 3.9.0. Setup and
allocation are excluded, and speedups use interval medians after subtracting the
measured 0.703 ns fixed C ABI overhead. With Pinocchio available through
`pkg-config`, rerun the raw benchmarks with:

```bash
cargo bench --features pinocchio-bench --bench pinocchio -- --quick
```

### Lightweight

Dynibo intentionally focuses on the most commonly used robot kinematics and
dynamics interfaces:

- `forward_kinematics` — target-link pose
- `jacobian` — target-link Jacobian
- `jacobian_derivative` — time derivative of the target-link Jacobian
- `forward_velocity_kinematics` — spatial velocity
- `forward_acceleration_kinematics` — spatial acceleration
- `inverse_kinematics` — damped least-squares IK
- `mass_matrix` — joint-space mass matrix
- `velocity_product_forces` — Coriolis and centrifugal generalized forces
- `gravity` — gravity compensation with optional external loads
- `inverse_dynamics` — recursive Newton–Euler inverse dynamics

The API is built around a small set of types: `Robot`, `Workspace`, `LinkId`,
`Frame`, `Twist`, and `Wrench`. Rust, Python, C, and C++ interfaces share the
same Rust implementation.

### Reliable

Dynibo is thoroughly unit-tested. Tests cover finite-difference kinematics,
dynamics regressions, branched robots and external loads, inverse kinematics,
invalid inputs, workspace ownership and reuse, and allocation-free calculation.
An independent Pinocchio oracle also compares complete FK, Jacobian, Jacobian
time-derivative, mass matrix, velocity-product forces, gravity, and RNEA outputs over
deterministic robot states.

## Dependencies

The Rust core has two direct runtime dependencies:

- [`nalgebra`](https://nalgebra.rs/) — linear algebra and numerical types
- [`urdf-rs`](https://github.com/openrr/urdf-rs) — URDF parsing

Python wheels bundle the native library and have no runtime Python dependencies.

## Quick start

### Rust

Add the Cargo package:

```bash
cargo add dynibo
```

Load a URDF, create a reusable workspace, and compute a target-link pose:

```rust
use dynibo::Robot;

fn main() -> dynibo::Result<()> {
    let robot = Robot::from_urdf("robot.urdf")?;
    let tool = robot.link_id("tool")?;
    let mut workspace = robot.workspace();
    let q = vec![0.0; robot.joint_count()];

    let pose = robot.forward_kinematics(&q, tool, &mut workspace)?;
    println!("translation: {}", pose.translation.vector.transpose());
    Ok(())
}
```

### Python

Install the Python package from PyPI:

```bash
python -m pip install dynibo
```

The Python binding owns its reusable native workspace:

```python
from dynibo import Robot

with Robot.from_urdf("robot.urdf") as robot:
    tool = robot.link_id("tool")
    q = [0.0] * robot.joint_count
    pose = robot.forward_kinematics(q, tool)
    print(pose.translation)
```

### C/C++

Build and install the CMake package from source. This requires Rust with Cargo
and CMake 3.16 or newer:

```bash
cmake -S . -B build/c -DCMAKE_BUILD_TYPE=Release
cmake --build build/c --parallel
cmake --install build/c --prefix /opt/dynibo
```

Use the installed package from another CMake project:

```cmake
find_package(dynibo CONFIG REQUIRED)
target_link_libraries(my_robot PRIVATE dynibo::dynibo)
```

If dynibo was installed to a custom prefix, configure the consumer with
`-DCMAKE_PREFIX_PATH=/opt/dynibo` (or the prefix you selected).

## Examples

Complete Rust, Python, and C examples are available in the
[`examples/`](examples/) directory. Each example exercises all of the main
kinematics and dynamics methods listed above.

## Supported models

Dynibo supports runtime-sized tree URDFs with revolute, continuous, prismatic,
and fixed joints. It rejects invalid topology and reports structured errors for
bad input lengths, model-mismatched handles, and solver failures.

## Testing

```bash
cargo fmt --all -- --check
cargo clippy --workspace --all-targets --locked -- -D warnings
cargo test --workspace --all-targets --locked
```

Run the complete local Rust, Python, C, and C++ verification suite with the
command below. Pinocchio reference tests are included when Pinocchio is
available through `pkg-config`.

```bash
bash ci/test-all.sh
```

## Contributing

Dynibo is still at an early stage, and contributions are welcome. See
[`CONTRIBUTING.md`](CONTRIBUTING.md) for development setup, required checks, and
pull request guidelines.

## Citation

If Dynibo is useful in your work, please cite it as:

```bibtex
@software{xue2026dynibo,
  author  = {Xue, Xiaojie},
  title   = {Dynibo: a Fast, Lightweight, and Reliable Robot Kinematics and Dynamics Library},
  year    = {2026},
  version = {0.2.0},
  url     = {https://github.com/xiaojie-xue/dynibo}
}
```
