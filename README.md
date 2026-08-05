# dynibo

[![Package CI](https://github.com/xiaojie-xue/dynibo/actions/workflows/package-ci.yml/badge.svg?branch=main)](https://github.com/xiaojie-xue/dynibo/actions/workflows/package-ci.yml)
[![codecov](https://codecov.io/gh/xiaojie-xue/dynibo/branch/main/graph/badge.svg)](https://codecov.io/gh/xiaojie-xue/dynibo)
[![GitHub Release](https://img.shields.io/github/v/release/xiaojie-xue/dynibo)](https://github.com/xiaojie-xue/dynibo/releases/latest)

English | [简体中文](README.zh.md)

`dynibo` is a fast, lightweight, and reliable Rust library for 
robot kinematics and dynamics. It loads robot topology from URDF at runtime and
provides allocation-free calculations through reusable workspaces. Python and
C/C++ interfaces are available on top of the same Rust core.

## Features

### Fast

Dynibo is written in Rust and keeps allocation outside the calculation loop.
After a `Workspace` and output buffers are created, the main kinematics and
dynamics routines reuse that memory without allocating or resizing.

The following Criterion results compare Dynibo with Pinocchio using the same URDF
models and joint states. Model construction, workspaces, Pinocchio `Data`, and
output allocation are excluded from the timed region. Speedups are calculated
from quick-mode interval medians after subtracting the measured 0.882 ns fixed
C ABI overhead from the Pinocchio times.

Across these benchmarks, Dynibo is 1.17–2.70× as fast as Pinocchio. Higher is
better.

| Model | FK | Jacobian | Gravity | RNEA |
|---|---:|---:|---:|---:|
| Serial chain (4 joints) | 1.17× | 1.58× | 1.59× | 1.66× |
| Serial chain (40 joints) | 1.24× | 1.72× | 1.63× | 1.91× |
| Two-leaf tree (7 joints) | 2.08× | 2.70× | 1.89× | 1.91× |

Measurements were collected on an Intel Core i9-14900K with rustc 1.97.1 and
Pinocchio 3.9.0. When Pinocchio is available through `pkg-config`,
reproduce them with:

```bash
cargo bench --features pinocchio-bench --bench pinocchio -- --quick
```

### Lightweight

Dynibo intentionally focuses on the most commonly used robot kinematics and
dynamics interfaces:

- `forward_kinematics` — target-link pose
- `jacobian` — target-link Jacobian
- `forward_velocity_kinematics` — spatial velocity
- `forward_acceleration_kinematics` — spatial acceleration
- `inverse_kinematics` — damped least-squares IK
- `gravity` — gravity compensation with optional external loads
- `inverse_dynamics` — recursive Newton–Euler inverse dynamics

The API is built around a small set of types: `Robot`, `Workspace`, `LinkId`,
`Frame`, `Twist`, and `Wrench`. Rust, Python, C, and C++ interfaces share the
same Rust implementation.

### Reliable

Dynibo is thoroughly unit-tested. Tests cover finite-difference kinematics,
dynamics regressions, branched robots and external loads, inverse kinematics,
invalid inputs, workspace ownership and reuse, and allocation-free calculation.
An independent Pinocchio oracle also compares complete FK, Jacobian, gravity,
and RNEA outputs over deterministic robot states.

The Rust core contains no project-owned `unsafe` code. CI requires at least 85%
line coverage and 75% branch coverage across the Rust workspace.

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

### Python

Install the Python package from PyPI:

```bash
python -m pip install dynibo
```

The package is imported as `dynibo`.

### C/C++

Build and install the CMake package:

```bash
cmake -S . -B build/c -DCMAKE_BUILD_TYPE=Release
cmake --build build/c --parallel
cmake --install build/c --prefix /opt/dynibo
```

CMake consumers can use the installed `dynibo::dynibo` target.

## Examples

Complete usage examples are available in the [`examples/`](examples/)
directory.

## Supported models

Dynibo supports runtime-sized tree URDFs with revolute, continuous, prismatic,
and fixed joints. It rejects invalid topology and reports structured errors for
bad input lengths, model-mismatched handles, and solver failures.

## Testing

```bash
cargo fmt --all -- --check
cargo clippy --all-targets -- -D warnings
cargo test --workspace --all-targets
```

Run the complete Rust, Pinocchio, Python, C, and C++ verification suite with:

```bash
bash ci/test-all.sh
```

## Contributing

Dynibo is still at an early stage, and we welcome you to help shape and build it
with us. Feel free to open an issue for bugs or ideas, submit a pull request with
improvements, or contact me anytime to discuss how the project could evolve.

## Citation

If Dynibo is useful in your work, please cite it as:

```bibtex
@software{xue2026dynibo,
  author  = {Xue, Xiaojie},
  title   = {Dynibo: a Fast, Lightweight, and Reliable Robot Kinematics and Dynamics Library},
  year    = {2026},
  version = {0.1.0},
  url     = {https://github.com/xiaojie-xue/dynibo}
}
```

## License

Dynibo is available under the [MIT License](LICENSE).
