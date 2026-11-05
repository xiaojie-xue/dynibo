<!-- markdownlint-disable MD033 MD041 -->

<div align="center">

<h1>dynibo</h1>

<p><strong>Robot kinematics and rigid-body dynamics with URDF, Jacobians, inverse kinematics, RNEA and ABA</strong></p>

<p>
  <a href="https://dynibo.readthedocs.io/">Documentation</a> &nbsp;&middot;&nbsp;
  <strong>English</strong> | <a href="https://github.com/xiaojie-xue/dynibo/blob/main/README.zh.md">简体中文</a>
</p>

<p>
  <a href="https://github.com/xiaojie-xue/dynibo/actions/workflows/package-ci.yml"><img alt="CI" src="https://github.com/xiaojie-xue/dynibo/actions/workflows/package-ci.yml/badge.svg?branch=main"></a>
  <a href="https://codecov.io/gh/xiaojie-xue/dynibo"><img alt="codecov" src="https://codecov.io/gh/xiaojie-xue/dynibo/branch/main/graph/badge.svg"></a>
  <a href="https://crates.io/crates/dynibo"><img alt="crates.io" src="https://img.shields.io/crates/v/dynibo.svg?color=CE422B&amp;logo=rust&amp;logoColor=white"></a>
  <a href="https://pypi.org/project/dynibo/"><img alt="PyPI" src="https://img.shields.io/pypi/v/dynibo.svg?color=3776AB&amp;logo=python&amp;logoColor=white"></a>
  <a href="https://github.com/xiaojie-xue/dynibo/blob/main/LICENSE"><img alt="License: MIT" src="https://img.shields.io/badge/license-MIT-blue.svg"></a>
</p>

</div>

`dynibo` is a robot kinematics and dynamics library for controller development.
It supports manipulators, humanoids, and other robots with fixed or floating
bases. It loads robot models from URDF and provides Rust, Python, C, and C++
interfaces.

## Algorithms and task recipes

Dynibo provides forward kinematics (FK), world geometric Jacobians and their
time derivatives, damped least-squares inverse kinematics (IK), mass matrices,
gravity compensation, recursive Newton–Euler inverse dynamics (RNEA), and
articulated-body forward dynamics (ABA). Fixed and floating bases use the same
Rust core; IK is available for fixed bases.

| Task | Complete, verified example |
|---|---|
| Load URDF and calculate a tool pose | [Forward kinematics](https://dynibo.readthedocs.io/en/latest/recipes/forward-kinematics/) |
| Compute a Jacobian and reshape for NumPy | [Jacobian and array layout](https://dynibo.readthedocs.io/en/latest/recipes/jacobian/) |
| Calculate holding torque | [Gravity compensation](https://dynibo.readthedocs.io/en/latest/recipes/gravity-compensation/) |
| Calculate torque or recover acceleration | [RNEA and ABA](https://dynibo.readthedocs.io/en/latest/recipes/inverse-forward-dynamics/) |
| Evaluate a floating-base robot | [Floating-base dynamics](https://dynibo.readthedocs.io/en/latest/recipes/floating-base/) |
| Reuse arrays in repeated calculations | [Output buffers](https://dynibo.readthedocs.io/en/latest/recipes/reuse-buffers/) |

Each Python recipe embeds a small URDF and checks its numerical output, with
no ROS or mesh downloads. Rust users can run the same six tasks with
`cargo run --example recipes` from this repository.
See [Choosing Dynibo and migrating from Pinocchio](https://dynibo.readthedocs.io/en/latest/getting-started/choosing-dynibo/)
for supported workloads, API mappings, and frame/array conversion rules.

## Features

### Fast

Dynibo is written in Rust and reuses per-robot storage. After a `Robot` and
output buffers are created, the main kinematics and dynamics routines do not
allocate or resize memory inside the calculation loop.

To put its computation speed in context, we benchmark Dynibo against
[Pinocchio](https://github.com/stack-of-tasks/pinocchio), an open-source library
for robot kinematics and dynamics. The benchmarks use Franka, a fixed-base
manipulator with 7 joints, and unitree G1, a floating-base humanoid with 29 joints.
The table below shows Dynibo's speedup over Pinocchio for each operation.

<table>
  <thead>
    <tr>
      <th rowspan="2">Operation</th>
      <th colspan="2" align="center">Rust</th>
      <th colspan="2" align="center">Python</th>
    </tr>
    <tr>
      <th>Franka</th><th>unitree G1</th>
      <th>Franka</th><th>unitree G1</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>Jacobian</td>
      <td align="right">1.61×</td><td align="right">1.78×</td>
      <td align="right">1.26×</td><td align="right">1.40×</td>
    </tr>
    <tr>
      <td>RNEA</td>
      <td align="right">1.72×</td><td align="right">1.80×</td>
      <td align="right">1.19×</td><td align="right">1.56×</td>
    </tr>
    <tr>
      <td>ABA</td>
      <td align="right">1.14×</td><td align="right">1.17×</td>
      <td align="right">1.76×</td><td align="right">1.94×</td>
    </tr>
  </tbody>
</table>

Source code to reproduce these results is available in [`benches/`](benches/).

### Reliable

Dynibo combines maintained fixtures with a seed-reproducible generated-URDF
corpus, covering serial and branched robots, fixed and floating bases, mixed
joint types, external loads, invalid inputs, and repeated workspace use. Results
are checked against finite-difference approximations, consistency relations
between related algorithms, and outputs from an independent Pinocchio oracle.
Separate tests verify allocation-free execution and the installed Rust, Python,
C, and C++ packages. See the
[test architecture](https://github.com/xiaojie-xue/dynibo/blob/main/tests/TESTING.md)
for details.

### Easy to Use

Dynibo's API is built around `Robot`: load a URDF, then call kinematics and
dynamics algorithms. `Robot` manages its internal calculation storage, so
users do not need to create and maintain separate `Model` and `Data` objects.
Rust, Python, C, and C++ interfaces share the same Rust core, making it easy to
integrate Dynibo into projects in different languages.

## Dependencies

The Rust core has two direct runtime dependencies:

- [`nalgebra`](https://nalgebra.rs/) — linear algebra and numerical types
- [`urdf-rs`](https://github.com/openrr/urdf-rs) — URDF parsing

Python wheels bundle the native library and require NumPy 1.23 or newer at runtime.

## Quick start

### Rust

Add the Cargo package:

```bash
cargo add dynibo
```

Load a URDF and compute a target-link pose:

```rust
use dynibo::Robot;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let urdf = r#"<robot name="arm">
      <link name="base"/>
      <link name="arm">
        <inertial>
          <origin xyz="0.5 0 0"/>
          <mass value="1"/>
          <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
        </inertial>
      </link>
      <joint name="hinge" type="revolute">
        <parent link="base"/><child link="arm"/>
        <axis xyz="0 1 0"/>
        <limit lower="-3.14" upper="3.14" effort="100" velocity="10"/>
      </joint>
      <link name="tool"/>
      <joint name="tip" type="fixed">
        <parent link="arm"/><child link="tool"/>
        <origin xyz="1 0 0"/>
      </joint>
    </robot>"#;
    let path = std::env::temp_dir()
        .join(format!("dynibo-quick-start-{}.urdf", std::process::id()));
    std::fs::write(&path, urdf)?;
    let loaded = Robot::from_urdf(&path);
    std::fs::remove_file(&path)?;
    let mut robot = loaded?;
    let tool = robot.link_id("tool")?;
    let q = [0.3_f64];
    let pose = robot.forward_kinematics(&q, tool)?;
    assert!((pose.translation.x - q[0].cos()).abs() < 1.0e-12);
    assert!((pose.translation.z + q[0].sin()).abs() < 1.0e-12);

    let mut jacobian = vec![0.0; 6 * robot.generalized_count()];
    robot.jacobian(&q, tool, &mut jacobian)?;
    let mut gravity = vec![0.0; robot.generalized_count()];
    robot.gravity(&q, &[], &mut gravity)?;
    println!("translation: {:?}; holding torque: {:?}", pose.translation, gravity);
    Ok(())
}
```

### Python

Install the Python package from PyPI:

```bash
python -m pip install dynibo
```

The Python binding owns its reusable native calculation storage:

```python
"""URDF forward kinematics: run with an installed dynibo package; no model download needed."""

from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np

from dynibo import Robot

URDF = """<robot name="pendulum">
  <link name="base">
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
  <link name="arm">
    <inertial>
      <origin xyz="0.5 0 0"/>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
  <joint name="hinge" type="revolute">
    <parent link="base"/><child link="arm"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="10"/>
  </joint>
  <link name="tool"/>
  <joint name="tip" type="fixed">
    <parent link="arm"/><child link="tool"/>
    <origin xyz="1 0 0"/>
  </joint>
</robot>"""

with TemporaryDirectory() as directory:
    path = Path(directory) / "pendulum.urdf"
    path.write_text(URDF, encoding="utf-8")
    with Robot.from_urdf(path) as robot:
        target = robot.link_id("tool")
        q = np.array([0.3], dtype=np.float64)
        pose = robot.forward_kinematics(q, target)
        expected = (np.cos(q[0]), 0.0, -np.sin(q[0]))
        np.testing.assert_allclose(pose.translation, expected, atol=1e-12)
        assert len(pose.rotation_xyzw) == 4
        print("world tool translation:", pose.translation)
```

### C/C++

C and C++ users can download a prebuilt package for Linux, macOS, or Windows
from [GitHub Releases](https://github.com/xiaojie-xue/dynibo/releases), or build
and install the package from source. Prebuilt packages contain the shared
library, C and C++ headers, pkg-config metadata, and a CMake package
configuration. Select the archive matching your operating system and CPU
architecture and verify it against the release's `SHA256SUMS`.

Building from source requires Rust with Cargo and CMake 3.16 or newer:

```bash
cmake -S . -B build/c -DCMAKE_BUILD_TYPE=Release
cmake --build build/c --parallel
cmake --install build/c --prefix /opt/dynibo
```

Use an extracted prebuilt package or a source installation from another CMake
project:

```cmake
find_package(dynibo CONFIG REQUIRED)
target_link_libraries(my_robot PRIVATE dynibo::dynibo)
```

Configure the consumer with `-DCMAKE_PREFIX_PATH` pointing to the extracted
archive directory or the installation prefix. See the
[installation guide](https://github.com/xiaojie-xue/dynibo/blob/main/docs/getting-started/installation.md)
for platform-specific runtime library paths.

## Examples

Complete Rust, Python, C++, and C examples are available in the
[`examples/`](https://github.com/xiaojie-xue/dynibo/tree/main/examples) directory.
Each example exercises all of the main kinematics and dynamics methods.

## Supported models

Dynibo supports both **fixed-base robots** and **floating-base robots**, using
runtime-sized tree URDFs with revolute, continuous, prismatic, and fixed joints.
It rejects invalid topology and reports structured errors for
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

## License

Dynibo code is licensed under
[MIT](https://github.com/xiaojie-xue/dynibo/blob/main/LICENSE).
Bundled robot descriptions retain their
[third-party licenses](https://github.com/xiaojie-xue/dynibo/blob/main/examples/data/README.md),
including Franka's Apache-2.0 license and Unitree's BSD-3-Clause license.

## Contributing

Dynibo is still at an early stage, and contributions are welcome. See
[`CONTRIBUTING.md`](https://github.com/xiaojie-xue/dynibo/blob/main/CONTRIBUTING.md)
for development setup, required checks, and pull request guidelines.

## Citation

If Dynibo is useful in your work, please cite it as:

```bibtex
@software{xue2026dynibo,
  author  = {Xue, Xiaojie},
  title   = {Dynibo: a Fast, Lightweight, and Reliable Robot Kinematics and Dynamics Library},
  year    = {2026},
  url     = {https://github.com/xiaojie-xue/dynibo}
}
```
