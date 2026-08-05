# Rust, Python, and C/C++ Installation and Releases

English | [简体中文](RELEASING.zh.md)

All three language interfaces share the `dynibo` Rust core. Kinematics and
dynamics algorithms are not reimplemented in the bindings:

| Distribution | Installation | Interface layer |
|---|---|---|
| Rust crate `dynibo` | crates.io / Cargo | Native Rust API |
| Python distribution `dynibo` | PyPI / pip | `ctypes` + bundled `dynibo-c` library |
| C/C++ package `dynibo` | CMake package / GitHub Release | Stable C ABI + C++17 RAII header |

The Python distribution and import name are both `dynibo`. The C ABI lives in the
separate `dynibo-c` workspace member, so the core
crate remains free of project-owned `unsafe` code.

## Rust crate

Use the current checkout as a dependency:

```toml
[dependencies]
dynibo = { path = "/path/to/dynibo" }
```

After publication, use the registry version:

```toml
[dependencies]
dynibo = "0.1"
```

```rust
use dynibo::{Frame, Robot};

let robot = Robot::from_urdf("robot.urdf")?;
let tool = robot.link_id("tool")?;
let mut workspace = robot.workspace();
let q = vec![0.0; robot.joint_count()];

let pose = robot.forward_kinematics(&q, tool, &mut workspace)?;
let mut jacobian = vec![0.0; 6 * robot.joint_count()];
robot.jacobian(&q, tool, &mut workspace, &mut jacobian)?;
let mut gravity = vec![0.0; robot.joint_count()];
robot.gravity(&q, &Frame::identity(), &[], &mut workspace, &mut gravity)?;
# Ok::<(), dynibo::Error>(())
```

After updating and committing the version in `Cargo.toml`, a publisher runs:

```bash
cargo package -p dynibo --locked
cargo publish -p dynibo --locked
```

`cargo publish` requires a crates.io token. The repository does not store that
token.

## Python package

Installing from the current checkout requires a Rust toolchain:

```bash
python -m pip install .
```

After publication to PyPI, users only need:

```bash
python -m pip install dynibo
```

The wheel contains the platform-native library. It does not require Rust,
NumPy, or other Python dependencies at runtime:

```python
import numpy as np
from dynibo import Load, Pose, Robot

with Robot("robot.urdf") as robot:
    tool = robot.link_id("tool")
    q = [0.0] * robot.joint_count
    pose = robot.forward_kinematics(q, tool)
    flat = robot.jacobian(q, tool)
    jacobian = np.asarray(flat).reshape((6, robot.joint_count), order="F")
    gravity = robot.gravity(q)
```

`Pose.rotation_xyzw` always uses `(x, y, z, w)` order. Jacobians are
column-major `6 x N` arrays; each column contains three angular components
followed by three linear components. Each `Robot` owns one reusable workspace
and must not be used for calculations from multiple threads at the same time.
Create a separate `Robot` for each worker thread.

Native invalid arguments are raised as `ValueError`. Model-loading failures
raise `dynibo.ModelError`, while iterative solver failures raise
`dynibo.SolverError`; both derive from `dynibo.DyniboError` and `RuntimeError`.

Build a wheel locally:

```bash
python -m pip install build
python -m build --wheel
```

Release wheels should be built separately for Linux, macOS, and Windows.
`cibuildwheel` is recommended and repairs Linux artifacts into PyPI-compatible
manylinux wheels:

```bash
python -m pip install cibuildwheel twine
python -m cibuildwheel --output-dir wheelhouse
python -m twine check wheelhouse/*
python -m twine upload wheelhouse/*
```

Before uploading, synchronize the versions in `pyproject.toml`, `setup.py`,
`bindings/python/dynibo/__init__.py`, and the Cargo packages. Supply the PyPI
token through a CI secret or trusted publishing.

## Automated registry publication

Publishing a non-prerelease GitHub Release triggers
[the release workflow](../.github/workflows/release.yml). The release tag must
use `vMAJOR.MINOR.PATCH`, for example `v0.1.0`. The workflow rejects the release
unless that version matches the Rust crate, Python distribution, C ABI crate,
CMake project, and runtime version constants.

The workflow first runs formatting, lint, and workspace tests. It then builds
and tests the Rust crate, the Python source distribution, and Linux, macOS, and
Windows wheels. Nothing is published until every package job succeeds. The
verified `.crate`, wheels, and source distribution remain available as workflow
artifacts.

One-time repository configuration is required before the first release:

1. Create a GitHub environment named `crates-io`. Add a crates.io publishing
   token as the environment secret `CRATES_IO_TOKEN`.
2. Create a GitHub environment named `pypi`. On PyPI, configure `dynibo` to
   trust this repository's `.github/workflows/release.yml` workflow and the
   `pypi` environment. PyPI then issues a short-lived OIDC credential; no PyPI
   API token is stored in GitHub.
3. Add required reviewers to both environments if releases should require a
   final manual approval.

To publish a version, update all version fields, run:

```bash
python ci/check-release-version.py v0.1.0
bash ci/test-all.sh
```

Commit and tag that exact revision, then publish a non-prerelease GitHub Release
for the tag. The workflow publishes only the `dynibo` core crate to crates.io;
the internal `dynibo-c` workspace member remains unpublished. It publishes the
Python sdist and all platform wheels together to PyPI.

## C/C++ package

Build and install from source:

```bash
cmake -S . -B build/c -DCMAKE_BUILD_TYPE=Release
cmake --build build/c --parallel
cmake --install build/c --prefix /opt/dynibo
```

Create a distributable binary archive:

```bash
cmake --build build/c --target package
```

The archive contains the shared library, `dynibo.h`, `dynibo.hpp`, a pkg-config
file, and a CMake package configuration. Build a separate package for every
target operating system and CPU architecture.

### C

```c
#include <dynibo/dynibo.h>

DyniboRobot *robot = NULL;
DyniboWorkspace *workspace = NULL;
if (dynibo_robot_load_urdf("robot.urdf", &robot) != DYNIBO_STATUS_OK) {
    fprintf(stderr, "%s\n", dynibo_last_error_message());
    return 1;
}
dynibo_workspace_create(robot, &workspace);

size_t tool;
dynibo_robot_link_id(robot, "tool", &tool);
size_t n = dynibo_robot_joint_count(robot);
double *q = calloc(n, sizeof(double));
DyniboPose pose;
dynibo_forward_kinematics(robot, workspace, q, n, tool, &pose);

free(q);
dynibo_workspace_destroy(workspace);
dynibo_robot_destroy(robot);
```

Every fallible C function returns `DyniboStatus`. When it is nonzero, use
`dynibo_last_error_message()` to read the current thread's error message. Every
opaque handle must be released with its matching `destroy` function.

Statuses are grouped by caller action: `DYNIBO_STATUS_INVALID_ARGUMENT`,
`DYNIBO_STATUS_MODEL_ERROR`, `DYNIBO_STATUS_SOLVER_ERROR`, and
`DYNIBO_STATUS_PANIC`. `DYNIBO_STATUS_ERROR` remains a compatibility alias for
`DYNIBO_STATUS_MODEL_ERROR`.

### C++17

```cpp
#include <dynibo/dynibo.hpp>

dynibo::Robot robot("robot.urdf");
auto tool = robot.link_id("tool");
std::vector<double> q(robot.joint_count(), 0.0);
auto pose = robot.forward_kinematics(q, tool);
auto jacobian = robot.jacobian(q, tool);
auto gravity = robot.gravity(q);
```

The C++ header manages Robot and Workspace lifetimes and converts C errors into
`dynibo::Error`. A wrapper object must likewise not be used concurrently for
calculations from multiple threads. `dynibo::Error::status()` preserves the
original `DyniboStatus` for programmatic handling.

For a CMake consumer:

```cmake
find_package(dynibo CONFIG REQUIRED)
target_link_libraries(my_robot_app PRIVATE dynibo::dynibo)
```

Alternatively, use pkg-config:

```bash
cc app.c $(pkg-config --cflags --libs dynibo)
```

The runtime loader must be able to find `libdynibo_c`. For a non-system prefix,
set the platform's shared-library search path or configure RPATH when installing
the application.

## Pre-release checks

The repository's [Package CI](../.github/workflows/package-ci.yml) builds real
release artifacts on every push, pull request, and manual dispatch, then
uploads them as GitHub Actions artifacts:

- Rust: creates the `.crate`, extracts it outside the source workspace, and
  runs every test included in the package.
- Python: uses cibuildwheel to build wheels on Linux, macOS, and Windows, tests
  each installed wheel from a temporary directory, and separately builds and
  verifies the sdist.
- C/C++: creates CPack archives on all three platforms, extracts them, and uses
  external CMake consumers with `find_package(dynibo)` to run the C and C++ tests.

Package CI does not upload to crates.io or PyPI and does not create a GitHub
Release. Only the separate release workflow publishes registry packages, after
the repository environments and credentials described above have been
configured.

Local unit tests cover only the Rust source workspace; they do not build or run
the Python, C, or C++ package tests:

```bash
cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace --all-targets
cargo +nightly llvm-cov --branch --workspace --all-targets
```

Run the complete local verification through one entry point:

```bash
bash ci/test-all.sh
```

Coverage CI runs `ci/check-coverage.py` against the LLVM JSON summary and
requires at least 85% line coverage and 75% branch coverage.

GitHub Actions invokes `ci/test-rust-package.sh`,
`ci/test-native-package.py`, and cibuildwheel's
`tests/python/test_package.py` to verify release artifacts. They are not part
of the default local unit-test workflow.
