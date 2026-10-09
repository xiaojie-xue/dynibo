# Contributing to dynibo

Contributions are welcome. For small fixes, feel free to open a pull request
directly. For larger API changes, please open an issue first so the design can
be discussed.

## Development setup

Development requires:

- stable Rust with `rustfmt` and `clippy`;
- Python 3.9 or newer for the Python binding;
- CMake 3.16 or newer and a C/C++ compiler for the native package.

Before submitting a pull request, run the Rust checks:

```bash
cargo fmt --all -- --check
cargo clippy --workspace --all-targets --locked -- -D warnings
cargo test --workspace --all-targets --locked
```

For changes to the C++ wrapper, use the repository's `.clang-format` settings:

```bash
clang-format -i bindings/c/include/dynibo/dynibo.hpp
```

For changes that affect packaging or language bindings, run the complete local
suite. Pinocchio reference tests are included when Pinocchio is available
through `pkg-config`.

```bash
bash ci/test-all.sh
```

## Source layout

- `src/model/` loads URDF descriptions and defines joints, links, and the validated tree.
- `src/robot.rs` defines calculation instances; `src/robot/model.rs` owns immutable
  runtime model data, shared handles, and metadata queries.
- `src/robot/topology.rs` owns link identifiers and index conversions;
  `loads.rs` owns public load values and reusable load buffers; `workspace.rs`
  owns internal calculation storage. Kinematics and dynamics stay grouped by algorithm.
- `bindings/c/src/` separates ABI values, boundary checks, model/workspace handles,
  kinematics, and dynamics. `lib.rs` preserves the public Rust and C entry points.
- `bindings/python/src/` separates spatial values, arrays, errors, loads, and the
  two robot classes. `calculation.rs` shares lifecycle rules; `lib.rs` registers
  the Python module.
- Larger test suites use submodules behind their existing entry points. See
  [the test architecture](tests/TESTING.md) for fixture and reference-test organization.

CMake tracks Rust modules recursively so edits to a binding or core submodule
also trigger an incremental native-library rebuild.

## Preparing a release

Before committing and tagging a release, update the canonical
`[workspace.package] version` in `Cargo.toml`. C, CMake, Python, runtime, and
test versions are derived automatically. Let Cargo refresh its generated lock
file entries before running the locked test suite:

```bash
cargo metadata --no-deps --format-version 1 > /dev/null
python3 ci/check-release-version.py vX.Y.Z
bash ci/test-all.sh
git commit -am "release: vX.Y.Z"
git tag vX.Y.Z
git push origin main vX.Y.Z
```

The release workflow repeats the tag/version consistency check before building
or publishing any artifacts.

## Guidelines

- The core library (`src/`) forbids unsafe Rust with `#![forbid(unsafe_code)]`.
  This restriction does not cover dependencies, language bindings, or the
  separate integration test and benchmark crates, which currently use unsafe
  code for FFI and allocation instrumentation.
- Keep changes focused and add tests for new behavior or bug fixes.
- Update public API documentation and examples when usage changes.
- Keep `README.md` and `README.zh.md` aligned.
- Include reproducible measurements and environment details for performance
  changes.

In the pull request description, briefly explain the change and list the checks
you ran.
