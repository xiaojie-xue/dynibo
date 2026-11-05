# Contributing to dynibo

Contributions are welcome. For small fixes, feel free to open a pull request
directly. For larger API changes, please open an issue first so the design can
be discussed.

## Development setup

Development requires:

- Rust 1.88 or newer, with `rustfmt` and `clippy` on stable;
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

Python wheel jobs in `package-ci.yml` and `release.yml` must target the same
runners. Linux aarch64 uses native `ubuntu-24.04-arm` with cibuildwheel's
`auto64` architecture selection and the `manylinux_2_28` image. It runs the
same installed-package tests as Linux x86-64, including typing artifacts and
task recipes. Its artifact name contains `Linux-ARM64`; the release's
`release-python-*` download includes it automatically, and publishing waits
for every wheel matrix entry to pass.

## Guidelines

- The core library (`src/`) forbids unsafe Rust with `#![forbid(unsafe_code)]`.
  This restriction does not cover dependencies, language bindings, or the
  separate integration test and benchmark crates, which currently use unsafe
  code for FFI and allocation instrumentation.
- Keep changes focused and add tests for new behavior or bug fixes.
- Update public API documentation and examples when usage changes.
- Keep `bindings/python/dynibo/_dynibo.pyi` aligned with the native public API.
  Its signatures and docstrings generate the Python reference without loading
  the extension. `__init__.pyi` re-exports the public names; both stubs and
  `py.typed` must remain in installed wheels and source distributions.
- Keep `README.md` and `README.zh.md` aligned.
- Include reproducible measurements and environment details for performance
  changes.

For Python API, typing, and documentation changes, install the package into a
virtual environment and run:

```bash
python -m pip install --group docs --group typing .
python -m mypy.stubtest dynibo
python -m mypy --strict tests/python/typing/*.py examples/python/recipes/*.py
mkdocs build --strict
python ci/check-docs.py
cargo test -p dynibo --doc --locked
cargo run --example recipes --locked
```

The installed-package suite also runs all six Python recipes and checks the
shipped typing artifacts. Documentation builds expand the recipe scripts via
MkDocs snippets; edit each script rather than duplicating its code in Markdown.

## Project listing metadata

Keep the GitHub About description aligned with `package.description` in
`Cargo.toml` and `project.description` in `pyproject.toml`. Link the About
website to `https://dynibo.readthedocs.io/`. Use these repository topics to
describe the supported tasks and interfaces:

```text
robotics kinematics robot-dynamics rigid-body-dynamics urdf
inverse-kinematics jacobian floating-base rust python
```

In the pull request description, briefly explain the change and list the checks
you ran.
