# Test architecture

The integration-test support code under `tests/support` provides four shared
building blocks:

- deterministic, seed-addressable URDF model generation;
- deterministic joint and floating-base states;
- absolute-plus-relative numerical assertions with complete case context;
- algorithm-matrix and workspace-sequence runners.

The `robot_arm` and `pinocchio_oracle` integration targets retain their root `.rs`
entry points and group test cases in same-named subdirectories. This keeps each
suite a single test executable. C ABI unit tests similarly share helpers in
`bindings/c/src/tests.rs` and group calculations, overlap checks, and ownership
checks under `bindings/c/src/tests/`.

`support/model_gen/` separates model specifications and validation, seeded
generation, corpus selection, temporary fixtures, and URDF serialization.
`support/pinocchio/` separates bridge declarations and calculation wrappers.
The installed Python suite still runs through `python/test_package.py`, with
test classes and shared fixtures in `python/package_cases/`. Allocation tests
remain independent targets with their own global allocators.

The generated pull-request corpus uses 24 reproducible pseudo-random `u64`
seeds and eight states per model. A versioned `ModelSpec` separates an explicit
24-case structural coverage plan from random physical parameters. The plan
includes fixed and floating bases independently of serial, single-branch,
balanced, wide, and unbalanced trees; it also covers absent, interleaved,
consecutive, and tool-frame fixed joints. The models cover revolute,
continuous, and prismatic joints, cardinal and non-axis-aligned axes, and
identity, offset, rotated, and offset-rotated physical inertial frames. Inertia
parameters deliberately remain in a normal, well-conditioned physical range.

Run the default suite with:

```bash
cargo test --workspace --all-targets --locked
```

When Pinocchio is available through `pkg-config`, the `pinocchio-tests` feature
adds two independent oracle layers. `pinocchio_oracle` exercises the maintained
serial, mixed-joint, branched, and free-flyer fixtures, including single- and
multi-link external loads in RNEA and ABA. `generated_pinocchio` runs the same
24 models and eight states per model through FK, velocity and acceleration
kinematics, Jacobian and its derivative, mass matrix, gravity,
velocity-product forces, RNEA, and ABA:

```bash
cargo test -p dynibo --locked --features pinocchio-tests --tests
```

The generated conformance suites accept the reproduction and corpus-size
environment variables documented below.

Reproduce one generated model with:

```bash
DYNIBO_TEST_SEED=0x1ea59f2878e51fb4 DYNIBO_TEST_CASE_ID=6 \
  cargo test --test generated_conformance -- --nocapture
```

Run a larger corpus locally with:

```bash
DYNIBO_TEST_CASES=512 \
  cargo test --test generated_conformance --release -- --nocapture
```

Run a fresh exploration corpus, seeded from the operating system, with:

```bash
DYNIBO_TEST_RANDOMIZE=1 DYNIBO_TEST_CASES=512 \
  cargo test --test generated_conformance --release -- --nocapture
```

The test reports its `master_seed`; rerun the same exploration corpus with
`DYNIBO_TEST_RANDOMIZE=1 DYNIBO_TEST_MASTER_SEED=...`. Individual failures
still report a case index and can be replayed with `DYNIBO_TEST_SEED` plus
`DYNIBO_TEST_CASE_ID`.

Set `DYNIBO_TEST_KEEP_URDF=1` to retain generated fixtures in the system
temporary directory and print their paths for inspection.

Every generated-case failure reports its seed, sample, base mode, algorithm,
target, and load case. During unwinding, the model URDF, `ModelSpec`, and a
reproduction command are kept under `target/test-failures`. Reproduce those
models with both `DYNIBO_TEST_SEED` and `DYNIBO_TEST_CASE_ID`. The generator is
versioned, so a seed continues to identify the same URDF within one generator
version.

Workspace sequence tests compare every operation on a reused `Robot` or
`FloatingRobot` against the same operation on a fresh `fork()`. The two typed
runners cover fixed and floating behavior separately. Invalid length and
foreign-link operations are interleaved with successful calculations to verify
recovery as well as scratch-buffer clearing.

Allocation tests remain separate because they own process-global allocators.
Installed C, C++, and Python package tests also remain black-box tests rather
than using Rust test helpers. They consume the versioned
`tests/data/pinocchio_reference_v1.tsv` corpus; the feature-gated Pinocchio
oracle verifies that committed corpus before package tests reuse it.

Python package tests additionally cover rotated floating-base motion with
stationary joints and with moving joints. Complete Jacobian derivatives,
velocities, accelerations, tool-point velocities, and velocity-product forces
are checked against the Pinocchio-verified corpus. Generalized-coordinate
ordering and column-major matrix layout are also checked through kinematic
identities. Both robot types exercise constructors, lifecycle errors, NumPy
input layouts, reusable outputs, and recovery after invalid calls. Non-finite
loads must raise `ValueError` before writing an output buffer.

## Rust implementation joint-test coverage

`ci/collect-coverage.sh` merges profiles from Rust workspace unit/integration
and executable example tests with calls into the Rust extension from installed
Python wheel tests. It measures the production Rust core and C/Python bindings,
not Python source, standalone C/C++ tests, or only Rust unit tests. The separate
`pinocchio-tests` CI job is not merged into this coverage run.

Test modules and test-only helpers use conditional `coverage(off)` attributes;
they still execute and contribute coverage to production code. Test files,
examples, benchmarks, and build scripts are excluded from the reported source
scope. Production validation, panic handling, and numerical safeguards remain
in scope. Both LLVM and Codecov exports use the same source filters.

The CI gate uses LLVM line and branch percentages from `coverage.json`.
`codecov.json` is the Codecov upload format; locally counting fully hit entries
in that file is neither LLVM branch coverage nor a confirmed Codecov service
result. Keep these metrics separate. Existing gates remain 85% lines and 75%
branches; changing the source scope establishes a new baseline, not a test gain.

The coverage job and local script share exact Rust, Python, and cargo-llvm-cov
versions in `ci/coverage.env`, plus Python dependencies in
`ci/coverage-requirements.txt`. Other jobs retain their existing toolchains.
On Linux, prepare and run the same environment as CI:

```bash
source ci/coverage.env
rustup toolchain install "$COVERAGE_RUST_TOOLCHAIN" --profile minimal --component llvm-tools-preview
cargo install cargo-llvm-cov --version "$COVERAGE_LLVM_COV_VERSION" --locked
# Install the exact Python release from coverage.env (for example using uv).
uv python install "$COVERAGE_PYTHON_VERSION"
uv venv --python "$COVERAGE_PYTHON_VERSION" --seed .venv-coverage
.venv-coverage/bin/python -m pip install -r ci/coverage-requirements.txt
mkdir -p target/coverage
PYTHON="$PWD/.venv-coverage/bin/python" bash ci/collect-coverage.sh target/coverage/coverage.json target/coverage/codecov.json
python3 ci/check-coverage.py target/coverage/coverage.json --min-lines 85 --min-branches 75
```

The script rejects mismatched Python/dependency/cargo-llvm-cov versions before
collecting profiles. Update pins deliberately and rebaseline when changing the
compiler: nightly branch instrumentation can change its measured denominator.

Baseline measured on 2026-10-10 with these pins on Linux x86_64: 117 Rust tests
and 24 installed-wheel Python tests passed. Production Rust coverage was
5,046/5,132 lines (98.32%) and 315/334 branch outcomes (94.31%). This baseline
excludes test implementations and is not directly comparable to older reports
that included them. It is a local measurement, not a Codecov service result.
