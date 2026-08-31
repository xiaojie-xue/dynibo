# Test architecture

The integration-test support code under `tests/support` provides four shared
building blocks:

- deterministic, seed-addressable URDF model generation;
- deterministic joint and floating-base states;
- absolute-plus-relative numerical assertions with complete case context;
- algorithm-matrix and workspace-sequence runners.

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
