# Test architecture

The integration-test support code under `tests/support` provides four shared
building blocks:

- deterministic, seed-addressable URDF model generation;
- deterministic joint and floating-base states;
- absolute-plus-relative numerical assertions with complete case context;
- algorithm-matrix and workspace-sequence runners.

The generated pull-request corpus uses 16 reproducible pseudo-random `u64`
seeds and eight states per model. SplitMix64 supplies uniformly distributed high
bits; the seed residue modulo 12 is stratified by corpus position so the first
twelve cases cover every model size from one through twelve joints together
with the associated fixed/floating and serial/branched patterns. The models
also cover mixed revolute, continuous, prismatic, and fixed joints,
non-axis-aligned joint axes, rotated inertial frames, and massless fixed
intermediary links.

Run the default suite with:

```bash
cargo test --workspace --all-targets --locked
```

Reproduce one generated model with:

```bash
DYNIBO_TEST_SEED=0x1ea59f2878e51fb4 \
  cargo test --test generated_conformance -- --nocapture
```

Run a larger corpus locally with:

```bash
DYNIBO_TEST_CASES=512 \
  cargo test --test generated_conformance --release -- --nocapture
```

Every generated-case failure reports its seed, sample, base mode, algorithm,
target, and load case. A seed must continue to identify the same URDF, so the
test-only stable random generator must not be replaced without intentionally
versioning the corpus.

Workspace sequence tests compare every operation on a reused `Robot` against
the same operation on a fresh `fork()`. Fixed- and floating-base sequences are
separate because base mode is a model property. Invalid length and foreign-link
operations are interleaved with successful calculations to verify recovery as
well as scratch-buffer clearing.

Allocation tests remain separate because they own process-global allocators.
Installed C, C++, and Python package tests also remain black-box tests rather
than using Rust test helpers.
