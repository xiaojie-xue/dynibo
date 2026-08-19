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

For changes that affect packaging or language bindings, run the complete local
suite. Pinocchio reference tests are included when Pinocchio is available
through `pkg-config`.

```bash
bash ci/test-all.sh
```

## Guidelines

- Keep changes focused and add tests for new behavior or bug fixes.
- Update public API documentation and examples when usage changes.
- Keep `README.md` and `README.zh.md` aligned.
- Include reproducible measurements and environment details for performance
  changes.

In the pull request description, briefly explain the change and list the checks
you ran.
