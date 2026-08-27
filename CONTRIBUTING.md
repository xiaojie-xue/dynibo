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

- Keep changes focused and add tests for new behavior or bug fixes.
- Update public API documentation and examples when usage changes.
- Keep `README.md` and `README.zh.md` aligned.
- Include reproducible measurements and environment details for performance
  changes.

In the pull request description, briefly explain the change and list the checks
you ran.
