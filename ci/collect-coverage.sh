#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_root}"

python_command="${PYTHON:-python3}"
report_path="${1:-coverage.json}"
codecov_path="${2:-}"
export RUSTUP_TOOLCHAIN="${RUSTUP_TOOLCHAIN:-nightly}"
coverage_work="$(mktemp -d "${TMPDIR:-/tmp}/dynibo-coverage.XXXXXX")"
cleanup() {
    rm -rf -- "${coverage_work}"
}
trap cleanup EXIT

# cargo-llvm-cov documents show-env as the entry point for external test
# processes. Maturin inherits this instrumentation when it builds the PyO3
# extension, and the installed extension writes profiles beside the Rust test
# profiles when the Python process exits.
eval "$(cargo +nightly llvm-cov --branch show-env --sh)"
cargo +nightly llvm-cov clean --workspace

cargo +nightly test --workspace --all-targets --locked

wheelhouse="${coverage_work}/wheelhouse"
installed="${coverage_work}/installed"
mkdir -p "${wheelhouse}" "${installed}"
"${python_command}" -m maturin build \
    --profile dev \
    --locked \
    --manifest-path bindings/python/Cargo.toml \
    --out "${wheelhouse}"
"${python_command}" -m pip install \
    --no-deps \
    --target "${installed}" \
    "${wheelhouse}"/*.whl
PYTHONPATH="${installed}" \
    "${python_command}" tests/python/test_package.py tests/data/test_arm.urdf \
    tests/data/pinocchio_reference_v1.tsv

coverage_packages=(-p dynibo -p dynibo-c -p dynibo-python)
cargo +nightly llvm-cov report \
    --branch \
    "${coverage_packages[@]}" \
    --json \
    --summary-only \
    --output-path "${report_path}"

if [[ -n "${codecov_path}" ]]; then
    cargo +nightly llvm-cov report \
        --branch \
        "${coverage_packages[@]}" \
        --codecov \
        --output-path "${codecov_path}"
fi
