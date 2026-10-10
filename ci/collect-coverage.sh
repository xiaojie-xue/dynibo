#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_root}"

python_command="${PYTHON:-python3}"
report_path="${1:-coverage.json}"
codecov_path="${2:-}"
# Shared with the CI coverage job; avoid silently measuring with another runtime.
source ci/coverage.env
export RUSTUP_TOOLCHAIN="${COVERAGE_RUST_TOOLCHAIN}"
export PYO3_PYTHON="${python_command}"
"${python_command}" - "${COVERAGE_PYTHON_VERSION}" <<'PYTHON'
import platform
import sys
from importlib.metadata import version
from pathlib import Path
if platform.python_version() != sys.argv[1]:
    raise SystemExit(f"Coverage requires Python {sys.argv[1]}, got {platform.python_version()}; see tests/TESTING.md")
for requirement in Path("ci/coverage-requirements.txt").read_text().splitlines():
    package, expected = requirement.split("==")
    if version(package) != expected:
        raise SystemExit(f"Coverage requires {requirement}; install ci/coverage-requirements.txt")
PYTHON
actual_llvm_cov="$(cargo llvm-cov --version)"
if [[ "${actual_llvm_cov}" != "cargo-llvm-cov ${COVERAGE_LLVM_COV_VERSION}" ]]; then
    echo "Coverage requires cargo-llvm-cov ${COVERAGE_LLVM_COV_VERSION}; got ${actual_llvm_cov}" >&2
    exit 1
fi
printf 'Rust implementation joint-test coverage: %s / Python %s / %s\n' \
    "${COVERAGE_RUST_TOOLCHAIN}" "${COVERAGE_PYTHON_VERSION}" "${actual_llvm_cov}"
coverage_work="$(mktemp -d "${TMPDIR:-/tmp}/dynibo-coverage.XXXXXX")"
cleanup() {
    rm -rf -- "${coverage_work}"
}
trap cleanup EXIT

# cargo-llvm-cov documents show-env as the entry point for external test
# processes. Maturin inherits this instrumentation when it builds the PyO3
# extension, and the installed extension writes profiles beside the Rust test
# profiles when the Python process exits.
eval "$(cargo llvm-cov --branch show-env --sh)"
cargo llvm-cov clean --workspace

cargo test --workspace --all-targets --locked

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

# Only production Rust implementation files. Test modules/helpers are also
# excluded at their definitions with coverage(off), without skipping execution.
coverage_packages=(-p dynibo -p dynibo-c -p dynibo-python)
coverage_filters=(--ignore-filename-regex '(^|/)(build\.rs|tests|benches|examples)(/|$)')
cargo llvm-cov report \
    --branch \
    "${coverage_packages[@]}" \
    "${coverage_filters[@]}" \
    --json \
    --summary-only \
    --output-path "${report_path}"

if [[ -n "${codecov_path}" ]]; then
    cargo llvm-cov report \
        --branch \
        "${coverage_packages[@]}" \
        "${coverage_filters[@]}" \
        --codecov \
        --output-path "${codecov_path}"
fi
