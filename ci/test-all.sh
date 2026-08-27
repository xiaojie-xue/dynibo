#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_root}"

cargo fmt --all -- --check
cargo clippy --workspace --all-targets --locked -- -D warnings
cargo test --workspace --all-targets --locked

if pkg-config --exists pinocchio; then
    cargo clippy -p dynibo --tests --locked --features pinocchio-tests -- -D warnings
    cargo test -p dynibo --locked --features pinocchio-tests --tests
elif [[ "${DYNIBO_REQUIRE_PINOCCHIO:-0}" == "1" ]]; then
    echo "Pinocchio is required, but pkg-config could not find it" >&2
    exit 1
else
    echo "Skipping Pinocchio reference tests; set PKG_CONFIG_PATH or DYNIBO_REQUIRE_PINOCCHIO=1"
fi

CARGO_NET_OFFLINE=true DYNIBO_ALLOW_DIRTY=1 bash ci/test-rust-package.sh

package_test_root="$(mktemp -d "${TMPDIR:-/tmp}/dynibo-package-test.XXXXXX")"
cleanup() {
    rm -rf -- "${package_test_root}"
}
trap cleanup EXIT

native_build="${package_test_root}/native"
cmake -S . -B "${native_build}" -DCMAKE_BUILD_TYPE=Release
cmake --build "${native_build}" --config Release --parallel
cmake --build "${native_build}" --config Release --target package
python_command="${PYTHON:-python3}"
"${python_command}" ci/test-native-package.py \
    --package-dir "${native_build}" --configuration Release

"${python_command}" -m pip wheel . --no-build-isolation --no-deps \
    --wheel-dir "${package_test_root}/wheelhouse"
"${python_command}" -m pip install --no-deps \
    --target "${package_test_root}/installed" \
    "${package_test_root}"/wheelhouse/*.whl
PYTHONPATH="${package_test_root}/installed" \
    "${python_command}" tests/python/test_package.py tests/data/test_arm.urdf \
    tests/data/pinocchio_reference_v1.tsv
