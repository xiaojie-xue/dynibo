#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_root}"

package_args=(package -p dyno --locked)
if [[ "${DYNO_ALLOW_DIRTY:-0}" == "1" ]]; then
    package_args+=(--allow-dirty)
fi
cargo "${package_args[@]}"

crate_file="$(find target/package -maxdepth 1 -type f -name 'dyno-*.crate' -print -quit)"
if [[ -z "${crate_file}" ]]; then
    echo "cargo package did not produce target/package/dyno-*.crate" >&2
    exit 1
fi

test_root="$(mktemp -d "${TMPDIR:-/tmp}/dyno-rust-package-test.XXXXXX")"
trap 'rm -rf "${test_root}"' EXIT
tar -xzf "${crate_file}" -C "${test_root}"

manifest="$(find "${test_root}" -mindepth 2 -maxdepth 2 -name Cargo.toml -print -quit)"
if [[ -z "${manifest}" ]]; then
    echo "could not find Cargo.toml in the extracted crate" >&2
    exit 1
fi

echo "Testing extracted crate: ${manifest}"
cargo test --manifest-path "${manifest}" --locked --all-targets
