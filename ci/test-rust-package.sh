#!/usr/bin/env bash
set -euo pipefail

if [[ $# -gt 1 ]]; then
    echo "usage: $0 [crate-output-directory]" >&2
    exit 2
fi

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_root}"

test_root="$(mktemp -d "${TMPDIR:-/tmp}/dynibo-rust-package-test.XXXXXX")"
trap 'rm -rf "${test_root}"' EXIT
package_target="${test_root}/package-target"

package_args=(package -p dynibo --locked --target-dir "${package_target}")
if [[ "${DYNIBO_ALLOW_DIRTY:-0}" == "1" ]]; then
    package_args+=(--allow-dirty)
fi
cargo "${package_args[@]}"

crate_file="$(find "${package_target}/package" -maxdepth 1 -type f -name 'dynibo-*.crate' -print -quit)"
if [[ -z "${crate_file}" ]]; then
    echo "cargo package did not produce dynibo-*.crate" >&2
    exit 1
fi

extract_root="${test_root}/extracted"
mkdir "${extract_root}"
tar -xzf "${crate_file}" -C "${extract_root}"

manifest="$(find "${extract_root}" -mindepth 2 -maxdepth 2 -name Cargo.toml -print -quit)"
if [[ -z "${manifest}" ]]; then
    echo "could not find Cargo.toml in the extracted crate" >&2
    exit 1
fi

echo "Testing extracted crate: ${manifest}"
cargo test --manifest-path "${manifest}" --locked --all-targets
RUSTDOCFLAGS="--html-in-header docs/rustdoc/katex-header.html" \
    cargo doc --manifest-path "${manifest}" --locked --no-deps

if [[ $# -eq 1 ]]; then
    crate_output_dir="$1"
    mkdir -p "${crate_output_dir}"
    cp "${crate_file}" "${crate_output_dir}/"
    echo "Exported crate to ${crate_output_dir}/$(basename "${crate_file}")"
fi
