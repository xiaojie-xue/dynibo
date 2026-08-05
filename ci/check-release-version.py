#!/usr/bin/env python3
"""Check that a release tag matches every published package version."""

from __future__ import annotations

import argparse
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
FINAL_RELEASE_TAG = re.compile(r"v(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)")


def capture(path: str, pattern: str, label: str) -> tuple[str, str]:
    text = (ROOT / path).read_text(encoding="utf-8")
    match = re.search(pattern, text, flags=re.MULTILINE | re.DOTALL)
    if match is None:
        raise SystemExit(f"could not read {label} from {path}")
    return label, match.group(1)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("tag", help="Git tag in vMAJOR.MINOR.PATCH form")
    args = parser.parse_args()

    tag_match = FINAL_RELEASE_TAG.fullmatch(args.tag)
    if tag_match is None:
        raise SystemExit(
            f"release tag {args.tag!r} must use the final-release form vMAJOR.MINOR.PATCH"
        )
    expected = args.tag[1:]

    versions = [
        capture(
            "Cargo.toml",
            r'^\[package\].*?^version\s*=\s*"([^"]+)"',
            "Rust crate",
        ),
        capture(
            "bindings/c/Cargo.toml",
            r'^\[package\].*?^version\s*=\s*"([^"]+)"',
            "C ABI crate",
        ),
        capture(
            "bindings/c/Cargo.toml",
            r'^dynibo\s*=\s*\{[^\n]*version\s*=\s*"([^"]+)"',
            "C ABI dependency",
        ),
        capture(
            "pyproject.toml",
            r'^\[project\].*?^version\s*=\s*"([^"]+)"',
            "Python project",
        ),
        capture("setup.py", r'^\s*version\s*=\s*"([^"]+)"', "setuptools"),
        capture(
            "bindings/python/dynibo/__init__.py",
            r'^__version__\s*=\s*"([^"]+)"',
            "Python runtime",
        ),
        capture(
            "CMakeLists.txt",
            r'project\(dynibo\s+VERSION\s+([^\s\)]+)',
            "CMake project",
        ),
        capture(
            "bindings/c/src/lib.rs",
            r'fn dynibo_version\(\).*?c"([^"]+)"',
            "C ABI runtime",
        ),
    ]

    header = (ROOT / "bindings/c/include/dynibo/dynibo.h").read_text(encoding="utf-8")
    header_parts = []
    for part in ("MAJOR", "MINOR", "PATCH"):
        match = re.search(rf"^#define DYNIBO_VERSION_{part}\s+([0-9]+)$", header, re.MULTILINE)
        if match is None:
            raise SystemExit(f"could not read C header {part.lower()} version")
        header_parts.append(match.group(1))
    versions.append(("C header", ".".join(header_parts)))

    mismatches = [(label, version) for label, version in versions if version != expected]
    for label, version in versions:
        print(f"{label:20} {version}")
    if mismatches:
        details = ", ".join(f"{label}={version}" for label, version in mismatches)
        raise SystemExit(f"release tag expects {expected}, but found: {details}")

    print(f"all release versions match {args.tag}")


if __name__ == "__main__":
    main()
