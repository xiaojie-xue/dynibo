#!/usr/bin/env python3
"""Check that a release tag matches the canonical Cargo package version."""

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
            r'^\[workspace\.package\].*?^version\s*=\s*"([^"]+)"',
            "Cargo workspace",
        )
    ]

    mismatches = [(label, version) for label, version in versions if version != expected]
    for label, version in versions:
        print(f"{label:20} {version}")
    if mismatches:
        details = ", ".join(f"{label}={version}" for label, version in mismatches)
        raise SystemExit(f"release tag expects {expected}, but found: {details}")

    print(f"all release versions match {args.tag}")


if __name__ == "__main__":
    main()
