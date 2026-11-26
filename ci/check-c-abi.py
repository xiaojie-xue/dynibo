#!/usr/bin/env python3
"""Verify 1-to-1 consistency between C header declarations and Rust extern "C" exports."""

from __future__ import annotations

import re
from pathlib import Path

PROJECT = Path(__file__).resolve().parents[1]
HEADER = PROJECT / "bindings" / "c" / "include" / "dynibo" / "dynibo.h"
RUST_SRC = PROJECT / "bindings" / "c" / "src"

C_FN_RE = re.compile(
    r"DYNIBO_API\s+[^;()]+?\b(dynibo_[a-z0-9_]+)\s*\(([^)]*)\)\s*;",
    re.MULTILINE | re.DOTALL,
)
RUST_FN_RE = re.compile(
    r"#\[unsafe\(no_mangle\)\]\s*pub\s+(?:unsafe\s+)?extern\s+\"C\"\s+fn\s+(dynibo_[a-z0-9_]+)\s*\(([^)]*)\)",
    re.MULTILINE | re.DOTALL,
)


def count_c_params(raw: str) -> int:
    cleaned = re.sub(r"/\*.*?\*/", "", raw, flags=re.DOTALL).strip()
    if not cleaned or cleaned == "void":
        return 0
    return len([part for part in cleaned.split(",") if part.strip()])


def count_rust_params(raw: str) -> int:
    cleaned = re.sub(r"//.*", "", raw).strip()
    if not cleaned:
        return 0
    # Split on top-level commas (none of our C ABI params use nested generics/tuples).
    return len([part for part in cleaned.split(",") if part.strip()])


def main() -> None:
    header_text = HEADER.read_text(encoding="utf-8")
    c_symbols: dict[str, int] = {}
    for match in C_FN_RE.finditer(header_text):
        name, params = match.group(1), match.group(2)
        if name in c_symbols:
            raise SystemExit(f"duplicate C declaration: {name}")
        c_symbols[name] = count_c_params(params)

    rust_symbols: dict[str, int] = {}
    for path in sorted(RUST_SRC.glob("*.rs")):
        text = path.read_text(encoding="utf-8")
        for match in RUST_FN_RE.finditer(text):
            name, params = match.group(1), match.group(2)
            if name in rust_symbols:
                raise SystemExit(f"duplicate Rust export: {name}")
            rust_symbols[name] = count_rust_params(params)

    missing_in_rust = sorted(set(c_symbols) - set(rust_symbols))
    missing_in_header = sorted(set(rust_symbols) - set(c_symbols))
    mismatched_arity = sorted(
        f"{name} (header={c_symbols[name]}, rust={rust_symbols[name]})"
        for name in set(c_symbols) & set(rust_symbols)
        if c_symbols[name] != rust_symbols[name]
    )

    errors: list[str] = []
    if missing_in_rust:
        errors.append(f"declared in dynibo.h but missing in Rust: {missing_in_rust}")
    if missing_in_header:
        errors.append(f"exported in Rust but missing in dynibo.h: {missing_in_header}")
    if mismatched_arity:
        errors.append(f"parameter count mismatch: {mismatched_arity}")
    if errors:
        raise SystemExit("\n".join(errors))
    print(f"Verified {len(c_symbols)} C ABI symbols match between dynibo.h and Rust.")


if __name__ == "__main__":
    main()
