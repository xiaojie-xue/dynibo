#!/usr/bin/env python3
"""Enforce line and branch percentages in an llvm-cov JSON summary."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("report", type=Path)
    parser.add_argument("--min-lines", type=float, required=True)
    parser.add_argument("--min-branches", type=float, required=True)
    args = parser.parse_args()

    report = json.loads(args.report.read_text(encoding="utf-8"))
    totals = report["data"][0]["totals"]
    lines = float(totals["lines"]["percent"])
    branches = float(totals["branches"]["percent"])
    print(f"coverage: lines={lines:.2f}% branches={branches:.2f}%")

    failures = []
    if lines < args.min_lines:
        failures.append(f"line coverage {lines:.2f}% is below {args.min_lines:.2f}%")
    if branches < args.min_branches:
        failures.append(f"branch coverage {branches:.2f}% is below {args.min_branches:.2f}%")
    if failures:
        raise SystemExit("; ".join(failures))


if __name__ == "__main__":
    main()
