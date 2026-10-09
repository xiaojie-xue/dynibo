"""Black-box tests run against an installed dynibo package."""

from __future__ import annotations

import sys
from pathlib import Path

import dynibo


URDF = (
    Path(sys.argv.pop(1)).resolve()
    if len(sys.argv) > 1
    else Path("tests/data/test_arm.urdf").resolve()
)
REFERENCE = (
    Path(sys.argv.pop(1)).resolve()
    if len(sys.argv) > 1
    else URDF.with_name("pinocchio_reference_v1.tsv")
)
SOURCE_PACKAGE = Path(__file__).resolve().parents[3] / "bindings" / "python" / "dynibo"
if Path(dynibo.__file__).resolve().parent == SOURCE_PACKAGE:
    raise RuntimeError("package test imported bindings/python/dynibo from the source tree")


def reference(key: str) -> tuple[float, ...]:
    for line in REFERENCE.read_text(encoding="utf-8").splitlines():
        if not line or line.startswith("#"):
            continue
        fields = line.split("\t")
        if fields[0] == key:
            return tuple(float(value) for value in fields[1:])
    raise RuntimeError(f"missing binding reference {key!r} in {REFERENCE}")


def motion_base() -> dynibo.BaseState:
    velocity = reference("floating_base_velocity")
    acceleration = reference("floating_base_acceleration")
    return dynibo.BaseState(
        dynibo.Pose(
            translation=reference("floating_base_translation"),
            rotation_xyzw=reference("floating_motion_rotation_xyzw"),
        ),
        dynibo.Twist(angular=velocity[:3], linear=velocity[3:]),
        dynibo.Twist(angular=acceleration[:3], linear=acceleration[3:]),
    )


def twist_values(twist: dynibo.Twist) -> tuple[float, ...]:
    return twist.angular + twist.linear
