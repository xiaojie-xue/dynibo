#!/usr/bin/env python3
"""Complete fixed-base dynibo example using the bundled Franka URDF."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Sequence

from dynibo import IkOptions, Load, Robot


def print_matrix(label: str, values: Sequence[float], rows: int, columns: int) -> None:
    """Print a flat column-major matrix without requiring NumPy."""
    print(f"{label} ({rows} x {columns}):")
    for row in range(rows):
        print("  " + " ".join(f"{values[row + column * rows]: .5f}" for column in range(columns)))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "urdf",
        nargs="?",
        type=Path,
        default=Path(__file__).resolve().parents[1]
        / "data"
        / "franka"
        / "franka_fer.urdf",
    )
    parser.add_argument("--target", default="fer_link8")
    args = parser.parse_args()

    robot = Robot.from_urdf(args.urdf)
    flange = robot.link_id(args.target)

    # Only non-fixed joints occupy state-vector entries; fer_joint8 is fixed.
    q = (0.0, -0.3, 0.0, -1.8, 0.0, 1.5, 0.7)
    qd = (0.10, -0.20, 0.15, 0.05, -0.10, 0.20, -0.05)
    qdd = (0.20, 0.10, -0.10, 0.05, 0.10, -0.05, 0.15)
    ik_initial_q = (0.05, -0.2, -0.05, -1.6, 0.05, 1.4, 0.6)

    if robot.joint_count != len(q):
        raise ValueError(
            f"this example expects {len(q)} non-fixed joints, "
            f"but {args.urdf} has {robot.joint_count}"
        )

    # forward_kinematics -- target-link pose in the world frame.
    pose = robot.forward_kinematics(q, flange)

    # jacobian and jacobian_derivative -- flat column-major matrices.
    jacobian = robot.jacobian(q, flange)
    jacobian_derivative = robot.jacobian_derivative(q, qd, flange)

    velocity = robot.forward_velocity_kinematics(q, qd, flange)
    acceleration = robot.forward_acceleration_kinematics(q, qd, qdd, flange)

    # inverse_kinematics -- recover a known, reachable pose.
    ik_solution = robot.inverse_kinematics(ik_initial_q, flange, pose, IkOptions())

    # Joint-space dynamics.
    mass_matrix = robot.mass_matrix(q)
    velocity_forces = robot.velocity_product_forces(q, qd)

    # A link-local downward force applied at the flange origin.
    loads = (Load(link_id=flange, force=(0.0, 0.0, -5.0)),)
    gravity = robot.gravity(q, loads=loads)
    joint_forces = robot.inverse_dynamics(q, qd, qdd, loads=loads)

    print(
        f"loaded {robot.name}: {robot.link_count} links, "
        f"{robot.joint_count} non-fixed joints"
    )
    print("forward_kinematics translation [m]:", pose.translation)
    print("forward_kinematics quaternion (x, y, z, w):", pose.rotation_xyzw)
    print_matrix("jacobian", jacobian, 6, robot.generalized_count)
    print_matrix(
        "jacobian_derivative", jacobian_derivative, 6, robot.generalized_count
    )
    print("forward_velocity_kinematics:", velocity)
    print("forward_acceleration_kinematics:", acceleration)
    print("inverse_kinematics:", ik_solution)
    print_matrix(
        "mass_matrix",
        mass_matrix,
        robot.generalized_count,
        robot.generalized_count,
    )
    print("velocity_product_forces:", velocity_forces)
    print("gravity (including external load):", gravity)
    print("inverse_dynamics (including external load):", joint_forces)


if __name__ == "__main__":
    main()
