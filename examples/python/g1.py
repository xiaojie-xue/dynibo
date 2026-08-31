#!/usr/bin/env python3
"""G1 floating-base kinematics, RNEA and ABA (no mesh assets needed)."""

from pathlib import Path

import numpy as np
from dynibo import BaseState, FloatingRobot, Pose, Twist


def main() -> None:
    path = Path(__file__).resolve().parents[1] / "data/unitree-g1/g1_29dof_mode_11.urdf"
    robot = FloatingRobot.from_urdf(path)
    target = robot.link_id("left_rubber_hand")
    n, g = robot.joint_count, robot.generalized_count
    assert (n, g) == (29, 35)
    base = BaseState(
        frame=Pose(translation=(0.0, 0.0, 0.8)),
        velocity=Twist(angular=(0.1, -0.05, 0.08), linear=(0.2, 0.0, -0.1)),
        acceleration=Twist(angular=(0.02, 0.03, -0.01), linear=(0.1, -0.2, 0.05)),
    )
    # Joint-only inputs use Dynibo's breadth-first URDF joint order.
    q = np.zeros(n)
    qd = 0.1 * np.cos(np.arange(1, n + 1, dtype=float))
    qdd = 0.2 * np.sin(np.arange(1, n + 1, dtype=float))
    pose = robot.forward_kinematics(base, q, target)
    jacobian = robot.jacobian(base, q, target).reshape((6, g), order="F")
    forces = robot.inverse_dynamics(base, q, qd, qdd)
    acceleration = robot.forward_dynamics(base, q, qd, forces)
    expected = np.r_[base.acceleration.angular, base.acceleration.linear, qdd]
    np.testing.assert_allclose(acceleration, expected, atol=1e-8, rtol=1e-8)
    print(f"{robot.name}: {n} joints, {g} generalized velocities (floating base)")
    print("left hand position [m]:", pose.translation)
    print(f"Jacobian: {jacobian.shape}; angular rows before linear rows")
    print("RNEA base wrench [torque, force]:", forces[:6])
    print("RNEA joint torques:", forces[6:])
    print("RNEA -> ABA maximum acceleration error:", np.max(np.abs(acceleration - expected)))
    # Zero base wrench models an unactuated base; no ground contacts are solved.
    forces[:6] = 0.0
    robot.forward_dynamics(base, q, qd, forces, out=acceleration)
    print("ABA unactuated-base acceleration [angular, linear]:", acceleration[:6])


if __name__ == "__main__":
    main()
