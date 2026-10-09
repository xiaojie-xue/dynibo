from __future__ import annotations

import unittest
import dynibo
import numpy as np

from .support import URDF, reference, motion_base, twist_values


class FloatingMotionTests(unittest.TestCase):
    def setUp(self) -> None:
        self.robot = dynibo.FloatingRobot(URDF)
        self.addCleanup(self.robot.close)
        self.target = self.robot.link_id("test_link_4")
        self.q = reference("q")

    def test_motion_matches_pinocchio(self) -> None:
        base = motion_base()
        tool = dynibo.Pose(translation=reference("floating_motion_tool_translation"))
        for case in ("base_only", "moving"):
            qd = reference("qd") if case == "moving" else (0.0,) * len(self.q)
            qdd = reference("qdd") if case == "moving" else (0.0,) * len(self.q)
            results = {
                "jacobian_derivative": self.robot.jacobian_derivative(
                    base, self.q, qd, self.target
                ),
                "velocity": twist_values(
                    self.robot.forward_velocity_kinematics(base, self.q, qd, self.target)
                ),
                "acceleration": twist_values(
                    self.robot.forward_acceleration_kinematics(base, self.q, qd, qdd, self.target)
                ),
                "tool_velocity": twist_values(
                    self.robot.forward_velocity_kinematics(base, self.q, qd, self.target, tool=tool)
                ),
                "velocity_product": self.robot.velocity_product_forces(base, self.q, qd),
            }
            for name, actual in results.items():
                with self.subTest(case=case, operation=name):
                    expected = reference(f"floating_motion_{case}_{name}")
                    self.assertEqual(np.shape(actual), np.shape(expected))
                    np.testing.assert_allclose(actual, expected, atol=3.0e-9, rtol=1.0e-9)

            # Independent kinematic identities also check the generalized-coordinate
            # ordering and the column-major matrix layout exposed by Python.
            n = self.robot.generalized_count
            jacobian = self.robot.jacobian(base, self.q, self.target).reshape((6, n), order="F")
            derivative = results["jacobian_derivative"].reshape((6, n), order="F")
            velocity = np.asarray(twist_values(base.velocity) + qd)
            acceleration = np.asarray(twist_values(base.acceleration) + qdd)
            np.testing.assert_allclose(
                jacobian @ velocity, results["velocity"], atol=2.0e-12, rtol=1.0e-10
            )
            np.testing.assert_allclose(
                jacobian @ acceleration + derivative @ velocity,
                results["acceleration"],
                atol=2.0e-12,
                rtol=1.0e-10,
            )

    def test_stationary_base_and_joints_have_zero_motion(self) -> None:
        base = dynibo.BaseState(frame=motion_base().frame)
        zero = (0.0,) * len(self.q)
        np.testing.assert_array_equal(
            self.robot.jacobian_derivative(base, self.q, zero, self.target),
            np.zeros(6 * self.robot.generalized_count),
        )
        self.assertEqual(
            self.robot.forward_velocity_kinematics(base, self.q, zero, self.target), dynibo.Twist()
        )
        self.assertEqual(
            self.robot.forward_acceleration_kinematics(base, self.q, zero, zero, self.target),
            dynibo.Twist(),
        )
        np.testing.assert_allclose(
            self.robot.velocity_product_forces(base, self.q, zero),
            np.zeros(self.robot.generalized_count),
            atol=1.0e-12,
            rtol=0,
        )
