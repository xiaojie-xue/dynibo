"""Black-box tests run against an installed dyno-robotics package."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import dyno


URDF = (
    Path(sys.argv.pop(1)).resolve()
    if len(sys.argv) > 1
    else Path("tests/data/test_arm.urdf").resolve()
)
SOURCE_PACKAGE = Path(__file__).resolve().parents[2] / "python" / "dyno"
if Path(dyno.__file__).resolve().parent == SOURCE_PACKAGE:
    raise RuntimeError("package test imported python/dyno from the source tree")


class PackageTests(unittest.TestCase):
    def setUp(self) -> None:
        self.robot = dyno.Robot(URDF)
        self.addCleanup(self.robot.close)
        self.target = self.robot.link_id("test_link_4")
        self.q = [0.0] * self.robot.joint_count

    def test_model_and_kinematics(self) -> None:
        self.assertEqual(dyno.__version__, "0.1.0")
        self.assertEqual(self.robot.name, "test_arm")
        self.assertEqual(self.robot.joint_count, 4)
        self.assertEqual(self.robot.link_count, 5)
        pose = self.robot.forward_kinematics(self.q, self.target)
        self.assertAlmostEqual(pose.translation[0], 0.62)
        self.assertAlmostEqual(pose.translation[1], 0.0)
        self.assertAlmostEqual(pose.translation[2], 0.108)
        self.assertEqual(len(self.robot.jacobian(self.q, self.target)), 24)
        self.assertEqual(self.robot.forward_velocity(self.q, self.q, self.target), dyno.Twist())
        self.assertEqual(
            self.robot.forward_acceleration(self.q, self.q, self.q, self.target), dyno.Twist()
        )

    def test_dynamics_and_inverse_kinematics(self) -> None:
        pose = self.robot.forward_kinematics(self.q, self.target)
        self.assertEqual(self.robot.inverse_kinematics(self.q, self.target, pose), tuple(self.q))
        gravity = self.robot.gravity(self.q)
        dynamics = self.robot.inverse_dynamics(self.q, self.q, self.q)
        self.assertEqual(len(gravity), self.robot.joint_count)
        self.assertEqual(len(dynamics), self.robot.joint_count)
        for left, right in zip(gravity, dynamics):
            self.assertAlmostEqual(left, right)

        loaded = self.robot.gravity(
            self.q, loads=[dyno.Load(self.target, force=(0.0, 1.0, 0.0))]
        )
        self.assertNotEqual(loaded, gravity)

    def test_errors_cross_the_package_boundary(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "does not exist"):
            self.robot.link_id("missing")
        with self.assertRaisesRegex(RuntimeError, "expected 4 elements"):
            self.robot.jacobian(self.q[:-1], self.target)
        pose = dyno.Pose(rotation_xyzw=(0.0, 0.0, 0.0, 0.0))
        with self.assertRaisesRegex(RuntimeError, "zero quaternion"):
            self.robot.inverse_kinematics(self.q, self.target, pose)


if __name__ == "__main__":
    unittest.main()
