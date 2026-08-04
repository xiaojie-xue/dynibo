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
SOURCE_PACKAGE = Path(__file__).resolve().parents[2] / "bindings" / "python" / "dyno"
if Path(dyno.__file__).resolve().parent == SOURCE_PACKAGE:
    raise RuntimeError("package test imported bindings/python/dyno from the source tree")


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
        with self.assertRaisesRegex(ValueError, "does not exist"):
            self.robot.link_id("missing")
        with self.assertRaisesRegex(ValueError, "expected 4 elements"):
            self.robot.jacobian(self.q[:-1], self.target)
        pose = dyno.Pose(rotation_xyzw=(0.0, 0.0, 0.0, 0.0))
        with self.assertRaisesRegex(ValueError, "zero quaternion"):
            self.robot.inverse_kinematics(self.q, self.target, pose)

        with self.assertRaises(dyno.ModelError):
            dyno.Robot(URDF.with_name("missing-model.urdf"))

        unreachable = dyno.Pose(translation=(100.0, 0.0, 0.0))
        options = dyno.IkOptions(max_iterations=1)
        with self.assertRaises(dyno.SolverError):
            self.robot.inverse_kinematics(self.q, self.target, unreachable, options)

    def test_non_default_frames_options_and_loads(self) -> None:
        moving = [0.1, -0.2, 0.3, -0.4]
        base = dyno.Pose(rotation_xyzw=(2**-0.5, 0.0, 0.0, 2**-0.5))
        tool = dyno.Pose(translation=(0.1, -0.03, 0.2))
        origin_velocity = self.robot.forward_velocity(self.q, moving, self.target)
        tool_velocity = self.robot.forward_velocity(
            self.q, moving, self.target, base=base, tool=tool
        )
        self.assertNotEqual(tool_velocity, origin_velocity)
        self.assertNotEqual(self.robot.gravity(self.q, base=base), self.robot.gravity(self.q))

        pose = self.robot.forward_kinematics(self.q, self.target)
        options = dyno.IkOptions(
            max_iterations=1,
            translation_tolerance=1.0e-8,
            rotation_tolerance=1.0e-8,
            damping=1.0e-4,
            max_step_norm=0.1,
        )
        self.assertEqual(
            self.robot.inverse_kinematics(self.q, self.target, pose, options), tuple(self.q)
        )

    def test_pinocchio_numeric_reference(self) -> None:
        q = [0.2, 1.0, -0.7, 0.4]
        qd = [-0.3, 0.5, -0.2, 0.8]
        qdd = [0.7, -0.4, 0.1, 0.3]
        pose = self.robot.forward_kinematics(q, self.target)
        for actual, expected in zip(
            pose.translation,
            (0.450338323287074, 0.09128809750443889, 0.46592677713692876),
        ):
            self.assertAlmostEqual(actual, expected, delta=2.0e-12)

        gravity = self.robot.gravity(q)
        expected_gravity = (
            1.7763568394002505e-15,
            39.629058959145354,
            17.60815765611755,
            0.053134179784508524,
        )
        dynamics = self.robot.inverse_dynamics(q, qd, qdd)
        expected_dynamics = (
            1.7649236924309104,
            38.319908179086525,
            17.136450444507805,
            0.05169960944426318,
        )
        for actual, expected in zip(gravity, expected_gravity):
            self.assertAlmostEqual(actual, expected, delta=2.0e-10)
        for actual, expected in zip(dynamics, expected_dynamics):
            self.assertAlmostEqual(actual, expected, delta=2.0e-10)

    def test_python_input_validation_and_lifecycle(self) -> None:
        with self.assertRaisesRegex(TypeError, "sequence of numbers"):
            self.robot.forward_kinematics(["not-a-number"] * 4, self.target)
        with self.assertRaisesRegex(ValueError, "q and qd must have the same length"):
            self.robot.forward_velocity(self.q, self.q[:-1], self.target)
        with self.assertRaisesRegex(ValueError, "q and qdd must have the same length"):
            self.robot.forward_acceleration(self.q, self.q, self.q[:-1], self.target)
        with self.assertRaisesRegex(ValueError, "q and qd must have the same length"):
            self.robot.inverse_dynamics(self.q, self.q[:-1], self.q)
        with self.assertRaisesRegex(ValueError, "pose translation must contain exactly 3"):
            self.robot.gravity(self.q, base=dyno.Pose(translation=(0.0, 0.0)))
        with self.assertRaisesRegex(ValueError, "load force must contain exactly 3"):
            self.robot.gravity(self.q, loads=[dyno.Load(self.target, force=(1.0, 2.0))])

        with dyno.Robot(URDF) as managed:
            self.assertEqual(managed.name, "test_arm")
        managed.close()
        with self.assertRaisesRegex(ValueError, "robot must not be null"):
            managed.forward_kinematics(self.q, self.target)


if __name__ == "__main__":
    unittest.main()
