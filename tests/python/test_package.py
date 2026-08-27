"""Black-box tests run against an installed dynibo package."""

from __future__ import annotations

import sys
import threading
import unittest
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
SOURCE_PACKAGE = Path(__file__).resolve().parents[2] / "bindings" / "python" / "dynibo"
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


class PackageTests(unittest.TestCase):
    def setUp(self) -> None:
        self.robot = dynibo.Robot.from_urdf(URDF)
        self.addCleanup(self.robot.close)
        self.target = self.robot.link_id("test_link_4")
        self.q = [0.0] * self.robot.joint_count

    def test_model_and_kinematics(self) -> None:
        self.assertEqual(dynibo.__version__, "0.4.0")
        self.assertEqual(self.robot.name, "test_arm")
        self.assertEqual(self.robot.joint_count, 4)
        self.assertEqual(self.robot.link_count, 5)
        pose = self.robot.forward_kinematics(self.q, self.target)
        self.assertAlmostEqual(pose.translation[0], 0.62)
        self.assertAlmostEqual(pose.translation[1], 0.0)
        self.assertAlmostEqual(pose.translation[2], 0.108)
        self.assertEqual(len(self.robot.jacobian(self.q, self.target)), 24)
        self.assertEqual(
            self.robot.forward_velocity_kinematics(self.q, self.q, self.target),
            dynibo.Twist(),
        )
        self.assertEqual(
            self.robot.forward_acceleration_kinematics(
                self.q, self.q, self.q, self.target
            ),
            dynibo.Twist(),
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
            self.q, loads=[dynibo.Load(self.target, force=(0.0, 1.0, 0.0))]
        )
        self.assertNotEqual(loaded, gravity)

    def test_second_order_dynamics_apis(self) -> None:
        q = [0.2, 1.0, -0.7, 0.4]
        qd = [-0.3, 0.5, -0.2, 0.8]
        zero = [0.0] * self.robot.joint_count
        mass = self.robot.mass_matrix(q)
        self.assertEqual(len(mass), self.robot.joint_count**2)
        velocity_product = self.robot.velocity_product_forces(q, qd)
        self.assertEqual(len(velocity_product), self.robot.generalized_count)
        derivative = self.robot.jacobian_derivative(q, qd, self.target)
        self.assertEqual(len(derivative), 6 * self.robot.joint_count)

        n = self.robot.joint_count
        for row in range(n):
            for column in range(n):
                self.assertAlmostEqual(
                    mass[column * n + row], mass[row * n + column], delta=1.0e-12
                )

        gravity = self.robot.gravity(q)
        bias = self.robot.inverse_dynamics(q, qd, zero)
        for row in range(n):
            reconstructed = gravity[row] + velocity_product[row]
            self.assertAlmostEqual(reconstructed, bias[row], delta=1.0e-10)

        expected_qdd = [0.7, -0.4, 0.1, 0.3]
        generalized_forces = self.robot.inverse_dynamics(q, qd, expected_qdd)
        recovered_qdd = self.robot.forward_dynamics(q, qd, generalized_forces)
        for actual, expected_value in zip(recovered_qdd, expected_qdd):
            self.assertAlmostEqual(actual, expected_value, delta=2.0e-10)

        acceleration = self.robot.forward_acceleration_kinematics(
            q, qd, zero, self.target
        )
        expected = tuple(acceleration.angular) + tuple(acceleration.linear)
        for row in range(6):
            contracted = sum(derivative[column * 6 + row] * qd[column] for column in range(n))
            self.assertAlmostEqual(contracted, expected[row], delta=1.0e-10)

    def test_floating_base_shapes_state_and_ik_contract(self) -> None:
        with dynibo.FloatingRobot.from_urdf(URDF) as robot:
            target = robot.link_id("test_link_4")
            q = reference("q")
            qd = reference("qd")
            qdd = reference("qdd")
            base_translation = reference("floating_base_translation")
            base_rotation = reference("floating_base_rotation_xyzw")
            base_velocity = reference("floating_base_velocity")
            base_acceleration = reference("floating_base_acceleration")
            base = dynibo.BaseState(
                dynibo.Pose(
                    translation=base_translation,
                    rotation_xyzw=base_rotation,
                ),
                dynibo.Twist(angular=base_velocity[:3], linear=base_velocity[3:]),
                dynibo.Twist(
                    angular=base_acceleration[:3], linear=base_acceleration[3:]
                ),
            )
            self.assertEqual(robot.generalized_count, robot.joint_count + 6)
            self.assertEqual(len(robot.jacobian(base, q, target)), 6 * robot.generalized_count)
            self.assertEqual(len(robot.mass_matrix(base, q)), robot.generalized_count**2)
            pose = robot.forward_kinematics(base, q, target)
            for actual, expected_value in zip(
                pose.translation, reference("floating_fk_translation")
            ):
                self.assertAlmostEqual(actual, expected_value, delta=2.0e-12)
            for actual, expected_value in zip(
                robot.gravity(base, q), reference("floating_gravity")
            ):
                self.assertAlmostEqual(actual, expected_value, delta=2.0e-10)
            forces = robot.inverse_dynamics(base, q, qd, qdd)
            for actual, expected_value in zip(forces, reference("floating_rnea")):
                self.assertAlmostEqual(actual, expected_value, delta=2.0e-10)
            recovered = robot.forward_dynamics(base, q, qd, forces)
            expected_acceleration = base_acceleration + qdd
            for actual, expected_value in zip(recovered, expected_acceleration):
                self.assertAlmostEqual(actual, expected_value, delta=2.0e-9)
            load_values = reference("floating_load")
            load = dynibo.Load(
                target,
                torque=load_values[:3],
                force=load_values[3:],
            )
            loaded_forces = robot.inverse_dynamics(base, q, qd, qdd, loads=[load])
            for actual, expected_value in zip(
                loaded_forces, reference("floating_rnea_loaded")
            ):
                self.assertAlmostEqual(actual, expected_value, delta=2.0e-10)
            loaded_recovered = robot.forward_dynamics(
                base, q, qd, loaded_forces, loads=[load]
            )
            for actual, expected_value in zip(
                loaded_recovered, expected_acceleration
            ):
                self.assertAlmostEqual(actual, expected_value, delta=2.0e-9)
            self.assertFalse(hasattr(robot, "inverse_kinematics"))

    def test_errors_cross_the_package_boundary(self) -> None:
        with self.assertRaisesRegex(ValueError, "does not exist"):
            self.robot.link_id("missing")
        with self.assertRaisesRegex(ValueError, "expected 4 elements"):
            self.robot.jacobian(self.q[:-1], self.target)
        pose = dynibo.Pose(rotation_xyzw=(0.0, 0.0, 0.0, 0.0))
        with self.assertRaisesRegex(ValueError, "zero quaternion"):
            self.robot.inverse_kinematics(self.q, self.target, pose)

        with self.assertRaises(dynibo.ModelError):
            dynibo.Robot.from_urdf(URDF.with_name("missing-model.urdf"))

        unreachable = dynibo.Pose(translation=(100.0, 0.0, 0.0))
        options = dynibo.IkOptions(max_iterations=1)
        with self.assertRaises(dynibo.SolverError):
            self.robot.inverse_kinematics(self.q, self.target, unreachable, options)

    def test_non_default_frames_options_and_loads(self) -> None:
        moving = [0.1, -0.2, 0.3, -0.4]
        base = dynibo.Pose(rotation_xyzw=(2**-0.5, 0.0, 0.0, 2**-0.5))
        tool = dynibo.Pose(translation=(0.1, -0.03, 0.2))
        identity_pose = self.robot.forward_kinematics(self.q, self.target)
        origin_velocity = self.robot.forward_velocity_kinematics(
            self.q, moving, self.target
        )
        identity_gravity = self.robot.gravity(self.q)
        self.robot.set_base_frame(base)
        transformed_pose = self.robot.forward_kinematics(self.q, self.target)
        tool_velocity = self.robot.forward_velocity_kinematics(
            self.q, moving, self.target, tool=tool
        )
        self.assertNotEqual(transformed_pose, identity_pose)
        self.assertNotEqual(tool_velocity, origin_velocity)
        self.assertNotEqual(self.robot.gravity(self.q), identity_gravity)

        pose = self.robot.forward_kinematics(self.q, self.target)
        options = dynibo.IkOptions(
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
        q = reference("q")
        qd = reference("qd")
        qdd = reference("qdd")
        pose = self.robot.forward_kinematics(q, self.target)
        for actual, expected in zip(
            pose.translation,
            reference("fk_translation"),
        ):
            self.assertAlmostEqual(actual, expected, delta=2.0e-12)

        gravity = self.robot.gravity(q)
        expected_gravity = reference("gravity")
        dynamics = self.robot.inverse_dynamics(q, qd, qdd)
        expected_dynamics = reference("rnea")
        for actual, expected in zip(gravity, expected_gravity):
            self.assertAlmostEqual(actual, expected, delta=2.0e-10)
        for actual, expected in zip(dynamics, expected_dynamics):
            self.assertAlmostEqual(actual, expected, delta=2.0e-10)

    def test_python_input_validation_and_lifecycle(self) -> None:
        class ChangingLengthSequence:
            def __init__(self) -> None:
                self.length_calls = 0

            def __len__(self) -> int:
                self.length_calls += 1
                return 3 if self.length_calls == 1 else 4

            def __iter__(self):
                return iter((0.0, 0.0, 0.0))

        with self.assertRaisesRegex(TypeError, "sequence of numbers"):
            self.robot.forward_kinematics(["not-a-number"] * 4, self.target)
        with self.assertRaisesRegex(ValueError, "expected 4 elements"):
            self.robot.mass_matrix(ChangingLengthSequence())
        with self.assertRaisesRegex(ValueError, "q and qd must have the same length"):
            self.robot.forward_velocity_kinematics(self.q, self.q[:-1], self.target)
        with self.assertRaisesRegex(ValueError, "q and qdd must have the same length"):
            self.robot.forward_acceleration_kinematics(
                self.q, self.q, self.q[:-1], self.target
            )
        with self.assertRaisesRegex(ValueError, "q and qd must have the same length"):
            self.robot.inverse_dynamics(self.q, self.q[:-1], self.q)
        with self.assertRaisesRegex(ValueError, "q and qd must have the same length"):
            self.robot.velocity_product_forces(self.q, self.q[:-1])
        with self.assertRaisesRegex(ValueError, "q and qd must have the same length"):
            self.robot.jacobian_derivative(self.q, self.q[:-1], self.target)
        with self.assertRaisesRegex(ValueError, "expected 4 elements"):
            self.robot.mass_matrix(self.q[:-1])
        with self.assertRaisesRegex(ValueError, "expected 4 elements"):
            self.robot.forward_dynamics(self.q, self.q, self.q[:-1])
        with self.assertRaisesRegex(ValueError, "pose translation must contain exactly 3"):
            self.robot.set_base_frame(dynibo.Pose(translation=(0.0, 0.0)))
        with self.assertRaisesRegex(ValueError, "load force must contain exactly 3"):
            self.robot.gravity(self.q, loads=[dynibo.Load(self.target, force=(1.0, 2.0))])
        self.assertFalse(hasattr(dynibo, "Base" + "Mode"))

        with dynibo.Robot.from_urdf(URDF) as managed:
            self.assertEqual(managed.name, "test_arm")
        managed.close()
        with self.assertRaisesRegex(RuntimeError, "robot is closed"):
            managed.forward_kinematics(self.q, self.target)

    def test_shared_robot_serializes_native_workspace_access(self) -> None:
        expected = self.robot.jacobian(self.q, self.target)
        barrier = threading.Barrier(4)
        failures: list[BaseException] = []

        def calculate() -> None:
            try:
                barrier.wait()
                for _ in range(50):
                    self.assertEqual(self.robot.jacobian(self.q, self.target), expected)
                    self.robot.mass_matrix(self.q)
            except BaseException as error:
                failures.append(error)

        threads = [threading.Thread(target=calculate) for _ in range(4)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
        if failures:
            raise failures[0]


if __name__ == "__main__":
    unittest.main()
