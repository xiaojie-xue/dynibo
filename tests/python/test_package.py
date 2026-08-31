"""Black-box tests run against an installed dynibo package."""

from __future__ import annotations

import sys
import threading
import unittest
from importlib.metadata import version as distribution_version
from pathlib import Path

import dynibo
import numpy as np


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
        self.assertEqual(dynibo.__version__, distribution_version("dynibo"))
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
        np.testing.assert_allclose(
            self.robot.inverse_kinematics(self.q, self.target, pose), self.q
        )
        gravity = self.robot.gravity(self.q)
        dynamics = self.robot.inverse_dynamics(self.q, self.q, self.q)
        self.assertEqual(len(gravity), self.robot.joint_count)
        self.assertEqual(len(dynamics), self.robot.joint_count)
        for left, right in zip(gravity, dynamics):
            self.assertAlmostEqual(left, right)

        loaded = self.robot.gravity(
            self.q, loads=[dynibo.Load(self.target, force=(0.0, 1.0, 0.0))]
        )
        self.assertFalse(np.array_equal(loaded, gravity))

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
            gravity_out = np.empty(robot.generalized_count, dtype=np.float64)
            self.assertIs(robot.gravity(base, q, out=gravity_out), gravity_out)
            np.testing.assert_allclose(
                gravity_out, reference("floating_gravity"), atol=2.0e-12
            )
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
        self.assertFalse(np.array_equal(self.robot.gravity(self.q), identity_gravity))

        pose = self.robot.forward_kinematics(self.q, self.target)
        options = dynibo.IkOptions(
            max_iterations=1,
            translation_tolerance=1.0e-8,
            rotation_tolerance=1.0e-8,
            damping=1.0e-4,
            max_step_norm=0.1,
        )
        np.testing.assert_allclose(
            self.robot.inverse_kinematics(self.q, self.target, pose, options), self.q
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

        with self.assertRaises((TypeError, ValueError)):
            self.robot.forward_kinematics(["not-a-number"] * 4, self.target)
        with self.assertRaisesRegex(ValueError, "max_iterations must be greater than zero"):
            dynibo.IkOptions(max_iterations=-1)
        with self.assertRaisesRegex(ValueError, "max_iterations must be greater than zero"):
            dynibo.IkOptions(max_iterations=0)
        with self.assertRaisesRegex(TypeError, "max_iterations must be an integer"):
            dynibo.IkOptions(max_iterations=1.5)
        for invalid in (float("nan"), float("inf"), float("-inf")):
            q = self.q.copy()
            q[0] = invalid
            with self.subTest(invalid=invalid):
                with self.assertRaisesRegex(ValueError, "q contains a non-finite value"):
                    self.robot.forward_kinematics(q, self.target)
        with self.assertRaises((TypeError, ValueError)):
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
        with self.assertRaisesRegex(ValueError, "sequence of length 3"):
            self.robot.set_base_frame(dynibo.Pose(translation=(0.0, 0.0)))
        with self.assertRaisesRegex(ValueError, "sequence of length 3"):
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
                    np.testing.assert_array_equal(
                        self.robot.jacobian(self.q, self.target), expected
                    )
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

    def test_numpy_fast_path_and_reusable_outputs(self) -> None:
        q = np.asarray(self.q, dtype=np.float64)
        qd = np.linspace(-0.2, 0.2, self.robot.joint_count, dtype=np.float64)

        jacobian = self.robot.jacobian(q, self.target)
        self.assertIsInstance(jacobian, np.ndarray)
        self.assertEqual(jacobian.dtype, np.float64)
        self.assertEqual(jacobian.shape, (6 * self.robot.generalized_count,))

        jacobian_out = np.empty_like(jacobian)
        returned = self.robot.jacobian(q, self.target, out=jacobian_out)
        self.assertIs(returned, jacobian_out)
        np.testing.assert_array_equal(returned, jacobian)

        mass_out = np.empty(self.robot.generalized_count**2, dtype=np.float64)
        returned_mass = self.robot.mass_matrix(q, out=mass_out)
        self.assertIs(returned_mass, mass_out)
        np.testing.assert_allclose(returned_mass, self.robot.mass_matrix(q))

        # A strided input remains supported through a temporary contiguous view.
        strided_q = np.repeat(q, 2)[::2]
        np.testing.assert_allclose(
            self.robot.velocity_product_forces(strided_q, qd),
            self.robot.velocity_product_forces(q, qd),
        )

        with self.assertRaisesRegex(ValueError, "out must contain exactly"):
            self.robot.gravity(q, out=np.empty(self.robot.generalized_count - 1))
        with self.assertRaises((TypeError, ValueError)):
            self.robot.gravity(q, out=np.empty(self.robot.generalized_count, dtype=np.float32))
        with self.assertRaises(ValueError):
            self.robot.gravity(q, out=q)


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


class ValueTypeTests(unittest.TestCase):
    def test_defaults_and_properties(self) -> None:
        self.assertEqual(dynibo.Pose().translation, (0.0, 0.0, 0.0))
        self.assertEqual(dynibo.Pose().rotation_xyzw, (0.0, 0.0, 0.0, 1.0))
        self.assertEqual(twist_values(dynibo.Twist()), (0.0,) * 6)
        base = dynibo.BaseState()
        self.assertEqual(base.frame, dynibo.Pose())
        self.assertEqual(base.velocity, dynibo.Twist())
        self.assertEqual(base.acceleration, dynibo.Twist())
        self.assertEqual(base, dynibo.BaseState(None, None, None))
        moving = motion_base()
        self.assertEqual(moving.frame.translation, reference("floating_base_translation"))
        self.assertEqual(moving.frame.rotation_xyzw, reference("floating_motion_rotation_xyzw"))
        self.assertEqual(twist_values(moving.velocity), reference("floating_base_velocity"))
        self.assertEqual(twist_values(moving.acceleration), reference("floating_base_acceleration"))
        self.assertNotEqual(moving, base)
        self.assertEqual(dynibo.BaseState(frame=moving.frame).velocity, dynibo.Twist())
        self.assertEqual(dynibo.BaseState(velocity=moving.velocity).frame, dynibo.Pose())

        load = dynibo.Load(3)
        self.assertEqual(load.link_id, 3)
        self.assertEqual(load.torque, (0.0,) * 3)
        self.assertEqual(load.force, (0.0,) * 3)
        load = dynibo.Load(2, torque=(0.1, 0.2, 0.3), force=(-1.0, 2.0, 4.0))
        self.assertEqual(load.link_id, 2)
        self.assertEqual(load.torque, (0.1, 0.2, 0.3))
        self.assertEqual(load.force, (-1.0, 2.0, 4.0))
        self.assertEqual(load, dynibo.Load(2, load.torque, load.force))

    def test_ik_options_defaults_and_custom_values(self) -> None:
        defaults = dict(
            max_iterations=100,
            translation_tolerance=1.0e-6,
            rotation_tolerance=1.0e-6,
            damping=1.0e-3,
            max_step_norm=0.5,
        )
        custom = dict(
            max_iterations=12,
            translation_tolerance=2.0e-7,
            rotation_tolerance=3.0e-7,
            damping=4.0e-4,
            max_step_norm=0.2,
        )
        for options, expected in (
            (dynibo.IkOptions(), defaults),
            (dynibo.IkOptions(**custom), custom),
        ):
            for name, value in expected.items():
                with self.subTest(name=name, value=value):
                    self.assertEqual(getattr(options, name), value)
        self.assertEqual(dynibo.IkOptions(), dynibo.IkOptions(**defaults))
        for invalid in (True, False, "10", 1.5):
            with self.subTest(invalid=invalid):
                with self.assertRaisesRegex(TypeError, "max_iterations must be an integer"):
                    dynibo.IkOptions(max_iterations=invalid)


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


class BindingContractTests(unittest.TestCase):
    def test_constructors_and_lifecycle(self) -> None:
        for robot_type in (dynibo.Robot, dynibo.FloatingRobot):
            with self.subTest(robot=robot_type.__name__):
                with robot_type(URDF) as direct, robot_type.from_urdf(URDF) as factory:
                    for name, expected in (
                        ("name", "test_arm"),
                        ("joint_count", 4),
                        ("link_count", 5),
                        ("generalized_count", 10 if robot_type is dynibo.FloatingRobot else 4),
                    ):
                        self.assertEqual(getattr(direct, name), expected)
                        self.assertEqual(getattr(factory, name), expected)
                    prefix = (dynibo.BaseState(),) if robot_type is dynibo.FloatingRobot else ()
                    target = direct.link_id("test_link_4")
                    args = prefix + (reference("q"), target)
                    self.assertEqual(
                        direct.forward_kinematics(*args), factory.forward_kinematics(*args)
                    )
                direct.close()
                for name in ("name", "joint_count", "generalized_count", "link_count"):
                    with self.assertRaisesRegex(RuntimeError, "robot is closed"):
                        getattr(direct, name)
                with self.assertRaisesRegex(RuntimeError, "robot is closed"):
                    direct.forward_kinematics(*args)
                with self.assertRaisesRegex(RuntimeError, "robot is closed"):
                    direct.__enter__()
                with self.assertRaisesRegex(RuntimeError, "context-body-error"):
                    with robot_type(URDF) as interrupted:
                        raise RuntimeError("context-body-error")
                with self.assertRaisesRegex(RuntimeError, "robot is closed"):
                    interrupted.link_id("test_link_4")
                for constructor in (robot_type, robot_type.from_urdf):
                    with self.assertRaises(dynibo.ModelError):
                        constructor(URDF.with_name("missing-model.urdf"))

    def test_array_inputs_and_output_reuse(self) -> None:
        for robot_type in (dynibo.Robot, dynibo.FloatingRobot):
            with robot_type(URDF) as robot:
                prefix = (motion_base(),) if robot_type is dynibo.FloatingRobot else ()
                target = robot.link_id("test_link_4")
                buffers = {}
                for shift in (0.0, 0.1):
                    values = np.asarray(reference("q")) + shift
                    for kind, q in (
                        ("list", values.tolist()),
                        ("contiguous", values),
                        ("strided", np.repeat(values, 2)[::2]),
                    ):
                        qd, qdd = np.asarray(reference("qd")), np.asarray(reference("qdd"))
                        forces = robot.inverse_dynamics(*prefix, q, qd, qdd)
                        calls = {
                            "jacobian": (q, target),
                            "jacobian_derivative": (q, qd, target),
                            "mass_matrix": (q,),
                            "gravity": (q,),
                            "velocity_product_forces": (q, qd),
                            "inverse_dynamics": (q, qd, qdd),
                            "forward_dynamics": (q, qd, forces),
                        }
                        if robot_type is dynibo.Robot:
                            calls["inverse_kinematics"] = (
                                q,
                                target,
                                robot.forward_kinematics(q, target),
                            )
                        for name, args in calls.items():
                            with self.subTest(
                                robot=robot_type.__name__, operation=name, shift=shift, input=kind
                            ):
                                method = getattr(robot, name)
                                expected = method(*prefix, *args)
                                canonical_args = (values.tolist(),) + args[1:]
                                np.testing.assert_allclose(
                                    expected,
                                    method(*prefix, *canonical_args),
                                    atol=1.0e-11,
                                    rtol=1.0e-10,
                                )
                                self.assertEqual(expected.dtype, np.float64)
                                out = buffers.setdefault(name, np.empty_like(expected))
                                out.fill(np.nan)
                                self.assertIs(method(*prefix, *args, out=out), out)
                                self.assertTrue(np.isfinite(out).all())
                                np.testing.assert_allclose(
                                    out, expected, atol=1.0e-11, rtol=1.0e-10
                                )

    def test_invalid_outputs_and_recovery(self) -> None:
        for robot_type in (dynibo.Robot, dynibo.FloatingRobot):
            with robot_type(URDF) as robot:
                prefix = (motion_base(),) if robot_type is dynibo.FloatingRobot else ()
                q = np.asarray(reference("q"))
                expected = robot.gravity(*prefix, q)
                n = robot.generalized_count
                readonly = np.empty(n)
                readonly.setflags(write=False)
                # Both the short q view and full out view refer to the same storage.
                storage = np.asarray(list(q) + [0.0] * (n - len(q)))
                cases = (
                    ("length", q, np.empty(n - 1)),
                    ("dtype", q, np.empty(n, dtype=np.float32)),
                    ("readonly", q, readonly),
                    ("strided", q, np.empty(2 * n)[::2]),
                    ("overlap", storage[: len(q)], storage),
                )
                for name, input_q, out in cases:
                    with self.subTest(robot=robot_type.__name__, case=name):
                        with self.assertRaises((TypeError, ValueError)):
                            robot.gravity(*prefix, input_q, out=out)
                        recovered = np.full(n, np.nan)
                        self.assertIs(robot.gravity(*prefix, q, out=recovered), recovered)
                        np.testing.assert_allclose(recovered, expected, atol=1.0e-11, rtol=1.0e-10)

    def test_invalid_arguments_and_recovery(self) -> None:
        q, qd, qdd = reference("q"), reference("qd"), reference("qdd")
        for robot_type in (dynibo.Robot, dynibo.FloatingRobot):
            with robot_type(URDF) as robot:
                prefix = (motion_base(),) if robot_type is dynibo.FloatingRobot else ()
                target = robot.link_id("test_link_4")
                expected = robot.mass_matrix(*prefix, q)
                forces = robot.inverse_dynamics(*prefix, q, qd, qdd)
                calls = (
                    ("jacobian_derivative", (q, qd[:-1], target)),
                    ("forward_velocity_kinematics", (q, qd[:-1], target)),
                    ("forward_acceleration_kinematics", (q, qd[:-1], qdd, target)),
                    ("forward_acceleration_kinematics", (q, qd, qdd[:-1], target)),
                    ("inverse_dynamics", (q, qd[:-1], qdd)),
                    ("inverse_dynamics", (q, qd, qdd[:-1])),
                    ("forward_dynamics", (q, qd[:-1], forces)),
                    ("forward_dynamics", (q, qd, forces[:-1])),
                    ("velocity_product_forces", (q, qd[:-1])),
                    ("mass_matrix", (q[:-1],)),
                    ("forward_kinematics", (q, robot.link_count)),
                    ("jacobian", (q, robot.link_count)),
                    ("jacobian_derivative", (q, qd, robot.link_count)),
                    ("forward_velocity_kinematics", (q, qd, robot.link_count)),
                    ("forward_acceleration_kinematics", (q, qd, qdd, robot.link_count)),
                )
                for name, args in calls:
                    with self.subTest(robot=robot_type.__name__, operation=name, args=args):
                        with self.assertRaises(ValueError):
                            getattr(robot, name)(*prefix, *args)
                        np.testing.assert_allclose(
                            robot.mass_matrix(*prefix, q), expected, atol=1.0e-11, rtol=1.0e-10
                        )
                with self.assertRaisesRegex(ValueError, "does not exist"):
                    robot.link_id("missing")
                with self.assertRaisesRegex(ValueError, "invalid link id"):
                    robot.gravity(*prefix, q, loads=[dynibo.Load(robot.link_count)])
                for invalid in (float("nan"), float("inf"), float("-inf")):
                    bad = (invalid,) + q[1:]
                    for name, args in (
                        ("mass_matrix", (bad,)),
                        ("velocity_product_forces", (q, bad)),
                        ("inverse_dynamics", (q, qd, bad)),
                    ):
                        with self.subTest(
                            robot=robot_type.__name__, operation=name, invalid=invalid
                        ):
                            with self.assertRaises(ValueError):
                                getattr(robot, name)(*prefix, *args)
                np.testing.assert_allclose(
                    robot.mass_matrix(*prefix, q), expected, atol=1.0e-11, rtol=1.0e-10
                )

    def test_nonfinite_loads_are_rejected_before_writing_output(self) -> None:
        q, qd, qdd = reference("q"), reference("qd"), reference("qdd")
        for robot_type in (dynibo.Robot, dynibo.FloatingRobot):
            with robot_type(URDF) as robot:
                prefix = (motion_base(),) if robot_type is dynibo.FloatingRobot else ()
                target = robot.link_id("test_link_4")
                valid_load = dynibo.Load(target, torque=(0.2, -0.1, 0.3), force=(-0.4, 0.6, 0.5))
                forces = robot.inverse_dynamics(*prefix, q, qd, qdd, loads=[valid_load])
                for name, args in (
                    ("gravity", (q,)),
                    ("inverse_dynamics", (q, qd, qdd)),
                    ("forward_dynamics", (q, qd, forces)),
                ):
                    method = getattr(robot, name)
                    expected = method(*prefix, *args, loads=[valid_load])
                    out = np.empty_like(expected)
                    for component in ("torque", "force"):
                        for axis in range(3):
                            for invalid in (float("nan"), float("inf"), float("-inf")):
                                values = [0.0, 0.0, 0.0]
                                values[axis] = invalid
                                load = dynibo.Load(target, **{component: values})
                                with self.subTest(
                                    robot=robot_type.__name__,
                                    operation=name,
                                    component=component,
                                    axis=axis,
                                    invalid=invalid,
                                ):
                                    out.fill(123.0)
                                    with self.assertRaisesRegex(
                                        ValueError, "load contains a non-finite value"
                                    ):
                                        method(*prefix, *args, loads=[valid_load, load], out=out)
                                    np.testing.assert_array_equal(out, np.full_like(out, 123.0))
                                    self.assertIs(
                                        method(*prefix, *args, loads=[valid_load], out=out), out
                                    )
                                    np.testing.assert_allclose(
                                        out, expected, atol=1.0e-11, rtol=1.0e-10
                                    )

    def test_invalid_frames_base_states_and_ik_options(self) -> None:
        q = reference("q")
        invalid_poses = (
            dynibo.Pose(rotation_xyzw=(0.0,) * 4),
            dynibo.Pose(rotation_xyzw=(float("inf"), 0.0, 0.0, 1.0)),
            dynibo.Pose(translation=(float("nan"), 0.0, 0.0)),
        )
        with dynibo.Robot(URDF) as fixed, dynibo.FloatingRobot(URDF) as floating:
            target = fixed.link_id("test_link_4")
            expected = fixed.forward_kinematics(q, target)
            for pose in invalid_poses:
                with self.assertRaisesRegex(ValueError, "pose contains"):
                    fixed.set_base_frame(pose)
                with self.assertRaisesRegex(ValueError, "pose contains"):
                    floating.forward_kinematics(dynibo.BaseState(frame=pose), q, target)
                for robot, prefix in ((fixed, ()), (floating, (motion_base(),))):
                    with self.assertRaisesRegex(ValueError, "pose contains"):
                        robot.forward_velocity_kinematics(
                            *prefix, q, reference("qd"), target, tool=pose
                        )
            for name in ("velocity", "acceleration"):
                for component in ("angular", "linear"):
                    twist = dynibo.Twist(**{component: (float("nan"), 0.0, 0.0)})
                    with self.subTest(state=name, component=component):
                        with self.assertRaises(ValueError):
                            floating.mass_matrix(dynibo.BaseState(**{name: twist}), q)
            for name in ("translation_tolerance", "rotation_tolerance", "damping", "max_step_norm"):
                for invalid in (-1.0, float("nan"), float("inf")):
                    with self.subTest(option=name, invalid=invalid):
                        options = dynibo.IkOptions(**{name: invalid})
                        with self.assertRaises(ValueError):
                            fixed.inverse_kinematics(q, target, expected, options)
            self.assertEqual(fixed.forward_kinematics(q, target), expected)
            np.testing.assert_allclose(
                fixed.inverse_kinematics(q, target, expected), q, atol=1.0e-12, rtol=0
            )


if __name__ == "__main__":
    unittest.main()
