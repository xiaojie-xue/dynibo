from __future__ import annotations

import unittest
import dynibo
import numpy as np

from .support import URDF, reference, motion_base


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

    def test_nonfinite_forces_and_load_overflow_allow_recovery(self) -> None:
        for robot_type in (dynibo.Robot, dynibo.FloatingRobot):
            with robot_type(URDF) as robot:
                prefix = (motion_base(),) if robot_type is dynibo.FloatingRobot else ()
                q = np.zeros(robot.joint_count)
                out = np.full(robot.generalized_count, 123.0)
                for invalid in (float("nan"), float("inf"), float("-inf")):
                    with self.assertRaisesRegex(ValueError, "generalized forces"):
                        robot.forward_dynamics(*prefix, q, q,
                            np.full(robot.generalized_count, invalid), out=out)
                    np.testing.assert_array_equal(out, 123.0)
                huge = dynibo.Load(robot.link_id("test_link_4"), torque=(1.7e308, 0.0, 0.0))
                with self.assertRaisesRegex(dynibo.SolverError, "load aggregation"):
                    robot.gravity(*prefix, q, loads=[huge, huge], out=out)
                np.testing.assert_array_equal(out, 123.0)
                expected = robot.gravity(*prefix, q)
                robot.gravity(*prefix, q, out=out)
                np.testing.assert_array_equal(out, expected)

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
