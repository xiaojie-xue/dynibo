from __future__ import annotations

import unittest
import dynibo

from .support import reference, motion_base, twist_values


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
