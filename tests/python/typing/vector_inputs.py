"""Static and runtime regression cases for NumPy spatial-vector arguments."""

import numpy as np

from dynibo import Load, LoadBuffer, Pose, Twist


def check_vector_inputs(loads: LoadBuffer, link_id: int) -> None:
    """Accept floating/integer arrays, strided views, lists, and tuples."""
    for vector in (
        np.array([1.0, 2.0, 3.0], dtype=np.float64),
        np.array([1.0, 2.0, 3.0], dtype=np.float32),
        np.array([1, 2, 3], dtype=np.int64),
        np.arange(1.0, 7.0)[::2],
        [1.0, 2.0, 3.0],
        (1.0, 2.0, 3.0),
    ):
        pose = Pose(translation=vector)
        twist = Twist(angular=vector, linear=vector)
        load = Load(link_id, torque=vector, force=vector)
        np.testing.assert_array_equal(pose.translation, vector)
        np.testing.assert_array_equal(twist.angular, vector)
        np.testing.assert_array_equal(twist.linear, vector)
        np.testing.assert_array_equal(load.torque, vector)
        np.testing.assert_array_equal(load.force, vector)
        loads.set(link_id, torque=vector, force=vector)
        loads.add(link_id, torque=vector, force=vector)
        assert len(loads) == 1

    for quaternion in (
        np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64),
        np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32),
        np.array([0, 0, 0, 1], dtype=np.int64),
        [0.0, 0.0, 0.0, 1.0],
        (0.0, 0.0, 0.0, 1.0),
    ):
        pose = Pose(rotation_xyzw=quaternion)
        np.testing.assert_array_equal(pose.rotation_xyzw, quaternion)
    loads.clear()
