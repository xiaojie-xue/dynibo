"""Floating-base dynamics: run with an installed dynibo package; no model download needed."""

from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np

from dynibo import BaseState, FloatingRobot, Pose, Twist

URDF = """<robot name="pendulum">
  <link name="base">
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
  <link name="arm">
    <inertial>
      <origin xyz="0.5 0 0"/>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
  <joint name="hinge" type="revolute">
    <parent link="base"/><child link="arm"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="10"/>
  </joint>
  <link name="tool"/>
  <joint name="tip" type="fixed">
    <parent link="arm"/><child link="tool"/>
    <origin xyz="1 0 0"/>
  </joint>
</robot>"""

with TemporaryDirectory() as directory:
    path = Path(directory) / "pendulum.urdf"
    path.write_text(URDF, encoding="utf-8")
    with FloatingRobot.from_urdf(path) as robot:
        base = BaseState(
            frame=Pose(translation=(0.2, 0.0, 1.0)),
            velocity=Twist(angular=(0.0, 0.1, 0.0), linear=(0.1, 0.0, 0.0)),
            acceleration=Twist(angular=(0.0, 0.02, 0.0), linear=(0.0, 0.0, 0.03)),
        )
        q = np.array([0.3], dtype=np.float64)
        qd = np.array([0.2], dtype=np.float64)
        qdd = np.array([-0.4], dtype=np.float64)
        assert robot.generalized_count == robot.joint_count + 6
        forces = robot.inverse_dynamics(base, q, qd, qdd)
        acceleration = robot.forward_dynamics(base, q, qd, forces)
        expected = np.r_[base.acceleration.angular, base.acceleration.linear, qdd]
        np.testing.assert_allclose(acceleration, expected, atol=1e-11)
        # An unactuated base supplies no root wrench. This is free flight, without contacts.
        forces[:6] = 0.0
        free = robot.forward_dynamics(base, q, qd, forces)
        assert free.shape == (robot.generalized_count,)
        assert np.isfinite(free).all()
        print("base angular / linear, then joint acceleration:", free)
