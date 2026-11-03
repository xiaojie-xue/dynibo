"""Gravity compensation: run with an installed dynibo package; no model download needed."""

from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np

from dynibo import Robot

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
    with Robot.from_urdf(path) as robot:
        q = np.array([0.3], dtype=np.float64)
        zero = np.zeros(robot.joint_count)
        tau = robot.gravity(q)
        # One 1 kg arm with its centre of mass 0.5 m along local x.
        np.testing.assert_allclose(tau, [-0.5 * 9.80665 * np.cos(q[0])], atol=1e-12)
        np.testing.assert_allclose(
            tau, robot.inverse_dynamics(q, zero, zero), atol=1e-12
        )
        np.testing.assert_allclose(
            robot.forward_dynamics(q, zero, tau), zero, atol=1e-12
        )
        print("holding torque (N m):", tau)
