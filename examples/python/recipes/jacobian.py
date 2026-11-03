"""Jacobian and NumPy layout: run with an installed dynibo package; no model download needed."""

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
        target = robot.link_id("tool")
        q = np.array([0.3], dtype=np.float64)
        flat = robot.jacobian(q, target)
        J = flat.reshape((6, robot.generalized_count), order="F")
        np.testing.assert_allclose(J[:3, 0], (0.0, 1.0, 0.0), atol=1e-12)
        eps = 1e-6
        p_plus = np.array(robot.forward_kinematics(q + eps, target).translation)
        p_minus = np.array(robot.forward_kinematics(q - eps, target).translation)
        np.testing.assert_allclose(J[3:, 0], (p_plus - p_minus) / (2 * eps), atol=1e-9)
        assert np.shares_memory(J, flat)
        print("Jacobian shape:", J.shape)
