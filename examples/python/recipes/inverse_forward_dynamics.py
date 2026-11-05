"""Inverse and forward dynamics (RNEA / ABA): run with an installed dynibo package; no model download needed."""

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
        qd = np.array([0.2], dtype=np.float64)
        qdd = np.array([-0.4], dtype=np.float64)
        tau = robot.inverse_dynamics(q, qd, qdd)
        recovered = robot.forward_dynamics(q, qd, tau)
        np.testing.assert_allclose(recovered, qdd, atol=1e-12)
        M = robot.mass_matrix(q).reshape((robot.generalized_count,) * 2, order="F")
        bias = robot.velocity_product_forces(q, qd) + robot.gravity(q)
        np.testing.assert_allclose(M @ qdd + bias, tau, atol=1e-12)
        np.testing.assert_allclose(M, [[0.35]], atol=1e-12)
        print("RNEA torque:", tau, "ABA acceleration:", recovered)
