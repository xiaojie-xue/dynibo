"""Reusing output buffers: run with an installed dynibo package; no model download needed."""

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
        q = np.zeros(robot.joint_count, dtype=np.float64)
        J_flat = np.empty(6 * robot.generalized_count, dtype=np.float64)
        J = J_flat.reshape((6, robot.generalized_count), order="F")
        tau = np.empty(robot.generalized_count, dtype=np.float64)
        loads = robot.load_buffer()
        loads.set(target, force=(0.0, 0.0, -1.0))
        for angle in (0.0, 0.1, 0.2):
            q[0] = angle
            assert robot.jacobian(q, target, out=J_flat) is J_flat
            assert robot.gravity(q, loads=loads, out=tau) is tau
            np.testing.assert_allclose(J[:3, 0], (0.0, 1.0, 0.0), atol=1e-12)
            # The resisting force rotates with the tool, giving a constant +1 N m.
            np.testing.assert_allclose(
                tau, [-0.5 * 9.80665 * np.cos(angle) + 1.0], atol=1e-12
            )
        assert np.shares_memory(J, J_flat)
        print("reused Jacobian:", J.shape, "reused effort:", tau)
