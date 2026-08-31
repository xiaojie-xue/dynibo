# Robot model licenses

These third-party robot descriptions retain their own licenses. Dynibo's MIT
license does not replace them.

- `franka/franka_fer.urdf`: dynamics-only derivative of the FER model from
  [Franka Robotics' franka_description](https://github.com/frankarobotics/franka_description),
  licensed under Apache-2.0. The complete upstream license, including its bundled
  BSD notice, is retained in [LICENSE](franka/LICENSE).
  As recorded in the URDF header, visual, collision, transmission and
  `ros2_control` elements were omitted. The URDF and its dynamics parameters
  have not been changed by this license update.
- `unitree-g1/g1_29dof_mode_11.urdf`: copied unchanged from
  `visibo/example/unitree-g1`, with the original Unitree Robotics BSD-3-Clause
  [LICENSE](unitree-g1/LICENSE). Mesh assets are not distributed here; the URDF
  still contains its original mesh references. Dynibo only loads the model's
  kinematic and inertial data and does not need those assets.

G1 has 29 actuated joints. Load it with `FloatingRobot` to add six base velocity
coordinates (35 generalized velocity/force entries); no floating joint needs to be added
to the URDF. Franka uses `Robot` with a fixed base and seven joint coordinates.
