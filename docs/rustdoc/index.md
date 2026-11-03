# Robot kinematics and rigid-body dynamics

Dynibo loads tree-structured URDF robots and computes forward kinematics,
Jacobians, inverse kinematics, mass matrices, gravity compensation,
recursive Newton–Euler inverse dynamics (RNEA), and articulated-body forward
dynamics (ABA). It provides a native Rust API and Python, C, and C++ bindings.
The minimum supported Rust version is **1.88**, checked in CI.

[`Robot`] owns fixed-base calculation storage. [`FloatingRobot`] adds six
generalized base degrees of freedom and receives an explicit [`BaseState`] on
each calculation. [`RobotModel`] shares immutable model data; use `fork()` to
create independent calculation instances for parallel work.

## Quick start

```bash
cargo add dynibo
```

This complete example creates its own URDF and verifies a one-metre tool offset.
Save it as `src/main.rs` in a Cargo binary project. It needs no ROS, meshes, or
additional dependencies.

```rust
use dynibo::Robot;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let urdf = r#"<robot name="arm">
      <link name="base"/>
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
    </robot>"#;
    let path = std::env::temp_dir()
        .join(format!("dynibo-quick-start-{}.urdf", std::process::id()));
    std::fs::write(&path, urdf)?;
    let loaded = Robot::from_urdf(&path);
    std::fs::remove_file(&path)?;
    let mut robot = loaded?;
    let tool = robot.link_id("tool")?;
    let q = [0.3_f64];
    let pose = robot.forward_kinematics(&q, tool)?;
    assert!((pose.translation.x - q[0].cos()).abs() < 1.0e-12);
    assert!((pose.translation.z + q[0].sin()).abs() < 1.0e-12);

    let mut jacobian = vec![0.0; 6 * robot.generalized_count()];
    robot.jacobian(&q, tool, &mut jacobian)?;
    let mut gravity = vec![0.0; robot.generalized_count()];
    robot.gravity(&q, &[], &mut gravity)?;
    println!("translation: {:?}; holding torque: {:?}", pose.translation, gravity);
    Ok(())
}
```

## Choose an operation

| Task | Fixed-base method | Output |
|---|---|---|
| Tool world pose | [`Robot::forward_kinematics`] | [`Frame`] |
| All link poses | [`Robot::forward_kinematics_all`] | One frame per link |
| Geometric Jacobian / derivative | [`Robot::jacobian`] / [`Robot::jacobian_derivative`] | `6 × G`, column-major |
| Link velocity / acceleration | [`Robot::forward_velocity_kinematics`] / [`Robot::forward_acceleration_kinematics`] | [`Twist`] |
| Target-pose IK | [`Robot::inverse_kinematics`] | `J` joint positions |
| Inertia matrix | [`Robot::mass_matrix`] | `G × G`, column-major |
| Coriolis / centrifugal effort | [`Robot::velocity_product_forces`] | `G` generalized entries |
| Gravity compensation | [`Robot::gravity`] | `G` generalized entries |
| RNEA inverse dynamics | [`Robot::inverse_dynamics`] | `G` generalized forces |
| ABA forward dynamics | [`Robot::forward_dynamics`] | `G` generalized accelerations |

[`FloatingRobot`] provides the same operations except inverse kinematics, with
`base: &BaseState` before joint inputs. Its root must have strictly positive
mass; ABA also requires solvable articulated inertia. Fixed-base robots may
have a massless URDF root.

## Array, frame, and load conventions

- `J = joint_count()`. q, qd, and qdd always contain `J` non-fixed scalar joint
  entries in breadth-first URDF order. Units are SI.
- `G = generalized_count()`: `J` for a fixed base and `J + 6` for a floating base.
  Floating generalized vectors begin with world-frame base angular then linear
  entries, followed by joint entries. Base coordinates are never prepended to q.
- Flat matrices are column-major: `values[column * rows + row]`. Jacobians have
  angular rows first, then linear rows at the target-link origin, in world axes.
- [`Frame`] maps local points to world. [`Twist`] uses angular then linear
  components; kinematic acceleration has classical linear acceleration.
- [`IndexedLoad`] contains a **resisting** [`Wrench`] in link-local axes at the
  link origin. It adds to required inverse-dynamics effort; negate a physical
  applied wrench when constructing it.
- Create caller-owned output buffers once. Each robot reuses its internal
  storage, and the main native calculation routines do not allocate in the
  calculation loop. [`LoadBuffer`] similarly reuses native load storage.
- Link identifiers and load buffers are model-scoped; they can be shared by
  forks, but not by independently loaded models.
- Fallible operations return [`Result`]. Inspect [`Error`] for the exact cause
  and [`Error::category`] for a coarse classification.

## Guides and executable tasks

- [Task recipes](https://dynibo.readthedocs.io/en/latest/recipes/)
- [Choosing Dynibo and migrating from Pinocchio](https://dynibo.readthedocs.io/en/latest/getting-started/choosing-dynibo/)
- [Rust example covering six tasks](https://github.com/xiaojie-xue/dynibo/blob/main/examples/rust/recipes.rs): `cargo run --example recipes`
- [Installation and supported toolchains](https://dynibo.readthedocs.io/en/latest/getting-started/installation/)
- [Python API reference](https://dynibo.readthedocs.io/en/latest/reference/python/)
