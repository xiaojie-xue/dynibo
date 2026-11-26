# Rust Guide

The Rust crate is dynibo's native interface. It exposes strongly typed link IDs,
instance-local workspaces, caller-owned output buffers, and detailed `Error` variants.

```bash
cargo add dynibo
```

## Typical setup

```rust
use dynibo::Robot;

fn main() -> dynibo::Result<()> {
    let mut robot = Robot::from_urdf("robot.urdf")?;
    let target = robot.link_id("tool")?;
    let q = vec![0.0; robot.joint_count()];

    let pose = robot.forward_kinematics(&q, target)?;
    println!("{}", pose.translation.vector.transpose());
    Ok(())
}
```

## Allocation, snapshots, and concurrency

Create output buffers once when repeatedly calculating matrices or generalized
forces. When querying multiple kinematics and dynamics quantities at the same
`(q, qd)` state in a control loop, use `robot.state(&q, &qd)` (or
`floating_robot.state(&base, &q, &qd)`) to compute joint transforms, poses, and
link velocities once and reuse them across `forward_kinematics`, `jacobian`,
`mass_matrix`, `inverse_dynamics`, and `forward_dynamics` without allocating.

Use `Robot::fork()` to create independent instances for concurrent threads.
`set_gravity(...)` configures per-instance gravitational acceleration (default
`[0.0, 0.0, -9.80665]` m/s²) and is inherited by `fork()`. A `LinkId` is
model-scoped and cannot be mixed with an independently loaded model.

## Types and errors

`Frame` is nalgebra's three-dimensional isometry (re-exported via
`dynibo::nalgebra`). `Twist` and `Wrench` expose named angular/linear and
torque/force components with arithmetic operators. Fallible calls return
`dynibo::Result<T>`; `Error::category()` provides a stable coarse category while
the error variant preserves detail.

[Open the Rust API reference](https://docs.rs/dynibo){ .md-button }
