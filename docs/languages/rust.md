# Rust Guide

The Rust crate is dynibo's native interface. It exposes strongly typed link IDs,
explicit workspaces, caller-owned output buffers, and detailed `Error` variants.

```bash
cargo add dynibo
```

## Typical setup

```rust
use dynibo::Robot;

fn main() -> dynibo::Result<()> {
    let robot = Robot::from_urdf("robot.urdf")?;
    let target = robot.link_id("tool")?;
    let mut workspace = robot.workspace();
    let q = vec![0.0; robot.joint_count()];

    let pose = robot.forward_kinematics(&q, target, &mut workspace)?;
    println!("{}", pose.translation.vector.transpose());
    Ok(())
}
```

## Allocation and concurrency

Create output buffers once when repeatedly calculating matrices or generalized
forces. A `Workspace` is model-scoped and mutable; use one per concurrent
calculation. A `LinkId` is also model-scoped and cannot be mixed with an
independently loaded model.

## Types and errors

`Frame` is nalgebra's three-dimensional isometry. `Twist` and `Wrench` expose
named angular/linear and torque/force components. Fallible calls return
`dynibo::Result<T>`; `Error::category()` provides a stable coarse category while
the error variant preserves detail.

[Open the Rust API reference](https://docs.rs/dynibo){ .md-button }
