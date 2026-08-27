# Rust 指南

Rust crate 是 dynibo 的原生接口，提供强类型 link ID、调用方持有的输出 buffer，以及
详细的 `Error` variant。

```bash
cargo add dynibo
```

## 典型设置

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

## 内存分配与并发

重复计算矩阵或广义力时，应预先创建并复用输出 buffer。每个并发计算需要独立的
`Robot::fork()` 实例。`LinkId` 同样属于模型，不能与独立加载的其他模型混用。

## 类型与错误

`Frame` 是 nalgebra 的三维 isometry。`Twist` 和 `Wrench` 提供有名称的
angular/linear 与 torque/force 分量。可能失败的调用返回 `dynibo::Result<T>`；
`Error::category()` 提供稳定的粗粒度类别，具体 error variant 则保留详细信息。

[打开 Rust API 参考](https://docs.rs/dynibo){ .md-button }
