# dyno

[English](README.md) | 简体中文

`dyno` 是一个轻量、可靠、基于 Rust 的树状机器人运动学与动力学库。它在运行时从
URDF 确定 link、关节数量和父子拓扑，并通过 slice 与显式 `Workspace` 提供一套统一的
动态尺寸计算 API。

数值计算基于 [`nalgebra`](https://nalgebra.rs/)，URDF 解析基于
[`urdf-rs`](https://github.com/openrr/urdf-rs)。

## 设计目标

- **运行时尺寸：** 同一个二进制可加载任意合法关节数的 URDF，无需提前枚举尺寸。
- **计算期零分配：** 创建 `Workspace` 和输出 buffer 后，运动学、动力学及 IK 调用不
  分配或调整容量。
- **可靠行为：** 核心库不包含项目自身的 `unsafe`；错误长度、错误模型的 Workspace、
  `LinkId` 和载荷都会明确返回错误。
- **FFI 友好：** 公共计算边界由 slice、调用方输出 buffer 和不透明 ID 构成，便于后续
  Python 与 C++ 绑定复用同一套算法。

## 公共类型

| 类型 | 用途 |
|---|---|
| `Robot` | 运行时拓扑的只读树模型 |
| `Workspace` | 与模型绑定、可重复使用的计算 scratch buffer |
| `LinkId` | 不透明、与模型绑定的 link 标识符 |
| `IndexedLoad` | 使用 `LinkId` 指定目标 link 的外载荷 |
| `InverseKinematicsOptions` | IK 的容差、阻尼、步长和迭代配置 |
| `Joint`、`JointType`、`Link` | URDF 模型信息 |
| `Frame` | `nalgebra::Isometry3<f64>` 刚体变换 |
| `Twist` | 角分量在前的空间速度或加速度 |
| `Wrench` | 力矩分量在前的空间力 |

## 基本使用

```rust
use dyno::{Frame, Robot};

let robot = Robot::from_urdf("robot.urdf")?;
let target = robot.link_id("tool")?;
let mut workspace = robot.workspace();

let q = vec![0.0; robot.joint_count()];
let mut jacobian = vec![0.0; 6 * robot.joint_count()];
let mut gravity = vec![0.0; robot.joint_count()];

let frame = robot.forward_kinematics(&q, target, &mut workspace)?;
robot.jacobian(&q, target, &mut workspace, &mut jacobian)?;
robot.gravity(
    &q,
    &Frame::identity(),
    &[],
    &mut workspace,
    &mut gravity,
)?;
# Ok::<(), dyno::Error>(())
```

Workspace 创建时一次性分配全部内部 buffer。实时循环中应复用它：

```rust
loop {
    robot.jacobian(&q, target, &mut workspace, &mut jacobian)?;
    // 使用 jacobian……
    # break;
}
# Ok::<(), dyno::Error>(())
```

并发计算应各自持有一个 Workspace；`Robot` 本身保持只读，可以共享。

## 计算接口

| 接口 | 结果 |
|---|---|
| `forward_kinematics(q, target, workspace)` | 指定 link 相对根坐标系的位姿 |
| `jacobian(q, target, workspace, output)` | 写入指定 link 的 `6 x N` Jacobian |
| `forward_velocity_kinematics(...)` | 指定 link/tool 的空间速度 |
| `forward_acceleration_kinematics(...)` | 指定 link 原点的空间加速度 |
| `gravity(q, base, loads, workspace, output)` | 写入重力及外载荷关节力 |
| `inverse_dynamics(...)` | 写入 Newton–Euler 逆动力学关节力 |
| `inverse_kinematics(..., options, workspace, output)` | 使用指定参数写入 IK 解 |

如果适用默认求解配置，显式传入 `InverseKinematicsOptions::default()` 即可。

所有关节输入和普通输出必须包含 `robot.joint_count()` 个元素。Jacobian 输出必须包含
`6 * robot.joint_count()` 个元素。长度不匹配会返回：

```rust
Error::WrongSliceLength {
    slice: "q",
    expected: robot.joint_count(),
    actual: q.len(),
}
```

接口不会自动 `resize`，也不会因为普通输入错误而退出进程。只有调用方主动对错误结果
使用 `unwrap()` 才会触发 panic。

## Jacobian 布局

Jacobian 是 column-major 的 `6 x N` 扁平数组。每个关节占连续 6 个元素：

```text
[angular_x, angular_y, angular_z, linear_x, linear_y, linear_z]
```

第 `joint` 列从 `jacobian[6 * joint]` 开始。该布局与 nalgebra 和 Eigen 默认列主序一致。

## LinkId、Workspace 与载荷归属

`LinkId` 和 `Workspace` 与产生它们的模型绑定：

```rust
let tool = robot.link_id("tool")?;
let mut workspace = robot.workspace();
```

将 Robot A 的 `LinkId`、Workspace 或 `IndexedLoad` 传给 Robot B 会分别返回
`Error::InvalidLinkId` 或 `Error::InvalidWorkspace`。`Robot::clone()` 表示同一模型，原有
ID 和 Workspace 对 clone 仍然有效。

`LinkId` 是进程内 handle，不承诺持久化、序列化或跨进程稳定性。

## 模型与动力学约定

`Robot` 支持任意分支数量和深度的合法树状 URDF。构建过程拒绝多根、重复名称、环、
断连、缺失 link 和一个 link 被多个 joint 重复连接的模型。支持 revolute、continuous、
prismatic 和 fixed joint。

关节 slice 当前按全部 URDF joint 排列，fixed joint 仍占一个元素，但不贡献运动或主动
关节力。根 link 保存在 `links()` 中，但兼容动力学不把根 link 自身惯性计入关节力。

兼容动力学保留已有正 Z 重力方向和惯量积符号约定。Pinocchio 桥接测试会转换这些约定后
逐元素比较结果。

IK 使用阻尼最小二乘
`J^T (J J^T + lambda^2 I)^-1`。迭代过程不主动施加关节限位，只在收敛后验证 URDF
限位。需要碰撞、冗余目标或其他约束时，应读取 Jacobian 并使用合适的优化器。

## 与 Pinocchio 的性能对比

以下数据在 Intel Core i9-14900K、rustc 1.97.1、Pinocchio 3.9.0 上使用当前动态
Workspace API 测得。Robot、Workspace、Pinocchio `Data` 和输出 buffer 都在计时区间外
创建并重复使用。Dyno 与 Pinocchio 使用相同 URDF 和关节输入。

数据来自 Criterion quick 模式报告区间的中值。Pinocchio 时间已扣除本次测得的
0.938 ns C ABI 固定开销；所有时间统一为 ns。

| 模型 | 操作 | Dyno 动态 API | Pinocchio | Dyno 加速比 |
|---|---|---:|---:|---:|
| 4 关节直链 | FK | 73.409 ns | 78.623 ns | 1.07x |
| 4 关节直链 | Jacobian | 84.467 ns | 129.422 ns | 1.53x |
| 4 关节直链 | Gravity | 119.120 ns | 187.782 ns | 1.58x |
| 4 关节直链 | RNEA | 181.740 ns | 304.192 ns | 1.67x |
| 40 关节直链 | FK | 730.120 ns | 810.432 ns | 1.11x |
| 40 关节直链 | Jacobian | 847.920 ns | 1327.462 ns | 1.57x |
| 40 关节直链 | Gravity | 1130.600 ns | 1830.562 ns | 1.62x |
| 40 关节直链 | RNEA | 1632.500 ns | 3147.862 ns | 1.93x |
| 7 关节双叶树 | FK | 112.200 ns | 138.442 ns | 1.23x |
| 7 关节双叶树 | Jacobian | 123.990 ns | 213.612 ns | 1.72x |
| 7 关节双叶树 | Gravity | 170.280 ns | 326.032 ns | 1.91x |
| 7 关节双叶树 | RNEA | 284.160 ns | 539.602 ns | 1.90x |

执行命令：

```bash
export PKG_CONFIG_PATH=/opt/ros/humble/lib/x86_64-linux-gnu/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/opt/ros/humble/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
cargo bench --features pinocchio-bench --bench pinocchio -- --quick
```

Quick 模式样本较少，以上数据用于展示当前机器上的性能趋势，不是跨平台性能承诺。
Pinocchio 对照测试还会在 32 组确定性状态下逐元素验证 FK、Jacobian、gravity 和 RNEA，
性能 benchmark 本身只测执行时间。

## 示例与验证

运行 Franka 示例：

```bash
cargo run --example franka
```

完整验证：

```bash
cargo fmt --all -- --check
cargo clippy --all-targets -- -D warnings
cargo test --all-targets
cargo bench --features core-bench --bench core
```

测试覆盖有限差分 Jacobian 与加速度、数值动力学回归、树模型多分支载荷、Workspace
残留、模型归属、错误长度、IK 以及计算期零分配。安装 Pinocchio 后还可运行逐元素交叉
验证。
