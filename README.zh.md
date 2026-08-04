# dyno

[![Package CI](https://github.com/xiaojie-xue/dyno/actions/workflows/package-ci.yml/badge.svg?branch=main)](https://github.com/xiaojie-xue/dyno/actions/workflows/package-ci.yml)
[![codecov](https://codecov.io/gh/xiaojie-xue/dyno/branch/main/graph/badge.svg)](https://codecov.io/gh/xiaojie-xue/dyno)

[English](README.md) | 简体中文

[Rust、Python 与 C/C++ 安装和发布指南](PACKAGING.zh.md)

`dyno` 是一个轻量、可靠、基于 Rust 的树状机器人运动学与动力学库。它在运行时从
URDF 确定 link、关节数量和父子拓扑，并通过 slice 与显式 `Workspace` 提供一套统一的
计算接口。

数值计算基于 [`nalgebra`](https://nalgebra.rs/)，URDF 解析基于
[`urdf-rs`](https://github.com/openrr/urdf-rs)。

## 设计目标

- **运行时尺寸：** 同一个二进制可加载任意合法关节数的 URDF，无需提前枚举尺寸。
- **计算期零分配：** 创建 `Workspace` 和输出 buffer 后，运动学、动力学及 IK 调用不
  分配或调整容量。
- **可靠行为：** 核心库不包含项目自身的 `unsafe`；错误长度、错误模型的 Workspace、
  `LinkId` 和载荷都会明确返回错误。
- **多语言接口：** 稳定 C ABI、C++17 RAII wrapper 与 Python package 已复用同一套
  Rust 算法实现。

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

Rust 用户可执行 `cargo add dyno`；Python 用户安装 `dyno-robotics`；C/C++ 用户使用仓库
提供的 CMake package。完整安装、调用和发布命令见[多语言打包指南](PACKAGING.zh.md)。

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

以下数据是 Dyno 与 Pinocchio 在 Intel Core i9-14900K、rustc 1.97.1、
Pinocchio 3.9.0 上的对比结果。Robot、Workspace、Pinocchio `Data` 和输出 buffer 都在
计时区间外创建并重复使用。Dyno 与 Pinocchio 使用相同 URDF 和关节输入。

数据来自 Criterion quick 模式报告区间的中值。Pinocchio 时间已扣除本次测得的
0.882 ns C ABI 固定开销；所有时间统一为 ns。

| 模型 | 操作 | Dyno | Pinocchio | Dyno 加速比 |
|---|---|---:|---:|---:|
| 4 关节直链 | FK | 66.414 ns | 77.627 ns | 1.17x |
| 4 关节直链 | Jacobian | 81.475 ns | 128.778 ns | 1.58x |
| 4 关节直链 | Gravity | 117.420 ns | 187.158 ns | 1.59x |
| 4 关节直链 | RNEA | 181.030 ns | 300.268 ns | 1.66x |
| 40 关节直链 | FK | 669.530 ns | 833.328 ns | 1.24x |
| 40 关节直链 | Jacobian | 784.060 ns | 1347.518 ns | 1.72x |
| 40 关节直链 | Gravity | 1120.500 ns | 1824.718 ns | 1.63x |
| 40 关节直链 | RNEA | 1643.800 ns | 3133.918 ns | 1.91x |
| 7 关节双叶树 | FK | 66.105 ns | 137.298 ns | 2.08x |
| 7 关节双叶树 | Jacobian | 80.325 ns | 217.188 ns | 2.70x |
| 7 关节双叶树 | Gravity | 170.080 ns | 322.128 ns | 1.89x |
| 7 关节双叶树 | RNEA | 284.220 ns | 543.928 ns | 1.91x |

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
cargo test --workspace --all-targets
cargo +nightly llvm-cov --branch --workspace --all-targets
cargo bench --features core-bench --bench core
```

本地单元测试只运行 Rust workspace 源码。GitHub Package CI 才会分别测试解包后的 Rust
`.crate`、已安装的 Python package 与解包后的 C/C++ CPack 包；具体行为见
[多语言打包指南](PACKAGING.zh.md#发布前检查)。Rust 测试覆盖有限差分 Jacobian 与加速度、
数值动力学回归、树模型多分支载荷、Workspace 残留、模型归属、错误长度、IK 以及计算期
零分配。
覆盖率 CI 会生成 LLVM JSON 报告，并要求 Rust workspace 的行覆盖率不低于 85%、
分支覆盖率不低于 75%。
