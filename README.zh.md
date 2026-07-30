# dyno

[English](README.md) | 简体中文

`dyno` 是一个轻量、可靠、基于 Rust 的串联机器人运动学与动力学库。模型构建时自动
确定关节数量，计算输入和输出仍保持固定尺寸。数值计算基于
[`nalgebra`](https://nalgebra.rs/)，URDF 解析基于
[`urdf-rs`](https://github.com/openrr/urdf-rs)。

## 设计目标

- **轻量运行时：** 计算路径使用固定尺寸、基于栈的向量、矩阵和工作数组。运动学、
  重力及逆动力学计算期间不进行堆内存分配。
- **可靠行为：** 运行时库自身不包含 `unsafe` 代码；遇到无效模型会明确返回错误；
  解析运动学通过有限差分和数值回归用例共同验证。可选 Pinocchio benchmark 所需的
  C ABI 被隔离在 benchmark harness 内。
- **基于 Rust：** 使用 const generics 保持计算输入和输出为固定尺寸，模型的关节
  数量则在构造时自动确定。

URDF 解析和名称查找只在模型构建阶段分配内存，不进入实时计算路径。这里的“可靠”
是指经过测试的安全 Rust 实现，不代表已获得功能安全认证。

## 公共接口

### 核心类型

| 类型 | 用途 |
|---|---|
| `RobotArm` | 运行时确定自由度、使用固定尺寸计算接口的串联模型 |
| `RobotJoint` | 关节变换、轴、限位和关节状态 |
| `RobotLink` | Link 的质量、质心和惯量 |
| `JointVector<N>` | 固定尺寸关节向量 |
| `Jacobian<N>` | 角运动分量在前的 `6 x N` 几何 Jacobian |
| `Frame` | 基于 `nalgebra::Isometry3<f64>` 的刚体变换 |
| `Motion` | 角运动分量在前的空间速度或加速度 |
| `Wrench` | 力矩分量在前的空间力 |

### 模型构建与访问

| 接口 | 结果 |
|---|---|
| `RobotArm::from_urdf(path)` | 从 URDF 文件路径构建模型 |
| `name()`、`joints()`、`links()` | 查看模型数据，`links()` 包含 root link |
| `link_count()` | 返回包含 root link 的 URDF link 数量 |
| `joint_count()` | 返回从模型中解析出的关节数量 |

### 计算接口

为保持库的定位专注、轻量，公开计算接口仅限下列操作。两个逆运动学接口用于明确后续
范围，目前尚未实现。

| 接口 | 状态与结果 |
|---|---|
| `forward_kinematics(q)` | 末端位姿 |
| `jacobian(q)` | 基座坐标系几何 Jacobian |
| `inverse_kinematics(...)` | 规划中，尚未实现 |
| `inverse_kinematics_with_boundary(...)` | 规划中，尚未实现 |
| `forward_velocity_kinematics(q, qd, base, tool)` | 末端空间速度 |
| `forward_acceleration_kinematics(q, qd, qdd)` | 直接递推加速度 `J * qdd + J_dot * qd` |
| `gravity(q, base, end_load)` | 关节重力和基座 Wrench |
| `inverse_dynamics(...)` | Newton–Euler 递推得到的关节力和基座 Wrench |

```rust
use dyno::{JointVector, RobotArm};

let arm = RobotArm::from_urdf("test_arm.urdf")?;
let q = JointVector::<4>::zeros();
let end = arm.forward_kinematics(&q)?;
let jacobian = arm.jacobian(&q)?;
# Ok::<(), dyno::Error>(())
```

计算尺寸 `N` 会从每次传入的 `JointVector<N>` 自动推导，不再属于 `RobotArm` 类型的
一部分。若模型与输入尺寸不一致，会在开始计算前返回 `Error::WrongJointCount`。

## 兼容范围

当前 `RobotArm` 面向串联机构。构建模型时会明确拒绝多分叉 URDF，而不会静默地将其
展平成串联链。支持多分叉机器人需要使用基于父节点索引的树形模型，适合作为后续
独立扩展。

兼容动力学内核有意保留已有的数值约定，包括正 Z 方向重力和既有的惯量积符号，使
C++ 数值回归结果可以复现。因此，下述重力和 RNEA benchmark 比较的是执行开销，
并不表示其数值结果与采用标准刚体动力学约定的 Pinocchio 完全等价。

## Pinocchio 性能基准

可选的 Criterion benchmark 分别在 `N=4` 和 `N=40` 下，使用两边完全相同的 URDF
和关节输入，对比 Dyno 与 Pinocchio 的正运动学、末端关节 Jacobian、重力和 RNEA。
模型构建及 URDF 解析均在计时区间之外；两边都会复用已解析的模型，Pinocchio 还会
复用其 `Data` 对象。另行测得的空操作用于修正 Rust 到 C ABI 的固定调用开销。

以下冒烟结果使用 `--quick` 在 Intel Core i9-14900K 上测得，工具链为 rustc 1.97.1、
Pinocchio 3.9.0；数值越小越好。它们用于展示当前机器上的性能趋势，不应视为跨平台
或具有严格统计意义的性能结论。

| 操作 | 自由度 | Dyno | Pinocchio | Dyno 加速比 |
|---|---:|---:|---:|---:|
| 正运动学 | 4 | 65.5 ns | 79.0 ns | 1.21x |
| 末端 Jacobian | 4 | 81.4 ns | 135.6 ns | 1.67x |
| 重力 | 4 | 91.5 ns | 187.6 ns | 2.05x |
| RNEA | 4 | 148.4 ns | 298.8 ns | 2.01x |
| 正运动学 | 40 | 646.4 ns | 819.2 ns | 1.27x |
| 末端 Jacobian | 40 | 786.1 ns | 1.351 µs | 1.72x |
| 重力 | 40 | 950.0 ns | 1.850 µs | 1.95x |
| RNEA | 40 | 1.462 µs | 3.209 µs | 2.19x |

上述 Pinocchio 数据已考虑并扣除测得的 C ABI 开销。如前所述，由于兼容内核与
Pinocchio 使用不同的数值约定，重力和 RNEA 两组数据只比较执行时间。

只有启用 `pinocchio-bench` feature 时才需要安装 Pinocchio。C++ 桥接、`cc`、
`pkg-config` 和 Criterion 都不会成为 Dyno 常规构建的运行时依赖。例如，在 x86-64
Linux 的 ROS Humble 环境中执行：

```bash
export PKG_CONFIG_PATH=/opt/ros/humble/lib/x86_64-linux-gnu/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/opt/ros/humble/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
cargo bench --features pinocchio-bench --bench pinocchio
```

仅测试 Dyno 的基准不依赖 Pinocchio：

```bash
cargo bench --features core-bench --bench core
```

其他 ROS 发行版或 CPU 架构需要相应调整路径。可添加 `-- --quick` 做快速冒烟验证；
需要正式比较时应省略该参数。

## 验证

```text
cargo fmt --all -- --check
cargo clippy --all-targets -- -D warnings
cargo test --all-targets
# 已安装 Pinocchio 时：
cargo clippy --features pinocchio-bench --bench pinocchio -- -D warnings
```

集成测试覆盖通用四轴测试 URDF、Jacobian 导数、加速度、逆动力学参考值、
Jacobian 及其导数的有限差分验证、旋转与移动关节、重力、关节限位和被动关节。
