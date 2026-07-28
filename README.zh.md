# dyno

[English](README.md) | 简体中文

`dyno` 是一个轻量、可靠、基于 Rust 的固定尺寸串联机器人运动学与动力学库。数值
计算基于 [`nalgebra`](https://nalgebra.rs/)，URDF 解析基于
[`urdf-rs`](https://github.com/openrr/urdf-rs)。

## 设计目标

- **轻量运行时：** 计算路径使用固定尺寸、基于栈的向量、矩阵和工作数组。FK、
  Jacobian、Jacobian 导数、重力及逆动力学计算期间不进行堆内存分配。
- **可靠行为：** 运行时库自身不包含 `unsafe` 代码；遇到无效模型会明确返回错误；
  解析运动学通过有限差分和数值回归用例共同验证。可选 Pinocchio benchmark 所需的
  C ABI 被隔离在 benchmark harness 内。
- **基于 Rust：** 使用 const generics 将关节数量编码进类型，并通过所有权和借用
  明确区分模型数据与计算输入。

URDF 解析和名称查找只在模型构建阶段分配内存，不进入实时计算路径。这里的“可靠”
是指经过测试的安全 Rust 实现，不代表已获得功能安全认证。

## 公共接口

### 核心类型

| 类型 | 用途 |
|---|---|
| `RobotArm<const N: usize>` | 固定自由度串联机器人模型和算法 |
| `RobotLink` | 关节变换、轴、限位、质量、质心和惯量 |
| `JointVector<N>` | 固定尺寸关节向量 |
| `Jacobian<N>` | 角运动分量在前的 `6 x N` 几何 Jacobian |
| `Frame` | 基于 `nalgebra::Isometry3<f64>` 的刚体变换 |
| `Motion` | 角运动分量在前的空间速度或加速度 |
| `Wrench` | 力矩分量在前的空间力 |

### 模型构建与访问

| 接口 | 结果 |
|---|---|
| `RobotArm::from_links(name, links)` | 从 `[RobotLink; N]` 构建模型 |
| `RobotArm::from_urdf_str(source)` | 解析 URDF 字符串 |
| `RobotArm::from_urdf_file(path)` | 解析 URDF 文件 |
| `name()`、`links()`、`link_mut()` | 查看或修改模型数据 |
| `replace_link(index, link)` | 替换连杆并刷新零位姿 |
| `home_end_frame()` | 返回关节零位时的末端位姿 |

### 运动学

| 接口 | 结果 |
|---|---|
| `forward_kinematics(q)` | 末端位姿 |
| `forward_kinematics_and_jacobian(q)` | 单次遍历同时得到末端位姿和 Jacobian |
| `jacobian(q)` | 基座坐标系几何 Jacobian |
| `jacobian_with_base(q, base)` | 旋转到指定基座坐标系的 Jacobian |
| `jacobian_with_tool(q, tool)` | 平移到工具点的 Jacobian |
| `forward_velocity_kinematics(q, qd, base, tool)` | 末端空间速度 |
| `jacobian_dot(q, qd)` | Jacobian 的解析时间导数 |
| `jacobian_dot_times_velocity(q, qd)` | 对流加速度 `J_dot * qd` |
| `forward_acceleration_kinematics(q, qd, qdd)` | 加速度 `J * qdd + J_dot * qd` |

### 动力学与关节工具

| 接口 | 结果 |
|---|---|
| `gravity_torque(q, base, end_load)` | 关节重力和基座 Wrench |
| `inverse_dynamics(...)` | Newton–Euler 递推得到的关节力和基座 Wrench |
| `joint_position_limits()` | 关节位置上下限向量 |
| `saturate_joint_position(lower, upper, q)` | 逐元素限制关节位置 |
| `PassiveJointMap` | 将主动坐标映射到全部关节，并把力映射回来 |
| `RobotWithPassiveJoints` | 被动关节运动学和动力学适配器 |

```rust
use dyno::{JointVector, RobotArm};

let arm = RobotArm::<4>::from_urdf_file("test_arm.urdf")?;
let q = JointVector::<4>::zeros();
let end = arm.forward_kinematics(&q);
let jacobian = arm.jacobian(&q);
# Ok::<(), dyno::Error>(())
```

## 兼容范围

当前 `RobotArm` 面向固定自由度串联机构。构建模型时会明确拒绝多分叉 URDF，而不会
静默地将其展平成串联链。支持多分叉机器人需要使用基于父节点索引的树形模型，适合
作为后续独立扩展。

兼容动力学内核有意保留已有的数值约定，包括正 Z 方向重力和既有的惯量积符号，使
C++ 数值回归结果可以复现。因此，下述重力和 RNEA benchmark 比较的是执行开销，
并不表示其数值结果与采用标准刚体动力学约定的 Pinocchio 完全等价。

## Pinocchio 性能基准

可选的 Criterion benchmark 分别在 `N=4` 和 `N=40` 下，使用两边完全相同的 URDF
和关节输入，对比 Dyno 与 Pinocchio 的正运动学、末端关节 Jacobian、重力和 RNEA。
模型构建及 URDF 解析均在计时区间之外；两边都会复用模型和计算工作区。此外还会
单独测量一次空操作，用来报告 Rust 到 C ABI 的固定调用开销。

以下冒烟结果使用 `--quick` 在 Intel Core i9-14900K 上测得，工具链为 rustc 1.97.1、
Pinocchio 3.9.0；数值越小越好。它们用于展示当前机器上的性能趋势，不应视为跨平台
或具有严格统计意义的性能结论。

| 操作 | 自由度 | Dyno | Pinocchio | Dyno 加速比 |
|---|---:|---:|---:|---:|
| 正运动学 | 4 | 66.6 ns | 81.7 ns | 1.23x |
| 末端 Jacobian | 4 | 80.9 ns | 137.6 ns | 1.70x |
| 重力 | 4 | 92.0 ns | 194.7 ns | 2.12x |
| RNEA | 4 | 148.0 ns | 311.5 ns | 2.11x |
| 正运动学 | 40 | 658.1 ns | 822.0 ns | 1.25x |
| 末端 Jacobian | 40 | 762.3 ns | 1.354 µs | 1.78x |
| 重力 | 40 | 951.3 ns | 1.835 µs | 1.93x |
| RNEA | 40 | 1.464 µs | 3.151 µs | 2.15x |

测得的 C ABI 空操作开销约为 0.704 ns。如前所述，由于兼容内核与 Pinocchio 使用
不同的数值约定，重力和 RNEA 两组数据只比较执行时间。

只有启用 `pinocchio-bench` feature 时才需要安装 Pinocchio。C++ 桥接、`cc`、
`pkg-config` 和 Criterion 都不会成为 Dyno 常规构建的运行时依赖。例如，在 x86-64
Linux 的 ROS Humble 环境中执行：

```bash
export PKG_CONFIG_PATH=/opt/ros/humble/lib/x86_64-linux-gnu/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/opt/ros/humble/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
cargo bench --features pinocchio-bench --bench pinocchio
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
