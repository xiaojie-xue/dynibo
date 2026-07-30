# dyno

[English](README.md) | 简体中文

`dyno` 是一个轻量、可靠、基于 Rust 的树状机器人运动学与动力学库。在当前支持的
关节类型范围内，可以加载任意分支数量和深度的合法树状 URDF。模型构建时自动确定
link、关节和父子拓扑，计算输入和输出仍保持固定尺寸。数值计算基于
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

URDF 解析和拓扑构建只在模型构建阶段分配内存；运动学和动力学计算路径不进行堆内存
分配。这里的“可靠”是指经过测试的安全 Rust 实现，不代表已获得功能安全认证。

## 公共接口

### 核心类型

| 类型 | 用途 |
|---|---|
| `Robot` | 运行时确定拓扑、使用固定尺寸计算接口的树模型 |
| `Joint` | 关节变换、轴、限位和关节状态 |
| `JointType` | Revolute、prismatic 或 fixed 关节运动类型 |
| `Link` | Link 的质量、质心和惯量 |
| `Load` | 施加在指定 link 原点、以该 link 坐标系表达的 Wrench |
| `JointVector<N>` | 固定尺寸关节向量 |
| `Jacobian<N>` | 角运动分量在前的 `6 x N` 几何 Jacobian |
| `Frame` | 基于 `nalgebra::Isometry3<f64>` 的刚体变换 |
| `Twist` | 角运动分量在前的空间速度或加速度 |
| `Wrench` | 力矩分量在前的空间力 |

### 模型构建与访问

| 接口 | 结果 |
|---|---|
| `Robot::from_urdf(path)` | 从 URDF 文件路径构建模型 |
| `name()`、`joints()`、`links()` | 查看模型数据，`links()` 包含父 link |
| `root_link()`、`leaf_links()` | 查看父 link 和所有子 link |
| `link(name)` | 按名称借用 `Link`，不存在时返回 `Error::UnknownLink` |
| `link_count()` | 返回包含父 link 的 URDF link 数量 |
| `joint_count()` | 返回从模型中解析出的关节数量 |

### 计算接口

为保持库的定位专注、轻量，公开计算接口仅限下列操作。

| 接口 | 状态与结果 |
|---|---|
| `forward_kinematics(q, target)` | 指定 link 的位姿 |
| `jacobian(q, target)` | 指定 link 的基座坐标系 Jacobian，非祖先关节列为零 |
| `inverse_kinematics(initial_q, target, desired)` | 使用默认参数的阻尼最小二乘位姿逆运动学 |
| `inverse_kinematics_with_options(...)` | 可配置阻尼、容差、步长和迭代上限的位姿逆运动学 |
| `forward_velocity_kinematics(q, qd, target, base, tool)` | 指定 link/tool 的空间速度 |
| `forward_acceleration_kinematics(q, qd, qdd, target)` | 指定 link 的直接递推加速度 |
| `gravity(q, base, loads)` | 支持多 link 外载荷的树形重力递推 |
| `inverse_dynamics(..., loads)` | 支持多 link 外载荷和分支汇聚的树形 RNEA |

```rust
use dyno::{JointVector, Robot};

let arm = Robot::from_urdf("test_arm.urdf")?;
let q = JointVector::<4>::zeros();
let target = arm.link("test_link_4")?;
let end = arm.forward_kinematics(&q, target)?;
let jacobian = arm.jacobian(&q, target)?;
let solved_q = arm.inverse_kinematics(&q, target, &end)?;
# Ok::<(), dyno::Error>(())
```

计算尺寸 `N` 会从每次传入的 `JointVector<N>` 自动推导，不再属于 `Robot` 类型的
一部分。若模型与输入尺寸不一致，会在开始计算前返回 `Error::WrongJointCount`。

`inverse_kinematics` 使用阻尼逆
`J^T (J J^T + lambda^2 I)^-1`。迭代过程不施加约束，但收敛结果会使用 URDF 中的关节
限位进行检查。求解错误直接通过 `Error` 的变体
明确区分非法配置、非有限输入、数值分解失败、关节限位越界和不收敛；不收敛错误还会
给出最终的平移与旋转残差。需要调整默认参数时，可使用
`inverse_kinematics_with_options`。

内置求解器仅适用于简单的位姿逆解。最终关节限位检查只会报告无效结果，并不会在优化
过程中施加约束。需要冗余控制、关节位置或速度约束、碰撞约束以及其他任务优先级时，
建议通过 `jacobian` 获取几何 Jacobian，再结合合适的 QP solver 自行构建约束 IK。

## 树模型约定与兼容范围

`Robot` 支持任意分支数量和深度的合法树状 URDF：模型具有唯一父 link，每个非父
link 只有一个父 joint。构建时会生成父先于子的拓扑顺序，并拒绝多父、重复名称、环、
断连、缺失 link 和一个 link 被多个 joint 重复连接的模型。当前
支持 revolute、continuous、prismatic 和 fixed joint；其他 URDF joint 类型仍会返回
`UnsupportedJoint`。

运动学接口直接接收目标 `&Link`，因此同一个模型可以计算任意分叉末端。重力和逆动力学
接口接收 `&[Load]`，可同时在任意多个 link 上施加载荷；空切片表示没有外载荷。
`JointVector<N>` 当前按全部 URDF joint 排列，fixed joint 仍占一个元素但其运动和主动
关节力为零。

父 link 会保存在 `links()` 中，但固定基座兼容动力学不把父 link 自身的惯性计入
关节力或基座 Wrench。`Load` 作用在 link 原点，并以该 link 坐标系表达。

兼容动力学内核保留已有的正 Z 方向重力和惯量积符号。Pinocchio 桥接层会转换相应
约定，正确性测试逐元素比较转换后的数值，性能基准只统计执行开销。

## 性能基准

树基准模型包含 7 个可动关节：一个公共 trunk 和左右两条三级分支，共有 2 个子 link。
以下结果使用相同的树状 URDF 和关节输入，对比 Dyno 与 Pinocchio 3.9.0。
数据通过 `cargo bench --features pinocchio-bench --bench pinocchio -- --quick` 在 Intel
Core i9-14900K 上测得；Pinocchio 时间已扣除本次测得的 0.70 ns C ABI 固定开销。

| 函数 | Dyno | Pinocchio | Dyno 加速比 |
|---|---:|---:|---:|
| `forward_kinematics` | 111.27 ns | 134.07 ns | 1.20x |
| `jacobian` | 138.88 ns | 214.00 ns | 1.54x |
| `gravity` | 163.51 ns | 321.75 ns | 1.97x |
| `inverse_dynamics` | 256.50 ns | 513.81 ns | 2.00x |

模型构建和 URDF 解析不在计时区间内，两边都会复用已解析的模型，Pinocchio 还会复用
其 `Data` 对象。Dyno 计算路径使用固定尺寸栈数组保存节点中间状态，不进行堆内存分配。
Criterion quick 模式样本较少，上述结果仅展示当前机器上的性能趋势，不应视为跨平台或
严格统计结论。

桥接层会统一关节顺序、空间向量行顺序和重力方向。另有集成测试逐元素比较 FK、
Jacobian、gravity 和 RNEA 的完整输出；性能基准本身只测执行时间。

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
cargo test --features pinocchio-bench --test tree_pinocchio
```

集成测试覆盖 Jacobian 导数、加速度、逆动力学参考值、Jacobian 及其导数的有限差分
验证、旋转与移动关节、重力、关节限位和被动关节。Pinocchio 交叉测试在 32 组确定性
配置下验证两条分支，并逐元素比较 FK、Jacobian、gravity 和 RNEA。性能基准使用同一份
包含公共 trunk、两条分支和两个子 link 的 7 关节树状 URDF。
