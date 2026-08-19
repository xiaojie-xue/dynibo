<!-- markdownlint-disable MD033 MD041 -->

<div align="center">

<h1>dynibo</h1>

<p><strong>Fast &middot; Lightweight &middot; Reliable</strong></p>

<p>
  <a href="https://docs.rs/dynibo">Rust API</a>
  &nbsp;&middot;&nbsp;
  <a href="https://dynibo.readthedocs.io/">Python API</a>
</p>

<p>
  <a href="https://github.com/xiaojie-xue/dynibo/actions/workflows/package-ci.yml"><img alt="CI" src="https://github.com/xiaojie-xue/dynibo/actions/workflows/package-ci.yml/badge.svg?branch=main"></a>
  <a href="https://codecov.io/gh/xiaojie-xue/dynibo"><img alt="codecov" src="https://codecov.io/gh/xiaojie-xue/dynibo/branch/main/graph/badge.svg"></a>
  <a href="https://crates.io/crates/dynibo"><img alt="crates.io" src="https://img.shields.io/crates/v/dynibo.svg?color=CE422B&amp;logo=rust&amp;logoColor=white"></a>
  <a href="https://pypi.org/project/dynibo/"><img alt="PyPI" src="https://img.shields.io/pypi/v/dynibo.svg?color=3776AB&amp;logo=python&amp;logoColor=white"></a>
  <a href="LICENSE"><img alt="License: MIT" src="https://img.shields.io/badge/license-MIT-blue.svg"></a>
</p>

</div>

[English](README.md) | 简体中文

`dynibo` 是一个快速、轻量且可靠的机器人运动学与动力学库。它在运行时从 URDF
加载机器人，并通过可复用的 Workspace 提供计算期零分配的接口；同时基于同一
套 Rust 核心开放 Python 与 C/C++ 接口。

## 特性

### 快速

在下列 benchmark 所测量的核心操作中，Dynibo 的运行速度是 Pinocchio 的
1.19–2.51 倍。Dynibo 基于 Rust 实现，并将内存分配移出计算循环。创建 `Workspace`
和输出 buffer 后，主要运动学与动力学接口会复用已有内存，不再分配或调整容量。

下表展示 Dynibo 相对 Pinocchio 在核心运动学与动力学接口上的加速比。

| 模型 | FK | Jacobian | Gravity | RNEA |
|---|---:|---:|---:|---:|
| 双末端树状模型（7 关节，固定基座） | 1.90× | 2.05× | 1.89× | 1.94× |
| 双末端树状模型（7 关节，浮动基座） | 2.16× | 2.51× | 2.15× | 2.20× |
| 串联模型（40 关节，固定基座） | 1.19× | 1.49× | 1.78× | 1.99× |
| 串联模型（40 关节，浮动基座） | 1.21× | 1.56× | 1.79× | 2.09× |

这些 Criterion quick 模式数据使用相同的 URDF 模型和关节状态，测试环境为 Intel Core
i9-14900K、rustc 1.97.1 和 Pinocchio 3.9.0。初始化和内存分配不计入耗时；加速比根据
报告区间的中值计算，并从 Pinocchio 耗时中扣除实测的 0.703 ns 固定 C ABI 开销。
确保 `pkg-config` 能找到 Pinocchio 后，可通过以下命令重新运行原始 benchmark：

```bash
cargo bench --features pinocchio-bench --bench pinocchio -- --quick
```

### 轻量

Dynibo 专注于最常用的机器人运动学与动力学接口：

- `forward_kinematics` — 目标 link 位姿
- `jacobian` — 目标 link 的 Jacobian
- `jacobian_derivative` — 目标 link Jacobian 的时间导数
- `forward_velocity_kinematics` — 空间速度
- `forward_acceleration_kinematics` — 空间加速度
- `inverse_kinematics` — 阻尼最小二乘逆运动学
- `mass_matrix` — 关节空间质量矩阵
- `velocity_product_forces` — 离心力 + 科氏广义力
- `gravity` — 重力补偿，可附加外部载荷
- `inverse_dynamics` — 递归 Newton–Euler 逆动力学

API 围绕少量核心类型构建：`Robot`、`Workspace`、`LinkId`、`Frame`、`Twist` 和
`Wrench`。Rust、Python、C 和 C++ 接口共用同一套 Rust 实现。

### 可靠

Dynibo 经过了深入的单元测试。测试覆盖有限差分运动学、动力学回归、树状机器人与外部
载荷、逆运动学、非法输入、Workspace 归属与复用，以及计算期零分配。独立的 Pinocchio
oracle 还会在确定性机器人状态下完整对比 FK、Jacobian、Jacobian 时间导数、质量矩阵、速度乘积力、gravity 和 RNEA 输出。

## 依赖

Rust 核心只有两个直接运行时依赖：

- [`nalgebra`](https://nalgebra.rs/) — 线性代数与数值类型
- [`urdf-rs`](https://github.com/openrr/urdf-rs) — URDF 解析

Python wheel 已包含原生库，无需额外的 Python 运行时依赖。

## 快速开始

### Rust

添加 Cargo package：

```bash
cargo add dynibo
```

加载 URDF、创建可复用的 workspace，并计算目标 link 位姿：

```rust
use dynibo::Robot;

fn main() -> dynibo::Result<()> {
    let robot = Robot::from_urdf("robot.urdf")?;
    let tool = robot.link_id("tool")?;
    let mut workspace = robot.workspace();
    let q = vec![0.0; robot.joint_count()];

    let pose = robot.forward_kinematics(&q, tool, &mut workspace)?;
    println!("translation: {}", pose.translation.vector.transpose());
    Ok(())
}
```

### Python

从 PyPI 安装 Python package：

```bash
python -m pip install dynibo
```

Python binding 会在内部持有并复用原生 workspace：

```python
from dynibo import Robot

with Robot.from_urdf("robot.urdf") as robot:
    tool = robot.link_id("tool")
    q = [0.0] * robot.joint_count
    pose = robot.forward_kinematics(q, tool)
    print(pose.translation)
```

### C/C++

从源码构建并安装 CMake package。构建需要 Rust、Cargo 和 CMake 3.16 或更高版本：

```bash
cmake -S . -B build/c -DCMAKE_BUILD_TYPE=Release
cmake --build build/c --parallel
cmake --install build/c --prefix /opt/dynibo
```

在另一个 CMake 项目中使用安装后的 package：

```cmake
find_package(dynibo CONFIG REQUIRED)
target_link_libraries(my_robot PRIVATE dynibo::dynibo)
```

如果 dynibo 安装在自定义目录中，请通过 `-DCMAKE_PREFIX_PATH=/opt/dynibo`
（或实际选择的安装目录）配置消费端项目。

## 示例

Rust、Python 和 C 的完整调用示例见 [`examples/`](examples/) 目录；每个示例均覆盖
上文列出的全部主要运动学与动力学方法。

## 支持的模型

Dynibo 支持运行时尺寸的树状 URDF，以及 revolute、continuous、prismatic 和 fixed joint。
无效拓扑、错误输入长度、模型不匹配的 handle 和求解失败都会返回结构化错误。

## 测试

```bash
cargo fmt --all -- --check
cargo clippy --workspace --all-targets --locked -- -D warnings
cargo test --workspace --all-targets --locked
```

通过以下命令运行完整的本地 Rust、Python、C 和 C++ 验证套件。当 `pkg-config`
能够找到 Pinocchio 时，该命令也会运行 Pinocchio 参考测试。

```bash
bash ci/test-all.sh
```

## 贡献

Dynibo 目前仍处于早期阶段，欢迎参与构建和完善。开发环境、必要检查和 pull request
要求请参阅 [`CONTRIBUTING.md`](CONTRIBUTING.md)。

## 引用

如果 Dynibo 对你的工作有帮助，请使用以下格式引用：

```bibtex
@software{xue2026dynibo,
  author  = {Xue, Xiaojie},
  title   = {Dynibo: a Fast, Lightweight, and Reliable Robot Kinematics and Dynamics Library},
  year    = {2026},
  version = {0.2.0},
  url     = {https://github.com/xiaojie-xue/dynibo}
}
```
