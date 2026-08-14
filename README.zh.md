# dynibo

[![Package CI](https://github.com/xiaojie-xue/dynibo/actions/workflows/package-ci.yml/badge.svg?branch=main)](https://github.com/xiaojie-xue/dynibo/actions/workflows/package-ci.yml)
[![codecov](https://codecov.io/gh/xiaojie-xue/dynibo/branch/main/graph/badge.svg)](https://codecov.io/gh/xiaojie-xue/dynibo)
[![GitHub Release](https://img.shields.io/github/v/release/xiaojie-xue/dynibo)](https://github.com/xiaojie-xue/dynibo/releases/latest)
[![Built with Rust](https://img.shields.io/badge/Built_with-Rust-000000?logo=rust&logoColor=white)](https://www.rust-lang.org/)
[![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

[English](README.md) | 简体中文

`dynibo` 是一个快速、轻量且可靠的机器人运动学与动力学库。它在运行时从 URDF
加载机器人，并通过可复用的 Workspace 提供计算期零分配的接口；同时基于同一
套 Rust 核心开放 Python 与 C/C++ 接口。

## 特性

### 快速

Dynibo 基于 Rust 实现，并将内存分配移出计算循环。创建 `Workspace` 和输出 buffer 后，
主要运动学与动力学接口会复用已有内存，不再分配或调整容量。

以下 Criterion 数据使用相同的 URDF 模型和关节状态对比 Dynibo 与 Pinocchio。Robot、
Workspace、Pinocchio `Data` 和输出 buffer 的创建均不计入耗时。加速比根据 quick 模式
报告区间的中值计算，并从 Pinocchio 耗时中扣除了实测的 0.882 ns 固定 C ABI 开销。

在这些 benchmark 中，Dynibo 相比 Pinocchio 快 1.17–2.70 倍，数值越高表示性能越好。

| 模型 | FK | Jacobian | Gravity | RNEA |
|---|---:|---:|---:|---:|
| 串联模型（4 关节） | 1.17× | 1.58× | 1.59× | 1.66× |
| 串联模型（40 关节） | 1.24× | 1.72× | 1.63× | 1.91× |
| 双末端树状模型（7 关节） | 2.08× | 2.70× | 1.89× | 1.91× |

测试环境为 Intel Core i9-14900K、rustc 1.97.1 和 Pinocchio 3.9.0。确保 `pkg-config` 能找到 Pinocchio 后，
可通过以下命令复现：

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
- `coriolis_matrix` — 离心力 + 科氏力矩阵
- `gravity` — 重力补偿，可附加外部载荷
- `inverse_dynamics` — 递归 Newton–Euler 逆动力学

API 围绕少量核心类型构建：`Robot`、`Workspace`、`LinkId`、`Frame`、`Twist` 和
`Wrench`。Rust、Python、C 和 C++ 接口共用同一套 Rust 实现。

### 可靠

Dynibo 经过了深入的单元测试。测试覆盖有限差分运动学、动力学回归、树状机器人与外部
载荷、逆运动学、非法输入、Workspace 归属与复用，以及计算期零分配。独立的 Pinocchio
oracle 还会在确定性机器人状态下完整对比 FK、Jacobian、Jacobian 时间导数、质量矩阵、科氏矩阵、gravity 和 RNEA 输出。

Rust 核心不包含项目自身的 `unsafe` 代码。CI 要求 Rust workspace 的行覆盖率不低于
85%，分支覆盖率不低于 75%。

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

### Python

从 PyPI 安装 Python package：

```bash
python -m pip install dynibo
```

安装后通过 `dynibo` 导入。

### C/C++

构建并安装 CMake package：

```bash
cmake -S . -B build/c -DCMAKE_BUILD_TYPE=Release
cmake --build build/c --parallel
cmake --install build/c --prefix /opt/dynibo
```

CMake 项目可使用安装后的 `dynibo::dynibo` target。

## 文档

- [Rust API 文档](https://docs.rs/dynibo)
- [Python API 文档](https://dynibo.readthedocs.io/)

## 示例

完整调用示例见 [`examples/`](examples/) 目录。

## 支持的模型

Dynibo 支持运行时尺寸的树状 URDF，以及 revolute、continuous、prismatic 和 fixed joint。
无效拓扑、错误输入长度、模型不匹配的 handle 和求解失败都会返回结构化错误。

## 测试

```bash
cargo fmt --all -- --check
cargo clippy --all-targets -- -D warnings
cargo test --workspace --all-targets
```

运行完整的 Rust、Pinocchio、Python、C 和 C++ 验证套件：

```bash
bash ci/test-all.sh
```

## 贡献

Dynibo 目前仍处于早期阶段，欢迎大家一起参与构建和完善。你可以通过 issue 反馈问题
或提出建议，也可以提交 pull request；如果你对项目的发展有任何想法，欢迎随时联系我。

## 引用

如果 Dynibo 对你的工作有帮助，请使用以下格式引用：

```bibtex
@software{xue2026dynibo,
  author  = {Xue, Xiaojie},
  title   = {Dynibo: a Fast, Lightweight, and Reliable Robot Kinematics and Dynamics Library},
  year    = {2026},
  version = {0.1.0},
  url     = {https://github.com/xiaojie-xue/dynibo}
}
```
