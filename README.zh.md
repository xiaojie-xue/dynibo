# dyno

[![Package CI](https://github.com/xiaojie-xue/dyno/actions/workflows/package-ci.yml/badge.svg?branch=main)](https://github.com/xiaojie-xue/dyno/actions/workflows/package-ci.yml)
[![codecov](https://codecov.io/gh/xiaojie-xue/dyno/branch/main/graph/badge.svg)](https://codecov.io/gh/xiaojie-xue/dyno)

[English](README.md) | 简体中文

`dyno` 是一个快速、轻量且可靠的 Rust 机器人运动学与动力学库。它在运行时从 URDF
加载树状机器人拓扑，并通过可复用的 Workspace 提供计算期零分配的接口；同时基于同一
套 Rust 核心开放 Python 与 C/C++ 接口。

## 特性

### 快速

Dyno 基于 Rust 实现，并将内存分配移出计算循环。创建 `Workspace` 和输出 buffer 后，
主要运动学与动力学接口会复用已有内存，不再分配或调整容量。

以下 Criterion 数据使用相同的 URDF 模型和关节状态对比 Dyno 与 Pinocchio。Robot、
Workspace、Pinocchio `Data` 和输出 buffer 的创建均不计入耗时。加速比根据 quick 模式
报告区间的中值计算，并从 Pinocchio 耗时中扣除了实测的 0.882 ns 固定 C ABI 开销。

在这些 benchmark 中，Dyno 相比 Pinocchio 快 1.17–2.70 倍，数值越高表示性能越好。

| 模型 | FK | Jacobian | Gravity | RNEA |
|---|---:|---:|---:|---:|
| 串联模型（4 关节） | 1.17× | 1.58× | 1.59× | 1.66× |
| 串联模型（40 关节） | 1.24× | 1.72× | 1.63× | 1.91× |
| 双末端树状模型（7 关节） | 2.08× | 2.70× | 1.89× | 1.91× |

测试环境为 Intel Core i9-14900K、rustc 1.97.1 和 Pinocchio 3.9.0。以上数据用于展示
当前机器上的性能趋势，不构成跨平台性能承诺。确保 `pkg-config` 能找到 Pinocchio 后，
可通过以下命令复现：

```bash
cargo bench --features pinocchio-bench --bench pinocchio -- --quick
```

### 轻量

Dyno 专注于最常用的机器人运动学与动力学接口：

- `forward_kinematics` — 目标 link 位姿
- `jacobian` — 目标 link 的 Jacobian
- `forward_velocity_kinematics` — 空间速度
- `forward_acceleration_kinematics` — 空间加速度
- `inverse_kinematics` — 阻尼最小二乘逆运动学
- `gravity` — 重力补偿，可附加外部载荷
- `inverse_dynamics` — 递归 Newton–Euler 逆动力学

API 围绕少量核心类型构建：`Robot`、`Workspace`、`LinkId`、`Frame`、`Twist` 和
`Wrench`。数值计算基于 [`nalgebra`](https://nalgebra.rs/)，URDF 解析基于
[`urdf-rs`](https://github.com/openrr/urdf-rs)。Rust、Python、C 和 C++ 接口共用同一套
Rust 实现。

### 可靠

Dyno 经过了深入的单元测试。测试覆盖有限差分运动学、动力学回归、树状机器人与外部
载荷、逆运动学、非法输入、Workspace 归属与复用，以及计算期零分配。独立的 Pinocchio
oracle 还会在确定性机器人状态下完整对比 FK、Jacobian、gravity 和 RNEA 输出。

Rust 核心不包含项目自身的 `unsafe` 代码。CI 要求 Rust workspace 的行覆盖率不低于
85%，分支覆盖率不低于 75%。

## 快速开始

### Rust

添加 Cargo package：

```bash
cargo add dyno
```

### Python

从 PyPI 安装 Python package：

```bash
python -m pip install dyno-robotics
```

安装后通过 `dyno` 导入。

### C/C++

构建并安装 CMake package：

```bash
cmake -S . -B build/c -DCMAKE_BUILD_TYPE=Release
cmake --build build/c --parallel
cmake --install build/c --prefix /opt/dyno
```

CMake 项目可使用安装后的 `dyno::dyno` target。package 与链接配置详见
[安装指南](docs/RELEASING.zh.md)。

## 示例

完整调用示例见 [`examples/`](examples/) 目录。

## 支持的模型

Dyno 支持运行时尺寸的树状 URDF，以及 revolute、continuous、prismatic 和 fixed joint。
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

欢迎参与贡献。你可以通过 issue 反馈问题或提出建议，也可以提交 pull request 改进项目。
如果你对改进项目有任何想法，欢迎随时联系我。

## 引用

如果 Dyno 对你的工作有帮助，请使用以下格式引用：

```bibtex
@software{xue2026dyno,
  author  = {Xue, Xiaojie},
  title   = {Dyno: Fast, Lightweight, and Reliable Robot Kinematics and Dynamics},
  year    = {2026},
  version = {0.1.0},
  url     = {https://github.com/xiaojie-xue/dyno}
}
```

## 许可证

Dyno 使用 [MIT License](LICENSE)。
