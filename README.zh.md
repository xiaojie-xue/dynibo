<!-- markdownlint-disable MD033 MD041 -->

<div align="center">

<h1>dynibo</h1>

<p><strong>Fast &middot; Reliable &middot; Easy to Use</strong></p>

<p>
  <a href="https://dynibo.readthedocs.io/en/latest/zh/">文档</a> &nbsp;&middot;&nbsp;
  <a href="README.md">English</a> | <strong>简体中文</strong>
</p>

<p>
  <a href="https://github.com/xiaojie-xue/dynibo/actions/workflows/package-ci.yml"><img alt="CI" src="https://github.com/xiaojie-xue/dynibo/actions/workflows/package-ci.yml/badge.svg?branch=main"></a>
  <a href="https://codecov.io/gh/xiaojie-xue/dynibo"><img alt="codecov" src="https://codecov.io/gh/xiaojie-xue/dynibo/branch/main/graph/badge.svg"></a>
  <a href="https://crates.io/crates/dynibo"><img alt="crates.io" src="https://img.shields.io/crates/v/dynibo.svg?color=CE422B&amp;logo=rust&amp;logoColor=white"></a>
  <a href="https://pypi.org/project/dynibo/"><img alt="PyPI" src="https://img.shields.io/pypi/v/dynibo.svg?color=3776AB&amp;logo=python&amp;logoColor=white"></a>
  <a href="LICENSE"><img alt="License: MIT" src="https://img.shields.io/badge/license-MIT-blue.svg"></a>
</p>

</div>

`dynibo` 是一个快速、可靠且易用的机器人运动学与动力学库，支持固定和浮动基座机器人。它在运行时从 URDF
加载机器人，并通过 Robot 内部的可复用存储提供计算期零分配的接口；同时基于同一
套 Rust 核心开放 Python 与 C/C++ 接口。

## 特性

### 快速

Dynibo 基于 Rust 实现，并复用每个机器人的内部存储。创建 `Robot` 和输出 buffer 后，
主要运动学与动力学接口不会在计算循环中分配内存或调整容量。

下表展示 Dynibo 相对 Pinocchio 在两种机器人上的加速比：Franka 是 7 关节固定基座机械臂，
unitree G1 是 29 关节浮动基座人形机器人。

<table>
  <thead>
    <tr>
      <th rowspan="2">运算</th>
      <th colspan="2" align="center">Rust</th>
      <th colspan="2" align="center">Python</th>
    </tr>
    <tr>
      <th>Franka</th><th>unitree G1</th>
      <th>Franka</th><th>unitree G1</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>Jacobian</td>
      <td align="right">1.59×</td><td align="right">1.80×</td>
      <td align="right">1.28×</td><td align="right">1.38×</td>
    </tr>
    <tr>
      <td>RNEA</td>
      <td align="right">1.74×</td><td align="right">1.81×</td>
      <td align="right">1.17×</td><td align="right">1.54×</td>
    </tr>
    <tr>
      <td>ABA</td>
      <td align="right">1.20×</td><td align="right">1.14×</td>
      <td align="right">1.81×</td><td align="right">1.89×</td>
    </tr>
  </tbody>
</table>

复现上述结果的源码见 [`benches/`](benches/)。

### 可靠

Dynibo 将随机生成的、可精确复现的 URDF 用例与长期维护的固定用例相结合，覆盖串联与树状机器人、固定基座与浮动基座、混合关节、外部载荷、非法输入及 workspace 重复使用。结果通过
有限差分近似、算法间一致性关系和独立 Pinocchio oracle 校验；另有专项测试验证计算期零分配
及安装后的 Rust、Python、C 和 C++ 包。详见[测试架构](tests/TESTING.zh.md)。

### 易用

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
- `forward_dynamics` — 线性时间复杂度的 articulated-body 正动力学

静态内存分配由库在内部隐藏式管理，用户无需分别构造 `Model` 和 `Data`。
Rust、Python、C 和 C++ 接口共用同一套 Rust 实现。

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

加载 URDF 并计算目标 link 位姿：

```rust
use dynibo::Robot;

fn main() -> dynibo::Result<()> {
    let mut robot = Robot::from_urdf("robot.urdf")?;
    let tool = robot.link_id("tool")?;
    let q = vec![0.0; robot.joint_count()];

    let pose = robot.forward_kinematics(&q, tool)?;
    println!("translation: {}", pose.translation.vector.transpose());
    Ok(())
}
```

### Python

从 PyPI 安装 Python package：

```bash
python -m pip install dynibo
```

Python binding 会在内部持有并复用原生计算存储：

```python
import numpy as np

from dynibo import Robot

with Robot.from_urdf("robot.urdf") as robot:
    tool = robot.link_id("tool")
    q = np.zeros(robot.joint_count)
    pose = robot.forward_kinematics(q, tool)
    jacobian = np.empty(6 * robot.generalized_count)
    robot.jacobian(q, tool, out=jacobian)
    print(pose.translation)
```

### C/C++

C 和 C++ 用户可以从
[GitHub Releases](https://github.com/xiaojie-xue/dynibo/releases)
下载适用于 Linux、macOS 或 Windows 的预编译包，也可以从源码构建并安装。
预编译包包含动态库、C/C++ 头文件、pkg-config 元数据和 CMake package 配置。
请选择与操作系统及 CPU 架构匹配的压缩包，并使用同一 Release 中的
`SHA256SUMS` 校验下载文件。

从源码构建需要 Rust、Cargo 和 CMake 3.16 或更高版本：

```bash
cmake -S . -B build/c -DCMAKE_BUILD_TYPE=Release
cmake --build build/c --parallel
cmake --install build/c --prefix /opt/dynibo
```

在另一个 CMake 项目中使用解压后的预编译包或源码安装结果：

```cmake
find_package(dynibo CONFIG REQUIRED)
target_link_libraries(my_robot PRIVATE dynibo::dynibo)
```

通过 `-DCMAKE_PREFIX_PATH` 指向压缩包解压目录或实际安装目录。各平台的运行时
动态库路径配置见[安装指南](docs/getting-started/installation.zh.md)。

## 示例

Rust、Python、C++ 和 C 的完整调用示例见 [`examples/`](examples/) 目录；每个示例均覆盖
上文列出的全部主要运动学与动力学方法。

## 支持的模型

Dynibo 支持**固定基座（fixed-base）机器人**和**浮动基座（floating-base）机器人**，
模型采用运行时尺寸的树状 URDF，支持 revolute、continuous、prismatic 和 fixed joint。
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

## 许可证

Dynibo 代码使用 [MIT 许可证](LICENSE)。随项目提供的机器人描述保留各自的
[第三方许可证](examples/data/README.md)，包括 Franka 的 Apache-2.0 和 Unitree 的 BSD-3-Clause。

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
  url     = {https://github.com/xiaojie-xue/dynibo}
}
```
