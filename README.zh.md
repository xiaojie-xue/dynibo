<!-- markdownlint-disable MD033 MD041 -->

<div align="center">

<h1>dynibo</h1>

<p><strong>Robot kinematics and rigid-body dynamics with URDF, Jacobians, inverse kinematics, RNEA and ABA</strong></p>

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

`dynibo` 是一个面向控制器设计的机器人运动学与动力学库，适用于机械臂、人形机器人等
多种形态，支持固定和浮动基座。它可从 URDF 加载机器人模型，并提供 Rust、Python、C 和 C++ 接口。

## 算法与任务示例

Dynibo 提供正运动学（FK）、世界几何雅可比及其时间导数、阻尼最小二乘逆运动学（IK）、
质量矩阵、重力补偿、递归牛顿–欧拉逆动力学（RNEA）和关节体正动力学（ABA）。
固定与浮动基座使用同一套 Rust 核心；IK 适用于固定基座。

| 任务 | 完整且经过验证的示例 |
|---|---|
| 加载 URDF 并计算工具位姿 | [正运动学](https://dynibo.readthedocs.io/en/latest/zh/recipes/forward-kinematics/) |
| 计算雅可比并转换 NumPy 布局 | [雅可比与数组布局](https://dynibo.readthedocs.io/en/latest/zh/recipes/jacobian/) |
| 计算保持力矩 | [重力补偿](https://dynibo.readthedocs.io/en/latest/zh/recipes/gravity-compensation/) |
| 计算力矩或恢复加速度 | [RNEA 与 ABA](https://dynibo.readthedocs.io/en/latest/zh/recipes/inverse-forward-dynamics/) |
| 计算浮动基座机器人 | [浮动基座动力学](https://dynibo.readthedocs.io/en/latest/zh/recipes/floating-base/) |
| 在重复计算中复用数组 | [输出缓冲区](https://dynibo.readthedocs.io/en/latest/zh/recipes/reuse-buffers/) |

每个 Python 示例都内嵌小型 URDF 并校验数值结果，无需 ROS 或下载 mesh。
Rust 用户可在仓库中执行 `cargo run --example recipes` 运行相同六类任务。
[选型与 Pinocchio 迁移指南](https://dynibo.readthedocs.io/en/latest/zh/getting-started/choosing-dynibo/)
介绍了适用范围、API 映射和参考系、数组转换约定。

## 特性

### 快速

Dynibo 基于 Rust 实现，并复用每个机器人的内部存储。创建 `Robot` 和输出 buffer 后，
主要运动学与动力学接口不会在计算循环中分配内存或调整容量。

为直观展示计算性能，我们选用开源机器人运动学与动力学库
[Pinocchio](https://github.com/stack-of-tasks/pinocchio) 作为参照，
在 Franka（7 关节固定基座机械臂）和 unitree G1（29 关节浮动基座人形机器人）上进行基准测试。
下表给出 Dynibo 相对 Pinocchio 在各项运算中的计算效率提升比例。

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
      <td align="right">1.61×</td><td align="right">1.78×</td>
      <td align="right">1.26×</td><td align="right">1.40×</td>
    </tr>
    <tr>
      <td>RNEA</td>
      <td align="right">1.72×</td><td align="right">1.80×</td>
      <td align="right">1.19×</td><td align="right">1.56×</td>
    </tr>
    <tr>
      <td>ABA</td>
      <td align="right">1.14×</td><td align="right">1.17×</td>
      <td align="right">1.76×</td><td align="right">1.94×</td>
    </tr>
  </tbody>
</table>

复现上述结果的源码见 [`benches/`](benches/)。

### 可靠

Dynibo 将随机生成的、可精确复现的 URDF 用例与长期维护的固定用例相结合，覆盖串联与树状机器人、固定基座与浮动基座、混合关节、外部载荷、非法输入及 workspace 重复使用。结果通过
有限差分近似、算法间一致性关系和独立 Pinocchio oracle 校验；另有专项测试验证计算期零分配
及安装后的 Rust、Python、C 和 C++ 包。详见[测试架构](tests/TESTING.zh.md)。

### 易用

Dynibo 的 API 围绕 `Robot` 展开：加载 URDF 后，即可调用运动学与动力学算法。
内部计算存储由 `Robot` 管理，无需分别创建和维护 `Model` 与 `Data` 对象。
Rust、Python、C 和 C++ 接口共用同一套 Rust 核心，便于接入不同语言的项目。

## 依赖

Rust 核心只有两个直接运行时依赖：

- [`nalgebra`](https://nalgebra.rs/) — 线性代数与数值类型
- [`urdf-rs`](https://github.com/openrr/urdf-rs) — URDF 解析

Python wheel 已包含原生库，运行时需要 NumPy 1.23 或更高版本。

## 快速开始

### Rust

添加 Cargo package：

```bash
cargo add dynibo
```

加载 URDF 并计算目标 link 位姿：

```rust
use dynibo::Robot;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let urdf = r#"<robot name="arm">
      <link name="base"/>
      <link name="arm">
        <inertial>
          <origin xyz="0.5 0 0"/>
          <mass value="1"/>
          <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
        </inertial>
      </link>
      <joint name="hinge" type="revolute">
        <parent link="base"/><child link="arm"/>
        <axis xyz="0 1 0"/>
        <limit lower="-3.14" upper="3.14" effort="100" velocity="10"/>
      </joint>
      <link name="tool"/>
      <joint name="tip" type="fixed">
        <parent link="arm"/><child link="tool"/>
        <origin xyz="1 0 0"/>
      </joint>
    </robot>"#;
    let path = std::env::temp_dir()
        .join(format!("dynibo-quick-start-{}.urdf", std::process::id()));
    std::fs::write(&path, urdf)?;
    let loaded = Robot::from_urdf(&path);
    std::fs::remove_file(&path)?;
    let mut robot = loaded?;
    let tool = robot.link_id("tool")?;
    let q = [0.3_f64];
    let pose = robot.forward_kinematics(&q, tool)?;
    assert!((pose.translation.x - q[0].cos()).abs() < 1.0e-12);
    assert!((pose.translation.z + q[0].sin()).abs() < 1.0e-12);

    let mut jacobian = vec![0.0; 6 * robot.generalized_count()];
    robot.jacobian(&q, tool, &mut jacobian)?;
    let mut gravity = vec![0.0; robot.generalized_count()];
    robot.gravity(&q, &[], &mut gravity)?;
    println!("translation: {:?}; holding torque: {:?}", pose.translation, gravity);
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
"""URDF forward kinematics: run with an installed dynibo package; no model download needed."""

from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np

from dynibo import Robot

URDF = """<robot name="pendulum">
  <link name="base">
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
  <link name="arm">
    <inertial>
      <origin xyz="0.5 0 0"/>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
  <joint name="hinge" type="revolute">
    <parent link="base"/><child link="arm"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="10"/>
  </joint>
  <link name="tool"/>
  <joint name="tip" type="fixed">
    <parent link="arm"/><child link="tool"/>
    <origin xyz="1 0 0"/>
  </joint>
</robot>"""

with TemporaryDirectory() as directory:
    path = Path(directory) / "pendulum.urdf"
    path.write_text(URDF, encoding="utf-8")
    with Robot.from_urdf(path) as robot:
        target = robot.link_id("tool")
        q = np.array([0.3], dtype=np.float64)
        pose = robot.forward_kinematics(q, target)
        expected = (np.cos(q[0]), 0.0, -np.sin(q[0]))
        np.testing.assert_allclose(pose.translation, expected, atol=1e-12)
        assert len(pose.rotation_xyzw) == 4
        print("world tool translation:", pose.translation)
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
全部主要运动学与动力学方法。

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
