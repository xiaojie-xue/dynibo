# dynibo

**快速 · 轻量 · 可靠**

Dynibo 是一个机器人运动学与动力学库。它在运行时从 URDF 加载机器人拓扑，通过复用
Workspace 将内存分配移出重复计算过程，并基于同一套 Rust 实现提供 Rust、Python、
C++ 和 C 接口。

[快速上手](getting-started/quick-start.md){ .md-button .md-button--primary }
[安装 dynibo](getting-started/installation.md){ .md-button }

## 为什么选择 dynibo

### 快速

核心算法使用 Rust 编写，并以复用内存为设计目标。创建 `Workspace` 和所需的输出
buffer 后，主要运动学与动力学接口不会在计算循环中分配内存或调整容量。在项目公布的
benchmark 中，所测核心操作的运行速度是 Pinocchio 的 1.19–2.51 倍。

### 轻量

Dynibo 专注于一组紧凑、常用的机器人算法，而不是构建庞大的框架。API 围绕少量共同
概念组织：`Robot`、`Workspace`、`LinkId`、`Frame`、`Twist` 和 `Wrench`；四种
语言接口共享这套词汇。

### 可靠

测试覆盖有限差分运动学、动力学回归、树状机器人、外部载荷、逆运动学、非法输入、
Workspace 复用和内存分配行为。核心数值结果还会与独立的 Pinocchio oracle 进行对比。

## 可以计算什么

| 领域 | 能力 | 继续阅读 |
|---|---|---|
| 模型 | 加载 URDF、查询 link、配置固定基座或浮动基座 | [机器人模型与 URDF](user-guide/robot-model-and-urdf.md) |
| 运动学 | 位姿、雅可比矩阵、雅可比时间导数、空间速度与加速度 | [运动学](user-guide/kinematics.md) |
| 逆运动学 | 使用阻尼最小二乘法求解目标位姿 | [运动学](user-guide/kinematics.md#逆运动学) |
| 动力学 | 质量矩阵、速度乘积力、重力项与逆动力学 | [动力学](user-guide/dynamics.md) |
| 载荷 | 向 link 施加外部 wrench | [外部载荷](user-guide/external-loads.md) |

在解释数值结果前，请先阅读[参考系与空间向量](user-guide/frames-and-spatial-vectors.md)，
了解各语言接口共同使用的顺序、矩阵布局、单位和参考系约定。

## 选择接口

| 接口 | 适用场景 | API 风格 |
|---|---|---|
| [Rust](languages/rust.md) | 原生 Rust 应用和显式内存控制 | `Robot` 方法，显式复用 `Workspace` |
| [Python](languages/python.md) | 科研、脚本和快速原型 | `Robot` 方法，内部持有 workspace |
| [C++](languages/cpp.md) | 需要 RAII 和异常处理的 C++ 应用 | 只可移动的 `dynibo::Robot` 封装 |
| [C](languages/c.md) | 稳定 ABI 和其他语言集成 | opaque handle 与 `dynibo_*` 函数 |

四种接口遵循各自语言的惯用表达，同时保留相同的概念和操作。[API 对照](languages/api-mapping.md)
并排展示了对应的名称、所有权规则和错误模型。

## 浏览文档

- [安装](getting-started/installation.md) — 选择并安装适合的 package。
- [快速上手](getting-started/quick-start.md) — 分别用 Rust、Python、C++ 或 C 完成第一次计算。
- [使用指南](user-guide/index.md) — 理解各接口共享的模型、坐标、参考系、Workspace、
  运动学与动力学语义。
- [API 对照](languages/api-mapping.md) — 查看同一操作在不同语言中的写法。
- API 参考 — 浏览 [Python](reference/python.md)、[C++](cpp-api/dynibo_8hpp.md)、
  [C](c-api/dynibo_8h.md) 或 [Rust](https://docs.rs/dynibo)。
- [GitHub 源代码](https://github.com/xiaojie-xue/dynibo) — 查看示例、issue、benchmark
  和开发信息。
