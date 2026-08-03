# 动态 Slice Kernel 与 Workspace 开发计划

- 状态：已实施，最终接口调整为仅保留动态 API
- 日期：2026-07-30
- 完成日期：2026-08-03

> 最终决策（2026-08-03）：删除固定尺寸公共计算 API、`JointVector<N>`、
> `Jacobian<N>` 和借用式 `Load`，只发布 `LinkId + Workspace` 动态 API。动态方法沿用原
> 固定 API 的名称，不使用 `_slice` 或 `_into` 后缀。
> 本文后续关于“保留固定 API”的内容用于记录重构过程，不再描述当前公共接口；当前用法
> 以仓库 README 为准。

## 背景

Dyno 当前使用 const generics 表达关节空间的固定尺寸：
`JointVector<N>`、`Jacobian<N>` 和计算过程中的 `[T; N]` 都由编译期常量
`N` 决定。这让已知关节数量的 Rust 调用能够使用栈上工作数组，但不适合直接提供给
Python、C++ FFI 或运行时加载任意 URDF 的 Rust 程序，因为这些调用方只在运行时知道
关节数量。

本计划把运动学和动力学实现重构为共享的 slice kernel：kernel 接收运行时长度的输入、
输出及 scratch slice，不负责分配内存。现有固定尺寸 API 使用栈数组调用 kernel；新增
动态 API 使用预分配的 `Workspace` 调用同一个 kernel。

```text
现有固定尺寸 Rust API ── 栈数组 ──────┐
                                      ├── 共享 slice kernel
新增动态 Rust/FFI API ──── Workspace ──┘
```

Python 和 C++ 绑定不属于本计划，但后续应只依赖这里建立的动态 API，不能复制核心
机器人学算法。

## 目标

- 不破坏现有 Rust 固定尺寸公开 API。
- 固定 API 和动态 API 共用同一套运动学、动力学及 IK 核心算法。
- 固定 API 继续使用栈上工作数组，保持计算路径无堆分配。
- 动态 API 只在创建 `Workspace` 时分配；后续计算不分配和调整容量。
- 支持运行时确定的任意合法模型关节数，不增加人为的最大关节数。
- 保持现有数值、错误、关节顺序、fixed joint 和动力学兼容约定。
- 保持 `Robot` 只读且可共享；并发计算由每个调用方持有独立 Workspace。
- 测量并记录固定与动态 API 的性能变化，不设置性能评审线。

## 非目标

本阶段不包含：

- Python/PyO3 绑定；
- C ABI、C++ RAII wrapper、CMake 或 Conan 包；
- crates.io、PyPI 或其他注册表发布；
- fixed joint 是否占用关节向量元素的语义变更；
- 重力方向、惯量积符号、空间向量顺序等兼容行为变更；
- “只计算目标祖先”等算法优化；
- 使用 `unsafe` 绕过初始化或边界检查；
- 将 Workspace 隐藏在 Robot 内部并用 mutex 串行化调用。

## 设计原则

### Kernel 只计算，不拥有内存

当前 `link_frames` 同时创建并返回 `[Frame; N]`。目标形式是由调用方提供内存：

```rust
fn link_frames_kernel(
    &self,
    q: &[f64],
    frames: &mut [Frame],
) -> Result<()>;
```

函数入口验证所有 slice 长度，热点循环统一使用同一个 `0..n` 范围，以便编译器消除
重复 bounds check。kernel 不允许创建 `Vec`、调整容量或复制完整输入输出。

### 固定 API 保持现有外观

现有调用不变：

```rust
let q = JointVector::<7>::zeros();
let frame = robot.forward_kinematics(&q, target)?;
let jacobian = robot.jacobian(&q, target)?;
```

固定 API 作为薄包装，创建 `[T; N]` 或固定尺寸 nalgebra 类型，将其转换为 slice 后
调用共享 kernel。

### 动态 API 显式持有 Workspace

暂定使用方式：

```rust
let target = robot.link_id("tool")?;
let mut workspace = robot.workspace();
let q = vec![0.0; robot.joint_count()];
let mut jacobian = vec![0.0; 6 * robot.joint_count()];

let frame = robot.forward_kinematics(&q, target, &mut workspace)?;
robot.jacobian(&q, target, &mut workspace, &mut jacobian)?;
```

原提案使用 `slice` 表示运行时尺寸输入，使用 `into` 表示输出内存由调用方提供；最终在
删除固定 API 后沿用原方法名，不再需要后缀区分。

### Workspace 与模型绑定

Workspace 保存创建它的 `model_id` 和 `joint_count`。每个动态入口必须验证：

- Workspace 属于当前 Robot；
- Workspace 的关节数等于当前 Robot；
- 输入和输出 slice 长度正确；
- Link ID 和所有 Load 都属于当前 Robot。

不匹配时返回错误，不能在计算入口自动 `resize`，否则会破坏“创建后无分配”的保证。

### 输出布局保持一致

动态 Jacobian 使用 `6 x N` column-major 布局，每个关节的一列连续存储，顺序保持
`[angular_x, angular_y, angular_z, linear_x, linear_y, linear_z]`。这与 nalgebra 和
Eigen 的默认布局一致，也允许固定 API 直接传入 `Jacobian<N>::as_mut_slice()`，避免
转置或完整复制。

## 初步公共类型

新增类型的字段全部私有：

```rust
pub struct Workspace {
    model_id: u64,
    joint_count: usize,
    frames: Vec<Frame>,
    angular_velocities: Vec<Vector3<f64>>,
    angular_accelerations: Vec<Vector3<f64>>,
    origin_accelerations: Vec<Vector3<f64>>,
    link_accelerations: Vec<Vector3<f64>>,
    link_loads: Vec<Wrench>,
    jacobian: Vec<f64>,
    q_work: Vec<f64>,
    step: Vec<f64>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LinkId {
    model_id: u64,
    index: usize,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct IndexedLoad {
    pub link: LinkId,
    pub wrench: Wrench,
}
```

初版优先使用有语义的 Workspace 字段，而不是把多组 `Vector3` buffer 合并为
`vec3_a`、`vec3_b`。对于常见机器人，额外内存很小；清晰的字段可以降低动力学递推中
误用 scratch buffer 的风险。获得 benchmark 和内存数据后，仍可在不影响公共 API 的
情况下修改私有布局。

`LinkId` 是不透明、可复制的模型内 handle。现有接收 `&Link` 的 API 保持不变；动态
API 使用 `LinkId`，避免让后续 FFI 暴露 Rust 引用和 lifetime。

## 初步代码结构

```text
src/
├── robot.rs
└── robot/
    ├── workspace.rs
    └── kernel/
        ├── mod.rs
        ├── kinematics.rs
        ├── dynamics.rs
        └── ik.rs
```

- `robot.rs`：`Robot`、现有固定 API、新增动态 API 和公开参数验证；
- `workspace.rs`：`Workspace`、`LinkId`、`IndexedLoad` 和归属验证；
- `kinematics.rs`：FK、Jacobian、速度及加速度 kernel；
- `dynamics.rs`：gravity 和 RNEA kernel；
- `ik.rs`：IK 迭代及固定 `6 x 6` 线性系统。

第一批变更可以先把 kernel 保留在 `robot.rs`，架构经过 FK/Jacobian 验证后再移动，避免
第一步同时产生大量代码移动和行为变更。

## 实施阶段

### 阶段 0：建立性能与分配基线

预计 0.5 至 1 天。

- 正式运行现有 FK、Jacobian、速度、加速度、gravity、RNEA 和 IK benchmark；
- 模型构建必须在计时区间外；
- 使用固定输入和迭代条件记录 IK；
- 记录 Rust、编译选项、CPU 和 benchmark 命令；
- 增加独立的 allocation-count 测试目标；
- 保存重构前的数值及性能结果。

性能数据不作为共享 CI 或人工评审门禁。CI 继续执行正确性、格式和 lint 检查。

验收条件：不改变核心行为，并获得可重复的重构前基线。

### 阶段 1：FK slice kernel 与 Workspace 骨架

预计 1 天。

将当前泛型 `link_frames` 改为：

```rust
fn link_frames_kernel(
    &self,
    q: &[f64],
    frames: &mut [Frame],
) -> Result<()>;
```

- 固定 FK 创建 `[Frame; N]` 后调用 kernel；
- 增加只包含模型标识和 frames 的最小 Workspace；
- 增加 `LinkId` 查找和验证；
- 增加动态 FK；
- 对会完整覆盖的 `frames` 禁止每次预清零。

验收条件：

- 现有 FK 测试不变并通过；
- 固定和动态结果一致；
- 根 link、错误长度、错误模型 LinkId 和错误 Workspace 均有测试；
- Workspace 重复调用不存在残留污染；
- 记录固定 FK 重构前后的数据及动态 FK 相对固定 API 的数据。

FK 是架构验证点。如果不能达到性能和可维护性目标，暂停后续迁移并调整设计。

### 阶段 2：Jacobian 与速度 kernel

预计 1 至 2 天。

拆分为：

```rust
fn jacobian_kernel(
    &self,
    frames: &[Frame],
    target_index: usize,
    output: &mut [f64],
) -> Result<Frame>;
```

- kernel 写入 `6 * N` column-major 输出；
- 每次计算先清零 Jacobian，保证非祖先列为零；
- 固定 API 直接使用 `Jacobian<N>::as_mut_slice()`；
- 动态 API 使用调用方提供的输出 buffer；
- 速度运动学复用 frames 和 Jacobian kernel，动态版本使用 Workspace 内部 Jacobian。

验收条件：

- 现有有限差分和树模型测试通过；
- 非祖先列保持为零；
- 固定与动态结果一致；
- 连续切换 target 不存在残留列；
- 记录固定 API 重构前后的数据及动态 API 相对固定 API 的数据。

### 阶段 3：加速度运动学 kernel

预计 1 天。

使用内部 scratch view 避免过长参数列表：

```rust
struct AccelerationScratch<'a> {
    frames: &'a mut [Frame],
    angular_velocities: &'a mut [Vector3<f64>],
    angular_accelerations: &'a mut [Vector3<f64>],
    linear_accelerations: &'a mut [Vector3<f64>],
}
```

kernel 接收 `q`、`qd`、`qdd` slice、目标索引及 scratch view。所有输入分别验证长度；
根 link 继续返回零 Twist。

验收条件：现有解析与有限差分测试通过，固定与动态结果一致，并记录性能变化。

### 阶段 4：Gravity kernel

预计 1 至 2 天。

引入内部 scratch view：

```rust
struct GravityScratch<'a> {
    transforms: &'a mut [Frame],
    gravity_at_link: &'a mut [Vector3<f64>],
    link_loads: &'a mut [Wrench],
}
```

- 正向和反向递推进入唯一的 slice kernel；
- 现有 `Load<'a>` 与动态 `IndexedLoad` 分别验证后写入同一个 `link_loads`；
- 不创建临时 `Vec<IndexedLoad>`，以免固定 API 因适配发生分配；
- 每次调用清零 `link_loads` 和输出；
- 保持根 link 外载荷和现有动力学约定。

验收条件覆盖空载荷、重复 link 载荷、多分支载荷、错误模型载荷和连续调用残留；固定 API
记录固定与动态 API 的性能变化。

### 阶段 5：Inverse Dynamics kernel

预计 2 天。

引入：

```rust
struct DynamicsScratch<'a> {
    transforms: &'a mut [Frame],
    angular_velocities: &'a mut [Vector3<f64>],
    angular_accelerations: &'a mut [Vector3<f64>],
    origin_accelerations: &'a mut [Vector3<f64>],
    link_accelerations: &'a mut [Vector3<f64>],
    link_loads: &'a mut [Wrench],
}
```

正向速度/加速度递推和反向 wrench 递推进入同一个 kernel。保持现有正 Z 重力兼容约定、
惯量积符号、根 link 行为、fixed joint 输出及外载荷坐标系约定。

验收条件：

- 现有 RNEA 回归测试全部通过；
- Pinocchio 可用时通过完整输出对照测试；
- 固定与动态输出一致；
- 连续调用不存在载荷残留；
- 记录固定与动态 API 的性能变化。

### 阶段 6：IK kernel

预计 2 至 3 天，是性能风险最高的阶段。

Workspace 增加 `jacobian`、`q_work` 和 `step`。`J J^T` 的结果始终为 `6 x 6`，继续
使用固定尺寸 `SMatrix<f64, 6, 6>`；只在关节维度上使用动态 slice 循环。`J^T e` 按每个
关节的连续 6 元素列计算，避免动态矩阵分配。

先尝试让固定和动态 API 完全调用同一个 IK kernel。如果测量显示固定 IK 明显变慢，允许
固定 API 保留 nalgebra 固定矩阵的线性代数包装，动态 API 使用 slice 循环；FK、
Jacobian、误差计算、收敛条件和错误处理仍必须共享。

验收条件：

- 收敛状态、迭代次数、结果及残差与当前实现一致；
- `InvalidOptions`、`NonFiniteInput`、`NumericalFailure`、
  `JointLimitViolation` 和 `NotConverged` 行为保持；
- Workspace 创建后，每轮及完整求解不分配；
- 记录固定与动态 IK 的性能变化。

### 阶段 7：API 整理与 FFI 准备

预计 1 至 2 天。

- 确定动态 API、Workspace 和 Load 类型的最终命名；
- 完成公开文档和动态 Rust 示例；
- 文档化并发规则、无分配范围、输出尺寸和 Jacobian 布局；
- 保证 Workspace 私有布局不进入兼容性承诺；
- 形成后续 C ABI 能直接包装的公开调用边界；
- 执行完整格式、lint、测试和正式 benchmark。

## 测试计划

### 固定与动态数值一致性

每个新增动态入口都必须在相同模型和输入下与固定 API 比较，覆盖：

- 直链和树形模型；
- revolute、prismatic 和 fixed joint；
- 根 link 和多个叶 link；
- 空载荷、同 link 多载荷和跨分支多载荷；
- 合法、错误长度及非有限输入；
- 当前模型验证允许时的零关节纯根模型。

### Workspace 残留

交替运行：

```text
输入 A + 载荷 A
输入 B + 空载荷
输入 A + 载荷 A
```

第一次和第三次结果必须一致，第二次不得包含 A 的状态。重点检查 Jacobian 非祖先列、
`link_loads`、IK step 及树模型左右分支。

### 模型归属

测试 Robot A 与 Workspace B、Robot A 与 LinkId B，以及 Robot clone 的预期行为。

### 分配计数

在独立测试目标中完成 Robot/Workspace 创建和一次预热后，开始计数并重复运行计算。
FK、Jacobian into、加速度、gravity into、RNEA into 和 IK 的计算期分配次数必须为零。
现有固定 API 的无分配保证也继续验证。

## 性能测量

不设置性能通过或失败阈值。每个阶段记录固定 API 重构前后的变化，以及动态 API 相对固定
API 的开销；数据用于发现问题和指导优化，不作为合并门禁。出现明显变化时按以下顺序排查：

1. 计算期是否出现堆分配或容量调整；
2. 是否清零了本应完整覆盖的 buffer；
3. 是否发生 Jacobian 转置或布局转换；
4. 热点循环是否保留重复 bounds check；
5. kernel 是否未被内联；
6. IK 是否丢失固定 `6 x 6` 优化；
7. 是否重复验证或复制完整输入输出；
8. 是否引入锁或 trait object 动态分派。

不能为追求性能引入 `unsafe`。如果安全 slice kernel 的测量结果不理想，应先评估 API 和代码
结构上的折中。

## 建议 PR 拆分

1. `bench: establish fixed-api and allocation baselines`
2. `refactor: share slice kernel for forward kinematics`
3. `refactor: share slice kernel for jacobian and velocity`
4. `refactor: add slice kernel for acceleration kinematics`
5. `refactor: share workspace kernels for gravity and RNEA`
6. `refactor: add workspace-based inverse kinematics`
7. `docs: finalize dynamic workspace API and guarantees`

每个 PR 必须：

- 保持现有固定 API 和测试可用；
- 增加对应的固定/动态数值一致性测试；
- 增加错误与 Workspace 残留测试；
- 提供重构前后 benchmark；
- 不混入无关格式化、算法优化或兼容行为修改。

## 风险

| 风险 | 影响 | 应对 |
|---|---|---|
| slice bounds check 未消除 | 固定和动态 API 变慢 | 入口统一验证，循环统一使用 `0..n` |
| Workspace 数据残留 | 第二次调用数值错误 | 明确覆盖/清零规则并增加交替输入测试 |
| IK 动态循环变慢 | 多次迭代累计回退 | 保留固定 `6 x 6`，必要时区分线性代数包装 |
| Jacobian 布局转换 | 产生完整复制 | 核心统一使用 column-major |
| Workspace 被跨模型使用 | 错误结果或越界 | `model_id` 和关节数双重验证 |
| Robot 内共享 Workspace | 并发调用串行化 | Workspace 由调用方独占 |
| 单个变更过大 | 难以定位数值或性能回归 | 按算法拆分、逐阶段验收 |
| 动态 API 过早固化 | 后续 FFI 使用不便 | FK 阶段先评审命名、布局和错误模型 |

## 工作量预估

| 阶段 | 预计时间 |
|---|---:|
| 基线和 allocation benchmark | 0.5～1 天 |
| FK 与 Workspace 骨架 | 1 天 |
| Jacobian 与 velocity | 1～2 天 |
| Acceleration | 1 天 |
| Gravity 与 RNEA | 2～3 天 |
| IK | 2～3 天 |
| 文档、分配验证和性能调优 | 1～2 天 |
| 合计 | 8～13 个工作日 |

如果 bounds check 或 IK 出现明显回退，额外预留 2 至 4 天 profiling 和针对性优化。

## 已确认决策

1. 使用统一 `Workspace`，还是拆成 `KinematicsWorkspace` 与
   `DynamicsWorkspace`：使用统一 Workspace。
2. 将字段私有的 `LinkId` 作为正式公开类型。
3. Jacobian 正式规定为 column-major，保持 nalgebra/Eigen 默认布局。
4. 删除固定 API 后，动态 API 沿用 `forward_kinematics`、`jacobian`、`gravity` 等原名称。
5. 性能只测量和记录，不设置人工评审线。
6. 动态 API 直接作为 0.x 正式公共接口发布，不标记为实验性接口。
