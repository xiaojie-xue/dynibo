# 测试架构

`tests/support` 下的集成测试支持代码提供四项共享能力：

- 可通过 seed 精确定位、可复现的 URDF 模型生成；
- 确定性的关节状态与浮动基座状态；
- 带完整 case 上下文的绝对误差加相对误差数值断言；
- 算法矩阵与 workspace 序列执行器。

`robot_arm` 和 `pinocchio_oracle` 集成测试保留根目录下的 `.rs` 入口，
具体用例按职责放在同名子目录中，每套测试仍只生成一个可执行文件。
C ABI 单元测试在 `bindings/c/src/tests.rs` 中共享辅助函数，计算、缓冲区重叠检查和
所有权检查分别放在 `bindings/c/src/tests/` 的子模块中。

`support/model_gen/` 分别组织模型规格与校验、确定性生成、语料选择、临时文件和
URDF 序列化；`support/pinocchio/` 分开桥接声明与计算封装。
Python 安装包测试仍通过 `python/test_package.py` 运行，测试类与共享 fixture 位于
`python/package_cases/`。分配测试继续作为独立目标，分别使用自己的全局分配器。

PR 使用的生成模型语料包含 24 个可复现的伪随机 `u64` seed，每个模型配合八组状态。
带版本号的 `ModelSpec` 将显式的 24-case 结构覆盖计划与随机物理参数分开。该计划让固定基和
浮动基分别覆盖串联、单分支、平衡、宽树和非平衡树；同时覆盖无 fixed joint、交错 fixed joint、
连续 fixed joint 与 tool-frame fixed joint。模型覆盖 revolute、continuous、prismatic 关节，标准轴
与非轴对齐轴，以及 identity、带偏移、带旋转、同时带偏移和旋转的物理惯性坐标系。惯性参数始终
保持在正常且数值条件良好的物理范围内。

运行默认测试套件：

```bash
cargo test --workspace --all-targets --locked
```

当 `pkg-config` 能找到 Pinocchio 时，`pinocchio-tests` feature 会增加两层独立 oracle。
`pinocchio_oracle` 覆盖长期维护的串联、混合关节、分叉树和 free-flyer fixture，包括 RNEA、ABA
的单 link 与多 link 外载荷；`generated_pinocchio` 则让同一套 24 个模型、每个八组状态分别对比
FK、速度和加速度运动学、Jacobian 与其导数、质量矩阵、重力、速度乘积力、RNEA 和 ABA：

```bash
cargo test -p dynibo --locked --features pinocchio-tests --tests
```

生成模型 conformance 测试接受下文记录的复现与语料规模环境变量。

复现一个生成模型：

```bash
DYNIBO_TEST_SEED=0x1ea59f2878e51fb4 DYNIBO_TEST_CASE_ID=6 \
  cargo test --test generated_conformance -- --nocapture
```

在本地运行更大的语料：

```bash
DYNIBO_TEST_CASES=512 \
  cargo test --test generated_conformance --release -- --nocapture
```

使用操作系统随机源生成一套新的探索语料：

```bash
DYNIBO_TEST_RANDOMIZE=1 DYNIBO_TEST_CASES=512 \
  cargo test --test generated_conformance --release -- --nocapture
```

测试会输出 `master_seed`；使用
`DYNIBO_TEST_RANDOMIZE=1 DYNIBO_TEST_MASTER_SEED=...` 可重跑同一套探索语料。单个失败会报告
case 索引，并可通过 `DYNIBO_TEST_SEED` 与 `DYNIBO_TEST_CASE_ID` 重放。

设置 `DYNIBO_TEST_KEEP_URDF=1` 会将生成的 fixture 保留在系统临时目录，并输出路径以便检查。

每个生成 case 的失败信息都会包含 seed、sample、base mode、算法、目标 link 与 load case。
发生 panic 展开时，模型 URDF、`ModelSpec` 和复现命令会保留在 `target/test-failures` 下。使用
`DYNIBO_TEST_SEED` 与 `DYNIBO_TEST_CASE_ID` 可复现这些模型。生成器带版本号，因此在同一个
生成器版本内，一个 seed 始终对应同一个 URDF。

Workspace 序列测试会逐步比较复用同一个 `Robot` 或 `FloatingRobot` 的每个操作与新建
`fork()` 上的相同操作。两个类型化 runner 分别覆盖固定基与浮动基行为。无效长度和
foreign-link 操作会与成功计算交错执行，以验证错误恢复和 scratch buffer 清理。

内存分配测试单独维护，因为它们使用进程全局 allocator。已安装的 C、C++、Python 包测试也
保持黑盒测试，不复用 Rust 测试辅助代码。它们共同读取带版本号的
`tests/data/pinocchio_reference_v1.tsv`；启用 feature 的 Pinocchio oracle 会先验证这份提交到
仓库中的参考数据，再由各语言包测试复用。

Python 包测试还覆盖旋转浮动基座下关节静止和关节运动两种状态，对照经 Pinocchio 校验的
参考数据检查完整雅可比导数、速度、加速度、工具点速度和速度乘积力，并通过运动学恒等式
检查广义坐标顺序和矩阵列主序布局。固定基与浮动基对象均测试构造器、生命周期错误、
NumPy 输入布局、输出缓冲区复用，以及非法调用后的恢复。含非有限数值的载荷必须在写入
输出缓冲区前抛出 `ValueError`。

## Rust 实现的联合测试覆盖率

`ci/collect-coverage.sh` 合并 Rust workspace 单元测试、集成测试和可执行 example
测试，以及安装后的 Python wheel 测试调用 Rust 扩展所产生的 profile。
统计对象为 Rust 核心及 C/Python 绑定的生产实现，不是纯 Rust 单元测试覆盖率，
也不统计 Python 源码或独立 C/C++ 测试。独立的 `pinocchio-tests` CI 任务不合并至此报告。

测试模块和仅供测试使用的辅助函数通过条件 `coverage(off)` 属性退出统计，但测试
继续执行，其调用的生产代码仍计入覆盖。测试文件、examples、benchmarks 和构建脚本
不进入报告；生产代码中的参数校验、panic 处理和数值保护仍保留统计。
LLVM 与 Codecov 导出使用相同的文件过滤规则。

CI 门槛依据 `coverage.json` 中 LLVM 的行覆盖率和分支覆盖率。
`codecov.json` 是上传格式，本地计算其完整命中条目比例，既不是 LLVM 分支率，
也不是 Codecov 服务端最终结果，应分别展示。暂时保持行覆盖率 85%、分支覆盖率
75% 的门槛。调整统计范围后建立新基线，不能将百分比变化直接当作测试改进。

本地与 CI 共用 `ci/coverage.env` 中精确的 Rust、Python 和 cargo-llvm-cov 版本，
Python 依赖固定在 `ci/coverage-requirements.txt`；其他 CI 任务的工具链不变。
Linux 环境安装及运行命令见 [英文说明](TESTING.md#rust-implementation-joint-test-coverage)。
脚本会在收集前拒绝 Python、依赖或 cargo-llvm-cov 版本不匹配的环境。
更新固定版本时需重新建立基线，尤其 nightly 的分支插桩可能改变统计分母。

2026-10-10 在 Linux x86_64 使用上述固定环境测得的新基线：117 个 Rust 测试、
24 个安装后 Python wheel 测试通过；Rust 生产实现行覆盖率为 5,046/5,132
（98.32%），分支覆盖率为 315/334（94.31%）。该基线排除了测试实现，不能与旧口径
直接比较；这是本地测量结果，不是 Codecov 服务端结果。
