# 测试架构

`tests/support` 下的集成测试支持代码提供四项共享能力：

- 可通过 seed 精确定位、可复现的 URDF 模型生成；
- 确定性的关节状态与浮动基座状态；
- 带完整 case 上下文的绝对误差加相对误差数值断言；
- 算法矩阵与 workspace 序列执行器。

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

Workspace 序列测试会逐步比较复用同一个 `Robot` 的每个操作与新建 `fork()` 上的相同操作。
固定基与浮动基序列分开测试，因为 base mode 是模型属性。无效长度和 foreign-link 操作会与
成功计算交错执行，以验证错误恢复和 scratch buffer 清理。

内存分配测试单独维护，因为它们使用进程全局 allocator。已安装的 C、C++、Python 包测试也
保持黑盒测试，不复用 Rust 测试辅助代码。
