# Dynamic Workspace 性能记录

> 本文保留固定 API 删除前的历史性能对照。当前公共接口仅包含动态 Workspace API。

- 日期：2026-08-03
- CPU：Intel Core i9-14900K
- Rust：rustc 1.97.1（LLVM 22.1.6）
- Profile：`release`，`codegen-units = 1`，fat LTO
- 命令：`cargo bench --features core-bench --bench core -- --quick`

本记录用于观察重构前后趋势，不设置性能评审线。Criterion quick 模式样本很少，结果会受
频率、温度及系统负载影响，不应解释为跨机器或严格统计结论。模型构建、Workspace 创建及
输入/输出 buffer 分配都在计时区间外。

## 固定 API 重构前后

表中使用 Criterion 报告区间的中值。40 关节数据统一换算为 ns。

| 模型 | 操作 | 重构前 | 重构后 | 变化 |
|---|---|---:|---:|---:|
| 4 关节直链 | FK | 72.583 ns | 73.395 ns | +1.12% |
| 4 关节直链 | Jacobian | 90.831 ns | 94.320 ns | +3.84% |
| 4 关节直链 | Acceleration | 107.01 ns | 113.38 ns | +5.95% |
| 4 关节直链 | Gravity | 105.85 ns | 116.41 ns | +9.98% |
| 4 关节直链 | RNEA | 164.66 ns | 181.06 ns | +9.96% |
| 40 关节直链 | FK | 747.13 ns | 754.58 ns | +1.00% |
| 40 关节直链 | Jacobian | 867.44 ns | 884.21 ns | +1.93% |
| 40 关节直链 | Acceleration | 1069.8 ns | 1084.8 ns | +1.40% |
| 40 关节直链 | Gravity | 1120.6 ns | 1177.8 ns | +5.10% |
| 40 关节直链 | RNEA | 1660.1 ns | 1723.1 ns | +3.79% |
| 7 关节双叶树 | FK | 109.69 ns | 111.47 ns | +1.62% |
| 7 关节双叶树 | Jacobian | 130.39 ns | 134.30 ns | +3.00% |
| 7 关节双叶树 | Acceleration | 189.82 ns | 199.54 ns | +5.12% |
| 7 关节双叶树 | Gravity | 167.33 ns | 182.97 ns | +9.35% |
| 7 关节双叶树 | RNEA | 280.81 ns | 296.02 ns | +5.42% |

重构前的 benchmark 未包含 velocity 和 IK，因此这两项只记录重构后的固定/动态对照。

## 动态 API 与重构后固定 API

负数表示动态 API 在本次测量中更快。动态路径复用 Workspace；固定包装每次创建栈上
scratch，并对返回值进行固定类型构造，因此动态路径在这些小模型上不一定更慢。

| 模型 | 操作 | 固定 API | 动态 API | 动态相对固定 |
|---|---|---:|---:|---:|
| 4 关节直链 | FK | 73.395 ns | 73.269 ns | -0.17% |
| 4 关节直链 | Jacobian | 94.320 ns | 84.906 ns | -9.98% |
| 4 关节直链 | Velocity | 111.46 ns | 108.55 ns | -2.61% |
| 4 关节直链 | Acceleration | 113.38 ns | 109.33 ns | -3.57% |
| 4 关节直链 | Gravity | 116.41 ns | 110.84 ns | -4.78% |
| 4 关节直链 | RNEA | 181.06 ns | 174.34 ns | -3.71% |
| 4 关节直链 | IK | 1370.4 ns | 1366.5 ns | -0.28% |
| 40 关节直链 | FK | 754.58 ns | 720.88 ns | -4.47% |
| 40 关节直链 | Jacobian | 884.21 ns | 833.57 ns | -5.73% |
| 40 关节直链 | Velocity | 936.45 ns | 898.67 ns | -4.03% |
| 40 关节直链 | Acceleration | 1084.8 ns | 1051.8 ns | -3.04% |
| 40 关节直链 | Gravity | 1177.8 ns | 1109.7 ns | -5.78% |
| 40 关节直链 | RNEA | 1723.1 ns | 1634.8 ns | -5.12% |
| 7 关节双叶树 | Gravity（双叶载荷） | 182.97 ns | 169.29 ns | -7.48% |
| 7 关节双叶树 | RNEA（双叶载荷） | 296.02 ns | 283.84 ns | -4.11% |

## 分配验证

独立 allocation-count 测试在 Robot/Workspace 创建和一次预热后，对动态 API 的 FK、
Jacobian、velocity、acceleration、gravity、RNEA 和 IK 各重复运行 10 次，记录到的计算期
分配次数为 0。
