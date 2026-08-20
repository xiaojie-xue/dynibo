# 错误处理与线程安全

不同语言接口使用符合各自习惯的机制表示相同的主要错误类别。

## 错误对应关系

| 情况 | Rust | Python | C++ | C |
|---|---|---|---|---|
| 无效参数、handle 或长度 | `InvalidInput` 类别的 `Error` | `ValueError` | `dynibo::Error` | `DYNIBO_STATUS_INVALID_ARGUMENT` |
| URDF/模型失败 | `Model` 类别的 `Error` | `ModelError` | `dynibo::Error` | `DYNIBO_STATUS_MODEL_ERROR` |
| IK 数值失败或不收敛 | `Solver` 类别的 `Error` | `SolverError` | `dynibo::Error` | `DYNIBO_STATUS_SOLVER_ERROR` |
| ABI 边界捕获 panic | 原生 Rust 调用不适用 | `PanicError` | `dynibo::Error` | `DYNIBO_STATUS_PANIC` |

不要根据供人阅读的错误文本进行程序分支。Rust 提供 `ErrorCategory`，C 提供稳定状态值，
Python 提供异常类型，C++ 的 `Error::status()` 会保留 C 状态。

## C 错误消息

`dynibo_last_error_message()` 返回线程局部字符串。它在同一线程下一次可能失败的 dynibo
调用之前有效；成功调用会将其清空。如果消息需要跨越下一次调用，应复制字符串。

## 线程安全规则

- 没有线程修改基座状态时，可以读取不可变模型信息。
- 一个可变 workspace 不能同时参与多个计算。
- Rust 和 C 的每个并发计算应分配独立 workspace。
- Python 会串行化同一 `Robot` 的方法；独立实例可以执行并行原生调用。
- C++ wrapper 没有内部锁；每个 worker 应使用独立 `dynibo::Robot`。
- 其他线程使用 handle 时，绝不能销毁或移动所属对象。

## 错误恢复

参数错误和求解器错误不会使 robot 或 workspace 失效，修正输入后可以再次调用。ABI
边界捕获 panic 可以防止 unwinding 穿过外部语言边界，但它表示意外的内部失败；决定
是否继续前，应记录错误消息和 dynibo 版本。
