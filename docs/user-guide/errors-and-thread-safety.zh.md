# 错误处理与线程安全

不同语言接口使用符合各自习惯的机制表示相同的主要错误类别。

## 错误对应关系

| 情况 | Rust | Python | C++ | C |
|---|---|---|---|---|
| 无效参数、句柄或长度 | `InvalidInput` 类别的 `Error` | `ValueError` | `dynibo::Error` | `DYNIBO_STATUS_INVALID_ARGUMENT` |
| URDF/模型失败 | `Model` 类别的 `Error` | `ModelError` | `dynibo::Error` | `DYNIBO_STATUS_MODEL_ERROR` |
| 数值失败或 IK 不收敛 | `Solver` 类别的 `Error` | `SolverError` | `dynibo::Error` | `DYNIBO_STATUS_SOLVER_ERROR` |
| ABI 边界捕获 panic | 原生 Rust 调用不适用 | `PanicError` | `dynibo::Error` | `DYNIBO_STATUS_PANIC` |

不要依赖错误提示文本做条件判断。Rust 提供 `ErrorCategory`，C 提供稳定的状态码，
Python 提供明确的异常类型，C++ 的 `Error::status()` 则保留对应的 C 状态码。

## C 错误消息

`dynibo_last_error_message()` 返回线程局部的错误字符串。该字符串在同一线程再次调用
可能失败的 dynibo 函数前保持有效，成功调用会将其清空；如需长期保存，请先复制字符串。

## 线程安全规则

- Rust `RobotModel` 不可变，可在线程间无锁共享。
- 没有线程修改固定基座 `Robot` 的基座位姿时，可以安全读取不可变模型信息。
- 每个 Rust `Robot` 或 `FloatingRobot` 都持有一个可变 workspace，计算方法需要可变借用；
  每个并发计算应通过 `fork()` 创建独立实例。
- C 的每个并发计算需要对应类型的独立 workspace：固定基座使用 `DyniboWorkspace`，浮动基座使用
  `DyniboFloatingWorkspace`。
- Python 会串行化同一 `Robot` 或 `FloatingRobot` 上的方法调用；`fork()` 共享模型数据并
  创建独立实例，可并行执行原生计算。
- C++ 封装没有内部锁，每个工作线程应使用独立的 `dynibo::Robot` 或 `dynibo::FloatingRobot`。
- 当其他线程正在使用句柄时，绝不能销毁或移动所属对象。

## 错误恢复

关节状态、广义力、外部载荷和 tool 位姿都必须包含有限数值。输入校验失败时，调用方提供的
计算输出缓冲区保持不变。有限载荷累加溢出或有限输入产生非有限计算结果均属于求解错误；
求解错误后输出缓冲区内容未定义，只有调用成功返回后才能使用计算结果。

参数错误和求解错误不会破坏机器人实例或 workspace 状态，修正输入后即可再次调用。ABI
边界捕获 panic 可防止栈展开穿过跨语言边界，但这表明发生了意外的内部故障；应记录错误
消息与 dynibo 版本，在 C ABI 中继续前必须销毁并重新创建受影响的 workspace。
