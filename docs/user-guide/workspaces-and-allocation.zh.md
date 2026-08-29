# Workspace 与内存分配

运行时尺寸的机器人算法需要临时数组保存变换、速度、复合惯量、求解器步长和遍历路径。
Dynibo 在模型局部的 workspace 中一次性分配这些 buffer，之后重复使用。

## 各语言接口的行为

| 接口 | Workspace 所有权 | 计算输出 |
|---|---|---|
| Rust | 每个 `Robot` 或 `FloatingRobot` 持有一个 workspace | 矩阵和广义力 buffer 由调用方提供 |
| Python | 每个 `Robot` 或 `FloatingRobot` 持有一个原生 workspace | 返回 NumPy 数组或值对象；`out=` 可复用调用方存储 |
| C++ | 每个 `dynibo::Robot` 或 `dynibo::FloatingRobot` 持有一个原生 workspace | 返回 `std::vector` 或值对象 |
| C | 显式 `DyniboWorkspace*` 或 `DyniboFloatingWorkspace*` | buffer 和结构体由调用方提供 |

Rust 和 C 可以直接控制输出内存：

=== "Rust"

    ```rust
    let mut jacobian = vec![0.0; 6 * robot.generalized_count()];
    robot.jacobian(&q, target, &mut jacobian)?;
    ```

=== "C"

    ```c
    DyniboWorkspace *workspace = NULL;
    check(dynibo_workspace_create(robot, &workspace));
    check(dynibo_jacobian(
        robot, workspace, q, J, target, jacobian, 6 * G));
    ```

创建 workspace 时会分配全部内部临时 buffer；复用时不会调整这些 buffer 的尺寸。
Python 可通过 `out=` 复用 NumPy 数组，未提供时才分配结果数组；C++ 会分配语言层的
返回容器。

## 模型作用域

每个 `Robot` 或 `FloatingRobot` 实例持有与其不可变模型绑定的 workspace。`fork()` 会共享模型、创建新的
计算存储。

## 并行计算

每个 Rust `Robot` 或 `FloatingRobot` 都是可变的，同一时刻只能参与一次计算。并行计算时应为每个任务调用
`fork()` 创建实例。Python 会串行化同一个 `Robot` 或 `FloatingRobot` 上的调用，需要并行时应使用独立 robot。
C++ 不提供内部锁，因此每个 worker 应使用独立 `Robot` 或 `FloatingRobot`。
