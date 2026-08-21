# Workspace 与内存分配

运行时尺寸的机器人算法需要临时数组保存变换、速度、复合惯量、求解器步长和遍历路径。
Dynibo 在模型局部的 workspace 中一次性分配这些 buffer，之后重复使用。

## 各语言接口的行为

| 接口 | Workspace 所有权 | 计算输出 |
|---|---|---|
| Rust | 由 `robot.workspace()` 创建并显式传入 | 矩阵和广义力 buffer 由调用方提供 |
| Python | 每个 `Robot` 持有一个原生 workspace | 返回 Python tuple 或值对象 |
| C++ | 每个 `dynibo::Robot` 持有一个原生 workspace | 返回 `std::vector` 或值对象 |
| C | 显式 `DyniboWorkspace*` | buffer 和结构体由调用方提供 |

Rust 和 C 可以直接控制输出内存：

=== "Rust"

    ```rust
    let mut workspace = robot.workspace();
    let mut jacobian = vec![0.0; 6 * robot.generalized_count()];
    robot.jacobian(&q, target, &mut workspace, &mut jacobian)?;
    ```

=== "C"

    ```c
    DyniboWorkspace *workspace = NULL;
    check(dynibo_workspace_create(robot, &workspace));
    check(dynibo_jacobian(
        robot, workspace, q, J, target, jacobian, 6 * G));
    ```

创建 workspace 时会分配全部内部临时 buffer；复用时不会调整这些 buffer 的尺寸。
Python 和 C++ 对矩阵等结果仍会分配语言层的返回容器。

## 模型作用域

Workspace 只属于创建它的模型，也可以用于该模型在 Rust 中的 clone。即使两个模型的
关节数量相同，把一个模型的 workspace 传给另一个模型仍然会报错。

## 并行计算

Workspace 是可变的，同一时刻只能参与一次计算。Rust 或 C 的每个并发调用应使用独立
workspace。Python 会串行化同一个 `Robot` 上的调用，需要并行时应使用独立 robot。
C++ 不提供内部锁，因此每个 worker 应使用独立 `Robot`。
