# Workspace 与内存分配

运行时尺寸的机器人算法需要临时数组保存变换、速度、复合惯量、求解器步长和遍历路径。
Dynibo 在模型局部的 workspace 中一次性分配这些 buffer，之后重复使用。

## 各语言接口的行为

| 接口 | Workspace 所有权 | 计算输出 |
|---|---|---|
| Rust | 每个 `Robot` 或 `FloatingRobot` 持有一个 workspace | 矩阵和广义力 buffer 由调用方提供 |
| Python | 每个 `Robot` 或 `FloatingRobot` 持有一个原生 workspace | 返回 NumPy 数组或值对象；`out=` 可复用调用方存储 |
| C++ | 每个 `dynibo::Robot` 或 `dynibo::FloatingRobot` 持有一个原生 workspace | 返回 `std::vector`，或使用 `*_into` 复用输出 vector |
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
Python 可通过 `out=` 复用 NumPy 数组，未提供时才分配结果数组。`LoadBuffer` 还能避免
每次调用重建原生载荷 vector，但 Python 对象处理本身不承诺零分配。
C++ 返回容器的方法会分配内存；`jacobian_into`、`jacobian_derivative_into`、
`mass_matrix_into`、`velocity_product_forces_into`、`gravity_into`、
`inverse_dynamics_into`、`forward_dynamics_into` 及固定基的 `inverse_kinematics_into`
接受尺寸精确匹配的输出 vector，并且不会调整其容量。输出位于必填输入之后、可选载荷
或 IK 选项之前，例如 `robot.inverse_dynamics_into(q, qd, qdd, output, loads)`。

## 模型作用域

每个 `Robot` 或 `FloatingRobot` 实例持有与其不可变模型绑定的 workspace。`fork()` 会共享模型、创建新的
计算存储。

Rust 另提供不含 workspace 的只读 `RobotModel`。C 模型句柄采用该表示，只有显式
创建的 C workspace 分配计算 buffer。Python 的 `robot.fork()` 共享模型和 link-ID
表，同时创建独立 workspace 和锁。关闭任一实例不影响其他实例；固定基 fork 初始
复制基座位姿，后续修改任一实例的基座位姿都不会影响另一个。

## 并行计算

每个 Rust `Robot` 或 `FloatingRobot` 都是可变的，同一时刻只能参与一次计算。并行计算时应为每个任务调用
`fork()` 创建实例。Python 会串行化同一个 `Robot` 或 `FloatingRobot` 上的调用，
需要并行时可使用 `fork()`，避免重复解析 URDF 或复制模型。
C++ 不提供内部锁，因此每个 worker 应使用独立 `Robot` 或 `FloatingRobot`。
