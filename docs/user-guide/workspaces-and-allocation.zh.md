# Workspace 与内存分配

运行时尺寸的机器人算法需要临时数组来保存变换、速度、复合惯量、求解器步长和遍历路径。
Dynibo 在绑定到所属模型的 workspace 中一次性分配这些缓冲区，之后重复复用。

## 各语言接口的行为

| 接口 | Workspace 所有权 | 计算输出 |
|---|---|---|
| Rust | 每个 `Robot` 或 `FloatingRobot` 持有一个 workspace | 矩阵和广义力缓冲区由调用方提供 |
| Python | 每个 `Robot` 或 `FloatingRobot` 持有一个原生 workspace | 返回 NumPy 数组或值对象；`out=` 可复用调用方缓冲区 |
| C++ | 每个 `dynibo::Robot` 或 `dynibo::FloatingRobot` 持有一个原生 workspace | 返回 `std::vector`，或使用 `*_into` 复用输出缓冲区 |
| C | 显式 `DyniboWorkspace*` 或 `DyniboFloatingWorkspace*` | 缓冲区和结构体由调用方提供 |

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

创建 workspace 时会一次性分配全部内部临时缓冲区，复用时不会调整它们的容量。不同语言在
输出和载荷上的内存分配行为如下：

- **Rust 与 C**：输出缓冲区由调用方持有，可在计算循环中实现完全零分配。
- **Python**：通过 `out=` 传入预分配的 NumPy 数组可避免每次调用创建新数组；使用
  `LoadBuffer` 可避免反复重建原生载荷存储（但 Python 对象本身的处理不承诺严格零分配）。
- **C++**：返回容器的方法会分配 `std::vector`；而 `*_into` 重载——包括
  `forward_kinematics_all_into`、`jacobian_into`、`jacobian_derivative_into`、
  `mass_matrix_into`、`velocity_product_forces_into`、`gravity_into`、
  `inverse_dynamics_into`、`forward_dynamics_into` 以及固定基座的
  `inverse_kinematics_into`——会直接写入尺寸精确匹配的输出向量，绝不重新分配或调整容量。
  输出参数位于必填输入之后、可选载荷或 IK 选项之前，例如
  `robot.inverse_dynamics_into(q, qd, qdd, output, loads)`。

## 状态快照复用（Rust）

在同一个控制步的 `(q, qd)` 状态下连续查询多项运动学和动力学量时，Rust 的
`robot.state(&q, &qd)`（以及 `floating_robot.state(&base, &q, &qd)`）会返回一个借用
实例预分配缓存的 `RobotState` 或 `FloatingRobotState`。关节变换、link 位姿和 link
速度只计算一次，随后即可零分配复用于 `forward_kinematics`、`forward_kinematics_all`、
`forward_velocity_kinematics`、`forward_acceleration_kinematics`、`jacobian`、
`jacobian_derivative`、`mass_matrix`、`gravity`、`velocity_product_forces`、
`inverse_dynamics` 和 `forward_dynamics`。

## 模型作用域

每个 `Robot` 或 `FloatingRobot` 实例持有绑定到其不可变模型的 workspace。`fork()`
会共享该模型，同时创建独立的计算存储。

Rust 另提供不含 workspace 的只读 `RobotModel`。C 模型句柄采用同一表示，只有显式创建的
C workspace 会分配计算缓冲区。Python 的 `robot.fork()` 共享模型与 link-ID 表，同时
创建独立的 workspace 和锁。关闭任一实例都不会影响其他副本；固定基座 fork 初始会复制
当前基座位姿，此后对任一实例的位姿修改互不影响。

## 并行计算

每个 Rust `Robot` 或 `FloatingRobot` 都是可变的，同一时刻只能参与一次计算；并行执行时
应为每个任务调用 `fork()` 创建独立实例。Python 会串行化同一实例上的调用，需要并行时同样
推荐使用 `fork()`，避免重复解析 URDF 或复制模型。C++ 不带内部锁，每个工作线程应使用
独立的 `Robot` 或 `FloatingRobot` 实例。
