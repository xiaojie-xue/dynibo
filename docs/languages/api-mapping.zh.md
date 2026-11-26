# API 对照

Dynibo 通过四种符合语言习惯的接口开放同一套数值模型。各接口的操作名称保持一致，
而资源所有权、缓冲区和错误处理则遵循各自语言的惯用方式。

Rust 使用 crate 与类型命名空间，Python 使用模块与类，C++ 使用 `namespace dynibo`。
C 语言没有命名空间，因此稳定 ABI 中的所有导出符号均带有 `dynibo_` 前缀——该前缀只是
命名空间在 C 中的表达形式，而非另一套数值 API。

| 概念 | Rust | Python | C++ | C |
|---|---|---|---|---|
| 加载固定基座模型 | `Robot::from_urdf` | `Robot.from_urdf` | `dynibo::Robot(path)` | `dynibo_robot_from_urdf` |
| 加载浮动基座模型 | `FloatingRobot::from_urdf` | `FloatingRobot.from_urdf` | `dynibo::FloatingRobot(path)` | `dynibo_floating_robot_from_urdf` |
| 查找固定/浮动基座 link | `robot.link_id` | `robot.link_id` | `robot.link_id` | `dynibo_robot_link_id` / `dynibo_floating_robot_link_id` |
| 固定基座正运动学 | `Robot::forward_kinematics` | `Robot.forward_kinematics` | `Robot.forward_kinematics` | `dynibo_forward_kinematics` |
| 浮动基座正运动学 | `FloatingRobot::forward_kinematics(base, …)` | `FloatingRobot.forward_kinematics(base, …)` | `FloatingRobot.forward_kinematics(base, …)` | `dynibo_floating_forward_kinematics` |
| 固定/浮动基座雅可比矩阵 | `Robot` / `FloatingRobot` 方法 | `Robot` / `FloatingRobot` 方法 | `Robot` / `FloatingRobot` 方法 | `dynibo_jacobian` / `dynibo_floating_jacobian` |
| 固定基座质量与动力学 | `Robot` 方法 | `Robot` 方法 | `Robot` 方法 | `dynibo_mass_matrix`、`dynibo_inverse_dynamics`、`dynibo_forward_dynamics` |
| 浮动基座质量与动力学 | 传入 `base` 的 `FloatingRobot` 方法 | 传入 `base` 的 `FloatingRobot` 方法 | 传入 `base` 的 `FloatingRobot` 方法 | `dynibo_floating_mass_matrix`、`dynibo_floating_inverse_dynamics`、`dynibo_floating_forward_dynamics` |
| 固定/浮动基座 workspace | 由对应类型的机器人实例持有 | 由对应类型的机器人实例持有 | 由对应类型的机器人实例持有 | `DyniboWorkspace` / `DyniboFloatingWorkspace` |
| 配置重力向量 | `set_gravity` / `gravity_vector` | `set_gravity` / `gravity_vector` | `set_gravity` / `gravity_vector` | `dynibo_[floating_]workspace_set_gravity` / `dynibo_[floating_]workspace_gravity` |
| 状态快照复用 | `Robot::state` / `FloatingRobot::state` | — | — | — |
| 矩阵输出 | 调用方缓冲区 | 一维 NumPy 数组或可复用的 `out=` | 一维 `std::vector` 或可复用的 `*_into` | 调用方缓冲区 |
| 错误处理 | `Result<T>` | 异常 | `dynibo::Error` | `DyniboStatus` |

## 值类型

| 含义 | Rust | Python | C++ | C |
|---|---|---|---|---|
| 位姿 | `Frame` | `Pose` | `DyniboPose` | `DyniboPose` |
| 空间运动 | `Twist` | `Twist` | `DyniboTwist` | `DyniboTwist` |
| 外部载荷 | `IndexedLoad` | `Load` | `DyniboLoad` | `DyniboLoad` |
| IK 配置 | `InverseKinematicsOptions` | `IkOptions` | `DyniboIkOptions` | `DyniboIkOptions` |
| Link 标识 | `LinkId` | `LinkId`（`int` 子类） | `dynibo::LinkId` | `size_t` |

C++ 封装有意复用与 ABI 兼容的 C 值结构体，同时为对象句柄增加 RAII、成员方法、
移动语义和异常处理。

共享语义请查看[使用指南](../user-guide/index.md)，具体集成方式请查看对应语言页面。
