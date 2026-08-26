# API 对照

Dynibo 通过四种符合语言习惯的接口开放同一套数值模型。操作名称保持对应，资源所有权、
buffer 和错误处理遵循各语言的惯用方式。

Rust 使用 crate 和 type namespace，Python 使用 module 和 class，C++ 使用
`namespace dynibo`。C 没有 namespace，因此稳定 ABI 中的每个符号都带有
`dynibo_` 库前缀。这个前缀是 namespace 在 C 中的表达方式，不是另一套数值 API。

| 概念 | Rust | Python | C++ | C |
|---|---|---|---|---|
| 加载固定基模型 | `Robot::from_urdf` | `Robot.from_urdf` | `dynibo::Robot(path)` | `dynibo_robot_from_urdf` |
| 加载浮动基模型 | `FloatingRobot::from_urdf` | `Robot.from_urdf_with_base` | `dynibo::Robot(path, mode)` | `dynibo_robot_from_urdf_with_base` |
| 查找 link | `robot.link_id` | `robot.link_id` | `robot.link_id` | `dynibo_robot_link_id` |
| 正运动学 | `robot.forward_kinematics` | `robot.forward_kinematics` | `robot.forward_kinematics` | `dynibo_forward_kinematics` |
| 雅可比矩阵 | `robot.jacobian` | `robot.jacobian` | `robot.jacobian` | `dynibo_jacobian` |
| 质量矩阵 | `robot.mass_matrix` | `robot.mass_matrix` | `robot.mass_matrix` | `dynibo_mass_matrix` |
| 逆动力学 | `robot.inverse_dynamics` | `robot.inverse_dynamics` | `robot.inverse_dynamics` | `dynibo_inverse_dynamics` |
| 正动力学 | `robot.forward_dynamics` | `robot.forward_dynamics` | `robot.forward_dynamics` | `dynibo_forward_dynamics` |
| Workspace | 由 `Robot` 持有 | 由 `Robot` 持有 | 由 `Robot` 持有 | 显式 handle |
| 矩阵输出 | 调用方 buffer | 一维 tuple | 一维 `std::vector` | 调用方 buffer |
| 错误 | `Result<T>` | 异常 | `dynibo::Error` | `DyniboStatus` |

## 值类型

| 含义 | Rust | Python | C++ | C |
|---|---|---|---|---|
| 位姿 | `Frame` | `Pose` | `DyniboPose` | `DyniboPose` |
| 空间运动 | `Twist` | `Twist` | `DyniboTwist` | `DyniboTwist` |
| 外部载荷 | `IndexedLoad` | `Load` | `DyniboLoad` | `DyniboLoad` |
| IK 配置 | `InverseKinematicsOptions` | `IkOptions` | `DyniboIkOptions` | `DyniboIkOptions` |
| Link 标识 | `LinkId` | `int` | `std::size_t` | `size_t` |

C++ wrapper 有意复用与 ABI 兼容的 C 值结构体，同时为对象 handle 增加 RAII、方法、
移动语义和异常。

共享语义请查看[使用指南](../user-guide/index.md)，集成方式请查看对应语言页面。
