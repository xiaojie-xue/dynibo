# 约定与 API 速查

本页集中汇总 Rust、Python、C++ 和 C 四种接口共享的全部数值约定、与 Pinocchio 的差异、
数学符号映射以及完整公开 API 列表。

## 核心约定

| 主题 | 规则 |
|---|---|
| 单位 | 国际单位制（SI）：米（m）、弧度（rad）、秒（s）、千克（kg）、牛顿（N）、牛·米（N·m） |
| 关节数量 `J` | URDF 中非固定关节（`revolute`、`continuous`、`prismatic`）的总数 |
| 关节顺序 | 从根 link 出发的**广度优先拓扑顺序**（同一父 link 下的子关节保持 URDF 声明顺序）；在 Rust 中可通过 `joint_name(i)` 查询 |
| 广义坐标维数 `G` | 固定基座：`G == J`；浮动基座：`G == J + 6` |
| 关节输入（`q`、`qd`、`qdd`） | 长度始终为 `J`，不要在开头拼接浮动基座位姿或速度 |
| 浮动基座广义量顺序 | 速度/加速度为 `[base_angular_xyz, base_linear_xyz, J 个关节]`；广义力为 `[base_torque_xyz, base_force_xyz, J 个关节力]`（世界坐标系，作用点为根 link 原点） |
| 四元数顺序 | Python、C++ 和 C 使用 `(x, y, z, w)` 单位四元数；Rust 使用 `nalgebra::Isometry3<f64>`（`Frame`） |
| Twist（`Twist` / `DyniboTwist`） | 角分量在前 `[wx, wy, wz, vx, vy, vz]`；世界坐标系下的**经典速度**与**经典加速度**（而非 Featherstone 空间加速度，二者相差 $\omega \times v$） |
| 外部载荷（`Wrench` / `Load` / `DyniboLoad`） | 力矩在前 `[tx, ty, tz, fx, fy, fz]`，作用于目标 link 原点并在该 link 局部坐标系表达，采用**阻力 wrench** 符号约定（`gravity`/`inverse_dynamics` 中相加，`forward_dynamics` 中相减；与施加在机器人上的物理外力反号） |
| 矩阵布局 | 一维 **column-major（列主序）**：`values[col * rows + row]`。雅可比/导数长度 `6 * G`，质量矩阵长度 `G * G`。Python/NumPy 需用 `arr.reshape((rows, cols), order="F")` |
| `forward_kinematics_all` 布局 | 按 link ID 顺序输出全部 `link_count` 个世界位姿（索引 `0` 为根 link）。Python 返回长度 `7 * link_count` 的一维数组（`reshape((link_count, 7))`，每行 `[x, y, z, qx, qy, qz, qw]`） |
| 默认重力 | 世界坐标系 `[0.0, 0.0, -9.80665]` m/s²；可通过 `set_gravity` 按实例/workspace 配置 |
| `forward_dynamics` 基座状态 | 使用 `BaseState.frame` 和 `BaseState.velocity`，**忽略** `BaseState.acceleration`；无驱动浮动基将前 6 个广义力设为 0 |
| 默认 IK 配置（仅固定基） | `max_iterations = 100`，`translation_tolerance = 1e-6` m，`rotation_tolerance = 1e-6` rad，`damping = 1e-3`，`max_step_norm = 0.5` |
| 并发规则 | 单个 `Robot`/`FloatingRobot`/C workspace 同一时刻仅执行一次计算；并行请用 `fork()`（Rust/Python）、独立实例（C++）或独立 workspace（C） |

## 与 Pinocchio 的主要差异

从 Pinocchio 迁移代码时，请特别留意以下约定区别：

| 约定项 | dynibo | Pinocchio（默认） |
|---|---|---|
| 空间向量 / 雅可比行顺序 | **角分量在前**：`[angular_xyz, linear_xyz]` | **线分量在前**：`[linear_xyz, angular_xyz]` |
| 雅可比与运动学参考系 | 世界坐标轴、目标原点（等价于 `LOCAL_WORLD_ALIGNED`），**经典加速度** | 可选 `LOCAL` / `WORLD` / `LOCAL_WORLD_ALIGNED`；区分空间加速度与经典加速度 |
| 浮动基表示 | 显式传入 `BaseState`；`q` 长度为 `J`；广义基座分量为世界系 `[ang, lin]` | 将自由浮动关节编入 `q`（`7 + J`，`[xyz, xyzw]`）与 `v`（`6 + J`，局部系 `[lin, ang]`） |
| 外力 wrench 符号 | link 局部系下的**阻力** wrench（RNEA 加、ABA 减） | 施加在机器人上的外力 `fext`（RNEA 减、ABA 加） |
| 分支树关节顺序 | 从根 link 出发的**广度优先（BFS）** | 从根 link 出发的**深度优先（DFS）** |
| 计算存储 | 由 `Robot` / `FloatingRobot` 持有（Rust 支持 `state()` 快照；C 使用显式 workspace） | 分离的 `Model` 与 `Data` 对象 |

## 数学符号映射

| 符号 | 物理含义 | 对应 API 与布局 |
|---|---|---|
| $q, \dot q, \ddot q \in \mathbb{R}^J$ | 活动关节位置、速度、加速度 | `q`、`qd`、`qdd`（长度 `J`，广度优先拓扑顺序） |
| $\nu, \dot\nu \in \mathbb{R}^G$ | 广义速度与广义经典加速度 | 固定基：$\dot q, \ddot q$；浮动基：`[base_ang, base_lin, qd/qdd]` |
| ${}^W T_{\text{target}}(q)$ | 目标 link 的世界位姿 | `forward_kinematics`（`Frame` / `Pose` / `DyniboPose`） |
| ${}^W V_{\text{target}} = J(q)\nu$ | 目标原点世界系经典速度 twist | `forward_velocity_kinematics`（`tool = identity` 时） |
| ${}^W A_{\text{target}} = J(q)\dot\nu + \dot J(q,\nu)\nu$ | 目标原点世界系经典加速度 twist | `forward_acceleration_kinematics` |
| $J(q), \dot J(q,\nu) \in \mathbb{R}^{6 \times G}$ | 目标原点世界系几何雅可比及时间导数 | `jacobian`、`jacobian_derivative`（一维列主序 `6 * G`） |
| $M(q) \in \mathbb{R}^{G \times G}$ | 对称广义惯性矩阵 | `mass_matrix`（一维列主序 `G * G`） |
| $C(q,\nu)\nu \in \mathbb{R}^G$ | 科里奥利力与离心力广义力 | `velocity_product_forces`（长度 `G`） |
| $g(q) \in \mathbb{R}^G$ | 重力广义力（可选叠加 $\tau_{\mathrm{load}}$） | `gravity`（长度 `G`） |
| $\tau = M\dot\nu + C\nu + g + \tau_{\mathrm{load}}$ | RNEA 所需广义驱动力 | `inverse_dynamics`（长度 `G`） |
| $\dot\nu = M^{-1}(\tau - C\nu - g - \tau_{\mathrm{load}})$ | ABA 广义加速度 | `forward_dynamics`（长度 `G`） |

## 四语言完整 API 对照表

浮动基操作需在参数开头传入 `base`（Rust 为 `&BaseState`，Python/C++ 为 `BaseState`，C 为 `const DyniboBaseState *`），并使用 `FloatingRobot` / `dynibo_floating_*`。

| 操作 | 输出尺寸 | Rust（`Robot` / `FloatingRobot`） | Python（`Robot` / `FloatingRobot`） | C++（`dynibo::Robot` / `FloatingRobot`） | C（`dynibo.h`） |
|---|---|---|---|---|---|
| 加载固定/浮动模型 | 句柄 | `Robot::from_urdf` / `FloatingRobot::from_urdf` | `Robot.from_urdf` / `FloatingRobot.from_urdf` | `Robot(path)` / `FloatingRobot(path)` | `dynibo_[floating_]robot_from_urdf` |
| 只读共享模型 | 句柄 | `RobotModel::from_urdf`、`robot.model()` | — | — | `Dynibo[Floating]Robot` |
| 并行独立副本 | 句柄 | `fork()` | `fork()` | 独立实例 | `dynibo_[floating_]workspace_create` |
| 维度与名称 | 标量 | `name`、`joint_count`、`generalized_count`、`link_count` | `.name`、`.joint_count`、`.generalized_count`、`.link_count` | `name()`、`joint_count()`、`generalized_count()`、`link_count()` | `dynibo_[floating_]robot_{name,joint_count,generalized_count,link_count}` |
| Link 查找 | `LinkId` / `size_t` | `link_id`、`link_id_at`、`root_link_id` | `link_id`、`link_id_at` | `link_id`、`link_id_at` | `dynibo_[floating_]robot_link_id` |
| 关节与 Link 元数据 | 标量/向量/矩阵 | `joint_{name,type,lower_limit,upper_limit,velocity_limit}`、`link_{name,mass,center_of_mass,inertia}` | — | — | — |
| 固定基世界位姿 | — | `set_base_frame`、`base_frame` | `set_base_frame` | `set_base_frame` | `dynibo_workspace_set_base_frame` |
| 重力向量 | 3 | `set_gravity`、`gravity_vector` | `set_gravity`、`.gravity_vector` | `set_gravity`、`gravity_vector` | `dynibo_[floating_]workspace_{set_gravity,gravity}` |
| 单 Link 正运动学 | 1 位姿 | `forward_kinematics` | `forward_kinematics` | `forward_kinematics` | `dynibo_[floating_]forward_kinematics` |
| 全 Link 正运动学 | `link_count` 位姿（Py 为 `7*L`） | `forward_kinematics_all` | `forward_kinematics_all(..., out=)` | `forward_kinematics_all[_into]` | `dynibo_[floating_]forward_kinematics_all` |
| 正向速度运动学 | 1 twist | `forward_velocity_kinematics` | `forward_velocity_kinematics` | `forward_velocity_kinematics` | `dynibo_[floating_]forward_velocity_kinematics` |
| 正向加速度运动学 | 1 twist | `forward_acceleration_kinematics` | `forward_acceleration_kinematics` | `forward_acceleration_kinematics` | `dynibo_[floating_]forward_acceleration_kinematics` |
| 几何雅可比 | `6 * G` | `jacobian` | `jacobian(..., out=)` | `jacobian[_into]` | `dynibo_[floating_]jacobian` |
| 雅可比导数 | `6 * G` | `jacobian_derivative` | `jacobian_derivative(..., out=)` | `jacobian_derivative[_into]` | `dynibo_[floating_]jacobian_derivative` |
| 逆运动学（仅固定基） | `J` | `inverse_kinematics` | `inverse_kinematics(..., out=)` | `inverse_kinematics[_into]` | `dynibo_inverse_kinematics` |
| 质量矩阵（CRBA） | `G * G` | `mass_matrix` | `mass_matrix(..., out=)` | `mass_matrix[_into]` | `dynibo_[floating_]mass_matrix` |
| 速度乘积力 | `G` | `velocity_product_forces` | `velocity_product_forces(..., out=)` | `velocity_product_forces[_into]` | `dynibo_[floating_]velocity_product_forces` |
| 重力（+ 载荷） | `G` | `gravity` | `gravity(..., loads=, out=)` | `gravity[_into]` | `dynibo_[floating_]gravity` |
| 逆动力学（RNEA） | `G` | `inverse_dynamics` | `inverse_dynamics(..., loads=, out=)` | `inverse_dynamics[_into]` | `dynibo_[floating_]inverse_dynamics` |
| 正动力学（ABA） | `G` | `forward_dynamics` | `forward_dynamics(..., loads=, out=)` | `forward_dynamics[_into]` | `dynibo_[floating_]forward_dynamics` |
| 可复用载荷缓冲区 | 缓冲区 | `load_buffer()` -> `LoadBuffer` | `load_buffer()` -> `LoadBuffer` | `make_load(...)` -> `DyniboLoad` | `DyniboLoad[]` 数组 |
| 状态快照复用 | 借用视图 | `state(&q, &qd)` / `state(&base, &q, &qd)` | — | — | — |
