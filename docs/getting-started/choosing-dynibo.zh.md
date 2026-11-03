# 选择 Dynibo 与从 Pinocchio 迁移

Dynibo 专注于基于 URDF 的机器人运动学与刚体动力学。需要在 Rust 中计算固定或浮动
基座机器人，或希望通过 Python/NumPy 使用同一套 Rust 核心时，可以考虑 Dynibo。
每个计算实例持有可复用存储，矩阵与向量运算支持调用者提供输出缓冲区。

## 核对所需能力

| 需求 | Dynibo 的支持与边界 |
|---|---|
| 运行时 URDF 模型 | 树状拓扑，支持 revolute、continuous、prismatic 和 fixed 关节 |
| 固定基座机械臂 | FK、世界几何雅可比及其导数、速度与加速度运动学、阻尼最小二乘 IK |
| 浮动基座动力学 | 显式基座位姿与运动、广义质量矩阵、重力、速度乘积力、RNEA 与 ABA |
| 重复 CPU 计算 | 实例存储复用；Rust 输出切片与 Python `out=` 数组 |
| 多语言接口 | 同一 Rust 实现上的 Rust、Python、C 与 C++ 接口 |
| 碰撞与可视化几何 | 不提供碰撞检测和渲染；数值计算不需要 mesh |
| 地面接触与约束 | 不提供接触求解器、闭环机构求解器或平衡控制器 |
| 可微与 GPU 工作流 | 无公开的自动微分或 GPU 后端 |
| 完整仿真 | 不内置时间积分器，积分和控制由应用实现 |

如果应用已经依赖 Pinocchio 特有算法或生态工具，迁移前先梳理依赖。Dynibo 的 API
并非直接兼容替代。性能取决于运算、模型、绑定和硬件，请用
[可复现 benchmark](https://github.com/xiaojie-xue/dynibo/tree/main/benches)
评估自己的负载。

## 转换固定基座计算

表中 `pin` 为 Pinocchio Python 模块，`robot` 为 Dynibo `Robot`，`target` 为 URDF
link 名称。Pinocchio 的 model/data 与 frame API 见其
[官方文档](https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/)。

| 任务 | Pinocchio Python | Dynibo Python |
|---|---|---|
| 加载 | `model = pin.buildModelFromUrdf(path)`；`data = model.createData()` | `robot = Robot.from_urdf(path)` |
| 查询目标 | `frame = model.getFrameId(target)` | `link = robot.link_id(target)` |
| 位姿 | `pin.framesForwardKinematics(model, data, q)`；`data.oMf[frame]` | `robot.forward_kinematics(q, link)` |
| 雅可比 | `pin.computeFrameJacobian(model, data, q, frame, pin.LOCAL_WORLD_ALIGNED)` | `robot.jacobian(q, link).reshape((6, robot.generalized_count), order="F")` |
| 重力 | `pin.computeGeneralizedGravity(model, data, q)` | `robot.gravity(q)` |
| 逆动力学 | `pin.rnea(model, data, q, qd, qdd)` | `robot.inverse_dynamics(q, qd, qdd)` |
| 正动力学 | `pin.aba(model, data, q, qd, tau)` | `robot.forward_dynamics(q, qd, tau)` |
| 质量矩阵 | `pin.crba(model, data, q)`；必要时补全上三角对应的下三角 | `robot.mass_matrix(q).reshape((G, G), order="F")` |
| 独立计算存储 | 另一个 `model.createData()` | `robot.fork()` |

## 显式转换约定

1. **按名称匹配关节。** Dynibo 使用非固定 URDF 关节的广度优先顺序，不能假定现有
   Pinocchio 数组采用相同顺序。Rust 提供 `joint_name(index)`；Python 用户需要先根据
   自己的模型建立映射。Pinocchio 的 continuous 关节可能用余弦/正弦对表示 q，而
   Dynibo 使用标量角度，因此还需要转换表示形式。
2. **匹配参考系与作用点。** Dynibo 目标是 URDF link，雅可比在该 link 原点以世界轴
   表达。应与 Pinocchio 对应 frame 的 `LOCAL_WORLD_ALIGNED` 比较；其 `WORLD`
   模式的作用点约定并不相同。
3. **交换空间行块。** Pinocchio Motion 向量与雅可比采用线分量在前、角分量在后；
   Dynibo 相反。固定基座模型对齐关节列后，可用
   `np.vstack((J_pin[3:, :], J_pin[:3, :]))` 与 Dynibo 雅可比比较。
4. **转换输出形状。** Dynibo 矩阵是列主序的扁平 float64 数组，应使用 `order="F"`。
   默认 reshape 顺序会改变矩阵含义。位姿通过 `translation`、`rotation_xyzw` 获取，
   不是 SE3 矩阵对象。
5. **转换外部载荷。** Dynibo 载荷是 link 原点处、局部坐标表达的阻力 wrench，力矩
   在前。需要变换作用点和参考系，并把物理外力取反后构造 `Load`。不能直接复制
   按 joint 索引的外力集合。

## 浮动基座迁移

Dynibo 把关节坐标与根状态分开。`BaseState` 包含根的世界位姿 `Pose`，以及世界坐标
表达的根角/线速度和经典加速度。q、qd、qdd 长度始终为 J；广义力、加速度和矩阵
坐标轴大小为 G = J + 6。

不能直接复制 Pinocchio free-flyer 的配置或切向量：配置表示、空间分量顺序、基座
速度参考系都需要转换。尤其要区分空间加速度与经典线加速度，它们相差速度叉乘项。
比较数值结果前先对齐这些约定。正动力学忽略传入的基座加速度，因为加速度是待求结果。

## 验证迁移

先在多个非零关节配置下比较位姿，再比较对齐顺序与参考系后的雅可比。检查质量矩阵
对称性，以及无载荷时的 `tau = M @ qdd + velocity_product_forces + gravity`。
最后把 RNEA 输出传给 ABA，验证恢复期望加速度，再对载荷和浮动基座转换重复验证。

从[可执行任务示例](../recipes/index.md)开始，然后查阅
[Python API 参考](../reference/python.md)与
[参考系约定](../user-guide/frames-and-spatial-vectors.md)。
