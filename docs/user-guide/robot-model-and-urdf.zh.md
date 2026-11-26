# 机器人模型与 URDF

Dynibo 在运行时从 URDF 文件加载完整的树形机器人模型。根 link 可以固定在世界坐标系中，
也可以作为拥有六个自由度的浮动基座。

## 支持的拓扑

模型可以包含 revolute、continuous、prismatic 和 fixed 关节，并支持分支结构：
机器人可以有多个叶子 link，计算可以选择任意 link 作为目标。Fixed 关节不占用关节
状态向量中的元素，但其变换、质量和惯量仍会影响后代 link 和动力学结果。

Dynibo 会拒绝无法表示的拓扑，包括不连通结构、环路和无效的父子关系。URDF 解析和
模型校验在加载阶段完成，失败时不会创建 workspace。

关节 origin 和活动关节轴必须为有限数值（不能包含 `NaN` 或 `Inf`）。Revolute 和
prismatic 关节的位置上下限必须为有限数值，且下限不大于上限；continuous 关节不设位置
限制。活动关节的速度上限必须为有限且非负的数值。目前遇到 mimic 约束会返回模型错误，
避免将从动关节静默当作独立自由度加载。

## 名称与 ID

`Robot.name` 来自 URDF 中的 robot 名称，link 名称也保持不变。可以先解析一次名称，
然后在重复计算中复用 link ID：

=== "Rust"

    ```rust
    let robot = Robot::from_urdf("robot.urdf")?;
    let tool = robot.link_id("tool")?;
    ```

=== "Python"

    ```python
    robot = Robot.from_urdf("robot.urdf")
    tool = robot.link_id("tool")
    ```

=== "C++"

    ```cpp
    dynibo::Robot robot("robot.urdf");
    const auto tool = robot.link_id("tool");
    ```

=== "C"

    ```c
    DyniboRobot *robot = NULL;
    size_t tool = 0;
    check(dynibo_robot_from_urdf("robot.urdf", &robot));
    check(dynibo_robot_link_id(robot, "tool", &tool));
    ```

Link ID 只对生成它的模型有效，不应作为模型数据长期保存，也不能用于另一个独立加载的
robot。

## 模型状态与计算状态

在 Rust 中，可以将不可变模型加载与可变计算存储分开管理：

- `RobotModel::from_urdf(path)` 加载并校验模型，但不分配 workspace，并提供与机器人实例
  相同的元数据查询方法。
- `model.robot()` 创建位于世界原点的固定基座实例；`model.floating_robot()?` 会先校验根
  link 质量严格为正，再创建浮动基座实例。
- `robot.model()` 与 `model.clone()` 共享不可变模型，且不分配 workspace。由同一模型派生
  的实例共享 link ID，而独立加载的 URDF 各自拥有独立的 ID 作用域。
- `RobotModel` 不保存固定基座位姿；如需复制当前固定基座位姿与重力配置并创建独立计算
  存储，请调用 `robot.fork()`。

拓扑和惯性参数来自 URDF，而关节位置、速度和加速度在每次计算时传入。固定基座 `Robot`
会持久保存基座位姿；浮动基座的位姿、速度和加速度则通过 `BaseState` 在每次调用
`FloatingRobot` 时显式传入，详见[固定基座与浮动基座](fixed-and-floating-bases.md)。
