# 机器人模型与 URDF

Dynibo 在运行时从 URDF 文件加载完整的树形机器人模型。根 link 可以固定在世界坐标系，
也可以作为具有六个自由度的浮动基。

## 支持的拓扑

模型可以包含 revolute、continuous、prismatic 和 fixed joint，并支持分支结构：
机器人可以有多个叶子 link，计算可以选择任意 link 作为目标。Fixed joint 不占用关节
状态向量中的元素，但其变换、质量和惯量仍会影响后代 link 和动力学结果。

Dynibo 会拒绝无法表示的拓扑，包括不连通结构、环和无效的父子关系。URDF 解析和
模型验证在加载阶段完成，失败时还不会创建 workspace。

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

拓扑和惯性参数来自 URDF；关节位置、速度和加速度在每次计算时传入。基座位姿和运动
保存在 robot 上，因为它们统一作用于所有计算，详见[固定基座与浮动基座](fixed-and-floating-bases.md)。
