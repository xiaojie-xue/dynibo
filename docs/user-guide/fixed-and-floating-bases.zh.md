# 固定基座与浮动基座

机器人类型决定根节点运动是否作为广义自由度参与计算。

## 固定基座

固定基座 `Robot` 满足 `G = J`。它默认使用单位位姿（identity）作为根节点位姿；调用
`Robot::set_base_frame(frame)` 可指定其他世界位姿。“固定”表示该位姿由外部给定，
而不作为待求解的广义坐标。

## 浮动基座

浮动基座机器人满足 `G = J + 6`，广义量开头依次为世界坐标系下的角运动和线运动。
URDF 根 link 必须声明惯性块（`<inertial>`），且质量严格为正。根 link 无质量的模型在
固定基座模式下仍然有效，但使用 `FloatingRobot` 加载时会被拒绝。正根质量属于加载期校验；
正动力学还会进一步检查完整铰接体惯量（articulated inertia），以检测转动惯量或关节子树奇异。

执行依赖速度或加速度的计算时，请传入完整的基座状态：

=== "Rust"

    ```rust
    let mut robot = FloatingRobot::from_urdf("robot.urdf")?;
    let base = BaseState::new(frame, velocity, acceleration)?;
    robot.inverse_dynamics(&base, &q, &qd, &qdd, &loads, &mut forces)?;
    ```

=== "Python"

    ```python
    robot = FloatingRobot.from_urdf("robot.urdf")
    base = BaseState(frame, velocity, acceleration)
    forces = robot.inverse_dynamics(base, q, qd, qdd)
    ```

=== "C++"

    ```cpp
    dynibo::FloatingRobot robot("robot.urdf");
    dynibo::BaseState base(frame, velocity, acceleration);
    auto forces = robot.inverse_dynamics(base, q, qd, qdd);
    ```

=== "C"

    ```c
    check(dynibo_floating_robot_from_urdf("robot.urdf", &robot));
    DyniboBaseState base = {frame, velocity, acceleration};
    check(dynibo_floating_inverse_dynamics(robot, workspace, &base,
        q, qd, qdd, joint_count, loads, load_count, forces, generalized_count));
    ```

关节数组长度仍然是 `J`，不要在开头拼接四元数或六个基座值。所有语言的浮动基座计算都
显式传入 `BaseState`，机器人句柄本身不保存可变的基座状态。

## 对计算的影响

- 位姿使用传入的 base frame。
- 速度和加速度包含传入的基座运动。
- 雅可比矩阵增加开头六个基座列。
- 质量矩阵和广义力增加六个基座行或元素。
- 正动力学在关节加速度之前返回六个世界坐标系基座加速度元素。
- 正动力学使用传入的位姿和速度，但忽略传入的基座加速度。
- 逆运动学只在 `Robot` 上可用，`FloatingRobot` 不提供该方法。

Rust 的基座状态是不可变计算输入，因此同一个 `FloatingRobot` 可以按顺序配合不同的
`BaseState` 复用。每个 `Robot` 和 `FloatingRobot` 都只拥有一个 workspace，计算方法
需要可变访问；并行计算应调用 `fork()` 创建相互独立的实例。
