# 固定基座与浮动基座

机器人类型决定根节点运动是否作为广义自由度参与计算。

## 固定基

固定基 `Robot` 满足 `G = J`。它默认使用 identity 根节点位姿；调用
`Robot::set_base_frame(frame)` 可以指定其他世界位姿。“固定”表示该位姿是给定状态，
而不是需要求解的广义坐标。

## 浮动基

浮动基机器人满足 `G = J + 6`，广义量最前面是世界坐标系下的角运动和线运动。
执行依赖速度或加速度的计算时，应传入完整基座状态：

URDF root link 必须声明 inertial block，并具有严格为正的质量。无质量 root 的模型在
固定基模式下仍然有效，但使用 `FloatingRobot` 加载时会被拒绝。正 root mass 是
加载期要求；正动力学还会检查完整 articulated inertia，以发现转动惯量或关节子树奇异。

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

关节数组长度仍然是 `J`，不要在开头添加四元数或六个基座值。Rust 计算方法显式接收
`BaseState`；所有语言的浮动计算都显式接收 state，handle 不保存可变状态。

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
