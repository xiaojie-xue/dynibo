# 固定基座与浮动基座

Base mode 决定根节点运动是否作为广义自由度参与计算。它在加载模型时确定，之后不能
更改。

## 固定基

固定基机器人满足 `G = J`。Rust 中，`BaseState::fixed()` 表示 identity 位姿和零运动，
`BaseState::fixed_at(frame)` 可以指定其他世界位姿；“固定”表示该位姿是给定状态，
而不是需要求解的广义坐标。

## 浮动基

浮动基机器人满足 `G = J + 6`，广义量最前面是世界坐标系下的角运动和线运动。
执行依赖速度或加速度的计算时，应传入完整基座状态：

=== "Rust"

    ```rust
    let robot = Robot::from_urdf_with_base(
        "robot.urdf", BaseMode::Floating)?;
    let base = BaseState::new(frame, velocity, acceleration)?;
    robot.inverse_dynamics(
        &base, &q, &qd, &qdd, &loads, &mut workspace, &mut forces)?;
    ```

=== "Python"

    ```python
    robot = Robot.from_urdf_with_base("robot.urdf", BaseMode.FLOATING)
    robot.set_floating_base_state(frame, velocity, acceleration)
    ```

=== "C++"

    ```cpp
    dynibo::Robot robot("robot.urdf", DYNIBO_BASE_FLOATING);
    robot.set_floating_base_state(frame, velocity, acceleration);
    ```

=== "C"

    ```c
    check(dynibo_robot_from_urdf_with_base(
        "robot.urdf", DYNIBO_BASE_FLOATING, &robot));
    check(dynibo_robot_set_floating_base_state(
        robot, &frame, velocity, acceleration));
    ```

关节数组长度仍然是 `J`，不要在开头添加四元数或六个基座值。Rust 计算方法显式接收
`BaseState`；当前 Python 和 C 系列适配层为了 API 兼容仍保留 setter 状态。

## 对计算的影响

- 位姿使用传入的 base frame。
- 速度和加速度包含传入的基座运动。
- 雅可比矩阵增加开头六个基座列。
- 质量矩阵和广义力增加六个基座行或元素。
- 逆运动学目前只支持固定基模型。

Rust 的基座状态是不可变计算输入，因此一个 robot 可以配合不同的状态和 workspace
并发计算。
