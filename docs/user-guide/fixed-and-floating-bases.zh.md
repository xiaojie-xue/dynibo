# 固定基座与浮动基座

Base mode 决定根节点运动是否作为广义自由度参与计算。它在加载模型时确定，之后不能
更改。

## 固定基

固定基机器人满足 `G = J`。仍然可以通过 `set_base_frame` 将根节点放置在世界坐标系
中的任意位置；“固定”表示该位姿是给定状态，而不是需要求解的广义坐标。

## 浮动基

浮动基机器人满足 `G = J + 6`，广义量最前面是世界坐标系下的角运动和线运动。
执行依赖速度或加速度的计算前，应设置完整基座状态：

=== "Rust"

    ```rust
    let mut robot = Robot::from_urdf_with_base(
        "robot.urdf", BaseMode::Floating)?;
    robot.set_floating_base_state(frame, velocity, acceleration)?;
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

关节数组长度仍然是 `J`，不要在开头添加四元数或六个基座值。基座状态通过 robot 对象
进入计算。

## 对计算的影响

- 位姿使用保存的 base frame。
- 速度和加速度包含保存的基座运动。
- 雅可比矩阵增加开头六个基座列。
- 质量矩阵和广义力增加六个基座行或元素。
- 逆运动学目前只支持固定基模型。

修改基座状态会改变 robot。不要在同一个 robot 正在计算时并发修改它。
