# 运动学

运动学描述关节及基座状态与 link 位姿、速度和加速度之间的关系。所有 target ID 都是
由 `link_id` 返回的模型局部标识。

## 正运动学

`forward_kinematics` 返回目标 link 在世界坐标系下的位姿。只有根节点到目标路径上的
关节会影响该位姿，其他分支不会参与。

## 雅可比矩阵及其导数

几何雅可比将广义速度映射为目标 link 原点处、世界坐标系下、角分量在前的 twist：

$$
{}^W V_{target} = J(q)\nu.
$$

`jacobian_derivative` 使用相同的 `6 x G` 尺寸、世界坐标系、目标原点和 column-major
约定。二者共同满足：

$$
{}^W A_{target} = J(q)\dot\nu + \dot J(q,\nu)\nu.
$$

不在目标祖先链上的关节对应列为零。浮动基雅可比的前六列对应基座运动。

## 正向速度与加速度

`forward_velocity_kinematics` 接受相对于目标 link 的 tool pose，并返回该工具点的
速度。`forward_acceleration_kinematics` 返回目标 link 原点的加速度。固定基计算使用
`Robot` 保存的 frame；浮动基计算使用该次调用显式传入的 `BaseState`。

## 逆运动学 { #inverse-kinematics }

逆运动学使用阻尼最小二乘求解一个固定基目标位姿：

$$
\Delta q = J^T(JJ^T + \lambda^2 I)^{-1}e.
$$

终止条件由平移和旋转容差、最大迭代次数、阻尼和最大步长范数控制。不收敛属于求解器
错误；目前不支持浮动基逆运动学。

## 调用方式

=== "Rust"

    ```rust
    let pose = robot.forward_kinematics(&q, target)?;
    robot.jacobian(&q, target, &mut jacobian)?;
    let velocity = robot.forward_velocity_kinematics(
        &q, &qd, target, &Frame::identity())?;
    let mut solution = vec![0.0; robot.joint_count()];
    robot.inverse_kinematics(
        &initial_q, target, &desired, options, &mut solution)?;
    ```

=== "Python"

    ```python
    pose = robot.forward_kinematics(q, target)
    jacobian = robot.jacobian(q, target)
    velocity = robot.forward_velocity_kinematics(q, qd, target)
    solution = robot.inverse_kinematics(initial_q, target, desired, options)
    ```

=== "C++"

    ```cpp
    const auto pose = robot.forward_kinematics(q, target);
    const auto jacobian = robot.jacobian(q, target);
    const auto velocity = robot.forward_velocity_kinematics(q, qd, target);
    const auto solution = robot.inverse_kinematics(initial_q, target, desired);
    ```

=== "C"

    ```c
    check(dynibo_forward_kinematics(
        robot, workspace, q, J, target, &pose));
    check(dynibo_jacobian(
        robot, workspace, q, J, target, jacobian, 6 * G));
    check(dynibo_inverse_kinematics(
        robot, workspace, initial_q, J, target, &desired,
        dynibo_ik_options_default(), solution, J));
    ```

精确函数签名和验证错误请查看对应的 [API 参考](../reference/python.md)或 docs.rs 上的
Rust reference。
