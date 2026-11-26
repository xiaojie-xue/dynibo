# 动力学

Dynibo 的动力学操作使用机器人动力学方程：

$$
\tau = M(q)\dot\nu + C(q,\nu)\nu + g(q).
$$

固定基座输出为标量关节力；浮动基座输出依次为世界坐标系下六元素根节点 wrench 和
各关节力。

## 质量矩阵

`mass_matrix` 计算对称的 `G x G` 广义惯性矩阵，采用列主序存储。Fixed 关节不占用行或列，
但其子树惯量仍然影响可运动的祖先关节。

## 速度乘积力

`velocity_product_forces` 计算科里奥利力和离心力对应的广义力，不包含重力、给定的
基座加速度和外部载荷。

## 重力

没有外部载荷时，`gravity` 是速度和加速度均为零的逆动力学项：

$$
g(q) = \tau(q,0,0).
$$

该函数也支持在零速度、零加速度下叠加 link 局部外部载荷。

## 逆动力学

`inverse_dynamics` 使用递归 Newton-Euler 算法（RNEA），综合考虑关节状态、显式传入的
浮动基座运动、重力和可选外部载荷。基座静止且无外载荷时满足上面的动力学方程。

## 正动力学

`forward_dynamics` 使用线性时间复杂度的铰接体算法（articulated-body algorithm，ABA）求解：

$$
\dot\nu = M(q)^{-1}\left(\tau-C(q,\nu)\nu-g(q)-\tau_{\mathrm{load}}\right).
$$

对于浮动基座，输入广义力和输出广义加速度均以世界坐标系下的基座角分量、线分量开头。
计算会使用传入的 `BaseState` 位姿和速度；由于基座加速度本身就是待求解的输出，传入的
`BaseState.acceleration` 会被忽略。当关节或浮动基座的铰接体惯量奇异时，函数会返回求解
错误，而不会产生非有限加速度。

浮动基座求解器会先按对角尺度对六个基座惯量坐标进行平衡缩放，再执行 Cholesky 分解，
以便稳健处理转动惯量远小于质量的情形；同时会拒绝奇异矩阵、条件数过差的缩放矩阵
（一范数条件数倒数低于 `sqrt(f64::EPSILON)`，在 Rust 中作为独立错误报告）、归一化残差
超限的解以及非有限中间结果。

## 调用方式

=== "Rust"

    ```rust
    robot.mass_matrix(&q, &mut mass)?;
    robot.velocity_product_forces(&q, &qd, &mut velocity)?;
    robot.gravity(&q, &loads, &mut gravity)?;
    robot.inverse_dynamics(&q, &qd, &qdd, &loads, &mut forces)?;
    robot.forward_dynamics(&q, &qd, &forces, &loads, &mut accelerations)?;
    ```

=== "Python"

    ```python
    mass = robot.mass_matrix(q)
    velocity = robot.velocity_product_forces(q, qd)
    gravity = robot.gravity(q, loads)
    forces = robot.inverse_dynamics(q, qd, qdd, loads)
    accelerations = robot.forward_dynamics(q, qd, forces, loads)
    ```

=== "C++"

    ```cpp
    const auto mass = robot.mass_matrix(q);
    const auto velocity = robot.velocity_product_forces(q, qd);
    const auto gravity = robot.gravity(q, loads);
    const auto forces = robot.inverse_dynamics(q, qd, qdd, loads);
    const auto accelerations = robot.forward_dynamics(q, qd, forces, loads);
    ```

=== "C"

    ```c
    check(dynibo_mass_matrix(
        robot, workspace, q, J, mass, G * G));
    check(dynibo_gravity(
        robot, workspace, q, J, loads, load_count, gravity, G));
    check(dynibo_inverse_dynamics(
        robot, workspace, q, qd, qdd, J,
        loads, load_count, forces, G));
    check(dynibo_forward_dynamics(
        robot, workspace, q, qd, J, forces, G,
        loads, load_count, accelerations, G));
    ```

传入载荷前请阅读[外部载荷](external-loads.md)，解释基座 wrench 或矩阵结果前请阅读
[参考系与空间向量](frames-and-spatial-vectors.md)。
