# 动力学

Dynibo 的动力学操作使用机器人动力学方程：

$$
\tau = M(q)\dot\nu + C(q,\nu)\nu + g(q).
$$

固定基输出包含标量关节力。浮动基输出首先包含世界坐标系下六元素根节点 wrench，
随后是关节力。

## 质量矩阵

`mass_matrix` 计算对称的 `G x G` 广义惯性矩阵，使用 column-major 顺序。Fixed
joint 不占用行或列，但其子树惯量仍然影响可以运动的祖先关节。

## 速度乘积力

`velocity_product_forces` 计算科里奥利力和离心力对应的广义力，不包含重力、给定的
基座加速度和外部载荷。

## 重力

没有外部载荷时，`gravity` 是速度和加速度均为零的逆动力学项：

$$
g(q) = \tau(q,0,0).
$$

该操作也可以加入 link 局部外部载荷，而不需要非零关节速度或加速度。

## 逆动力学

`inverse_dynamics` 使用递归 Newton--Euler 算法，包含关节状态、保存的基座运动、
重力和可选外部载荷。基座静止且无载荷时满足上面的动力学方程。

## 调用方式

=== "Rust"

    ```rust
    let base = BaseState::fixed();
    robot.mass_matrix(&base, &q, &mut workspace, &mut mass)?;
    robot.velocity_product_forces(&base, &q, &qd, &mut workspace, &mut velocity)?;
    robot.gravity(&base, &q, &loads, &mut workspace, &mut gravity)?;
    robot.inverse_dynamics(
        &base, &q, &qd, &qdd, &loads, &mut workspace, &mut forces)?;
    ```

=== "Python"

    ```python
    mass = robot.mass_matrix(q)
    velocity = robot.velocity_product_forces(q, qd)
    gravity = robot.gravity(q, loads)
    forces = robot.inverse_dynamics(q, qd, qdd, loads)
    ```

=== "C++"

    ```cpp
    const auto mass = robot.mass_matrix(q);
    const auto velocity = robot.velocity_product_forces(q, qd);
    const auto gravity = robot.gravity(q, loads);
    const auto forces = robot.inverse_dynamics(q, qd, qdd, loads);
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
    ```

传入载荷前请阅读[外部载荷](external-loads.md)，解释基座 wrench 或矩阵结果前请阅读
[参考系与空间向量](frames-and-spatial-vectors.md)。
