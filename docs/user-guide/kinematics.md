# Kinematics

Kinematics relates joint and base state to link pose, velocity, and
acceleration. All target IDs are model-scoped values returned by `link_id`.

## Forward kinematics

`forward_kinematics` returns the target-link pose in the world frame. Only
joints on the root-to-target path affect the pose; branches outside that path
do not.

## Jacobian and its derivative

The geometric Jacobian maps generalized velocity to an angular-first target
twist at the target-link origin:

$$
{}^W V_{target} = J(q)\nu.
$$

`jacobian_derivative` uses the same `6 x G`, world-frame, target-origin and
column-major conventions. Together they satisfy:

$$
{}^W A_{target} = J(q)\dot\nu + \dot J(q,\nu)\nu.
$$

Columns for joints outside the target's ancestor chain are zero. Floating-base
Jacobians begin with six base-motion columns.

## Forward velocity and acceleration

`forward_velocity_kinematics` accepts a tool pose relative to the target link
and returns velocity at that tool point. `forward_acceleration_kinematics`
returns acceleration at the target-link origin. Both include the base state
stored on the robot.

## Inverse kinematics

Inverse kinematics solves one fixed-base target pose with damped least squares:

$$
\Delta q = J^T(JJ^T + \lambda^2 I)^{-1}e.
$$

Termination is controlled by translation and rotation tolerances, maximum
iterations, damping, and maximum step norm. Non-convergence is a solver error;
floating-base inverse kinematics is not currently supported.

## Calling the operations

=== "Rust"

    ```rust
    let pose = robot.forward_kinematics(&q, target, &mut workspace)?;
    robot.jacobian(&q, target, &mut workspace, &mut jacobian)?;
    let velocity = robot.forward_velocity_kinematics(
        &q, &qd, target, &Frame::identity(), &mut workspace)?;
    let mut solution = vec![0.0; robot.joint_count()];
    robot.inverse_kinematics(
        &initial_q, target, &desired, options,
        &mut workspace, &mut solution)?;
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

For exact signatures and validation errors, use the relevant [API
Reference](../reference/python.md) or Rust reference on docs.rs.
