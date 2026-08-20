# Fixed and Floating Bases

The base mode determines whether root motion contributes generalized degrees of
freedom. It is selected when loading the model and cannot be changed afterward.

## Fixed base

A fixed-base robot has `G = J`. Its root pose may still be placed anywhere in
the world with `set_base_frame`; "fixed" means the pose is prescribed rather
than solved as a generalized coordinate.

## Floating base

A floating-base robot has `G = J + 6`. Its generalized ordering begins with
world-expressed angular and linear base motion. Set the complete base state
before calculations that depend on velocity or acceleration:

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

The joint arrays remain length `J`; do not prepend a quaternion or six base
values. Base state enters calculations through the robot object.

## Effects on calculations

- Poses use the stored base frame.
- Velocity and acceleration include stored base motion.
- Jacobians gain six leading base columns.
- Mass matrices and generalized forces gain six base rows or entries.
- Inverse kinematics currently accepts fixed-base models only.

Changing base state mutates the robot. Do not change it concurrently with a
calculation on the same robot.
