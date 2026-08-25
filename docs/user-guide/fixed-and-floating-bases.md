# Fixed and Floating Bases

The base mode determines whether root motion contributes generalized degrees of
freedom. It is selected when loading the model and cannot be changed afterward.

## Fixed base

A fixed-base robot has `G = J`. In Rust, `BaseState::fixed()` supplies the
identity pose and zero motion; `BaseState::fixed_at(frame)` prescribes another
world pose. "Fixed" means the pose is prescribed rather than solved as a
generalized coordinate.

## Floating base

A floating-base robot has `G = J + 6`. Its generalized ordering begins with
world-expressed angular and linear base motion. Supply the complete base state
to calculations that depend on velocity or acceleration:

The URDF root link must declare an inertial block with strictly positive mass.
Models with a massless root remain valid in fixed-base mode, but are rejected
when loaded with `BaseMode::Floating`. A positive root mass is a load-time
requirement; forward dynamics additionally checks the complete articulated
inertia for rotational or joint-subtree singularities.

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

The joint arrays remain length `J`; do not prepend a quaternion or six base
values. Rust calculation methods receive `BaseState` explicitly. The current
Python and C-family adapters retain setter-based state for API compatibility.

## Effects on calculations

- Poses use the supplied base frame.
- Velocity and acceleration include the supplied base motion.
- Jacobians gain six leading base columns.
- Mass matrices and generalized forces gain six base rows or entries.
- Forward dynamics returns six world-frame base acceleration entries before joint acceleration.
- Forward dynamics uses the supplied pose and velocity but ignores the stored acceleration.
- Inverse kinematics currently accepts fixed-base models only.

Rust base states are immutable calculation inputs, so one robot can be shared
across calculations using different states and workspaces.
