# Fixed and Floating Bases

The robot type determines whether root motion contributes generalized degrees
of freedom.

## Fixed base

A fixed-base [`Robot`] has `G = J`. It starts with an identity root pose; use
[`Robot::set_base_frame`] to prescribe another world pose. "Fixed" means the
pose is prescribed rather than solved as a generalized coordinate.

## Floating base

A floating-base robot has `G = J + 6`. Its generalized ordering begins with
world-expressed angular and linear base motion. Supply the complete base state
to calculations that depend on velocity or acceleration:

The URDF root link must declare an inertial block with strictly positive mass.
Models with a massless root remain valid in fixed-base mode, but are rejected
when loaded as a [`FloatingRobot`]. A positive root mass is a load-time
requirement; forward dynamics additionally checks the complete articulated
inertia for rotational or joint-subtree singularities.

=== "Rust"

    ```rust
    let mut robot = FloatingRobot::from_urdf("robot.urdf")?;
    let base = BaseState::new(frame, velocity, acceleration)?;
    robot.inverse_dynamics(&base, &q, &qd, &qdd, &loads, &mut forces)?;
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
- Inverse kinematics is available on `Robot`, not `FloatingRobot`.

Rust base states are immutable calculation inputs, so one robot can be shared
across calculations using different states and workspaces.
