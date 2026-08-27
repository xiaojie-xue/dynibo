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

The joint arrays remain length `J`; do not prepend a quaternion or six base
values. Every language's floating calculation methods receive `BaseState`
explicitly; no floating robot handle stores mutable state.

## Effects on calculations

- Poses use the supplied base frame.
- Velocity and acceleration include the supplied base motion.
- Jacobians gain six leading base columns.
- Mass matrices and generalized forces gain six base rows or entries.
- Forward dynamics returns six world-frame base acceleration entries before joint acceleration.
- Forward dynamics uses the supplied pose and velocity; the base acceleration is ignored.
- Inverse kinematics is available on `Robot`, not `FloatingRobot`.

Rust base states are immutable calculation inputs, so one `FloatingRobot` may
be reused *sequentially* with different `BaseState` values. Each `Robot` and
`FloatingRobot` owns one workspace and calculation methods require mutable
access; call `fork()` to create independent instances for parallel work.
