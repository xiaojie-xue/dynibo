# Robot Model and URDF

Dynibo loads a complete tree-structured robot model from a URDF file at runtime.
The root link may be fixed to the world or treated as a six-degree-of-freedom
floating base.

## Supported topology

Models may contain revolute, continuous, prismatic, and fixed joints. Branching
is supported: a robot may have multiple leaf links and calculations may target
any link. Fixed joints do not occupy an entry in joint-state vectors, but their
transforms, masses, and inertias still affect descendants and dynamics.

Dynibo rejects topology it cannot represent, including disconnected structures,
cycles, and invalid parent-child relationships. URDF parse and model validation
fail while loading, before a workspace is created.

Joint origins and moving-joint axes must be finite. Revolute and prismatic
position limits must be finite and ordered; continuous joints have unbounded
position limits. Moving-joint velocity limits must be finite and non-negative.
Mimic constraints are currently rejected with a model error rather than loaded
as independent degrees of freedom.

## Names and IDs

`Robot.name` comes from the URDF robot name. Links retain their URDF names.
Resolve a name once and reuse the returned link ID in repeated calculations:

=== "Rust"

    ```rust
    let robot = Robot::from_urdf("robot.urdf")?;
    let tool = robot.link_id("tool")?;
    ```

=== "Python"

    ```python
    robot = Robot.from_urdf("robot.urdf")
    tool = robot.link_id("tool")
    ```

=== "C++"

    ```cpp
    dynibo::Robot robot("robot.urdf");
    const auto tool = robot.link_id("tool");
    ```

=== "C"

    ```c
    DyniboRobot *robot = NULL;
    size_t tool = 0;
    check(dynibo_robot_from_urdf("robot.urdf", &robot));
    check(dynibo_robot_link_id(robot, "tool", &tool));
    ```

A link ID is scoped to the model that produced it. Do not persist it as model
data or use it with an independently loaded robot.

## Model state and calculation state

Rust can load `RobotModel::from_urdf(path)` without allocating calculation
storage. Its metadata methods match those on a robot. `model.robot()` creates
a fixed instance at the world origin, and `model.floating_robot()?` creates a
floating instance after checking positive root mass. `robot.model()` and
`model.clone()` share the immutable model without allocating a workspace.
All these instances share link IDs, while independent URDF loads have distinct
ID scopes. A model handle contains no fixed-base pose; use `robot.fork()` to
copy the current fixed pose together with independent calculation storage.

Topology and inertial data come from URDF. Joint position, velocity, and
acceleration are supplied to each calculation. A fixed `Robot` persists its
base frame; floating pose, velocity, and acceleration are the `BaseState`
passed to each `FloatingRobot` calculation. See [Fixed and Floating
Bases](fixed-and-floating-bases.md).
