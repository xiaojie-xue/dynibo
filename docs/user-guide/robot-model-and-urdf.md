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

Topology and inertial data come from URDF. Joint position, velocity, and
acceleration are supplied to each calculation. A fixed `Robot` persists its
base frame; floating pose, velocity, and acceleration are the `BaseState`
passed to each `FloatingRobot` calculation. See [Fixed and Floating
Bases](fixed-and-floating-bases.md).
