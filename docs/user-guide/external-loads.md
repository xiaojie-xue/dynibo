# External Loads

Gravity and inverse dynamics can include wrenches applied to link origins. Each
load pairs a model-scoped link ID with torque and force components.

## Frame and point

A load is expressed in the selected link's local frame and applied at that link
origin. If a force is applied at an offset point, first shift it to an equivalent
wrench at the link origin. Components use torque-first order.

## Creating loads

=== "Rust"

    ```rust
    use dynibo::{IndexedLoad, Wrench};
    use nalgebra::Vector3;

    let load = IndexedLoad {
        link: tool,
        wrench: Wrench::new(
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, -10.0),
        ),
    };
    ```

=== "Python"

    ```python
    from dynibo import Load

    load = Load(
        link_id=tool,
        torque=(0.0, 0.0, 0.0),
        force=(0.0, 0.0, -10.0),
    )
    gravity = robot.gravity(q, [load])
    ```

=== "C++"

    ```cpp
    DyniboLoad load{
        tool,
        {0.0, 0.0, 0.0},
        {0.0, 0.0, -10.0},
    };
    const auto gravity = robot.gravity(q, {load});
    ```

=== "C"

    ```c
    const DyniboLoad load = {
        .link_id = tool,
        .torque = {0.0, 0.0, 0.0},
        .force = {0.0, 0.0, -10.0},
    };
    check(dynibo_gravity(
        robot, workspace, q, J, &load, 1, output, G));
    ```

Every link ID must come from the robot used for the calculation. Multiple loads
may target the same or different links; dynibo accumulates their contribution.

## No-load calls

Rust, Python, and C++ accept an empty collection. In C, pass `NULL` only when
`load_count` is zero. The caller owns the load array, and dynibo does not retain
it after the call.
