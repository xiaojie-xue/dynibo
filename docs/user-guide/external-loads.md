# External Loads

Gravity, inverse dynamics, and forward dynamics can include resisting wrenches
at link origins. Each load pairs a model-scoped link ID with torque and force
components. The wrench is added to the generalized effort required by inverse
dynamics and subtracted from the effort available to forward dynamics; use the
opposite sign when starting from a physical force applied to the robot.

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

## Reusable loads

Rust and Python robots provide `load_buffer()`. It reserves capacity once for
one load per link. `set` replaces a load, `add` accumulates it, and `remove` or
`clear` retain capacity. Rejected updates leave existing loads unchanged.
Rust passes `buffer.as_slice()` to dynamics. Python accepts the buffer directly:

```python
loads = robot.load_buffer()
loads.set(tool, force=(0.0, 0.0, -10.0))
robot.gravity(q, loads=loads, out=output)
loads.clear()
```

A Python `LoadBuffer` belongs to its creating model.
An independently loaded robot rejects it, even when empty. The
buffer is borrowed during a native call; concurrent mutation is rejected.
Existing Python lists of `Load` remain supported and are converted per call.

## No-load calls

Rust, Python, and C++ accept an empty collection. In C, pass `NULL` only when
`load_count` is zero. The caller owns the load array, and dynibo does not retain
it after the call.
