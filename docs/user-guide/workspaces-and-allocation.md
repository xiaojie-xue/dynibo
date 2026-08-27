# Workspaces and Allocation

Runtime-sized robot algorithms need scratch arrays for transforms, velocities,
composite inertias, solver steps, and traversal paths. Dynibo allocates these
buffers once in a model-scoped workspace and reuses them.

## Binding behavior

| Interface | Workspace ownership | Calculation outputs |
|---|---|---|
| Rust | One workspace owned by each `Robot` or `FloatingRobot` | Caller supplies matrix and force buffers |
| Python | One native workspace owned by each `Robot` or `FloatingRobot` | Python tuples/value objects are returned |
| C++ | One native workspace owned by each `dynibo::Robot` or `dynibo::FloatingRobot` | `std::vector` or value objects are returned |
| C | Explicit `DyniboWorkspace*` or `DyniboFloatingWorkspace*` | Caller supplies buffers and structs |

Rust and C give direct control over output allocation:

=== "Rust"

    ```rust
    let mut jacobian = vec![0.0; 6 * robot.generalized_count()];
    robot.jacobian(&q, target, &mut jacobian)?;
    ```

=== "C"

    ```c
    DyniboWorkspace *workspace = NULL;
    check(dynibo_workspace_create(robot, &workspace));
    check(dynibo_jacobian(
        robot, workspace, q, J, target, jacobian, 6 * G));
    ```

Creating a workspace allocates all internal scratch buffers. Reusing it does not
resize those buffers. Python and C++ still allocate language-level return
containers for results such as matrices.

## Model scope

Each `Robot` or `FloatingRobot` instance owns a workspace scoped to its immutable model. `fork()`
creates fresh calculation storage while sharing that model.

## Parallel calculations

Each Rust `Robot` or `FloatingRobot` is mutable and may participate in only one
calculation at a time. Use `fork()` to create an instance per concurrent calculation. Python
serializes calls on one `Robot` or `FloatingRobot`; use separate robot instances for parallel work.
C++ performs no internal locking, so use a separate `Robot` or `FloatingRobot` per worker.
