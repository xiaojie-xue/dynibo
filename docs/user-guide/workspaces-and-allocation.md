# Workspaces and Allocation

Runtime-sized robot algorithms need scratch arrays for transforms, velocities,
composite inertias, solver steps, and traversal paths. Dynibo allocates these
buffers once in a model-scoped workspace and reuses them.

## Binding behavior

| Interface | Workspace ownership | Calculation outputs |
|---|---|---|
| Rust | Created with `robot.workspace()` and passed explicitly | Caller supplies matrix and force buffers |
| Python | One native workspace owned by each `Robot` | Python tuples/value objects are returned |
| C++ | One native workspace owned by each `dynibo::Robot` | `std::vector` or value objects are returned |
| C | Explicit `DyniboWorkspace*` | Caller supplies buffers and structs |

Rust and C give direct control over output allocation:

=== "Rust"

    ```rust
    let base = BaseState::fixed();
    let mut workspace = robot.workspace();
    let mut jacobian = vec![0.0; 6 * robot.generalized_count()];
    robot.jacobian(&base, &q, target, &mut workspace, &mut jacobian)?;
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

A workspace belongs to the model that created it, including Rust clones of that
model. Passing a workspace from an unrelated model is an error even if both
models have the same number of joints.

## Parallel calculations

A workspace is mutable and may participate in only one calculation at a time.
Use a separate workspace per concurrent Rust or C call. Python serializes calls
on one `Robot`; use separate robot instances for parallel work. C++ performs no
internal locking, so use a separate `Robot` per worker.
