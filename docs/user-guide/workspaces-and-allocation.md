# Workspaces and Allocation

Runtime-sized robot algorithms need scratch arrays for transforms, velocities,
composite inertias, solver steps, and traversal paths. Dynibo allocates these
buffers once in a model-scoped workspace and reuses them.

## Binding behavior

| Interface | Workspace ownership | Calculation outputs |
|---|---|---|
| Rust | One workspace owned by each `Robot` or `FloatingRobot` | Caller supplies matrix and force buffers |
| Python | One native workspace owned by each `Robot` or `FloatingRobot` | NumPy arrays/value objects are returned; `out=` reuses caller storage |
| C++ | One native workspace owned by each `dynibo::Robot` or `dynibo::FloatingRobot` | Return a `std::vector`, or reuse an output vector with `*_into` |
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
resize those buffers. Python can reuse an `out=` NumPy array; without one it
allocates a result array. A Python `LoadBuffer` also avoids rebuilding the native
load vector on each call; Python object handling is not an allocation-free API.
C++ return-container methods allocate, while `jacobian_into`,
`jacobian_derivative_into`, `mass_matrix_into`, `velocity_product_forces_into`,
`gravity_into`, `inverse_dynamics_into`, `forward_dynamics_into`, and fixed-base
`inverse_kinematics_into` accept an exactly sized output vector. They never resize
it. Put the output after required inputs and before optional loads/IK options,
for example `robot.inverse_dynamics_into(q, qd, qdd, output, loads)`.

## Model scope

Each `Robot` or `FloatingRobot` instance owns a workspace scoped to its immutable model. `fork()`
creates fresh calculation storage while sharing that model.

Rust also exposes a read-only `RobotModel` without workspace storage. The C
model handles use this representation; only explicit C workspaces allocate
calculation buffers. Python `robot.fork()` shares the model and link-ID table,
allocating a fresh workspace and lock. Closing either Python instance leaves
the others usable. Fixed-base forks initially copy the base pose, and later
changes to either instance's pose are independent.

## Parallel calculations

Each Rust `Robot` or `FloatingRobot` is mutable and may participate in only one
calculation at a time. Use `fork()` to create an instance per concurrent calculation. Python
serializes calls on one `Robot` or `FloatingRobot`; use `fork()` for parallel work
without reparsing URDF or duplicating the model.
C++ performs no internal locking, so use a separate `Robot` or `FloatingRobot` per worker.
