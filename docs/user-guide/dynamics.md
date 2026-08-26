# Dynamics

Dynibo's dynamics operations use the manipulator equation:

$$
\tau = M(q)\dot\nu + C(q,\nu)\nu + g(q).
$$

For a fixed base, outputs contain scalar joint forces. For a floating base,
they begin with a six-element world-frame root wrench followed by joint forces.

## Mass matrix

`mass_matrix` computes the symmetric `G x G` generalized inertia matrix in
column-major order. Fixed joints occupy no row or column, but their subtree
inertia contributes to moving ancestors.

## Velocity-product forces

`velocity_product_forces` computes Coriolis and centrifugal generalized forces.
Gravity, prescribed base acceleration, and external loads are excluded.

## Gravity

With no external loads, `gravity` is the zero-velocity, zero-acceleration
inverse-dynamics term:

$$
g(q) = \tau(q,0,0).
$$

The operation can include link-local external loads without requiring nonzero
joint velocity or acceleration.

## Inverse dynamics

`inverse_dynamics` uses recursive Newton--Euler dynamics and includes joint
state, supplied base motion, gravity, and optional external loads. With a
stationary base and no loads, it satisfies the manipulator equation above.

## Forward dynamics

`forward_dynamics` uses the linear-time articulated-body algorithm (ABA) to
solve

$$
\dot\nu = M(q)^{-1}\left(\tau-C(q,\nu)\nu-g(q)-\tau_{\mathrm{load}}\right).
$$

For a floating base, input forces and output accelerations begin with
world-frame angular and linear base components. The supplied base pose and
velocity participate in the calculation; a floating base's stored acceleration
is ignored because it is part of the result. A singular joint or floating-base articulated
inertia produces a solver error rather than non-finite acceleration.

## Calling the operations

=== "Rust"

    ```rust
    robot.mass_matrix(&q, &mut mass)?;
    robot.velocity_product_forces(&q, &qd, &mut velocity)?;
    robot.gravity(&q, &loads, &mut gravity)?;
    robot.inverse_dynamics(&q, &qd, &qdd, &loads, &mut forces)?;
    robot.forward_dynamics(&q, &qd, &forces, &loads, &mut accelerations)?;
    ```

=== "Python"

    ```python
    mass = robot.mass_matrix(q)
    velocity = robot.velocity_product_forces(q, qd)
    gravity = robot.gravity(q, loads)
    forces = robot.inverse_dynamics(q, qd, qdd, loads)
    accelerations = robot.forward_dynamics(q, qd, forces, loads)
    ```

=== "C++"

    ```cpp
    const auto mass = robot.mass_matrix(q);
    const auto velocity = robot.velocity_product_forces(q, qd);
    const auto gravity = robot.gravity(q, loads);
    const auto forces = robot.inverse_dynamics(q, qd, qdd, loads);
    const auto accelerations = robot.forward_dynamics(q, qd, forces, loads);
    ```

=== "C"

    ```c
    check(dynibo_mass_matrix(
        robot, workspace, q, J, mass, G * G));
    check(dynibo_gravity(
        robot, workspace, q, J, loads, load_count, gravity, G));
    check(dynibo_inverse_dynamics(
        robot, workspace, q, qd, qdd, J,
        loads, load_count, forces, G));
    check(dynibo_forward_dynamics(
        robot, workspace, q, qd, J, forces, G,
        loads, load_count, accelerations, G));
    ```

See [External Loads](external-loads.md) before supplying loads and [Frames and
Spatial Vectors](frames-and-spatial-vectors.md) before interpreting base wrench
or matrix results.
