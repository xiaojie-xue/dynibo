# Choosing Dynibo and Migrating from Pinocchio

Dynibo is a focused library for URDF-based robot kinematics and rigid-body
dynamics. Choose it when you need fixed- or floating-base calculations in Rust,
or a Python/NumPy interface backed by the same Rust core. Each calculation
instance owns its reusable storage, and matrix/vector operations support
caller-owned output buffers.

## Check the required capabilities

| Requirement | Dynibo support and boundary |
|---|---|
| Runtime URDF models | Tree topology with revolute, continuous, prismatic, and fixed joints |
| Fixed-base manipulation | FK, world geometric Jacobians and derivatives, velocity/acceleration kinematics, damped least-squares IK |
| Floating-base dynamics | Explicit root pose/motion, generalized mass, gravity, velocity-product forces, RNEA and ABA |
| Repeated CPU calculations | Reusable instance storage; Rust output slices and Python `out=` arrays |
| Multiple language interfaces | Rust, Python, C, and C++ over one Rust implementation |
| Collision or visual geometry | No collision detection or renderer; meshes are not needed for calculations |
| Ground contacts or constraints | No contact solver, closed-loop mechanism solver, or balance controller |
| Differentiable or GPU workflows | No public automatic-differentiation or GPU backend |
| Complete simulation | No built-in time stepper; integration and control belong to the application |

If your application already depends on Pinocchio-specific algorithms or
ecosystem tools, audit those dependencies before migrating. Dynibo does not
provide a drop-in compatible API. Performance depends on operation, model,
bindings, and hardware; use the [reproducible benchmarks](https://github.com/xiaojie-xue/dynibo/tree/main/benches)
to evaluate your workload.

## Translate a fixed-base calculation

The table uses `pin` for the Pinocchio Python module and `robot` for a Dynibo
`Robot`. `target` is a URDF link name. Pinocchio's model/data and frame APIs are
described in its [official documentation](https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/).

| Task | Pinocchio Python | Dynibo Python |
|---|---|---|
| Load | `model = pin.buildModelFromUrdf(path)`; `data = model.createData()` | `robot = Robot.from_urdf(path)` |
| Resolve target | `frame = model.getFrameId(target)` | `link = robot.link_id(target)` |
| Pose | `pin.framesForwardKinematics(model, data, q)`; `data.oMf[frame]` | `robot.forward_kinematics(q, link)` |
| Jacobian | `pin.computeFrameJacobian(model, data, q, frame, pin.LOCAL_WORLD_ALIGNED)` | `robot.jacobian(q, link).reshape((6, robot.generalized_count), order="F")` |
| Gravity | `pin.computeGeneralizedGravity(model, data, q)` | `robot.gravity(q)` |
| Inverse dynamics | `pin.rnea(model, data, q, qd, qdd)` | `robot.inverse_dynamics(q, qd, qdd)` |
| Forward dynamics | `pin.aba(model, data, q, qd, tau)` | `robot.forward_dynamics(q, qd, tau)` |
| Mass matrix | `pin.crba(model, data, q)`; mirror its upper triangle if needed | `robot.mass_matrix(q).reshape((G, G), order="F")` |
| Independent calculation storage | Another `model.createData()` | `robot.fork()` |

## Convert conventions explicitly

1. **Match joints by name.** Dynibo uses breadth-first non-fixed URDF joint
   order. Do not assume an existing Pinocchio vector uses that order. Rust
   exposes `joint_name(index)`; for the Python interface, establish the mapping
   from your model before transferring vectors. For a Pinocchio continuous
   joint, q may use a cosine/sine pair rather than Dynibo's scalar angle;
   convert the representation as well as the order.
2. **Match frames and points.** Dynibo's target is a URDF link, and its
   Jacobian is world-aligned at that link origin. Compare with Pinocchio's
   `LOCAL_WORLD_ALIGNED` for the corresponding frame. Pinocchio's `WORLD`
   reference does not mean the same point convention.
3. **Swap spatial row blocks.** Pinocchio Motion vectors/Jacobians use linear
   then angular components. Dynibo uses angular then linear. For a matched
   fixed-base model, compare Dynibo's Jacobian with
   `np.vstack((J_pin[3:, :], J_pin[:3, :]))`, after mapping joint columns.
4. **Reshape outputs.** Dynibo matrices are flat float64 arrays in column-major
   order. Use `order="F"`; the default NumPy reshape order changes the matrix.
   Dynibo poses expose `translation` and `rotation_xyzw`, not an SE3 matrix.
5. **Map external loads.** Dynibo loads are link-local resisting wrenches at
   link origins, torque first. Transform the point and frame, and negate a
   physical applied wrench before constructing a Dynibo `Load`. Do not copy a
   joint-indexed external-force collection directly.

## Floating-base migration

Dynibo keeps joint coordinates separate from the root. Pass a `BaseState`
containing a world-from-root `Pose` and world-expressed root angular/linear
velocity and classical acceleration. q, qd, and qdd remain length J.
Generalized effort, acceleration, and matrix axes have G = J + 6 entries.

A Pinocchio free-flyer configuration/tangent vector cannot be copied directly:
the configuration representation, spatial ordering, and base-velocity frame
must all be converted. In particular, spatial acceleration and classical
linear acceleration differ by a velocity cross term. Match these conventions
before comparing numerical outputs. Forward dynamics ignores the supplied
base acceleration because acceleration is its result.

## Verify a migration

First compare poses for several nonzero joint configurations, then Jacobians
with matched ordering and frames. Check mass-matrix symmetry and the identity
`tau = M @ qdd + velocity_product_forces + gravity` without loads. Finally,
pass RNEA's effort to ABA and verify that it recovers the requested
acceleration. Repeat with your load conversion and floating-base mapping.

Start with the [executable task recipes](../recipes/index.md), then consult the
[Python API reference](../reference/python.md) and
[frame conventions](../user-guide/frames-and-spatial-vectors.md).
