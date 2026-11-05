# Python API Reference

This reference is generated from the typed public interface shipped in the
Python wheel. Import all public types from `dynibo`, not `dynibo._dynibo`.

## Shared calculation contract

- `J = robot.joint_count`; `G = robot.generalized_count`; `L = robot.link_count`.
  `G = J` for `Robot` and `G = J + 6` for `FloatingRobot`.
- q, qd, and qdd are finite one-dimensional joint arrays of length J, in
  breadth-first non-fixed URDF joint order. `numpy.typing.ArrayLike` in the
  signatures describes accepted values, not arbitrary dimensions: scalars and
  two-dimensional inputs are rejected. Contiguous float64 NumPy inputs avoid
  copies; sequences and other supported numeric dtypes are converted.
- `Pose`, `Twist`, `Load`, and `LoadBuffer.set` / `add` also accept
  one-dimensional numeric NumPy arrays or Python sequences for vector inputs.
  Vectors must contain exactly three values, or four for `rotation_xyzw`;
  these lengths are checked at runtime.
- Array results are one-dimensional float64 NumPy arrays. Jacobians use
  `reshape((6, G), order="F")`; mass matrices use
  `reshape((G, G), order="F")`. The first three spatial rows are angular.
  All-link poses instead use consecutive seven-value records and
  `reshape((L, 7))`: x, y, z, qx, qy, qz, qw.
- Optional `out` must be writable, contiguous, one-dimensional float64 storage
  of exactly the documented length. It must not overlap inputs. A successful
  call returns that same array. Treat output contents as unspecified on error.
- Poses and kinematic motion use world axes at the selected origin. Base
  generalized entries are world angular then linear; joint entries follow.
  SI units are used throughout.
- `Load` and `LoadBuffer` contain **resisting** wrenches expressed in link-local
  axes at link origins. Negate a physical applied wrench before supplying it.
- Invalid numerical inputs raise `ValueError`; incompatible Python/NumPy types
  raise `TypeError`. Model failures raise `ModelError`, solver failures raise
  `SolverError`, and caught native panics raise `PanicError`. The last three
  derive from `DyniboError`, which derives from `RuntimeError`. Using a closed
  robot raises `RuntimeError`.
- Use a context manager or `close()`. Calls on one instance serialize; use
  `fork()` for independent calculation storage. A link index or load buffer
  belongs to its creating model and can be used by its forks.

For complete executable examples, see the [task recipes](../recipes/index.md).

## Construction

Native classes implement construction through `__new__`; call the class
normally using these signatures. The typed wheel also includes their precise
constructor annotations.

| Call | Defaults and meaning |
|---|---|
| `Robot(path)` / `Robot.from_urdf(path)` | Filesystem path as `str` or `os.PathLike[str]` |
| `FloatingRobot(path)` / `FloatingRobot.from_urdf(path)` | Same, with positive root mass required |
| `Pose(translation=(0, 0, 0), rotation_xyzw=(0, 0, 0, 1))` | Three metre values and four xyzw quaternion values |
| `Twist(angular=(0, 0, 0), linear=(0, 0, 0))` | Two three-element motion sequences |
| `BaseState(frame=None, velocity=None, acceleration=None)` | Identity `Pose`, zero velocity and classical acceleration |
| `IkOptions(max_iterations=None, translation_tolerance=1e-6, rotation_tolerance=1e-6, damping=1e-3, max_step_norm=0.5)` | `None` means 100 iterations; tolerances in metres and radians |
| `Load(link_id, torque=(0, 0, 0), force=(0, 0, 0))` | Three local N·m values and three local N values |
| `robot.load_buffer()` | The supported way to construct a `LoadBuffer` |

## Fixed-base robot

::: dynibo.Robot

## Floating-base robot

::: dynibo.FloatingRobot

## Spatial values and solver settings

::: dynibo.Pose

::: dynibo.Twist

::: dynibo.BaseState

::: dynibo.IkOptions

## Resisting loads

::: dynibo.Load

::: dynibo.LoadBuffer

## Exceptions

::: dynibo.DyniboError

::: dynibo.ModelError

::: dynibo.SolverError

::: dynibo.PanicError
