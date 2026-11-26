"""Python interface to the dynibo robot kinematics and dynamics library.

Shared conventions across all classes and methods:

- Units: SI (metres, radians, seconds, kilograms, newtons, newton-metres).
- Joint ordering: `q`, `qd`, and `qdd` contain `joint_count` values for
  non-fixed URDF joints in breadth-first topological order from the root link
  (preserving URDF joint order among children of the same parent).
- Generalized coordinates: `G == joint_count` for fixed-base `Robot` and
  `G == joint_count + 6` for `FloatingRobot`. Floating-base generalized vectors
  prepend world-frame angular (3) then linear (3) base components at the
  root-link origin.
- Twists: world-frame classical velocity or classical acceleration in
  angular-first order `[wx, wy, wz, vx, vy, vz]`.
- External loads: resisting wrenches `[tx, ty, tz, fx, fy, fz]` at the target
  link origin, expressed in that link's local frame (opposite sign to a
  physical force applied to the robot).
- Matrices: flat 1-D `float64` NumPy arrays in column-major order. Use
  `arr.reshape((rows, cols), order="F")` to view as a 2-D array.
"""

from __future__ import annotations

from os import PathLike
from types import TracebackType
from typing import Sequence

import numpy as np
from numpy.typing import ArrayLike, NDArray

__version__: str

class DyniboError(RuntimeError):
    """Base exception for errors raised by the native dynibo library."""

class ModelError(DyniboError):
    """Raised when a URDF file cannot be parsed or describes an invalid model."""

class SolverError(DyniboError):
    """Raised on numerical failure, singular inertia, or IK non-convergence."""

class PanicError(DyniboError):
    """Raised when an internal Rust panic is caught at the native boundary."""

class LinkId(int):
    """Model-scoped link identifier.

    Subclasses `int` so index usage such as `poses[link_id]` works directly,
    while tagging instances created by `robot.link_id(...)` or
    `robot.link_id_at(...)` with the owning model to reject cross-model use.
    """

    def __new__(cls, index: int) -> LinkId:
        """Create an untagged link identifier from a non-negative integer index."""

    @property
    def index(self) -> int:
        """Zero-based link enumeration index (0 is the root link)."""

class Pose:
    """Rigid-body pose with translation in metres and a unit `(x, y, z, w)` quaternion."""

    def __init__(
        self,
        translation: Sequence[float] = (0.0, 0.0, 0.0),
        rotation_xyzw: Sequence[float] = (0.0, 0.0, 0.0, 1.0),
    ) -> None:
        """Create a rigid-body pose.

        Args:
            translation: Position `(x, y, z)` in metres.
            rotation_xyzw: Unit orientation quaternion `(x, y, z, w)`.
        """

    @property
    def translation(self) -> tuple[float, float, float]:
        """Position `(x, y, z)` in metres."""

    @property
    def rotation_xyzw(self) -> tuple[float, float, float, float]:
        """Unit orientation quaternion `(x, y, z, w)`."""

class Twist:
    """Angular-first classical velocity or classical acceleration vector.

    Components are ordered `[angular_x, angular_y, angular_z, linear_x, linear_y, linear_z]`.
    Kinematics outputs and floating-base states are expressed in the world frame.
    """

    def __init__(
        self,
        angular: Sequence[float] = (0.0, 0.0, 0.0),
        linear: Sequence[float] = (0.0, 0.0, 0.0),
    ) -> None:
        """Create a twist from angular and linear 3-vectors.

        Args:
            angular: Angular velocity (rad/s) or angular acceleration (rad/s^2).
            linear: Linear velocity (m/s) or linear acceleration (m/s^2).
        """

    @property
    def angular(self) -> tuple[float, float, float]:
        """Angular component `(wx, wy, wz)`."""

    @property
    def linear(self) -> tuple[float, float, float]:
        """Linear component `(vx, vy, vz)`."""

class BaseState:
    """Explicit floating-root pose, classical velocity, and classical acceleration.

    Passed as the first argument to every `FloatingRobot` calculation. All
    motion components are expressed in the world frame at the root-link origin.
    """

    def __init__(
        self,
        frame: Pose | None = None,
        velocity: Twist | None = None,
        acceleration: Twist | None = None,
    ) -> None:
        """Create a floating-base state.

        Args:
            frame: Root-link world pose. Defaults to identity.
            velocity: World-frame classical velocity at the root origin. Defaults to zero.
            acceleration: World-frame classical acceleration at the root origin.
                Defaults to zero; ignored by `forward_dynamics`.
        """

    @property
    def frame(self) -> Pose:
        """Root-link pose in the world frame."""

    @property
    def velocity(self) -> Twist:
        """World-frame classical velocity at the root-link origin."""

    @property
    def acceleration(self) -> Twist:
        """World-frame classical acceleration at the root-link origin."""

class Load:
    """Resisting external wrench applied at a link's origin in that link's local frame.

    Added to the generalized effort required by `gravity` and `inverse_dynamics`,
    and subtracted from the effort available to `forward_dynamics`. Use the opposite
    sign when starting from a physical force applied to the robot.
    """

    def __init__(
        self,
        link_id: LinkId | int,
        torque: Sequence[float] = (0.0, 0.0, 0.0),
        force: Sequence[float] = (0.0, 0.0, 0.0),
    ) -> None:
        """Create a link-local resisting external load.

        Args:
            link_id: Model-scoped `LinkId` from `robot.link_id(...)`. Plain `int` is deprecated.
            torque: Resisting torque `(tx, ty, tz)` in N*m, in the link frame.
            force: Resisting force `(fx, fy, fz)` in N, in the link frame.
        """

    @property
    def link_id(self) -> LinkId | int:
        """Target link identifier."""

    @property
    def torque(self) -> tuple[float, float, float]:
        """Resisting torque `(tx, ty, tz)` in N*m in the link frame."""

    @property
    def force(self) -> tuple[float, float, float]:
        """Resisting force `(fx, fy, fz)` in N in the link frame."""

class LoadBuffer:
    """Reusable, model-scoped storage for at most one resisting wrench per link.

    Created via `robot.load_buffer()` and accepted directly as the `loads`
    argument to dynamics methods. Can be shared across `fork()` instances of
    the same model, but is rejected by independently loaded robots.
    """

    def set(
        self,
        link_id: LinkId | int,
        torque: Sequence[float] = (0.0, 0.0, 0.0),
        force: Sequence[float] = (0.0, 0.0, 0.0),
    ) -> None:
        """Replace a link's resisting wrench, inserting it if absent.

        Args:
            link_id: Model-scoped link identifier.
            torque: Resisting torque `(tx, ty, tz)` in N*m in the link frame.
            force: Resisting force `(fx, fy, fz)` in N in the link frame.

        Raises:
            ValueError: If `link_id` is invalid or any component is non-finite.
        """

    def add(
        self,
        link_id: LinkId | int,
        torque: Sequence[float] = (0.0, 0.0, 0.0),
        force: Sequence[float] = (0.0, 0.0, 0.0),
    ) -> None:
        """Accumulate a resisting wrench onto a link's stored load.

        Raises:
            ValueError: If `link_id` is invalid or any component is non-finite.
            SolverError: If accumulation overflows to a non-finite value.
        """

    def remove(self, link_id: LinkId | int) -> None:
        """Remove a link's stored load if present, retaining capacity."""

    def clear(self) -> None:
        """Remove all stored loads while retaining allocated capacity."""

    def __len__(self) -> int:
        """Return the number of links currently carrying a stored load."""

class IkOptions:
    """Configuration for damped-least-squares inverse kinematics."""

    def __init__(
        self,
        max_iterations: int | None = None,
        translation_tolerance: float | None = None,
        rotation_tolerance: float | None = None,
        damping: float | None = None,
        max_step_norm: float | None = None,
    ) -> None:
        """Create inverse-kinematics solver options.

        Omitted (`None`) fields use the library defaults.

        Args:
            max_iterations: Maximum update iterations (default `100`).
            translation_tolerance: Position error norm threshold in metres (default `1e-6`).
            rotation_tolerance: Rotation-vector error norm threshold in radians (default `1e-6`).
            damping: Damping factor `lambda` (default `1e-3`).
            max_step_norm: Maximum Euclidean norm per joint step (default `0.5`).
        """

    @property
    def max_iterations(self) -> int:
        """Maximum number of joint-update iterations."""

    @property
    def translation_tolerance(self) -> float:
        """Accepted Euclidean translation error in metres."""

    @property
    def rotation_tolerance(self) -> float:
        """Accepted rotation-vector error norm in radians."""

    @property
    def damping(self) -> float:
        """Damping factor `lambda` in `J^T (J J^T + lambda^2 I)^-1`."""

    @property
    def max_step_norm(self) -> float:
        """Maximum Euclidean norm of a single joint update."""

class Robot:
    """Fixed-base robot model with an owned, reusable native workspace.

    Calls on the same instance are serialized by an internal lock. Use `fork()`
    to run parallel calculations on the same URDF model without reparsing it.
    Use as a context manager (`with Robot.from_urdf(...) as r:`) or call
    `close()` to release native resources deterministically.
    """

    def __init__(self, path: str | PathLike[str]) -> None:
        """Load and validate a fixed-base robot model from a URDF file.

        Raises:
            ModelError: If the URDF cannot be read, parsed, or validated.
        """

    @classmethod
    def from_urdf(cls, path: str | PathLike[str]) -> Robot:
        """Load and validate a fixed-base robot model from a URDF file.

        Raises:
            ModelError: If the URDF cannot be read, parsed, or validated.
        """

    def fork(self) -> Robot:
        """Create an independent robot sharing the immutable model with fresh storage.

        Copies the current fixed base frame and gravity vector; subsequent
        changes on either instance are independent.
        """

    def load_buffer(self) -> LoadBuffer:
        """Create a reusable `LoadBuffer` scoped to this robot's model."""

    def close(self) -> None:
        """Release the underlying native robot and workspace handles."""

    def __enter__(self) -> Robot:
        """Enter context manager and return this robot."""

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_value: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        """Close the robot on context exit."""

    @property
    def name(self) -> str:
        """Robot name declared in the URDF."""

    @property
    def joint_count(self) -> int:
        """Number of non-fixed (active) URDF joints, `J`."""

    @property
    def generalized_count(self) -> int:
        """Generalized coordinate count, `G == J` for fixed-base robots."""

    @property
    def link_count(self) -> int:
        """Total number of links in the model, including the root link."""

    @property
    def gravity_vector(self) -> tuple[float, float, float]:
        """World-frame gravitational acceleration `(gx, gy, gz)` in m/s^2.

        Defaults to `(0.0, 0.0, -9.80665)`.
        """

    def set_gravity(self, gravity: Sequence[float]) -> None:
        """Replace the world-frame gravitational acceleration `(gx, gy, gz)` in m/s^2.

        Raises:
            ValueError: If any component is non-finite.
        """

    def link_id(self, name: str) -> LinkId:
        """Resolve a URDF link name to a model-scoped `LinkId`.

        Raises:
            ValueError: If no link with `name` exists in this model.
        """

    def link_id_at(self, index: int) -> LinkId:
        """Return the model-scoped `LinkId` at link enumeration `index`.

        Index `0` is the root link. Raises `ValueError` if out of range.
        """

    def set_base_frame(self, frame: Pose) -> None:
        """Replace the fixed root-link pose in the world frame.

        Raises:
            ValueError: If `frame` is non-finite or has a zero quaternion.
        """

    def forward_kinematics_all(
        self, q: ArrayLike, out: NDArray[np.float64] | None = None
    ) -> NDArray[np.float64]:
        """Compute all link world poses in link-ID order (including the root).

        Args:
            q: Joint positions of length `joint_count` in BFS topological order.
            out: Optional contiguous 1-D `float64` buffer of length `7 * link_count`.

        Returns:
            Flat 1-D `float64` array of length `7 * link_count`. Reshaped to
            `(link_count, 7)`, each row is `[x, y, z, qx, qy, qz, qw]`.
        """

    def forward_kinematics(self, q: ArrayLike, target: LinkId | int) -> Pose:
        """Compute the world-frame pose of `target` link.

        Args:
            q: Joint positions of length `joint_count` in BFS topological order.
            target: Model-scoped `LinkId` (plain `int` is deprecated).

        Returns:
            Target-link `Pose` in the world frame.
        """

    def jacobian(
        self, q: ArrayLike, target: LinkId | int, out: NDArray[np.float64] | None = None
    ) -> NDArray[np.float64]:
        """Compute the world-frame, target-origin `6 x G` geometric Jacobian.

        Args:
            q: Joint positions of length `joint_count`.
            target: Model-scoped target `LinkId`.
            out: Optional contiguous 1-D `float64` buffer of length `6 * generalized_count`.

        Returns:
            Flat 1-D column-major `float64` array of length `6 * G` with
            angular rows `0..3` before linear rows `3..6`. Reshape with
            `J.reshape((6, robot.generalized_count), order="F")`.
        """

    def jacobian_derivative(
        self, q: ArrayLike, qd: ArrayLike, target: LinkId | int, out: NDArray[np.float64] | None = None
    ) -> NDArray[np.float64]:
        """Compute the time derivative of the `6 x G` geometric Jacobian.

        Satisfies `A_target = J(q) @ qdd + Jdot(q, qd) @ qd` for classical
        acceleration at the target-link origin.

        Args:
            q: Joint positions of length `joint_count`.
            qd: Joint velocities of length `joint_count`.
            target: Model-scoped target `LinkId`.
            out: Optional contiguous 1-D `float64` buffer of length `6 * generalized_count`.

        Returns:
            Flat 1-D column-major `float64` array of length `6 * G`. Reshape
            with `Jdot.reshape((6, robot.generalized_count), order="F")`.
        """

    def forward_velocity_kinematics(
        self, q: ArrayLike, qd: ArrayLike, target: LinkId | int, tool: Pose | None = None
    ) -> Twist:
        """Compute world-frame classical velocity of a point rigidly attached to `target`.

        Args:
            q: Joint positions of length `joint_count`.
            qd: Joint velocities of length `joint_count`.
            target: Model-scoped target `LinkId`.
            tool: Optional pose relative to `target`. Defaults to link origin.

        Returns:
            World-frame angular-first `Twist` at the tool point.
        """

    def forward_acceleration_kinematics(
        self, q: ArrayLike, qd: ArrayLike, qdd: ArrayLike, target: LinkId | int
    ) -> Twist:
        """Compute world-frame classical acceleration at the `target` link origin.

        Args:
            q: Joint positions of length `joint_count`.
            qd: Joint velocities of length `joint_count`.
            qdd: Joint accelerations of length `joint_count`.
            target: Model-scoped target `LinkId`.

        Returns:
            World-frame angular-first classical acceleration `Twist`.
        """

    def gravity(
        self,
        q: ArrayLike,
        loads: Sequence[Load] | LoadBuffer | None = None,
        out: NDArray[np.float64] | None = None,
    ) -> NDArray[np.float64]:
        """Compute generalized gravity forces, optionally including external loads.

        Args:
            q: Joint positions of length `joint_count`.
            loads: Optional sequence of `Load` or a `LoadBuffer` containing
                link-local resisting wrenches.
            out: Optional contiguous 1-D `float64` buffer of length `generalized_count`.

        Returns:
            1-D `float64` array of generalized forces of length `G`.
        """

    def inverse_dynamics(
        self,
        q: ArrayLike,
        qd: ArrayLike,
        qdd: ArrayLike,
        loads: Sequence[Load] | LoadBuffer | None = None,
        out: NDArray[np.float64] | None = None,
    ) -> NDArray[np.float64]:
        """Compute generalized forces via recursive Newton-Euler (RNEA).

        Args:
            q: Joint positions of length `joint_count`.
            qd: Joint velocities of length `joint_count`.
            qdd: Joint accelerations of length `joint_count`.
            loads: Optional link-local resisting wrenches.
            out: Optional contiguous 1-D `float64` buffer of length `generalized_count`.

        Returns:
            1-D `float64` array of generalized forces of length `G`.
        """

    def forward_dynamics(
        self,
        q: ArrayLike,
        qd: ArrayLike,
        forces: ArrayLike,
        loads: Sequence[Load] | LoadBuffer | None = None,
        out: NDArray[np.float64] | None = None,
    ) -> NDArray[np.float64]:
        """Compute generalized accelerations via the articulated-body algorithm (ABA).

        Args:
            q: Joint positions of length `joint_count`.
            qd: Joint velocities of length `joint_count`.
            forces: Applied generalized forces of length `generalized_count`.
            loads: Optional link-local resisting wrenches (subtracted from effort).
            out: Optional contiguous 1-D `float64` buffer of length `generalized_count`.

        Returns:
            1-D `float64` array of joint accelerations of length `G`.

        Raises:
            SolverError: If articulated inertia is singular or results are non-finite.
        """

    def mass_matrix(
        self, q: ArrayLike, out: NDArray[np.float64] | None = None
    ) -> NDArray[np.float64]:
        """Compute the symmetric `G x G` generalized inertia matrix (CRBA).

        Args:
            q: Joint positions of length `joint_count`.
            out: Optional contiguous 1-D `float64` buffer of length `G * G`.

        Returns:
            Flat 1-D column-major `float64` array of length `G * G`. Reshape
            with `M.reshape((robot.generalized_count, robot.generalized_count), order="F")`.
        """

    def velocity_product_forces(
        self, q: ArrayLike, qd: ArrayLike, out: NDArray[np.float64] | None = None
    ) -> NDArray[np.float64]:
        """Compute Coriolis and centrifugal generalized forces `C(q, qd) @ qd`.

        Excludes gravity and external loads.

        Args:
            q: Joint positions of length `joint_count`.
            qd: Joint velocities of length `joint_count`.
            out: Optional contiguous 1-D `float64` buffer of length `generalized_count`.

        Returns:
            1-D `float64` array of length `G`.
        """

    def inverse_kinematics(
        self,
        q: ArrayLike,
        target: LinkId | int,
        desired: Pose,
        options: IkOptions | None = None,
        out: NDArray[np.float64] | None = None,
    ) -> NDArray[np.float64]:
        """Solve fixed-base inverse kinematics with damped least squares.

        Args:
            q: Initial joint positions of length `joint_count`.
            target: Model-scoped target `LinkId`.
            desired: Desired world-frame `Pose` of `target`.
            options: Optional `IkOptions`. Defaults to `IkOptions()`.
            out: Optional contiguous 1-D `float64` buffer of length `joint_count`.

        Returns:
            1-D `float64` array of solved joint positions of length `J`.

        Raises:
            SolverError: If the solver fails to converge, hits a linear-solve
                failure, or violates URDF joint limits.
        """

class FloatingRobot:
    """Floating-base robot model with an owned, reusable native workspace.

    Never stores mutable base state: supply a `BaseState` explicitly as the
    first argument to every calculation. `generalized_count == joint_count + 6`;
    generalized outputs and inputs prepend world-frame angular (3) then linear (3)
    base components at the root-link origin before the `J` joint entries.

    Calls on the same instance are serialized by an internal lock. Use `fork()`
    for parallel execution without reparsing the URDF.
    """

    def __init__(self, path: str | PathLike[str]) -> None:
        """Load and validate a floating-base robot model from a URDF file.

        Requires a root link with strictly positive mass.

        Raises:
            ModelError: If the URDF cannot be read, parsed, or validated.
        """

    @classmethod
    def from_urdf(cls, path: str | PathLike[str]) -> FloatingRobot:
        """Load and validate a floating-base robot model from a URDF file.

        Requires a root link with strictly positive mass.

        Raises:
            ModelError: If the URDF cannot be read, parsed, or validated.
        """

    def fork(self) -> FloatingRobot:
        """Create an independent floating robot sharing the immutable model.

        Copies the current gravity vector; subsequent changes are independent.
        """

    def load_buffer(self) -> LoadBuffer:
        """Create a reusable `LoadBuffer` scoped to this robot's model."""

    def close(self) -> None:
        """Release the underlying native robot and workspace handles."""

    def __enter__(self) -> FloatingRobot:
        """Enter context manager and return this robot."""

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_value: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        """Close the robot on context exit."""

    @property
    def name(self) -> str:
        """Robot name declared in the URDF."""

    @property
    def joint_count(self) -> int:
        """Number of non-fixed (active) URDF joints, `J`."""

    @property
    def generalized_count(self) -> int:
        """Generalized coordinate count, `G == J + 6` for floating-base robots."""

    @property
    def link_count(self) -> int:
        """Total number of links in the model, including the root link."""

    @property
    def gravity_vector(self) -> tuple[float, float, float]:
        """World-frame gravitational acceleration `(gx, gy, gz)` in m/s^2.

        Defaults to `(0.0, 0.0, -9.80665)`.
        """

    def set_gravity(self, gravity: Sequence[float]) -> None:
        """Replace the world-frame gravitational acceleration `(gx, gy, gz)` in m/s^2.

        Raises:
            ValueError: If any component is non-finite.
        """

    def link_id(self, name: str) -> LinkId:
        """Resolve a URDF link name to a model-scoped `LinkId`.

        Raises:
            ValueError: If no link with `name` exists in this model.
        """

    def link_id_at(self, index: int) -> LinkId:
        """Return the model-scoped `LinkId` at link enumeration `index`.

        Index `0` is the root link. Raises `ValueError` if out of range.
        """

    def forward_kinematics_all(
        self, base: BaseState, q: ArrayLike, out: NDArray[np.float64] | None = None
    ) -> NDArray[np.float64]:
        """Compute all link world poses in link-ID order (including the root).

        Args:
            base: Floating-base state (uses `base.frame`).
            q: Joint positions of length `joint_count` in BFS topological order.
            out: Optional contiguous 1-D `float64` buffer of length `7 * link_count`.

        Returns:
            Flat 1-D `float64` array of length `7 * link_count`. Reshaped to
            `(link_count, 7)`, each row is `[x, y, z, qx, qy, qz, qw]`.
        """

    def forward_kinematics(self, base: BaseState, q: ArrayLike, target: LinkId | int) -> Pose:
        """Compute the world-frame pose of `target` link.

        Args:
            base: Floating-base state (uses `base.frame`).
            q: Joint positions of length `joint_count` in BFS topological order.
            target: Model-scoped target `LinkId`.

        Returns:
            Target-link `Pose` in the world frame.
        """

    def jacobian(
        self, base: BaseState, q: ArrayLike, target: LinkId | int, out: NDArray[np.float64] | None = None
    ) -> NDArray[np.float64]:
        """Compute the world-frame, target-origin `6 x G` geometric Jacobian.

        The first six columns correspond to world-frame angular then linear base
        velocity at the root origin, followed by the `J` joint columns.

        Args:
            base: Floating-base state (uses `base.frame`).
            q: Joint positions of length `joint_count`.
            target: Model-scoped target `LinkId`.
            out: Optional contiguous 1-D `float64` buffer of length `6 * generalized_count`.

        Returns:
            Flat 1-D column-major `float64` array of length `6 * G`. Reshape
            with `J.reshape((6, robot.generalized_count), order="F")`.
        """

    def jacobian_derivative(
        self,
        base: BaseState,
        q: ArrayLike,
        qd: ArrayLike,
        target: LinkId | int,
        out: NDArray[np.float64] | None = None,
    ) -> NDArray[np.float64]:
        """Compute the time derivative of the `6 x G` geometric Jacobian.

        Satisfies `A_target = J(q) @ nu_dot + Jdot(q, nu) @ nu` for world-frame
        classical acceleration at the target-link origin.

        Args:
            base: Floating-base state (uses `base.frame` and `base.velocity`).
            q: Joint positions of length `joint_count`.
            qd: Joint velocities of length `joint_count`.
            target: Model-scoped target `LinkId`.
            out: Optional contiguous 1-D `float64` buffer of length `6 * generalized_count`.

        Returns:
            Flat 1-D column-major `float64` array of length `6 * G`. Reshape
            with `Jdot.reshape((6, robot.generalized_count), order="F")`.
        """

    def forward_velocity_kinematics(
        self, base: BaseState, q: ArrayLike, qd: ArrayLike, target: LinkId | int, tool: Pose | None = None
    ) -> Twist:
        """Compute world-frame classical velocity of a point rigidly attached to `target`.

        Args:
            base: Floating-base state (uses `base.frame` and `base.velocity`).
            q: Joint positions of length `joint_count`.
            qd: Joint velocities of length `joint_count`.
            target: Model-scoped target `LinkId`.
            tool: Optional pose relative to `target`. Defaults to link origin.

        Returns:
            World-frame angular-first `Twist` at the tool point.
        """

    def forward_acceleration_kinematics(
        self, base: BaseState, q: ArrayLike, qd: ArrayLike, qdd: ArrayLike, target: LinkId | int
    ) -> Twist:
        """Compute world-frame classical acceleration at the `target` link origin.

        Args:
            base: Floating-base state (uses `frame`, `velocity`, and `acceleration`).
            q: Joint positions of length `joint_count`.
            qd: Joint velocities of length `joint_count`.
            qdd: Joint accelerations of length `joint_count`.
            target: Model-scoped target `LinkId`.

        Returns:
            World-frame angular-first classical acceleration `Twist`.
        """

    def gravity(
        self,
        base: BaseState,
        q: ArrayLike,
        loads: Sequence[Load] | LoadBuffer | None = None,
        out: NDArray[np.float64] | None = None,
    ) -> NDArray[np.float64]:
        """Compute generalized gravity forces, optionally including external loads.

        Args:
            base: Floating-base state (uses `base.frame`).
            q: Joint positions of length `joint_count`.
            loads: Optional link-local resisting wrenches.
            out: Optional contiguous 1-D `float64` buffer of length `generalized_count`.

        Returns:
            1-D `float64` array of length `G` containing the world-frame root
            wrench `[torque_xyz, force_xyz]` followed by `J` joint forces.
        """

    def inverse_dynamics(
        self,
        base: BaseState,
        q: ArrayLike,
        qd: ArrayLike,
        qdd: ArrayLike,
        loads: Sequence[Load] | LoadBuffer | None = None,
        out: NDArray[np.float64] | None = None,
    ) -> NDArray[np.float64]:
        """Compute generalized forces via recursive Newton-Euler (RNEA).

        Args:
            base: Floating-base state (uses `frame`, `velocity`, and `acceleration`).
            q: Joint positions of length `joint_count`.
            qd: Joint velocities of length `joint_count`.
            qdd: Joint accelerations of length `joint_count`.
            loads: Optional link-local resisting wrenches.
            out: Optional contiguous 1-D `float64` buffer of length `generalized_count`.

        Returns:
            1-D `float64` array of length `G` containing the world-frame root
            wrench `[torque_xyz, force_xyz]` followed by `J` joint forces.
        """

    def forward_dynamics(
        self,
        base: BaseState,
        q: ArrayLike,
        qd: ArrayLike,
        forces: ArrayLike,
        loads: Sequence[Load] | LoadBuffer | None = None,
        out: NDArray[np.float64] | None = None,
    ) -> NDArray[np.float64]:
        """Compute generalized accelerations via the articulated-body algorithm (ABA).

        Uses `base.frame` and `base.velocity`; `base.acceleration` is ignored
        because base acceleration is part of the solved output. Set `forces[:6]`
        to zero for an unactuated floating base.

        Args:
            base: Floating-base state (pose and velocity).
            q: Joint positions of length `joint_count`.
            qd: Joint velocities of length `joint_count`.
            forces: Applied generalized forces of length `G` (base wrench then joints).
            loads: Optional link-local resisting wrenches (subtracted from effort).
            out: Optional contiguous 1-D `float64` buffer of length `G`.

        Returns:
            1-D `float64` array of length `G` containing world-frame classical
            base acceleration `[angular_xyz, linear_xyz]` followed by `J` joint
            accelerations.

        Raises:
            SolverError: If articulated inertia is singular, ill-conditioned, or
                produces non-finite results.
        """

    def mass_matrix(
        self, base: BaseState, q: ArrayLike, out: NDArray[np.float64] | None = None
    ) -> NDArray[np.float64]:
        """Compute the symmetric `G x G` generalized inertia matrix (CRBA).

        Args:
            base: Floating-base state (uses `base.frame`).
            q: Joint positions of length `joint_count`.
            out: Optional contiguous 1-D `float64` buffer of length `G * G`.

        Returns:
            Flat 1-D column-major `float64` array of length `G * G`. Reshape
            with `M.reshape((robot.generalized_count, robot.generalized_count), order="F")`.
        """

    def velocity_product_forces(
        self, base: BaseState, q: ArrayLike, qd: ArrayLike, out: NDArray[np.float64] | None = None
    ) -> NDArray[np.float64]:
        """Compute Coriolis and centrifugal generalized forces `C(q, nu) @ nu`.

        Excludes gravity, prescribed base acceleration, and external loads.

        Args:
            base: Floating-base state (uses `base.frame` and `base.velocity`).
            q: Joint positions of length `joint_count`.
            qd: Joint velocities of length `joint_count`.
            out: Optional contiguous 1-D `float64` buffer of length `generalized_count`.

        Returns:
            1-D `float64` array of length `G` (root wrench then joint forces).
        """
