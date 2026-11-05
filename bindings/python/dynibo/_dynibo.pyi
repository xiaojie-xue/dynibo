"""Typed public API for the native robot kinematics and dynamics extension.

Joint arrays are one-dimensional, finite, and in breadth-first non-fixed URDF
joint order. Units are SI. Matrix outputs are flat float64 arrays in column-major
order. See the Python API reference for shared array and exception contracts.
"""

from os import PathLike
from types import TracebackType
from typing import NoReturn, Sequence, final

import numpy as np
from numpy.typing import ArrayLike, NDArray

class DyniboError(RuntimeError):
    """Base exception for native model, solver, and panic failures.

    Invalid calculation inputs raise ValueError or TypeError separately.
    """

class ModelError(DyniboError):
    """URDF loading, parsing, topology, or inertial-model failure."""

class SolverError(DyniboError):
    """Numerical failure, singular inertia, or inverse-kinematics non-convergence."""

class PanicError(DyniboError):
    """A caught native panic or a poisoned calculation lock."""

@final
class Pose:
    """Immutable rigid pose with translation in metres and an xyzw quaternion.

    A pose returned by forward kinematics maps target-link points to world.
    Quaternion normalization and finite-value checks occur when used in a
    calculation; a zero quaternion raises ValueError.
    """

    def __new__(
        cls,
        translation: ArrayLike = ...,
        rotation_xyzw: ArrayLike = ...,
    ) -> Pose:
        """Create a pose from exactly three translation and four rotation values."""
        ...
    @property
    def translation(self) -> tuple[float, float, float]:
        """Translation (x, y, z), in metres."""
        ...
    @property
    def rotation_xyzw(self) -> tuple[float, float, float, float]:
        """Quaternion coefficients (x, y, z, w)."""
        ...
    def __eq__(self, other: object, /) -> bool: ...

@final
class Twist:
    """Immutable angular-first spatial motion value.

    Kinematic results use world axes at the target origin (or selected tool
    point). Angular velocity is rad/s; linear velocity is m/s. Acceleration
    results use rad/s² and m/s² and contain classical linear acceleration.
    """

    def __new__(
        cls,
        angular: ArrayLike = ...,
        linear: ArrayLike = ...,
    ) -> Twist:
        """Create a motion value from two sequences of exactly three values."""
        ...
    @property
    def angular(self) -> tuple[float, float, float]:
        """Angular (x, y, z) components."""
        ...
    @property
    def linear(self) -> tuple[float, float, float]:
        """Linear (x, y, z) components."""
        ...
    def __eq__(self, other: object, /) -> bool: ...

@final
class BaseState:
    """Immutable floating-base world pose, velocity, and classical acceleration.

    Motion is expressed in world axes at the URDF root origin. Missing values
    default to identity pose and zero motion. Joint vectors remain length J;
    do not concatenate base coordinates onto them.
    """

    def __new__(
        cls,
        frame: Pose | None = None,
        velocity: Twist | None = None,
        acceleration: Twist | None = None,
    ) -> BaseState:
        """Create an explicit base-state input for FloatingRobot calculations."""
        ...
    @property
    def frame(self) -> Pose:
        """World pose of the URDF root link."""
        ...
    @property
    def velocity(self) -> Twist:
        """World-expressed root angular and linear velocity."""
        ...
    @property
    def acceleration(self) -> Twist:
        """World-expressed root angular and classical linear acceleration."""
        ...
    def __eq__(self, other: object, /) -> bool: ...

@final
class IkOptions:
    """Immutable settings for fixed-base damped least-squares inverse kinematics.

    Tolerances, damping, and step norm must be finite and positive when used.
    max_iterations must be a positive integer; bool is rejected.
    """

    def __new__(
        cls,
        max_iterations: int | None = None,
        translation_tolerance: float = 1.0e-6,
        rotation_tolerance: float = 1.0e-6,
        damping: float = 1.0e-3,
        max_step_norm: float = 0.5,
    ) -> IkOptions:
        """Create settings; None selects 100 iterations.

        Raises:
            TypeError: max_iterations is not an integer or is a bool.
            ValueError: max_iterations is not positive.
        """
        ...
    @property
    def max_iterations(self) -> int:
        """Maximum joint updates (default 100)."""
        ...
    @property
    def translation_tolerance(self) -> float:
        """Accepted Euclidean position error in metres."""
        ...
    @property
    def rotation_tolerance(self) -> float:
        """Accepted rotation-vector norm in radians."""
        ...
    @property
    def damping(self) -> float:
        """Damping lambda in Jᵀ(JJᵀ + lambda²I)⁻¹."""
        ...
    @property
    def max_step_norm(self) -> float:
        """Maximum Euclidean norm of a joint update."""
        ...
    def __eq__(self, other: object, /) -> bool: ...

@final
class Load:
    """Immutable resisting wrench at a link origin, expressed in link-local axes.

    Torque is in N·m; force is in N. Loads add to required inverse-dynamics
    effort and subtract from available forward-dynamics effort. Negate a
    physical applied wrench before supplying it as a resisting load.
    """

    def __new__(
        cls,
        link_id: int,
        torque: ArrayLike = ...,
        force: ArrayLike = ...,
    ) -> Load:
        """Create a load using a link ID from the robot receiving it."""
        ...
    @property
    def link_id(self) -> int:
        """Index returned by the receiving robot's link_id method."""
        ...
    @property
    def torque(self) -> tuple[float, float, float]:
        """Link-local torque about the link origin, in N·m."""
        ...
    @property
    def force(self) -> tuple[float, float, float]:
        """Link-local force at the link origin, in N."""
        ...
    def __eq__(self, other: object, /) -> bool: ...

@final
class LoadBuffer:
    """Reusable native load storage obtained from robot.load_buffer().

    There is no public constructor. Capacity is reserved for one wrench per
    link. Only the creating model and its fork() instances may consume this
    buffer. Rejected updates leave existing entries unchanged.
    """

    def __new__(cls) -> NoReturn: ...
    def set(
        self,
        link_id: int,
        torque: ArrayLike = ...,
        force: ArrayLike = ...,
    ) -> None:
        """Replace a link's resisting wrench in local axes at its origin.

        Raises:
            ValueError: The link ID is invalid or a component is not finite.
        """
        ...
    def add(
        self,
        link_id: int,
        torque: ArrayLike = ...,
        force: ArrayLike = ...,
    ) -> None:
        """Accumulate a local resisting wrench, inserting it if absent.

        Raises:
            ValueError: The link ID or resulting wrench is invalid.
        """
        ...
    def remove(self, link_id: int) -> None:
        """Remove a valid link's entry if present; retain reserved capacity.

        Raises:
            ValueError: The link ID is invalid.
        """
        ...
    def clear(self) -> None:
        """Remove all entries while retaining reserved capacity."""
        ...
    def __len__(self) -> int:
        """Number of stored link wrenches."""
        ...

@final
class Robot:
    """Fixed-base robot with reusable native calculation storage.

    The root pose is prescribed; G = J. Use set_base_frame to change it.
    Use as a context manager or call close(). Calls on one instance serialize;
    fork() provides independent storage for parallel calculations.
    """

    def __new__(cls, path: str | PathLike[str]) -> Robot:
        """Load a tree URDF from a filesystem path.

        Raises:
            ModelError: The file, topology, joint types, or inertial model is invalid."""
        ...
    @classmethod
    def from_urdf(cls, path: str | PathLike[str]) -> Robot:
        """Load a URDF; equivalent to the constructor.

        Raises:
            ModelError: URDF loading or model validation fails."""
        ...
    def fork(self) -> Robot:
        """Share immutable model data and create independent calculation storage.

        The fork remains usable after the original is closed. Link IDs and load
        buffers from this model remain valid on the fork."""
        ...
    def load_buffer(self) -> LoadBuffer:
        """Create reusable resisting-load storage scoped to this model."""
        ...
    def close(self) -> None:
        """Release this instance's native resources. Repeated calls are harmless.

        Subsequent calculations and metadata access raise RuntimeError."""
        ...
    def __enter__(self) -> Robot:
        """Return the open robot for a with block."""
        ...
    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_value: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        """Close the robot without suppressing exceptions."""
        ...
    @property
    def name(self) -> str:
        """URDF robot name."""
        ...
    @property
    def joint_count(self) -> int:
        """J: number of non-fixed scalar joints in breadth-first URDF order."""
        ...
    @property
    def generalized_count(self) -> int:
        """G: generalized vector size, equal to J."""
        ...
    @property
    def link_count(self) -> int:
        """Number of URDF links, including root and fixed links."""
        ...
    def link_id(self, name: str) -> int:
        """Resolve a URDF link name to an index for this model.

        Do not reuse integer IDs from independently loaded models.

        Raises:
            ValueError: The name does not identify a link."""
        ...
    def set_base_frame(self, frame: Pose) -> None:
        """Prescribe the root pose in world coordinates (identity by default).

        Raises:
            ValueError: The pose is non-finite or has a zero quaternion."""
        ...
    def forward_kinematics_all(
        self, q: ArrayLike, out: NDArray[np.float64] | None = None
    ) -> NDArray[np.float64]:
        """Compute every world link pose, in link-index order.

        Args:
            q: Joint positions, shape (J,), in radians or metres.
            out: Optional writable contiguous float64 array of shape (7 * L,).

        Returns:
            Flat float64 array of length 7 * L; each consecutive record contains
            x, y, z, qx, qy, qz, qw. L is link_count. Returns out itself when supplied.

        Raises:
            ValueError: Invalid lengths, non-finite inputs, target, or output buffer.
            TypeError: An input has an incompatible Python or NumPy type."""
        ...
    def forward_kinematics(self, q: ArrayLike, target: int) -> Pose:
        """Compute the target-link world pose.

        Args:
            q: Joint positions, shape (J,), in radians or metres.
            target: Link index returned by link_id.

        Returns:
            World-from-target transform; translation in metres, quaternion in xyzw order.

        Raises:
            ValueError: Invalid lengths, non-finite inputs, target, or output buffer.
            TypeError: An input has an incompatible Python or NumPy type."""
        ...
    def jacobian(
        self, q: ArrayLike, target: int, out: NDArray[np.float64] | None = None
    ) -> NDArray[np.float64]:
        """Compute the world geometric Jacobian at the target-link origin.

        Args:
            q: Joint positions, shape (J,), in radians or metres.
            target: Link index returned by link_id.
            out: Optional writable contiguous float64 array of shape (6 * G,).

        Returns:
            Flat column-major 6 x G array. Reshape with (6, G), order="F".
            Rows are angular then linear, expressed in world axes at target origin.
            Returns out itself when supplied.

        Raises:
            ValueError: Invalid lengths, non-finite inputs, target, or output buffer.
            TypeError: An input has an incompatible Python or NumPy type."""
        ...
    def jacobian_derivative(
        self,
        q: ArrayLike,
        qd: ArrayLike,
        target: int,
        out: NDArray[np.float64] | None = None,
    ) -> NDArray[np.float64]:
        """Compute the time derivative of the world geometric Jacobian.

        Args:
            q: Joint positions, shape (J,), in radians or metres.
            qd: Joint velocities, shape (J,).
            target: Link index returned by link_id.
            out: Optional writable contiguous float64 array of shape (6 * G,).

        Returns:
            Flat column-major 6 x G array. Reshape with (6, G), order="F".
            Rows are angular then linear, expressed in world axes at target origin.
            Returns out itself when supplied.

        Raises:
            ValueError: Invalid lengths, non-finite inputs, target, or output buffer.
            TypeError: An input has an incompatible Python or NumPy type."""
        ...
    def forward_velocity_kinematics(
        self, q: ArrayLike, qd: ArrayLike, target: int, tool: Pose | None = None
    ) -> Twist:
        """Compute world velocity at the target origin or a rigidly attached tool.

        Args:
            q: Joint positions, shape (J,), in radians or metres.
            qd: Joint velocities, shape (J,).
            target: Link index returned by link_id.
            tool: Tool pose relative to target; None uses the target origin.

        Returns:
            World-expressed angular (rad/s) and linear (m/s) velocity at the selected point.

        Raises:
            ValueError: Invalid lengths, non-finite inputs, target, or output buffer.
            TypeError: An input has an incompatible Python or NumPy type."""
        ...
    def forward_acceleration_kinematics(
        self, q: ArrayLike, qd: ArrayLike, qdd: ArrayLike, target: int
    ) -> Twist:
        """Compute world acceleration at the target-link origin.

        Args:
            q: Joint positions, shape (J,), in radians or metres.
            qd: Joint velocities, shape (J,).
            qdd: Joint accelerations, shape (J,).
            target: Link index returned by link_id.

        Returns:
            Angular (rad/s²) and classical linear (m/s²) acceleration in world axes.

        Raises:
            ValueError: Invalid lengths, non-finite inputs, target, or output buffer.
            TypeError: An input has an incompatible Python or NumPy type."""
        ...
    def gravity(
        self,
        q: ArrayLike,
        loads: Sequence[Load] | LoadBuffer | None = None,
        out: NDArray[np.float64] | None = None,
    ) -> NDArray[np.float64]:
        """Compute gravity compensation and optional resisting-load effort.

        Joint and base velocities and accelerations are ignored.

        Args:
            q: Joint positions, shape (J,), in radians or metres.
            loads: Link-local resisting wrenches; None means no loads. A LoadBuffer
                must belong to this model or one of its forks.
            out: Optional writable contiguous float64 array of shape (G,).

        Returns:
            Float64 generalized effort array of shape (G,).
            Returns out itself when supplied.

        Raises:
            ValueError: Invalid lengths, non-finite inputs, target, or output buffer.
            TypeError: An input has an incompatible Python or NumPy type."""
        ...
    def inverse_dynamics(
        self,
        q: ArrayLike,
        qd: ArrayLike,
        qdd: ArrayLike,
        loads: Sequence[Load] | LoadBuffer | None = None,
        out: NDArray[np.float64] | None = None,
    ) -> NDArray[np.float64]:
        """Compute required generalized effort using recursive Newton-Euler (RNEA).

        Args:
            q: Joint positions, shape (J,), in radians or metres.
            qd: Joint velocities, shape (J,).
            qdd: Joint accelerations, shape (J,).
            loads: Link-local resisting wrenches; None means no loads. A LoadBuffer
                must belong to this model or one of its forks.
            out: Optional writable contiguous float64 array of shape (G,).

        Returns:
            Float64 generalized effort array of shape (G,).
            Returns out itself when supplied.

        Raises:
            ValueError: Invalid lengths, non-finite inputs, target, or output buffer.
            TypeError: An input has an incompatible Python or NumPy type."""
        ...
    def forward_dynamics(
        self,
        q: ArrayLike,
        qd: ArrayLike,
        forces: ArrayLike,
        loads: Sequence[Load] | LoadBuffer | None = None,
        out: NDArray[np.float64] | None = None,
    ) -> NDArray[np.float64]:
        """Solve generalized acceleration using the articulated-body algorithm (ABA).

        Args:
            q: Joint positions, shape (J,), in radians or metres.
            qd: Joint velocities, shape (J,).
            forces: Generalized effort, shape (G,), torque in N·m, force in N.
            loads: Link-local resisting wrenches; None means no loads. A LoadBuffer
                must belong to this model or one of its forks.
            out: Optional writable contiguous float64 array of shape (G,).

        Returns:
            Float64 generalized acceleration array of shape (G,).
            Returns out itself when supplied.

        Raises:
            ValueError: Invalid lengths, non-finite inputs, target, or output buffer.
            TypeError: An input has an incompatible Python or NumPy type.
            SolverError: Articulated inertia is singular, ill-conditioned, or fails to solve."""
        ...
    def mass_matrix(
        self, q: ArrayLike, out: NDArray[np.float64] | None = None
    ) -> NDArray[np.float64]:
        """Compute the generalized inertia matrix.

        Args:
            q: Joint positions, shape (J,), in radians or metres.
            out: Optional writable contiguous float64 array of shape (G * G,).

        Returns:
            Flat column-major G x G matrix. Reshape with (G, G), order="F".
            Returns out itself when supplied.

        Raises:
            ValueError: Invalid lengths, non-finite inputs, target, or output buffer.
            TypeError: An input has an incompatible Python or NumPy type."""
        ...
    def velocity_product_forces(
        self, q: ArrayLike, qd: ArrayLike, out: NDArray[np.float64] | None = None
    ) -> NDArray[np.float64]:
        """Compute Coriolis and centrifugal effort, excluding gravity and loads.

        Prescribed base acceleration is excluded.

        Args:
            q: Joint positions, shape (J,), in radians or metres.
            qd: Joint velocities, shape (J,).
            out: Optional writable contiguous float64 array of shape (G,).

        Returns:
            Float64 generalized effort array of shape (G,).
            Returns out itself when supplied.

        Raises:
            ValueError: Invalid lengths, non-finite inputs, target, or output buffer.
            TypeError: An input has an incompatible Python or NumPy type."""
        ...
    def inverse_kinematics(
        self,
        q: ArrayLike,
        target: int,
        desired: Pose,
        options: IkOptions | None = None,
        out: NDArray[np.float64] | None = None,
    ) -> NDArray[np.float64]:
        """Solve a fixed-base target pose using damped least squares.

        Args:
            q: Initial joint positions, shape (J,).
            target: Link index returned by link_id.
            desired: Desired world-from-target pose.
            options: Solver settings; None selects IkOptions defaults.
            out: Optional writable contiguous float64 array of shape (J,).

        Returns:
            Joint-position solution, shape (J,). Returns out itself when supplied.
            Output contents are unspecified on failure.

        Raises:
            ValueError: Invalid lengths, non-finite inputs, target, or output buffer.
            TypeError: An input has an incompatible Python or NumPy type.
            SolverError: No convergence, joint-limit violation, or numerical failure."""
        ...

@final
class FloatingRobot:
    """Floating-base robot with reusable native calculation storage.

    Base state is explicit on every calculation. G = J + 6.
    Use as a context manager or call close(). Calls on one instance serialize;
    fork() provides independent storage for parallel calculations.
    """

    def __new__(cls, path: str | PathLike[str]) -> FloatingRobot:
        """Load a tree URDF from a filesystem path.

        Raises:
            ModelError: The file, topology, joint types, or inertial model is invalid.
                A floating root must have strictly positive mass."""
        ...
    @classmethod
    def from_urdf(cls, path: str | PathLike[str]) -> FloatingRobot:
        """Load a URDF; equivalent to the constructor.

        Raises:
            ModelError: URDF loading or model validation fails."""
        ...
    def fork(self) -> FloatingRobot:
        """Share immutable model data and create independent calculation storage.

        The fork remains usable after the original is closed. Link IDs and load
        buffers from this model remain valid on the fork."""
        ...
    def load_buffer(self) -> LoadBuffer:
        """Create reusable resisting-load storage scoped to this model."""
        ...
    def close(self) -> None:
        """Release this instance's native resources. Repeated calls are harmless.

        Subsequent calculations and metadata access raise RuntimeError."""
        ...
    def __enter__(self) -> FloatingRobot:
        """Return the open robot for a with block."""
        ...
    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_value: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        """Close the robot without suppressing exceptions."""
        ...
    @property
    def name(self) -> str:
        """URDF robot name."""
        ...
    @property
    def joint_count(self) -> int:
        """J: number of non-fixed scalar joints in breadth-first URDF order."""
        ...
    @property
    def generalized_count(self) -> int:
        """G: generalized vector size, J + 6."""
        ...
    @property
    def link_count(self) -> int:
        """Number of URDF links, including root and fixed links."""
        ...
    def link_id(self, name: str) -> int:
        """Resolve a URDF link name to an index for this model.

        Do not reuse integer IDs from independently loaded models.

        Raises:
            ValueError: The name does not identify a link."""
        ...
    def forward_kinematics_all(
        self, base: BaseState, q: ArrayLike, out: NDArray[np.float64] | None = None
    ) -> NDArray[np.float64]:
        """Compute every world link pose, in link-index order.

        Args:
            base: World pose and world-expressed root motion.
            q: Joint positions, shape (J,), in radians or metres.
            out: Optional writable contiguous float64 array of shape (7 * L,).

        Returns:
            Flat float64 array of length 7 * L; each consecutive record contains
            x, y, z, qx, qy, qz, qw. L is link_count. Returns out itself when supplied.

        Raises:
            ValueError: Invalid lengths, non-finite inputs, target, or output buffer.
            TypeError: An input has an incompatible Python or NumPy type."""
        ...
    def forward_kinematics(self, base: BaseState, q: ArrayLike, target: int) -> Pose:
        """Compute the target-link world pose.

        Args:
            base: World pose and world-expressed root motion.
            q: Joint positions, shape (J,), in radians or metres.
            target: Link index returned by link_id.

        Returns:
            World-from-target transform; translation in metres, quaternion in xyzw order.

        Raises:
            ValueError: Invalid lengths, non-finite inputs, target, or output buffer.
            TypeError: An input has an incompatible Python or NumPy type."""
        ...
    def jacobian(
        self,
        base: BaseState,
        q: ArrayLike,
        target: int,
        out: NDArray[np.float64] | None = None,
    ) -> NDArray[np.float64]:
        """Compute the world geometric Jacobian at the target-link origin.

        Args:
            base: World pose and world-expressed root motion.
            q: Joint positions, shape (J,), in radians or metres.
            target: Link index returned by link_id.
            out: Optional writable contiguous float64 array of shape (6 * G,).

        Returns:
            Flat column-major 6 x G array. Reshape with (6, G), order="F".
            Rows are angular then linear, expressed in world axes at target origin.
            Six leading columns correspond to world angular then linear base velocity.
            Returns out itself when supplied.

        Raises:
            ValueError: Invalid lengths, non-finite inputs, target, or output buffer.
            TypeError: An input has an incompatible Python or NumPy type."""
        ...
    def jacobian_derivative(
        self,
        base: BaseState,
        q: ArrayLike,
        qd: ArrayLike,
        target: int,
        out: NDArray[np.float64] | None = None,
    ) -> NDArray[np.float64]:
        """Compute the time derivative of the world geometric Jacobian.

        Args:
            base: World pose and world-expressed root motion.
            q: Joint positions, shape (J,), in radians or metres.
            qd: Joint velocities, shape (J,).
            target: Link index returned by link_id.
            out: Optional writable contiguous float64 array of shape (6 * G,).

        Returns:
            Flat column-major 6 x G array. Reshape with (6, G), order="F".
            Rows are angular then linear, expressed in world axes at target origin.
            Six leading columns correspond to world angular then linear base velocity.
            Returns out itself when supplied.

        Raises:
            ValueError: Invalid lengths, non-finite inputs, target, or output buffer.
            TypeError: An input has an incompatible Python or NumPy type."""
        ...
    def forward_velocity_kinematics(
        self,
        base: BaseState,
        q: ArrayLike,
        qd: ArrayLike,
        target: int,
        tool: Pose | None = None,
    ) -> Twist:
        """Compute world velocity at the target origin or a rigidly attached tool.

        Args:
            base: World pose and world-expressed root motion.
            q: Joint positions, shape (J,), in radians or metres.
            qd: Joint velocities, shape (J,).
            target: Link index returned by link_id.
            tool: Tool pose relative to target; None uses the target origin.

        Returns:
            World-expressed angular (rad/s) and linear (m/s) velocity at the selected point.

        Raises:
            ValueError: Invalid lengths, non-finite inputs, target, or output buffer.
            TypeError: An input has an incompatible Python or NumPy type."""
        ...
    def forward_acceleration_kinematics(
        self, base: BaseState, q: ArrayLike, qd: ArrayLike, qdd: ArrayLike, target: int
    ) -> Twist:
        """Compute world acceleration at the target-link origin.

        Args:
            base: World pose and world-expressed root motion.
            q: Joint positions, shape (J,), in radians or metres.
            qd: Joint velocities, shape (J,).
            qdd: Joint accelerations, shape (J,).
            target: Link index returned by link_id.

        Returns:
            Angular (rad/s²) and classical linear (m/s²) acceleration in world axes.

        Raises:
            ValueError: Invalid lengths, non-finite inputs, target, or output buffer.
            TypeError: An input has an incompatible Python or NumPy type."""
        ...
    def gravity(
        self,
        base: BaseState,
        q: ArrayLike,
        loads: Sequence[Load] | LoadBuffer | None = None,
        out: NDArray[np.float64] | None = None,
    ) -> NDArray[np.float64]:
        """Compute gravity compensation and optional resisting-load effort.

        Joint and base velocities and accelerations are ignored.

        Args:
            base: World pose and world-expressed root motion.
            q: Joint positions, shape (J,), in radians or metres.
            loads: Link-local resisting wrenches; None means no loads. A LoadBuffer
                must belong to this model or one of its forks.
            out: Optional writable contiguous float64 array of shape (G,).

        Returns:
            Float64 generalized effort array of shape (G,).
            Base angular then linear entries precede scalar joint entries, all in world axes.
            Returns out itself when supplied.

        Raises:
            ValueError: Invalid lengths, non-finite inputs, target, or output buffer.
            TypeError: An input has an incompatible Python or NumPy type."""
        ...
    def inverse_dynamics(
        self,
        base: BaseState,
        q: ArrayLike,
        qd: ArrayLike,
        qdd: ArrayLike,
        loads: Sequence[Load] | LoadBuffer | None = None,
        out: NDArray[np.float64] | None = None,
    ) -> NDArray[np.float64]:
        """Compute required generalized effort using recursive Newton-Euler (RNEA).

        Args:
            base: World pose and world-expressed root motion.
            q: Joint positions, shape (J,), in radians or metres.
            qd: Joint velocities, shape (J,).
            qdd: Joint accelerations, shape (J,).
            loads: Link-local resisting wrenches; None means no loads. A LoadBuffer
                must belong to this model or one of its forks.
            out: Optional writable contiguous float64 array of shape (G,).

        Returns:
            Float64 generalized effort array of shape (G,).
            Base angular then linear entries precede scalar joint entries, all in world axes.
            Returns out itself when supplied.

        Raises:
            ValueError: Invalid lengths, non-finite inputs, target, or output buffer.
            TypeError: An input has an incompatible Python or NumPy type."""
        ...
    def forward_dynamics(
        self,
        base: BaseState,
        q: ArrayLike,
        qd: ArrayLike,
        forces: ArrayLike,
        loads: Sequence[Load] | LoadBuffer | None = None,
        out: NDArray[np.float64] | None = None,
    ) -> NDArray[np.float64]:
        """Solve generalized acceleration using the articulated-body algorithm (ABA).

        Args:
            base: World pose and world-expressed root motion.
            q: Joint positions, shape (J,), in radians or metres.
            qd: Joint velocities, shape (J,).
            forces: Generalized effort, shape (G,), torque in N·m, force in N.
            loads: Link-local resisting wrenches; None means no loads. A LoadBuffer
                must belong to this model or one of its forks.
            out: Optional writable contiguous float64 array of shape (G,).

        Returns:
            Float64 generalized acceleration array of shape (G,).
            Base angular then linear entries precede scalar joint entries, all in world axes.
            Base acceleration in the input is ignored; output contains classical
            base acceleration followed by joint accelerations.
            Returns out itself when supplied.

        Raises:
            ValueError: Invalid lengths, non-finite inputs, target, or output buffer.
            TypeError: An input has an incompatible Python or NumPy type.
            SolverError: Articulated inertia is singular, ill-conditioned, or fails to solve."""
        ...
    def mass_matrix(
        self, base: BaseState, q: ArrayLike, out: NDArray[np.float64] | None = None
    ) -> NDArray[np.float64]:
        """Compute the generalized inertia matrix.

        Args:
            base: World pose and world-expressed root motion.
            q: Joint positions, shape (J,), in radians or metres.
            out: Optional writable contiguous float64 array of shape (G * G,).

        Returns:
            Flat column-major G x G matrix. Reshape with (G, G), order="F".
            Base angular then linear entries precede scalar joint entries
            in both matrix axes.
            Returns out itself when supplied.

        Raises:
            ValueError: Invalid lengths, non-finite inputs, target, or output buffer.
            TypeError: An input has an incompatible Python or NumPy type."""
        ...
    def velocity_product_forces(
        self,
        base: BaseState,
        q: ArrayLike,
        qd: ArrayLike,
        out: NDArray[np.float64] | None = None,
    ) -> NDArray[np.float64]:
        """Compute Coriolis and centrifugal effort, excluding gravity and loads.

        Prescribed base acceleration is excluded.

        Args:
            base: World pose and world-expressed root motion.
            q: Joint positions, shape (J,), in radians or metres.
            qd: Joint velocities, shape (J,).
            out: Optional writable contiguous float64 array of shape (G,).

        Returns:
            Float64 generalized effort array of shape (G,).
            Base angular then linear entries precede scalar joint entries, all in world axes.
            Returns out itself when supplied.

        Raises:
            ValueError: Invalid lengths, non-finite inputs, target, or output buffer.
            TypeError: An input has an incompatible Python or NumPy type."""
        ...

__all__ = [
    "Pose",
    "Twist",
    "BaseState",
    "Load",
    "LoadBuffer",
    "IkOptions",
    "Robot",
    "FloatingRobot",
    "DyniboError",
    "ModelError",
    "SolverError",
    "PanicError",
]
