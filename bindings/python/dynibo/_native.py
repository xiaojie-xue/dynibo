"""Dependency-free ctypes wrapper around dynibo's stable C ABI."""

from __future__ import annotations

import ctypes as ct
import ctypes.util
import os
import sys
import threading
from dataclasses import dataclass
from enum import IntEnum
from functools import wraps
from pathlib import Path
from typing import Iterable, Sequence


def _library_name() -> str:
    if sys.platform == "win32":
        return "dynibo_c.dll"
    if sys.platform == "darwin":
        return "libdynibo_c.dylib"
    return "libdynibo_c.so"


def _load_library() -> ct.CDLL:
    candidates = []
    override = os.environ.get("DYNIBO_LIBRARY_PATH")
    if override:
        candidates.append(override)
    candidates.append(str(Path(__file__).with_name(_library_name())))
    system = ctypes.util.find_library("dynibo_c")
    if system:
        candidates.append(system)
    errors = []
    for candidate in candidates:
        try:
            return ct.CDLL(candidate)
        except OSError as error:
            errors.append(f"{candidate}: {error}")
    raise ImportError("could not load dynibo native library:\n" + "\n".join(errors))


class _Pose(ct.Structure):
    _fields_ = [("translation", ct.c_double * 3), ("rotation_xyzw", ct.c_double * 4)]


class _Twist(ct.Structure):
    _fields_ = [("angular", ct.c_double * 3), ("linear", ct.c_double * 3)]


class _Load(ct.Structure):
    _fields_ = [
        ("link_id", ct.c_size_t),
        ("torque", ct.c_double * 3),
        ("force", ct.c_double * 3),
    ]


class _IkOptions(ct.Structure):
    _fields_ = [
        ("max_iterations", ct.c_size_t),
        ("translation_tolerance", ct.c_double),
        ("rotation_tolerance", ct.c_double),
        ("damping", ct.c_double),
        ("max_step_norm", ct.c_double),
    ]


class BaseMode(IntEnum):
    """Connection mode of the URDF root link."""

    FIXED = 0
    FLOATING = 1


@dataclass(frozen=True)
class Pose:
    """A rigid-body pose.

    Attributes:
        translation: Translation in metres as `(x, y, z)`.
        rotation_xyzw: Unit quaternion in `(x, y, z, w)` order.

    Notes:
        The identity pose is used by default. Quaternion normalization is
        validated by native calculations that consume the pose.
    """

    translation: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rotation_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)


@dataclass(frozen=True)
class Twist:
    """An angular-first spatial velocity or acceleration.

    Its flat order is `(angular_x, angular_y, angular_z, linear_x, linear_y,
    linear_z)`. Kinematics results are expressed in the world frame.

    Attributes:
        angular: Angular component as `(x, y, z)`.
        linear: Linear component as `(x, y, z)`.
    """

    angular: tuple[float, float, float] = (0.0, 0.0, 0.0)
    linear: tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class Load:
    """An external wrench applied at a link origin.

    Attributes:
        link_id: Identifier returned by `Robot.link_id()`.
        torque: Link-local torque as `(x, y, z)`; it precedes `force` in the
            corresponding wrench vector.
        force: Link-local force as `(x, y, z)`.
    """

    link_id: int
    torque: tuple[float, float, float] = (0.0, 0.0, 0.0)
    force: tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class IkOptions:
    """Damped-least-squares inverse-kinematics configuration.

    Attributes:
        max_iterations: Maximum number of joint updates.
        translation_tolerance: Accepted Euclidean position error in metres.
        rotation_tolerance: Accepted rotation-vector norm in radians.
        damping: Damping factor in the least-squares update.
        max_step_norm: Maximum Euclidean norm of one joint update.
    """

    max_iterations: int = 100
    translation_tolerance: float = 1.0e-6
    rotation_tolerance: float = 1.0e-6
    damping: float = 1.0e-3
    max_step_norm: float = 0.5


class DyniboError(RuntimeError):
    """Base class for native dynibo model and calculation errors."""


class ModelError(DyniboError):
    """A robot description could not be loaded or represented."""


class SolverError(DyniboError):
    """An iterative numerical calculation did not produce a valid result."""


class PanicError(DyniboError):
    """The native library caught an unexpected internal panic."""


_lib = _load_library()
_robot_p = ct.c_void_p
_workspace_p = ct.c_void_p
_double_p = ct.POINTER(ct.c_double)

_lib.dynibo_last_error_message.restype = ct.c_char_p
_lib.dynibo_version.restype = ct.c_char_p
_lib.dynibo_robot_from_urdf.argtypes = [ct.c_char_p, ct.POINTER(_robot_p)]
_lib.dynibo_robot_from_urdf.restype = ct.c_int
_lib.dynibo_robot_from_urdf_with_base.argtypes = [
    ct.c_char_p, ct.c_int, ct.POINTER(_robot_p),
]
_lib.dynibo_robot_from_urdf_with_base.restype = ct.c_int
_lib.dynibo_robot_destroy.argtypes = [_robot_p]
_lib.dynibo_robot_name.argtypes = [_robot_p]
_lib.dynibo_robot_name.restype = ct.c_char_p
_lib.dynibo_robot_joint_count.argtypes = [_robot_p]
_lib.dynibo_robot_joint_count.restype = ct.c_size_t
_lib.dynibo_robot_generalized_count.argtypes = [_robot_p]
_lib.dynibo_robot_generalized_count.restype = ct.c_size_t
_lib.dynibo_robot_set_base_frame.argtypes = [_robot_p, ct.POINTER(_Pose)]
_lib.dynibo_robot_set_base_frame.restype = ct.c_int
_lib.dynibo_robot_set_floating_base_state.argtypes = [
    _robot_p, ct.POINTER(_Pose), _Twist, _Twist,
]
_lib.dynibo_robot_set_floating_base_state.restype = ct.c_int
_lib.dynibo_robot_link_count.argtypes = [_robot_p]
_lib.dynibo_robot_link_count.restype = ct.c_size_t
_lib.dynibo_robot_link_id.argtypes = [_robot_p, ct.c_char_p, ct.POINTER(ct.c_size_t)]
_lib.dynibo_robot_link_id.restype = ct.c_int
_lib.dynibo_workspace_create.argtypes = [_robot_p, ct.POINTER(_workspace_p)]
_lib.dynibo_workspace_create.restype = ct.c_int
_lib.dynibo_workspace_destroy.argtypes = [_workspace_p]
_lib.dynibo_forward_kinematics.argtypes = [
    _robot_p, _workspace_p, _double_p, ct.c_size_t, ct.c_size_t, ct.POINTER(_Pose)
]
_lib.dynibo_forward_kinematics.restype = ct.c_int
_lib.dynibo_jacobian.argtypes = [
    _robot_p, _workspace_p, _double_p, ct.c_size_t, ct.c_size_t,
    _double_p, ct.c_size_t,
]
_lib.dynibo_jacobian.restype = ct.c_int
_lib.dynibo_jacobian_derivative.argtypes = [
    _robot_p, _workspace_p, _double_p, _double_p, ct.c_size_t, ct.c_size_t,
    _double_p, ct.c_size_t,
]
_lib.dynibo_jacobian_derivative.restype = ct.c_int
_lib.dynibo_mass_matrix.argtypes = [
    _robot_p, _workspace_p, _double_p, ct.c_size_t, _double_p, ct.c_size_t,
]
_lib.dynibo_mass_matrix.restype = ct.c_int
_lib.dynibo_velocity_product_forces.argtypes = [
    _robot_p, _workspace_p, _double_p, _double_p, ct.c_size_t, _double_p, ct.c_size_t,
]
_lib.dynibo_velocity_product_forces.restype = ct.c_int
_lib.dynibo_inverse_kinematics.argtypes = [
    _robot_p, _workspace_p, _double_p, ct.c_size_t, ct.c_size_t,
    ct.POINTER(_Pose), _IkOptions, _double_p, ct.c_size_t,
]
_lib.dynibo_inverse_kinematics.restype = ct.c_int
_lib.dynibo_forward_velocity_kinematics.argtypes = [
    _robot_p, _workspace_p, _double_p, _double_p, ct.c_size_t, ct.c_size_t,
    ct.POINTER(_Pose), ct.POINTER(_Twist),
]
_lib.dynibo_forward_velocity_kinematics.restype = ct.c_int
_lib.dynibo_forward_acceleration_kinematics.argtypes = [
    _robot_p, _workspace_p, _double_p, _double_p, _double_p,
    ct.c_size_t, ct.c_size_t, ct.POINTER(_Twist),
]
_lib.dynibo_forward_acceleration_kinematics.restype = ct.c_int
_lib.dynibo_gravity.argtypes = [
    _robot_p, _workspace_p, _double_p, ct.c_size_t, ct.POINTER(_Load),
    ct.c_size_t, _double_p, ct.c_size_t,
]
_lib.dynibo_gravity.restype = ct.c_int
_lib.dynibo_inverse_dynamics.argtypes = [
    _robot_p, _workspace_p, _double_p, _double_p, _double_p, ct.c_size_t,
    ct.POINTER(_Load), ct.c_size_t, _double_p, ct.c_size_t,
]
_lib.dynibo_inverse_dynamics.restype = ct.c_int
_lib.dynibo_forward_dynamics.argtypes = [
    _robot_p, _workspace_p, _double_p, _double_p, ct.c_size_t,
    _double_p, ct.c_size_t, ct.POINTER(_Load), ct.c_size_t,
    _double_p, ct.c_size_t,
]
_lib.dynibo_forward_dynamics.restype = ct.c_int


def _check(status: int) -> None:
    if status != 0:
        raw_message = _lib.dynibo_last_error_message()
        message = raw_message.decode("utf-8", "replace") if raw_message else "unknown dynibo error"
        if status == 1:
            raise ValueError(message)
        if status == 2:
            raise ModelError(message)
        if status == 3:
            raise PanicError(message)
        if status == 4:
            raise SolverError(message)
        raise DyniboError(message)


def _array(values: Sequence[float], name: str) -> ct.Array[ct.c_double]:
    try:
        snapshot = tuple(float(value) for value in values)
        return (ct.c_double * len(snapshot))(*snapshot)
    except (TypeError, ValueError) as error:
        raise TypeError(f"{name} must be a finite-sized sequence of numbers") from error


def _fixed_array(values: Sequence[float], length: int, name: str) -> ct.Array[ct.c_double]:
    result = _array(values, name)
    if len(result) != length:
        raise ValueError(f"{name} must contain exactly {length} elements")
    return result


def _require_same_length(
    q: ct.Array[ct.c_double], **states: ct.Array[ct.c_double]
) -> None:
    for name, values in states.items():
        if len(values) != len(q):
            raise ValueError(f"q and {name} must have the same length")


def _pose(value: Pose) -> _Pose:
    return _Pose(
        _fixed_array(value.translation, 3, "pose translation"),
        _fixed_array(value.rotation_xyzw, 4, "pose quaternion"),
    )


def _twist(value: Twist) -> _Twist:
    return _Twist(
        _fixed_array(value.angular, 3, "twist angular"),
        _fixed_array(value.linear, 3, "twist linear"),
    )


def _loads(values: Iterable[Load]) -> ct.Array[_Load]:
    values = tuple(values)
    return (_Load * len(values))(*(
        _Load(
            value.link_id,
            _fixed_array(value.torque, 3, "load torque"),
            _fixed_array(value.force, 3, "load force"),
        )
        for value in values
    ))


def _synchronized(method):
    """Serialize access to one Robot's native handles and mutable workspace."""
    @wraps(method)
    def locked(self, *args, **kwargs):
        with self._lock:
            return method(self, *args, **kwargs)

    return locked


class Robot:
    """A URDF robot with a reusable calculation workspace.

    Args:
        urdf_path: Path to the URDF file to load.
        base_mode: Whether the URDF root link is fixed or floating.

    Raises:
        ModelError: If the file cannot be read, parsed, or represented as a
            supported tree model.
        PanicError: If the native library encounters an unexpected failure.

    Notes:
        A `Robot` is a context manager and should be closed after use. Its
        native calls are serialized so one instance can be shared safely
        between threads. Use distinct instances for calculations that must run
        in parallel.
    """

    def __init__(
        self,
        urdf_path: str | os.PathLike[str],
        base_mode: BaseMode | str = BaseMode.FIXED,
    ):
        self._lock = threading.RLock()
        self._robot = _robot_p()
        self._workspace = _workspace_p()
        if isinstance(base_mode, str):
            try:
                base_mode = BaseMode[base_mode.upper()]
            except KeyError as error:
                raise ValueError("base_mode must be 'fixed' or 'floating'") from error
        elif not isinstance(base_mode, BaseMode):
            raise TypeError("base_mode must be BaseMode, 'fixed', or 'floating'")
        _check(_lib.dynibo_robot_from_urdf_with_base(
            os.fsencode(urdf_path), int(base_mode), ct.byref(self._robot),
        ))
        try:
            _check(_lib.dynibo_workspace_create(self._robot, ct.byref(self._workspace)))
        except Exception:
            _lib.dynibo_robot_destroy(self._robot)
            self._robot = _robot_p()
            raise

    @classmethod
    def from_urdf(cls, urdf_path: str | os.PathLike[str]) -> "Robot":
        """Load a fixed-base robot from a URDF file."""
        return cls(urdf_path, BaseMode.FIXED)

    @classmethod
    def from_urdf_with_base(
        cls,
        urdf_path: str | os.PathLike[str],
        base_mode: BaseMode | str,
    ) -> "Robot":
        """Load a robot from a URDF file with an explicit base mode."""
        return cls(urdf_path, base_mode)

    @_synchronized
    def close(self) -> None:
        """Release native resources; calling this more than once is safe."""
        if self._workspace:
            _lib.dynibo_workspace_destroy(self._workspace)
            self._workspace = _workspace_p()
        if self._robot:
            _lib.dynibo_robot_destroy(self._robot)
            self._robot = _robot_p()

    def __enter__(self) -> "Robot":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def __del__(self) -> None:
        self.close()

    @property
    @_synchronized
    def name(self) -> str:
        """The robot name declared in the URDF."""
        value = _lib.dynibo_robot_name(self._robot)
        if value is None:
            raise RuntimeError("robot is closed")
        return value.decode("utf-8")

    @property
    @_synchronized
    def joint_count(self) -> int:
        """The number of non-fixed joints in the model."""
        return int(_lib.dynibo_robot_joint_count(self._robot))

    @property
    @_synchronized
    def generalized_count(self) -> int:
        """The generalized-vector size selected by the base mode.

        A floating-base vector starts with world-frame angular then linear
        components `(omega_x, omega_y, omega_z, v_x, v_y, v_z)`, followed by
        non-fixed joints in URDF order.
        """
        return int(_lib.dynibo_robot_generalized_count(self._robot))

    @_synchronized
    def set_base_frame(self, frame: Pose) -> None:
        """Set the root-link pose used by every calculation.

        This operation is valid for fixed-base and floating-base robots.
        """
        frame_c = _pose(frame)
        _check(_lib.dynibo_robot_set_base_frame(
            self._robot,
            ct.byref(frame_c),
        ))

    @_synchronized
    def set_floating_base_state(
        self,
        frame: Pose,
        velocity: Twist = Twist(),
        acceleration: Twist = Twist(),
    ) -> None:
        """Replace the pose and classical motion of a floating base.

        `velocity` and `acceleration` are angular-first and expressed in the
        world frame at the root-link origin.
        """
        frame_c = _pose(frame)
        _check(_lib.dynibo_robot_set_floating_base_state(
            self._robot,
            ct.byref(frame_c),
            _twist(velocity),
            _twist(acceleration),
        ))

    @property
    @_synchronized
    def link_count(self) -> int:
        """The number of links in the model, including the root link."""
        return int(_lib.dynibo_robot_link_count(self._robot))

    @_synchronized
    def link_id(self, name: str) -> int:
        """Look up a link identifier by its URDF name.

        Args:
            name: Link name exactly as declared in the URDF.

        Returns:
            An identifier for use as a `target` or `Load.link_id` with this
            robot.

        Raises:
            ValueError: If the link does not exist.
        """
        result = ct.c_size_t()
        _check(_lib.dynibo_robot_link_id(self._robot, name.encode(), ct.byref(result)))
        return int(result.value)

    @_synchronized
    def forward_kinematics(self, q: Sequence[float], target: int) -> Pose:
        r"""Compute the pose of a target link.

        For the joints on the root-to-target path, the returned pose is

        \[
        {}^W T_{\mathrm{target}}(q) = {}^W T_{\mathrm{base}}
        \prod_{i \in \mathrm{path}} {}^{i-1}T_i(q_i).
        \]

        Args:
            q: Positions of non-fixed joints in URDF order.
            target: Link identifier returned by `link_id()`.

        Returns:
            Target-link pose in the world frame.

        Raises:
            ValueError: If an input length or link identifier is invalid.
            DyniboError: If the native calculation fails.
        """
        q_array = _array(q, "q")
        output = _Pose()
        _check(_lib.dynibo_forward_kinematics(
            self._robot, self._workspace, q_array, len(q_array), target, ct.byref(output)
        ))
        return Pose(tuple(output.translation), tuple(output.rotation_xyzw))

    @_synchronized
    def jacobian(self, q: Sequence[float], target: int) -> tuple[float, ...]:
        r"""Compute the geometric Jacobian of a target link.

        \[
        {}^0 V_{\mathrm{target}} = J(q) \dot q, \qquad
        J(q) = \begin{bmatrix} J_\omega(q) \\ J_v(q) \end{bmatrix}.
        \]

        Args:
            q: Non-fixed joint positions in URDF order.
            target: Link identifier returned by `link_id()`.

        Returns:
            A flat, column-major world-frame, target-origin `6 x
            generalized_count` tuple. Each column is
            angular-first: `(angular_x, angular_y, angular_z, linear_x,
            linear_y, linear_z)`. Floating-base columns precede joint columns
            and are world-frame angular then linear.

        Raises:
            ValueError: If an input length or link identifier is invalid.
            DyniboError: If the native calculation fails.
        """
        q_array = _array(q, "q")
        output = (ct.c_double * (6 * self.generalized_count))()
        _check(_lib.dynibo_jacobian(
            self._robot, self._workspace, q_array, len(q_array), target,
            output, len(output),
        ))
        return tuple(output)

    @_synchronized
    def jacobian_derivative(
        self, q: Sequence[float], qd: Sequence[float], target: int
    ) -> tuple[float, ...]:
        r"""Compute the time derivative of the geometric Jacobian.

        \[
        {}^0 A_{\mathrm{target}} = J(q) \ddot q + \dot J(q, \dot q) \dot q.
        \]

        Args:
            q: Non-fixed joint positions in URDF order.
            qd: Non-fixed joint velocities in URDF order.
            target: Link identifier returned by `link_id()`.

        Returns:
            A flat, column-major world-frame, target-origin `6 x
            generalized_count` tuple with the same angular-first column layout
            as `jacobian()`. The result satisfies
            `forward_acceleration_kinematics(q, qd, 0) == J_dot @ qd`.

        Raises:
            ValueError: If an input length or link identifier is invalid.
            DyniboError: If the native calculation fails.
        """
        q_array = _array(q, "q")
        qd_array = _array(qd, "qd")
        _require_same_length(q_array, qd=qd_array)
        output = (ct.c_double * (6 * self.generalized_count))()
        _check(_lib.dynibo_jacobian_derivative(
            self._robot, self._workspace, q_array, qd_array, len(q_array), target,
            output, len(output),
        ))
        return tuple(output)

    @_synchronized
    def mass_matrix(self, q: Sequence[float]) -> tuple[float, ...]:
        r"""Compute the joint-space mass matrix.

        \[
        \tau = M(q) \ddot q + C(q, \dot q) \dot q + g(q).
        \]

        Args:
            q: Non-fixed joint positions in URDF order.

        Returns:
            A flat, column-major `generalized_count x generalized_count` tuple.
            For a floating base, rows and columns are world-frame angular,
            world-frame linear, then non-fixed URDF joints. Fixed joints do not
            occupy rows or columns.

        Raises:
            ValueError: If an input length is invalid.
            DyniboError: If the native calculation fails.
        """
        q_array = _array(q, "q")
        output = (ct.c_double * (self.generalized_count * self.generalized_count))()
        _check(_lib.dynibo_mass_matrix(
            self._robot, self._workspace, q_array, len(q_array), output, len(output),
        ))
        return tuple(output)

    @_synchronized
    def velocity_product_forces(
        self, q: Sequence[float], qd: Sequence[float]
    ) -> tuple[float, ...]:
        r"""Compute velocity-induced generalized forces.

        \[
        \tau_v = C(q, \dot q)\dot q.
        \]

        Args:
            q: Non-fixed joint positions in URDF order.
            qd: Non-fixed joint velocities in URDF order.

        Returns:
            A generalized-force tuple equal to `C(q, qd) @ qd`. Gravity is
            excluded.

        Raises:
            ValueError: If an input length is invalid.
            DyniboError: If the native calculation fails.
        """
        q_array = _array(q, "q")
        qd_array = _array(qd, "qd")
        _require_same_length(q_array, qd=qd_array)
        output = (ct.c_double * self.generalized_count)()
        _check(_lib.dynibo_velocity_product_forces(
            self._robot, self._workspace, q_array, qd_array, len(q_array),
            output, len(output),
        ))
        return tuple(output)

    @_synchronized
    def inverse_kinematics(
        self, initial_q: Sequence[float], target: int, desired: Pose,
        options: IkOptions = IkOptions(),
    ) -> tuple[float, ...]:
        r"""Solve for joint positions that reach a desired target pose.

        Each iteration applies the damped-least-squares update

        \[
        \Delta q = J^T\left(JJ^T + \lambda^2 I\right)^{-1} e,
        \qquad q_{k+1} = q_k + \Delta q.
        \]

        Args:
            initial_q: Initial non-fixed joint positions in URDF order.
            target: Link identifier returned by `link_id()`.
            desired: Desired target-link pose relative to the root link.
            options: Solver tolerances and iteration limits.

        Returns:
            Solved non-fixed joint positions in URDF order.

        Raises:
            ValueError: If an input, pose, link identifier, or option is
                invalid.
            SolverError: If the solver fails numerically or does not converge.
            DyniboError: If another native calculation failure occurs.
        """
        q_array = _array(initial_q, "initial_q")
        desired_c = _pose(desired)
        options_c = _IkOptions(
            options.max_iterations, options.translation_tolerance,
            options.rotation_tolerance, options.damping, options.max_step_norm,
        )
        output = (ct.c_double * self.joint_count)()
        _check(_lib.dynibo_inverse_kinematics(
            self._robot, self._workspace, q_array, len(q_array), target,
            ct.byref(desired_c), options_c, output, len(output),
        ))
        return tuple(output)

    @_synchronized
    def forward_velocity_kinematics(
        self, q: Sequence[float], qd: Sequence[float], target: int,
        tool: Pose = Pose(),
    ) -> Twist:
        r"""Compute spatial velocity at a point on a target link.

        \[
        V_{\mathrm{tool}} = J_{\mathrm{tool}}(q) \dot q.
        \]

        Args:
            q: Non-fixed joint positions in URDF order.
            qd: Non-fixed joint velocities in URDF order.
            target: Link identifier returned by `link_id()`.
            tool: Target-to-tool pose; its translation selects the point whose
                linear velocity is returned.

        Returns:
            World-expressed, angular-first spatial velocity at the selected
            tool point. The root pose and motion come from this Robot's stored
            base state.

        Raises:
            ValueError: If an input length, pose, or link identifier is
                invalid.
            DyniboError: If the native calculation fails.
        """
        q_array, qd_array = _array(q, "q"), _array(qd, "qd")
        _require_same_length(q_array, qd=qd_array)
        tool_c, output = _pose(tool), _Twist()
        _check(_lib.dynibo_forward_velocity_kinematics(
            self._robot, self._workspace, q_array, qd_array, len(q_array), target,
            ct.byref(tool_c), ct.byref(output),
        ))
        return Twist(tuple(output.angular), tuple(output.linear))

    @_synchronized
    def forward_acceleration_kinematics(
        self, q: Sequence[float], qd: Sequence[float],
        qdd: Sequence[float], target: int,
    ) -> Twist:
        r"""Compute spatial acceleration at a target-link origin.

        \[
        A_{\mathrm{target}} = J(q) \ddot q + \dot J(q, \dot q) \dot q.
        \]

        Args:
            q: Non-fixed joint positions in URDF order.
            qd: Non-fixed joint velocities in URDF order.
            qdd: Non-fixed joint accelerations in URDF order.
            target: Link identifier returned by `link_id()`.

        Returns:
            World-expressed, angular-first spatial acceleration at the
            target-link origin.

        Raises:
            ValueError: If an input length or link identifier is invalid.
            DyniboError: If the native calculation fails.
        """
        q_array = _array(q, "q")
        qd_array = _array(qd, "qd")
        qdd_array = _array(qdd, "qdd")
        _require_same_length(q_array, qd=qd_array, qdd=qdd_array)
        output = _Twist()
        _check(_lib.dynibo_forward_acceleration_kinematics(
            self._robot, self._workspace, q_array, qd_array, qdd_array,
            len(q_array), target, ct.byref(output),
        ))
        return Twist(tuple(output.angular), tuple(output.linear))

    @_synchronized
    def gravity(
        self, q: Sequence[float], loads: Iterable[Load] = (),
    ) -> tuple[float, ...]:
        r"""Compute gravity-compensation joint forces.

        With no external loads, the returned vector is

        \[
        g(q) = \tau(q, 0, 0).
        \]

        Args:
            q: Non-fixed joint positions in URDF order.
            loads: Optional external link-local wrenches.

        Returns:
            Generalized forces in base-then-joint order. Gravity direction is
            determined by this Robot's stored base frame.

        Raises:
            ValueError: If an input or load is invalid.
            DyniboError: If the native calculation fails.
        """
        q_array, loads_c = _array(q, "q"), _loads(loads)
        output = (ct.c_double * self.generalized_count)()
        _check(_lib.dynibo_gravity(
            self._robot, self._workspace, q_array, len(q_array), loads_c,
            len(loads_c), output, len(output),
        ))
        return tuple(output)

    @_synchronized
    def inverse_dynamics(
        self, q: Sequence[float], qd: Sequence[float], qdd: Sequence[float],
        loads: Iterable[Load] = (),
    ) -> tuple[float, ...]:
        r"""Compute recursive Newton-Euler inverse dynamics.

        Gravity is included in the result.

        The root pose and motion come from this Robot's stored base state. With
        a stationary base and no external loads, the returned generalized
        forces satisfy

        \[
        \tau = M(q) \ddot q + C(q, \dot q) \dot q + g(q).
        \]

        Args:
            q: Non-fixed joint positions in URDF order.
            qd: Non-fixed joint velocities in URDF order.
            qdd: Non-fixed joint accelerations in URDF order.
            loads: Optional external link-local wrenches.

        Returns:
            Generalized forces in base-then-joint order.

        Raises:
            ValueError: If an input or load is invalid.
            DyniboError: If the native calculation fails.
        """
        q_array = _array(q, "q")
        qd_array = _array(qd, "qd")
        qdd_array = _array(qdd, "qdd")
        _require_same_length(q_array, qd=qd_array, qdd=qdd_array)
        loads_c = _loads(loads)
        output = (ct.c_double * self.generalized_count)()
        _check(_lib.dynibo_inverse_dynamics(
            self._robot, self._workspace, q_array, qd_array, qdd_array, len(q_array),
            loads_c, len(loads_c), output, len(output),
        ))
        return tuple(output)

    @_synchronized
    def forward_dynamics(
        self, q: Sequence[float], qd: Sequence[float],
        generalized_forces: Sequence[float], loads: Iterable[Load] = (),
    ) -> tuple[float, ...]:
        r"""Compute articulated-body forward dynamics.

        The stored base pose and velocity participate in the calculation. A
        floating base's stored acceleration is ignored. For a floating base,
        input forces and output accelerations are ordered as world-frame angular
        base, world-frame linear base, then non-fixed joints.

        Args:
            q: Non-fixed joint positions in URDF order.
            qd: Non-fixed joint velocities in URDF order.
            generalized_forces: Applied generalized forces in base-then-joint order.
            loads: Optional link-local resisting wrenches.

        Returns:
            Generalized accelerations in base-then-joint order.

        Raises:
            ValueError: If an input or load is invalid.
            SolverError: If an articulated inertia is singular.
            DyniboError: If another native calculation failure occurs.
        """
        q_array = _array(q, "q")
        qd_array = _array(qd, "qd")
        _require_same_length(q_array, qd=qd_array)
        force_array = _array(generalized_forces, "generalized_forces")
        loads_c = _loads(loads)
        output = (ct.c_double * self.generalized_count)()
        _check(_lib.dynibo_forward_dynamics(
            self._robot, self._workspace, q_array, qd_array, len(q_array),
            force_array, len(force_array), loads_c, len(loads_c),
            output, len(output),
        ))
        return tuple(output)
