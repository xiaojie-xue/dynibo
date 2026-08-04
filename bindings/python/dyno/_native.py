"""Dependency-free ctypes wrapper around dyno's stable C ABI."""

from __future__ import annotations

import ctypes as ct
import ctypes.util
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence


def _library_name() -> str:
    if sys.platform == "win32":
        return "dyno_c.dll"
    if sys.platform == "darwin":
        return "libdyno_c.dylib"
    return "libdyno_c.so"


def _load_library() -> ct.CDLL:
    candidates = []
    override = os.environ.get("DYNO_LIBRARY_PATH")
    if override:
        candidates.append(override)
    candidates.append(str(Path(__file__).with_name(_library_name())))
    system = ctypes.util.find_library("dyno_c")
    if system:
        candidates.append(system)
    errors = []
    for candidate in candidates:
        try:
            return ct.CDLL(candidate)
        except OSError as error:
            errors.append(f"{candidate}: {error}")
    raise ImportError("could not load dyno native library:\n" + "\n".join(errors))


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


@dataclass(frozen=True)
class Pose:
    """A translation and an `(x, y, z, w)` unit quaternion."""

    translation: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rotation_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)


@dataclass(frozen=True)
class Twist:
    """Angular-first spatial velocity or acceleration."""

    angular: tuple[float, float, float] = (0.0, 0.0, 0.0)
    linear: tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class Load:
    """A link-local external wrench."""

    link_id: int
    torque: tuple[float, float, float] = (0.0, 0.0, 0.0)
    force: tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class IkOptions:
    """Damped-least-squares inverse-kinematics configuration."""

    max_iterations: int = 100
    translation_tolerance: float = 1.0e-6
    rotation_tolerance: float = 1.0e-6
    damping: float = 1.0e-3
    max_step_norm: float = 0.5


class DynoError(RuntimeError):
    """Base class for native dyno model and calculation errors."""


class ModelError(DynoError):
    """A robot description could not be loaded or represented."""


class SolverError(DynoError):
    """An iterative numerical calculation did not produce a valid result."""


class PanicError(DynoError):
    """The native library caught an unexpected internal panic."""


_lib = _load_library()
_robot_p = ct.c_void_p
_workspace_p = ct.c_void_p
_double_p = ct.POINTER(ct.c_double)

_lib.dyno_last_error_message.restype = ct.c_char_p
_lib.dyno_version.restype = ct.c_char_p
_lib.dyno_robot_load_urdf.argtypes = [ct.c_char_p, ct.POINTER(_robot_p)]
_lib.dyno_robot_load_urdf.restype = ct.c_int
_lib.dyno_robot_destroy.argtypes = [_robot_p]
_lib.dyno_robot_name.argtypes = [_robot_p]
_lib.dyno_robot_name.restype = ct.c_char_p
_lib.dyno_robot_joint_count.argtypes = [_robot_p]
_lib.dyno_robot_joint_count.restype = ct.c_size_t
_lib.dyno_robot_link_count.argtypes = [_robot_p]
_lib.dyno_robot_link_count.restype = ct.c_size_t
_lib.dyno_robot_link_id.argtypes = [_robot_p, ct.c_char_p, ct.POINTER(ct.c_size_t)]
_lib.dyno_robot_link_id.restype = ct.c_int
_lib.dyno_workspace_create.argtypes = [_robot_p, ct.POINTER(_workspace_p)]
_lib.dyno_workspace_create.restype = ct.c_int
_lib.dyno_workspace_destroy.argtypes = [_workspace_p]
_lib.dyno_forward_kinematics.argtypes = [
    _robot_p, _workspace_p, _double_p, ct.c_size_t, ct.c_size_t, ct.POINTER(_Pose)
]
_lib.dyno_forward_kinematics.restype = ct.c_int
_lib.dyno_jacobian.argtypes = [
    _robot_p, _workspace_p, _double_p, ct.c_size_t, ct.c_size_t,
    _double_p, ct.c_size_t,
]
_lib.dyno_jacobian.restype = ct.c_int
_lib.dyno_inverse_kinematics.argtypes = [
    _robot_p, _workspace_p, _double_p, ct.c_size_t, ct.c_size_t,
    ct.POINTER(_Pose), _IkOptions, _double_p, ct.c_size_t,
]
_lib.dyno_inverse_kinematics.restype = ct.c_int
_lib.dyno_forward_velocity.argtypes = [
    _robot_p, _workspace_p, _double_p, _double_p, ct.c_size_t, ct.c_size_t,
    ct.POINTER(_Pose), ct.POINTER(_Pose), ct.POINTER(_Twist),
]
_lib.dyno_forward_velocity.restype = ct.c_int
_lib.dyno_forward_acceleration.argtypes = [
    _robot_p, _workspace_p, _double_p, _double_p, _double_p,
    ct.c_size_t, ct.c_size_t, ct.POINTER(_Twist),
]
_lib.dyno_forward_acceleration.restype = ct.c_int
_lib.dyno_gravity.argtypes = [
    _robot_p, _workspace_p, _double_p, ct.c_size_t, ct.POINTER(_Pose),
    ct.POINTER(_Load), ct.c_size_t, _double_p, ct.c_size_t,
]
_lib.dyno_gravity.restype = ct.c_int
_lib.dyno_inverse_dynamics.argtypes = [
    _robot_p, _workspace_p, _double_p, _double_p, _double_p, ct.c_size_t,
    ct.POINTER(_Pose), _Twist, _Twist, ct.POINTER(_Load), ct.c_size_t,
    _double_p, ct.c_size_t,
]
_lib.dyno_inverse_dynamics.restype = ct.c_int


def _check(status: int) -> None:
    if status != 0:
        raw_message = _lib.dyno_last_error_message()
        message = raw_message.decode("utf-8", "replace") if raw_message else "unknown dyno error"
        if status == 1:
            raise ValueError(message)
        if status == 2:
            raise ModelError(message)
        if status == 3:
            raise PanicError(message)
        if status == 4:
            raise SolverError(message)
        raise DynoError(message)


def _array(values: Sequence[float], name: str) -> ct.Array[ct.c_double]:
    try:
        return (ct.c_double * len(values))(*(float(value) for value in values))
    except (TypeError, ValueError) as error:
        raise TypeError(f"{name} must be a finite-sized sequence of numbers") from error


def _fixed_array(values: Sequence[float], length: int, name: str) -> ct.Array[ct.c_double]:
    if len(values) != length:
        raise ValueError(f"{name} must contain exactly {length} elements")
    return _array(values, name)


def _require_same_length(q: Sequence[float], **states: Sequence[float]) -> None:
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


class Robot:
    """A URDF robot with one reusable, non-thread-safe calculation workspace."""

    def __init__(self, urdf_path: str | os.PathLike[str]):
        self._robot = _robot_p()
        self._workspace = _workspace_p()
        _check(_lib.dyno_robot_load_urdf(os.fsencode(urdf_path), ct.byref(self._robot)))
        try:
            _check(_lib.dyno_workspace_create(self._robot, ct.byref(self._workspace)))
        except Exception:
            _lib.dyno_robot_destroy(self._robot)
            self._robot = _robot_p()
            raise

    def close(self) -> None:
        """Release native resources; calling this more than once is safe."""
        if self._workspace:
            _lib.dyno_workspace_destroy(self._workspace)
            self._workspace = _workspace_p()
        if self._robot:
            _lib.dyno_robot_destroy(self._robot)
            self._robot = _robot_p()

    def __enter__(self) -> "Robot":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def __del__(self) -> None:
        self.close()

    @property
    def name(self) -> str:
        value = _lib.dyno_robot_name(self._robot)
        if value is None:
            raise RuntimeError("robot is closed")
        return value.decode("utf-8")

    @property
    def joint_count(self) -> int:
        return int(_lib.dyno_robot_joint_count(self._robot))

    @property
    def link_count(self) -> int:
        return int(_lib.dyno_robot_link_count(self._robot))

    def link_id(self, name: str) -> int:
        result = ct.c_size_t()
        _check(_lib.dyno_robot_link_id(self._robot, name.encode(), ct.byref(result)))
        return int(result.value)

    def forward_kinematics(self, q: Sequence[float], target: int) -> Pose:
        q_array = _array(q, "q")
        output = _Pose()
        _check(_lib.dyno_forward_kinematics(
            self._robot, self._workspace, q_array, len(q), target, ct.byref(output)
        ))
        return Pose(tuple(output.translation), tuple(output.rotation_xyzw))

    def jacobian(self, q: Sequence[float], target: int) -> tuple[float, ...]:
        """Return a flat, column-major `6 x joint_count` Jacobian."""
        q_array = _array(q, "q")
        output = (ct.c_double * (6 * self.joint_count))()
        _check(_lib.dyno_jacobian(
            self._robot, self._workspace, q_array, len(q), target,
            output, len(output),
        ))
        return tuple(output)

    def inverse_kinematics(
        self, initial_q: Sequence[float], target: int, desired: Pose,
        options: IkOptions = IkOptions(),
    ) -> tuple[float, ...]:
        q_array = _array(initial_q, "initial_q")
        desired_c = _pose(desired)
        options_c = _IkOptions(
            options.max_iterations, options.translation_tolerance,
            options.rotation_tolerance, options.damping, options.max_step_norm,
        )
        output = (ct.c_double * self.joint_count)()
        _check(_lib.dyno_inverse_kinematics(
            self._robot, self._workspace, q_array, len(initial_q), target,
            ct.byref(desired_c), options_c, output, len(output),
        ))
        return tuple(output)

    def forward_velocity(
        self, q: Sequence[float], qd: Sequence[float], target: int,
        base: Pose = Pose(), tool: Pose = Pose(),
    ) -> Twist:
        _require_same_length(q, qd=qd)
        q_array, qd_array = _array(q, "q"), _array(qd, "qd")
        base_c, tool_c, output = _pose(base), _pose(tool), _Twist()
        _check(_lib.dyno_forward_velocity(
            self._robot, self._workspace, q_array, qd_array, len(q), target,
            ct.byref(base_c), ct.byref(tool_c), ct.byref(output),
        ))
        return Twist(tuple(output.angular), tuple(output.linear))

    def forward_acceleration(
        self, q: Sequence[float], qd: Sequence[float],
        qdd: Sequence[float], target: int,
    ) -> Twist:
        _require_same_length(q, qd=qd, qdd=qdd)
        q_array = _array(q, "q")
        qd_array = _array(qd, "qd")
        qdd_array = _array(qdd, "qdd")
        output = _Twist()
        _check(_lib.dyno_forward_acceleration(
            self._robot, self._workspace, q_array, qd_array, qdd_array,
            len(q), target, ct.byref(output),
        ))
        return Twist(tuple(output.angular), tuple(output.linear))

    def gravity(
        self, q: Sequence[float], base: Pose = Pose(),
        loads: Iterable[Load] = (),
    ) -> tuple[float, ...]:
        q_array, base_c, loads_c = _array(q, "q"), _pose(base), _loads(loads)
        output = (ct.c_double * self.joint_count)()
        _check(_lib.dyno_gravity(
            self._robot, self._workspace, q_array, len(q), ct.byref(base_c),
            loads_c, len(loads_c), output, len(output),
        ))
        return tuple(output)

    def inverse_dynamics(
        self, q: Sequence[float], qd: Sequence[float], qdd: Sequence[float],
        base: Pose = Pose(), base_velocity: Twist = Twist(),
        base_acceleration: Twist = Twist(), loads: Iterable[Load] = (),
    ) -> tuple[float, ...]:
        _require_same_length(q, qd=qd, qdd=qdd)
        q_array = _array(q, "q")
        qd_array = _array(qd, "qd")
        qdd_array = _array(qdd, "qdd")
        base_c, loads_c = _pose(base), _loads(loads)
        output = (ct.c_double * self.joint_count)()
        _check(_lib.dyno_inverse_dynamics(
            self._robot, self._workspace, q_array, qd_array, qdd_array, len(q),
            ct.byref(base_c), _twist(base_velocity), _twist(base_acceleration),
            loads_c, len(loads_c), output, len(output),
        ))
        return tuple(output)
