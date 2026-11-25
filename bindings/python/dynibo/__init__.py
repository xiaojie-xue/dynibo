"""Python interface to the dynibo robot kinematics and dynamics library."""

from importlib.metadata import PackageNotFoundError, version as distribution_version

from ._dynibo import (
    BaseState,
    DyniboError,
    FloatingRobot,
    IkOptions,
    Load,
    LoadBuffer,
    ModelError,
    PanicError,
    Pose,
    Robot,
    SolverError,
    Twist,
    __version__ as _native_version,
)

__all__ = [
    "BaseState",
    "DyniboError",
    "IkOptions",
    "Load",
    "LoadBuffer",
    "ModelError",
    "PanicError",
    "Pose",
    "FloatingRobot",
    "Robot",
    "SolverError",
    "Twist",
]
try:
    __version__ = distribution_version("dynibo")
except PackageNotFoundError:
    __version__ = _native_version
