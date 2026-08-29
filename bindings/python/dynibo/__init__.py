"""Python interface to the dynibo robot kinematics and dynamics library."""

from importlib.metadata import version as distribution_version

try:
    from ._dynibo import (
        BaseState,
        DyniboError,
        FloatingRobot,
        IkOptions,
        Load,
        ModelError,
        PanicError,
        Pose,
        Robot,
        SolverError,
        Twist,
    )
except ImportError:  # pragma: no cover - transitional source-tree fallback
    from ._native import (
        BaseState,
        DyniboError,
        FloatingRobot,
        IkOptions,
        Load,
        ModelError,
        PanicError,
        Pose,
        Robot,
        SolverError,
        Twist,
    )

__all__ = [
    "BaseState",
    "DyniboError",
    "IkOptions",
    "Load",
    "ModelError",
    "PanicError",
    "Pose",
    "FloatingRobot",
    "Robot",
    "SolverError",
    "Twist",
]
__version__ = distribution_version("dynibo")
