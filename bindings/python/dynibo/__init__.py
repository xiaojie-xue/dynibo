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


class LinkId(int):
    """Model-scoped link identifier.

    Subclasses ``int`` so index usage such as ``poses[link_id]`` continues to
    work, while carrying the owning model's identity to reject cross-model use.
    """

    def __new__(cls, index: int, *, _model_tag: object = None) -> "LinkId":
        if isinstance(index, bool) or not isinstance(index, int):
            raise TypeError("link_id must be an integer")
        if index < 0:
            raise ValueError("link_id must be non-negative")
        obj = super().__new__(cls, index)
        object.__setattr__(obj, "_model_tag", _model_tag)
        return obj

    def __setattr__(self, name: str, value: object) -> None:
        raise AttributeError("LinkId is immutable")

    def __delattr__(self, name: str) -> None:
        raise AttributeError("LinkId is immutable")

    @property
    def index(self) -> int:
        return int(self)

    def __repr__(self) -> str:
        return f"LinkId({int(self)})"


__all__ = [
    "BaseState",
    "DyniboError",
    "IkOptions",
    "LinkId",
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
