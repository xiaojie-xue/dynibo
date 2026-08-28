"""ctypes bindings for dynibo's typed fixed- and floating-base C ABI."""
from __future__ import annotations
import ctypes as ct
import ctypes.util
import os
import operator
import sys
import threading
from dataclasses import dataclass, field
from functools import wraps
from pathlib import Path
from typing import Iterable, Sequence

def _library_name() -> str:
    return "dynibo_c.dll" if sys.platform == "win32" else "libdynibo_c.dylib" if sys.platform == "darwin" else "libdynibo_c.so"
def _load_library() -> ct.CDLL:
    candidates = [x for x in (os.environ.get("DYNIBO_LIBRARY_PATH"), str(Path(__file__).with_name(_library_name())), ctypes.util.find_library("dynibo_c")) if x]
    errors=[]
    for path in candidates:
        try: return ct.CDLL(path)
        except OSError as error: errors.append(f"{path}: {error}")
    raise ImportError("could not load dynibo native library:\n" + "\n".join(errors))
class _Pose(ct.Structure): _fields_=[("translation",ct.c_double*3),("rotation_xyzw",ct.c_double*4)]
class _Twist(ct.Structure): _fields_=[("angular",ct.c_double*3),("linear",ct.c_double*3)]
class _BaseState(ct.Structure): _fields_=[("frame",_Pose),("velocity",_Twist),("acceleration",_Twist)]
class _Load(ct.Structure): _fields_=[("link_id",ct.c_size_t),("torque",ct.c_double*3),("force",ct.c_double*3)]
class _IkOptions(ct.Structure): _fields_=[("max_iterations",ct.c_size_t),("translation_tolerance",ct.c_double),("rotation_tolerance",ct.c_double),("damping",ct.c_double),("max_step_norm",ct.c_double)]
@dataclass(frozen=True)
class Pose: translation: tuple[float,float,float]=(0.,0.,0.); rotation_xyzw: tuple[float,float,float,float]=(0.,0.,0.,1.)
@dataclass(frozen=True)
class Twist: angular: tuple[float,float,float]=(0.,0.,0.); linear: tuple[float,float,float]=(0.,0.,0.)
@dataclass(frozen=True)
class BaseState:
    """Immutable world-frame state supplied to each floating-base calculation.

    ``velocity`` and ``acceleration`` are angular-first quantities at the root
    origin. Native validation rejects non-finite values and invalid quaternions.
    """
    frame: Pose = field(default_factory=Pose)
    velocity: Twist = field(default_factory=Twist)
    acceleration: Twist = field(default_factory=Twist)
@dataclass(frozen=True)
class Load: link_id: int; torque: tuple[float,float,float]=(0.,0.,0.); force: tuple[float,float,float]=(0.,0.,0.)
@dataclass(frozen=True)
class IkOptions:
    max_iterations: int = 100
    translation_tolerance: float = 1e-6
    rotation_tolerance: float = 1e-6
    damping: float = 1e-3
    max_step_norm: float = .5

    def __post_init__(self) -> None:
        if isinstance(self.max_iterations, bool):
            raise TypeError("max_iterations must be an integer")
        try:
            value = operator.index(self.max_iterations)
        except TypeError as error:
            raise TypeError("max_iterations must be an integer") from error
        if value <= 0:
            raise ValueError("max_iterations must be greater than zero")
        size_t_max = (1 << (8 * ct.sizeof(ct.c_size_t))) - 1
        if value > size_t_max:
            raise OverflowError("max_iterations does not fit in size_t")
        object.__setattr__(self, "max_iterations", value)
class DyniboError(RuntimeError): pass
class ModelError(DyniboError): pass
class SolverError(DyniboError): pass
class PanicError(DyniboError): pass

_lib=_load_library(); _robot_p=ct.c_void_p; _workspace_p=ct.c_void_p; _double_p=ct.POINTER(ct.c_double)
_lib.dynibo_last_error_message.argtypes=[]; _lib.dynibo_last_error_message.restype=ct.c_char_p
_lib.dynibo_version.argtypes=[]; _lib.dynibo_version.restype=ct.c_char_p
_lib.dynibo_ik_options_default.argtypes=[]; _lib.dynibo_ik_options_default.restype=_IkOptions
def _bind(name: str, args: list[object]) -> None: getattr(_lib,name).argtypes=args; getattr(_lib,name).restype=ct.c_int
for prefix in ("dynibo", "dynibo_floating"):
    r = _robot_p; w = _workspace_p
    _bind(prefix+"_forward_kinematics",[r,w]+([ct.POINTER(_BaseState)] if prefix.endswith("floating") else [])+[_double_p,ct.c_size_t,ct.c_size_t,ct.POINTER(_Pose)])
    _bind(prefix+"_forward_velocity_kinematics",[r,w]+([ct.POINTER(_BaseState)] if prefix.endswith("floating") else [])+[_double_p,_double_p,ct.c_size_t,ct.c_size_t,ct.POINTER(_Pose),ct.POINTER(_Twist)])
    _bind(prefix+"_forward_acceleration_kinematics",[r,w]+([ct.POINTER(_BaseState)] if prefix.endswith("floating") else [])+[_double_p,_double_p,_double_p,ct.c_size_t,ct.c_size_t,ct.POINTER(_Twist)])
    _bind(prefix+"_jacobian",[r,w]+([ct.POINTER(_BaseState)] if prefix.endswith("floating") else [])+[_double_p,ct.c_size_t,ct.c_size_t,_double_p,ct.c_size_t])
    _bind(prefix+"_jacobian_derivative",[r,w]+([ct.POINTER(_BaseState)] if prefix.endswith("floating") else [])+[_double_p,_double_p,ct.c_size_t,ct.c_size_t,_double_p,ct.c_size_t])
    _bind(prefix+"_mass_matrix",[r,w]+([ct.POINTER(_BaseState)] if prefix.endswith("floating") else [])+[_double_p,ct.c_size_t,_double_p,ct.c_size_t])
    _bind(prefix+"_velocity_product_forces",[r,w]+([ct.POINTER(_BaseState)] if prefix.endswith("floating") else [])+[_double_p,_double_p,ct.c_size_t,_double_p,ct.c_size_t])
    _bind(prefix+"_gravity",[r,w]+([ct.POINTER(_BaseState)] if prefix.endswith("floating") else [])+[_double_p,ct.c_size_t,ct.POINTER(_Load),ct.c_size_t,_double_p,ct.c_size_t])
    _bind(prefix+"_inverse_dynamics",[r,w]+([ct.POINTER(_BaseState)] if prefix.endswith("floating") else [])+[_double_p,_double_p,_double_p,ct.c_size_t,ct.POINTER(_Load),ct.c_size_t,_double_p,ct.c_size_t])
    _bind(prefix+"_forward_dynamics",[r,w]+([ct.POINTER(_BaseState)] if prefix.endswith("floating") else [])+[_double_p,_double_p,ct.c_size_t,_double_p,ct.c_size_t,ct.POINTER(_Load),ct.c_size_t,_double_p,ct.c_size_t])
_bind("dynibo_inverse_kinematics",[_robot_p,_workspace_p,_double_p,ct.c_size_t,ct.c_size_t,ct.POINTER(_Pose),_IkOptions,_double_p,ct.c_size_t])
_lib.dynibo_robot_from_urdf.argtypes=[ct.c_char_p,ct.POINTER(_robot_p)]; _lib.dynibo_robot_from_urdf.restype=ct.c_int
_lib.dynibo_floating_robot_from_urdf.argtypes=[ct.c_char_p,ct.POINTER(_robot_p)]; _lib.dynibo_floating_robot_from_urdf.restype=ct.c_int
for prefix in ("dynibo", "dynibo_floating"):
    getattr(_lib,prefix+"_robot_destroy").argtypes=[_robot_p]
    getattr(_lib,prefix+"_robot_destroy").restype=None
    getattr(_lib,prefix+"_workspace_destroy").argtypes=[_workspace_p]
    getattr(_lib,prefix+"_workspace_destroy").restype=None
    getattr(_lib,prefix+"_workspace_create").argtypes=[_robot_p,ct.POINTER(_workspace_p)]; getattr(_lib,prefix+"_workspace_create").restype=ct.c_int
    getattr(_lib,prefix+"_robot_name").argtypes=[_robot_p]; getattr(_lib,prefix+"_robot_name").restype=ct.c_char_p
    for suffix in ("joint_count","generalized_count","link_count"):
        f=getattr(_lib,prefix+"_robot_"+suffix); f.argtypes=[_robot_p]; f.restype=ct.c_size_t
    f=getattr(_lib,prefix+"_robot_link_id"); f.argtypes=[_robot_p,ct.c_char_p,ct.POINTER(ct.c_size_t)]; f.restype=ct.c_int
_lib.dynibo_robot_set_base_frame.argtypes=[_robot_p,ct.POINTER(_Pose)]; _lib.dynibo_robot_set_base_frame.restype=ct.c_int

def _check(status:int)->None:
    if status:
        raw=_lib.dynibo_last_error_message(); message=raw.decode("utf-8","replace") if raw else "unknown dynibo error"
        if status==1: raise ValueError(message)
        if status==2: raise ModelError(message)
        if status==3: raise PanicError(message)
        if status==4: raise SolverError(message)
        raise DyniboError(message)
def _array(values:Sequence[float],name:str):
    try: return (ct.c_double*len(values))(*(float(x) for x in values))
    except (TypeError,ValueError) as error: raise TypeError(f"{name} must be a finite-sized sequence of numbers") from error
def _fixed(values:Sequence[float],n:int,name:str):
    x=_array(values,name)
    if len(x)!=n: raise ValueError(f"{name} must contain exactly {n} elements")
    return x
def _pose(x:Pose)->_Pose: return _Pose(_fixed(x.translation,3,"pose translation"),_fixed(x.rotation_xyzw,4,"pose quaternion"))
def _twist(x:Twist)->_Twist: return _Twist(_fixed(x.angular,3,"twist angular"),_fixed(x.linear,3,"twist linear"))
def _base(x:BaseState)->_BaseState:
    if not isinstance(x,BaseState): raise TypeError("base must be BaseState")
    return _BaseState(_pose(x.frame),_twist(x.velocity),_twist(x.acceleration))
def _loads(x:Iterable[Load]):
    x=tuple(x); return (_Load*len(x))(*(_Load(v.link_id,_fixed(v.torque,3,"load torque"),_fixed(v.force,3,"load force")) for v in x))
def _same(q,**kwargs):
    for name,x in kwargs.items():
        if len(q)!=len(x): raise ValueError(f"q and {name} must have the same length")
def _synchronized(method):
    @wraps(method)
    def locked(self,*args,**kwargs):
        with self._lock: return method(self,*args,**kwargs)
    return locked

class _RobotHandle:
    _prefix="dynibo"; _floating=False
    def __init__(self,path:str|os.PathLike[str]):
        self._lock=threading.RLock(); self._robot=_robot_p(); self._workspace=_workspace_p()
        _check(getattr(_lib,self._prefix+"_robot_from_urdf")(os.fsencode(path),ct.byref(self._robot)))
        try: _check(getattr(_lib,self._prefix+"_workspace_create")(self._robot,ct.byref(self._workspace)))
        except Exception: getattr(_lib,self._prefix+"_robot_destroy")(self._robot); self._robot=_robot_p(); raise
    @classmethod
    def from_urdf(cls,path):
        """Load a URDF and allocate one reusable native workspace.

        Raises ``ModelError`` when the URDF cannot represent this model type.
        """
        return cls(path)
    @_synchronized
    def close(self):
        """Release native resources. Calling this method repeatedly is safe."""
        if self._workspace: getattr(_lib,self._prefix+"_workspace_destroy")(self._workspace); self._workspace=_workspace_p()
        if self._robot: getattr(_lib,self._prefix+"_robot_destroy")(self._robot); self._robot=_robot_p()
    def __enter__(self): return self
    def __exit__(self,*_): self.close()
    def __del__(self):
        try: self.close()
        except AttributeError: pass
    def _open(self):
        if not self._robot: raise RuntimeError("robot is closed")
    @property
    @_synchronized
    def name(self):
        """URDF model name."""
        self._open(); return getattr(_lib,self._prefix+"_robot_name")(self._robot).decode()
    @property
    @_synchronized
    def joint_count(self):
        """Number of non-fixed joints in URDF order."""
        self._open(); return int(getattr(_lib,self._prefix+"_robot_joint_count")(self._robot))
    @property
    @_synchronized
    def generalized_count(self):
        """Length of generalized force, acceleration, and velocity outputs."""
        self._open(); return int(getattr(_lib,self._prefix+"_robot_generalized_count")(self._robot))
    @property
    @_synchronized
    def link_count(self):
        """Number of links, including the root link."""
        self._open(); return int(getattr(_lib,self._prefix+"_robot_link_count")(self._robot))
    @_synchronized
    def link_id(self,name:str):
        """Resolve a model-scoped link identifier for use as a target or load ID."""
        self._open(); out=ct.c_size_t(); _check(getattr(_lib,self._prefix+"_robot_link_id")(self._robot,name.encode(),ct.byref(out))); return int(out.value)
    def _args(self,base): return [self._robot,self._workspace]+([ct.byref(base)] if self._floating else [])

class Robot(_RobotHandle):
    """Fixed-base URDF model with an owned, thread-serialized workspace.

    Joint vectors are in non-fixed URDF order and have ``joint_count`` values.
    Fixed-base ``generalized_count == joint_count``. Methods raise ``ValueError``
    for invalid input, ``DyniboError`` for native failures, and ``RuntimeError``
    after :meth:`close`.
    """
    @_synchronized
    def set_base_frame(self,frame:Pose):
        """Persist a finite world-frame root pose for later fixed calculations."""
        self._open(); x=_pose(frame); _check(_lib.dynibo_robot_set_base_frame(self._robot,ct.byref(x)))
    @_synchronized
    def forward_kinematics(self,q,target):
        """Return the world-frame :class:`Pose` of ``target`` at joint positions ``q``."""
        return self._fk(None,q,target)
    @_synchronized
    def jacobian(self,q,target):
        """Return a column-major, world-frame target-origin ``6 x G`` Jacobian tuple."""
        return self._jac(None,q,target)
    @_synchronized
    def jacobian_derivative(self,q,qd,target):
        """Return the column-major ``6 x G`` Jacobian time derivative for ``q, qd``."""
        return self._jd(None,q,qd,target)
    @_synchronized
    def mass_matrix(self,q):
        """Return the column-major ``G x G`` generalized mass matrix."""
        return self._mass(None,q)
    @_synchronized
    def velocity_product_forces(self,q,qd):
        """Return the ``G`` Coriolis/centrifugal generalized-force values."""
        return self._velprod(None,q,qd)
    @_synchronized
    def forward_velocity_kinematics(self,q,qd,target,tool:Pose=Pose()):
        """Return world-frame angular-first velocity at ``tool`` on ``target``."""
        return self._fv(None,q,qd,target,tool)
    @_synchronized
    def forward_acceleration_kinematics(self,q,qd,qdd,target):
        """Return world-frame angular-first acceleration at the target origin."""
        return self._fa(None,q,qd,qdd,target)
    @_synchronized
    def gravity(self,q,loads=()):
        """Return ``G`` gravity and resisting external-load generalized forces."""
        return self._gravity(None,q,loads)
    @_synchronized
    def inverse_dynamics(self,q,qd,qdd,loads=()):
        """Return ``G`` Newton--Euler generalized forces for the requested motion."""
        return self._id(None,q,qd,qdd,loads)
    @_synchronized
    def forward_dynamics(self,q,qd,forces,loads=()):
        """Return ``G`` articulated-body generalized accelerations."""
        return self._fd(None,q,qd,forces,loads)
    @_synchronized
    def inverse_kinematics(self,q,target,desired:Pose,options:IkOptions=IkOptions()):
        """Solve fixed-base IK and return ``joint_count`` values, or raise ``SolverError``."""
        self._open(); q=_array(q,"initial_q"); desired=_pose(desired); out=(ct.c_double*self.joint_count)(); native=_IkOptions(options.max_iterations,options.translation_tolerance,options.rotation_tolerance,options.damping,options.max_step_norm); _check(_lib.dynibo_inverse_kinematics(self._robot,self._workspace,q,len(q),target,ct.byref(desired),native,out,len(out))); return tuple(out)

class FloatingRobot(_RobotHandle):
    """Floating-base URDF model with no stored base state.

    Every calculation accepts :class:`BaseState` as its first argument. Joint
    arrays remain length ``joint_count`` while outputs use
    ``generalized_count == joint_count + 6`` with angular then linear base
    components first. Floating-base inverse kinematics is intentionally absent.
    """
    _prefix="dynibo_floating"; _floating=True
    @_synchronized
    def forward_kinematics(self,base,q,target):
        """Return the target world pose using this call's explicit ``base`` state."""
        return self._fk(_base(base),q,target)
    @_synchronized
    def jacobian(self,base,q,target):
        """Return a column-major, world-frame target-origin ``6 x G`` Jacobian."""
        return self._jac(_base(base),q,target)
    @_synchronized
    def jacobian_derivative(self,base,q,qd,target):
        """Return the ``6 x G`` Jacobian derivative for explicit ``base, q, qd``."""
        return self._jd(_base(base),q,qd,target)
    @_synchronized
    def mass_matrix(self,base,q):
        """Return the column-major ``G x G`` mass matrix for ``base`` and ``q``."""
        return self._mass(_base(base),q)
    @_synchronized
    def velocity_product_forces(self,base,q,qd):
        """Return ``G`` velocity-product forces for explicit ``base``."""
        return self._velprod(_base(base),q,qd)
    @_synchronized
    def forward_velocity_kinematics(self,base,q,qd,target,tool:Pose=Pose()):
        """Return angular-first world velocity at ``tool`` using ``base``."""
        return self._fv(_base(base),q,qd,target,tool)
    @_synchronized
    def forward_acceleration_kinematics(self,base,q,qd,qdd,target):
        """Return angular-first world acceleration at the target origin."""
        return self._fa(_base(base),q,qd,qdd,target)
    @_synchronized
    def gravity(self,base,q,loads=()):
        """Return ``G`` gravity and external-load forces for ``base``."""
        return self._gravity(_base(base),q,loads)
    @_synchronized
    def inverse_dynamics(self,base,q,qd,qdd,loads=()):
        """Return ``G`` inverse-dynamics forces for an explicit base state."""
        return self._id(_base(base),q,qd,qdd,loads)
    @_synchronized
    def forward_dynamics(self,base,q,qd,forces,loads=()):
        """Return ``G`` forward-dynamics accelerations for an explicit base state."""
        return self._fd(_base(base),q,qd,forces,loads)

def _fk(self,b,q,target):
    self._open(); q=_array(q,"q"); out=_Pose(); _check(getattr(_lib,self._prefix+"_forward_kinematics")(*self._args(b),q,len(q),target,ct.byref(out))); return Pose(tuple(out.translation),tuple(out.rotation_xyzw))
def _jac(self,b,q,target):
    self._open(); q=_array(q,"q"); out=(ct.c_double*(6*self.generalized_count))(); _check(getattr(_lib,self._prefix+"_jacobian")(*self._args(b),q,len(q),target,out,len(out))); return tuple(out)
def _jd(self,b,q,qd,target):
    self._open(); q=_array(q,"q"); qd=_array(qd,"qd"); _same(q,qd=qd); out=(ct.c_double*(6*self.generalized_count))(); _check(getattr(_lib,self._prefix+"_jacobian_derivative")(*self._args(b),q,qd,len(q),target,out,len(out))); return tuple(out)
def _mass(self,b,q):
    self._open(); q=_array(q,"q"); out=(ct.c_double*(self.generalized_count**2))(); _check(getattr(_lib,self._prefix+"_mass_matrix")(*self._args(b),q,len(q),out,len(out))); return tuple(out)
def _velprod(self,b,q,qd):
    self._open(); q=_array(q,"q"); qd=_array(qd,"qd"); _same(q,qd=qd); out=(ct.c_double*self.generalized_count)(); _check(getattr(_lib,self._prefix+"_velocity_product_forces")(*self._args(b),q,qd,len(q),out,len(out))); return tuple(out)
def _fv(self,b,q,qd,target,tool):
    self._open(); q=_array(q,"q"); qd=_array(qd,"qd"); _same(q,qd=qd); tool=_pose(tool); out=_Twist(); _check(getattr(_lib,self._prefix+"_forward_velocity_kinematics")(*self._args(b),q,qd,len(q),target,ct.byref(tool),ct.byref(out))); return Twist(tuple(out.angular),tuple(out.linear))
def _fa(self,b,q,qd,qdd,target):
    self._open(); q=_array(q,"q"); qd=_array(qd,"qd"); qdd=_array(qdd,"qdd"); _same(q,qd=qd,qdd=qdd); out=_Twist(); _check(getattr(_lib,self._prefix+"_forward_acceleration_kinematics")(*self._args(b),q,qd,qdd,len(q),target,ct.byref(out))); return Twist(tuple(out.angular),tuple(out.linear))
def _gravity(self,b,q,values):
    self._open(); q=_array(q,"q"); values=_loads(values); out=(ct.c_double*self.generalized_count)(); _check(getattr(_lib,self._prefix+"_gravity")(*self._args(b),q,len(q),values,len(values),out,len(out))); return tuple(out)
def _id(self,b,q,qd,qdd,values):
    self._open(); q=_array(q,"q"); qd=_array(qd,"qd"); qdd=_array(qdd,"qdd"); _same(q,qd=qd,qdd=qdd); values=_loads(values); out=(ct.c_double*self.generalized_count)(); _check(getattr(_lib,self._prefix+"_inverse_dynamics")(*self._args(b),q,qd,qdd,len(q),values,len(values),out,len(out))); return tuple(out)
def _fd(self,b,q,qd,forces,values):
    self._open(); q=_array(q,"q"); qd=_array(qd,"qd"); _same(q,qd=qd); forces=_array(forces,"forces"); values=_loads(values); out=(ct.c_double*self.generalized_count)(); _check(getattr(_lib,self._prefix+"_forward_dynamics")(*self._args(b),q,qd,len(q),forces,len(forces),values,len(values),out,len(out))); return tuple(out)
for _name,_function in {"_fk":_fk,"_jac":_jac,"_jd":_jd,"_mass":_mass,"_velprod":_velprod,"_fv":_fv,"_fa":_fa,"_gravity":_gravity,"_id":_id,"_fd":_fd}.items(): setattr(_RobotHandle,_name,_function)
