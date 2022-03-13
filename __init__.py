from envpool.python.api import py_env

from .classic_control_envpool import _CurlingEnvPool, _CurlingEnvSpec,cross_prod#,point2line

CurlingEnvSpec, CurlingDMEnvPool, CurlingGymEnvPool = py_env(
  _CurlingEnvSpec, _CurlingEnvPool
)

__all__ = [
  "CurlingEnvSpec",
  "CurlingDMEnvPool",
  "CurlingGymEnvPool",
  "cross_prod",
  # "point2line"
]