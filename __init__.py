from envpool.python.api import py_env

from .classic_control_envpool import _CurlingEnvPool, _CurlingEnvSpec

CurlingEnvSpec, CurlingDMEnvPool, CurlingGymEnvPool = py_env(
  _CurlingEnvSpec, _CurlingEnvPool
)


from .classic_control_envpool import  _CurlingSimpleEnvPool, _CurlingSimpleEnvSpec
CurlingSimpleEnvSpec, CurlingSimpleDMEnvPool, CurlingSimpleGymEnvPool = py_env(
   _CurlingSimpleEnvSpec,_CurlingSimpleEnvPool
)


__all__ = [
  "CurlingEnvSpec",
  "CurlingDMEnvPool",
  "CurlingGymEnvPool",
  "CurlingSimpleEnvSpec",
  "CurlingSimpleDMEnvPool",
  "CurlingSimpleGymEnvPool",
]