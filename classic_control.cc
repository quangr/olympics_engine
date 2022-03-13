#include <pybind11/eigen.h>

#include "core.h"
#include "envpool/classic_control/curling.h"
#include "envpool/core/py_envpool.h"
// generate python-side (raw) CurlingEnvSpec
typedef PyEnvSpec<classic_control::CurlingEnvSpec> CurlingEnvSpec;
// generate python-side (raw) CurlingEnvPool
typedef PyEnvPool<classic_control::CurlingEnvPool> CurlingEnvPool;

// generate classic_control_envpool.so
PYBIND11_MODULE(classic_control_envpool, m) {
  REGISTER(m, CurlingEnvSpec, CurlingEnvPool);
  m.def("cross_prod", &cross_prod);
  // m.def("point2line", &point2line);
}
