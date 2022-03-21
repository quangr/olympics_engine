#include <pybind11/eigen.h>

#include "core.h"
#include "envpool/classic_control/curling.h"
#include "envpool/core/py_envpool.h"
// generate python-side (raw) CurlingEnvSpec
typedef PyEnvSpec<classic_control::CurlingEnvSpec> CurlingEnvSpec;
// generate python-side (raw) CurlingEnvPool
typedef PyEnvPool<classic_control::CurlingEnvPool> CurlingEnvPool;
typedef PyEnvSpec<classic_control::CurlingSimpleEnvSpec> CurlingSimpleEnvSpec;
// generate python-side (raw) CurlingEnvPool
typedef PyEnvPool<classic_control::CurlingSimpleEnvPool> CurlingSimpleEnvPool;
// generate classic_control_envpool.so
PYBIND11_MODULE(classic_control_envpool, m) {
  REGISTER(m, CurlingSimpleEnvSpec, CurlingSimpleEnvPool);
  REGISTER(m, CurlingEnvSpec, CurlingEnvPool);
  // m.def("readjson", &readjson);
  // py::class_<Viewer>(m, "Viewer")
  //     .def(py::init<int, int, int>(), "width"_a, "height"_a, "edge"_a);
  // py::class_<curling>(m, "curling")
  //     .def(py::init<>())
  //     .def("step", &curling::step)
  //     .def("reset", &curling::reset, py::arg("reset_game") = false)
  //     .def_readwrite("current_team", &curling::current_team);
}
