#include <pybind11/eigen.h>

#include "core.h"
#include "envpool/classic_control/curling.h"
#include "envpool/classic_control/viewer.h"
#include "envpool/core/py_envpool.h"
// generate python-side (raw) CurlingEnvSpec
typedef PyEnvSpec<classic_control::CurlingEnvSpec> CurlingEnvSpec;
// generate python-side (raw) CurlingEnvPool
typedef PyEnvPool<classic_control::CurlingEnvPool> CurlingEnvPool;
// generate classic_control_envpool.so
PYBIND11_MODULE(classic_control_envpool, m) {
  REGISTER(m, CurlingEnvSpec, CurlingEnvPool);
  m.def("cross_prod", &helperfunction::cross_prod);
  m.def("point2line", &helperfunction::point2line);
  m.def("line_intersect", &helperfunction::line_intersect);
  m.def("closest_point", &helperfunction::closest_point);
  m.def("rotate2", &helperfunction::rotate2);
  m.def("rotate", &helperfunction::rotate);
  m.def("get_distance", &helperfunction::get_distance);
  m.def("check_radian", &helperfunction::check_radian);
  // m.def("readjson", &readjson);
  // py::class_<Viewer>(m, "Viewer")
  //     .def(py::init<int, int, int>(), "width"_a, "height"_a, "edge"_a);
  py::class_<curling>(m, "curling")
      .def(py::init<>())
      .def("step", &curling::step)
      .def("reset", &curling::reset, py::arg("reset_game") = false);
}
