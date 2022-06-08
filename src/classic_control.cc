#include <pybind11/eigen.h>

#include "core.h"
// #include "curling.h"
// #include "envpool/core/py_envpool.h"
// generate python-side (raw) CurlingEnvSpec
// typedef PyEnvSpec<classic_control::CurlingEnvSpec> CurlingEnvSpec;
// // generate python-side (raw) CurlingEnvPool
// typedef PyEnvPool<classic_control::CurlingEnvPool> CurlingEnvPool;
// typedef PyEnvSpec<classic_control::CurlingSimpleEnvSpec> CurlingSimpleEnvSpec;
// // generate python-side (raw) CurlingEnvPool
// typedef PyEnvPool<classic_control::CurlingSimpleEnvPool> CurlingSimpleEnvPool;
// generate classic_control_envpool.so

class Publicist : public curling { // helper type for exposing protected functions
public:
    using curling::map; // inherited with different access modifier
};

PYBIND11_MODULE(classic_control_envpool, m) {
//   REGISTER(m, CurlingSimpleEnvSpec, CurlingSimpleEnvPool);
//   REGISTER(m, CurlingEnvSpec, CurlingEnvPool);
  // m.def("readjson", &readjson);
  // py::class_<Viewer>(m, "Viewer")
  //     .def(py::init<int, int, int>(), "width"_a, "height"_a, "edge"_a);
  py::class_<agent_t>(m, "agent_t")
      .def_readwrite("r", &agent_t::r)
      .def_readwrite("color", &agent_t::color);
  py::enum_<Color>(m, "Color")
      .value("light_green", light_green)
      .value("green", green)
      .value("sky_blue", sky_blue)
      .value("yellow", yellow)
      .value("grey", grey)
      .value("purple", purple)
      .value("black", black)
      .value("red", red)
      .value("blue", blue)
      .export_values();
  py::class_<view_t>(m, "view_t")
      .def_readwrite("width", &view_t::width)
      .def_readwrite("height", &view_t::height)
      .def_readwrite("edge", &view_t::edge)
      .def_readwrite("init_obs", &view_t::init_obs);
  py::class_<object_t>(m, "object_t")
      .def_readwrite("color", &object_t::color);
  
  py::class_<component_t,object_t>(m, "component_t")
      .def_readwrite("cur_pos", &component_t::cur_pos)
      .def_readwrite("cur_pos_rotated", &component_t::cur_pos_rotated);
      

  py::class_<map_view_t>(m, "map_view_t")
      .def_readwrite("view", &map_view_t::view)
      .def_readwrite("objects", &map_view_t::objects);

  py::class_<map_t,map_view_t>(m, "map_t");
  py::class_<curling>(m, "curling")
      .def(py::init<std::string>())
      .def("step", &curling::step)
      .def("reset", &curling::reset, py::arg("reset_game") = false)
      .def_readwrite("agent_pos", &curling::agent_pos)
      .def_readwrite("agent_accel", &curling::agent_accel)
      .def_readwrite("current_team", &curling::current_team)
      .def_readwrite("cmap", &Publicist::map)
      .def_readwrite("agent_list", &curling::agent_list);
}
