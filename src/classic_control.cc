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

class Publicist : public OlympicsBase { // helper type for exposing protected functions
public:
    using OlympicsBase::map,OlympicsBase::obs_boundary,OlympicsBase::agent_num,OlympicsBase::reset,OlympicsBase::agent_init_pos; // inherited with different access modifier
    
};

PYBIND11_MODULE(classic_control_envpool, m) {
//   REGISTER(m, CurlingSimpleEnvSpec, CurlingSimpleEnvPool);
//   REGISTER(m, CurlingEnvSpec, CurlingEnvPool);
  // m.def("readjson", &readjson);
  // py::class_<Viewer>(m, "Viewer")
  //     .def(py::init<int, int, int>(), "width"_a, "height"_a, "edge"_a);
  py::class_<agent_t>(m, "agent_t")
      .def_readwrite("r", &agent_t::r)
      .def_readwrite("energy_cap", &agent_t::energy_cap)
      .def_readwrite("energy", &agent_t::energy)
      .def_readwrite("_color", &agent_t::color)
      .def_readwrite("is_ball", &agent_t::is_ball)
      .def_readwrite("alive", &agent_t::alive)
      .def("to_ball", &agent_t::to_ball);
      
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
      .def_readwrite("_color", &object_t::color);
  
  py::class_<component_t,object_t>(m, "component_t")
      .def_readwrite("cur_pos", &component_t::cur_pos)
      .def_readwrite("cur_pos_rotated", &component_t::cur_pos_rotated);
      

  py::class_<map_view_t>(m, "map_view_t")
      .def_readwrite("view", &map_view_t::view)
      .def_readwrite("objects", &map_view_t::objects);

  py::class_<map_t,map_view_t>(m, "map_t");
  py::class_<OlympicsBase>(m, "OlympicsBase")
      .def(py::init<std::string>())
    //   .def("reset", &Publicist::reset, py::return_value_policy::reference_internal)
    //   .def("get_obs", &OlympicsBase::get_obs, py::return_value_policy::reference_internal)
    //   .def("stepPhysics", &OlympicsBase::stepPhysics, py::return_value_policy::reference_internal)
    //   .def("add_agent", &OlympicsBase::add_agent, py::return_value_policy::reference_internal)
    //   .def("clear_agent", &OlympicsBase::clear_agent, py::return_value_policy::reference_internal)
      .def("reset", &Publicist::reset)
      .def("get_obs", &OlympicsBase::get_obs)
      .def("stepPhysics", &OlympicsBase::stepPhysics)
      .def("remove_agent", &OlympicsBase::remove_agent)
      .def("add_agent", &OlympicsBase::add_agent)
      .def("clear_agent", &OlympicsBase::clear_agent)
      .def("init_state", &OlympicsBase::init_state)
      .def_readwrite("agent_pos", &OlympicsBase::agent_pos)
      .def_readwrite("agent_v", &OlympicsBase::agent_v)
      .def_readwrite("agent_num", &Publicist::agent_num)
      .def_readwrite("tau", &OlympicsBase::tau)
      .def_readwrite("gamma", &OlympicsBase::gamma)
      .def_readwrite("wall_restitution", &OlympicsBase::wall_restitution)
      .def_readwrite("circle_restitution", &OlympicsBase::circle_restitution)
      .def_readwrite("agent_accel", &OlympicsBase::agent_accel)
      .def_readwrite("obs_boundary", &Publicist::obs_boundary)
      .def_readwrite("agent_init_pos", &Publicist::agent_init_pos)
      .def_readwrite("cmap", &Publicist::map)
      .def_readwrite("obs_list", &OlympicsBase::obs_list)
      .def_readwrite("step_cnt", &OlympicsBase::step_cnt)
      .def_readwrite("agent_list", &OlympicsBase::agent_list);






//   py::class_<curling>(m, "curling")
//       .def(py::init<std::string>())
//       .def("step", &curling::step)
//       .def("reset", &curling::reset, py::arg("reset_game") = false)
//       .def_readwrite("agent_pos", &curling::agent_pos)
//       .def_readwrite("agent_accel", &curling::agent_accel)
//       .def_readwrite("current_team", &curling::current_team)
//       .def_readwrite("obs_boundary", &Publicist::obs_boundary)
//       .def_readwrite("cmap", &Publicist::map)
//       .def_readwrite("obs_list", &Publicist::obs_list)
//       .def_readwrite("step_cnt", &curling::step_cnt)
//       .def_readwrite("agent_list", &curling::agent_list);

}
