#ifndef ENVPOOL_CURLING_CORE_ENV_H_
#define ENVPOOL_CURLING_CORE_ENV_H_
#include <Eigen/Dense>

#include "objects.hpp"
using point2 = Eigen::Vector2d;
using mat2 = Eigen::Matrix2d;

using obslist_t = std::vector<Eigen::MatrixXd>;

class OlympicsBase {
 protected:
  point2 action_f{-100, 200};
  point2 action_theta{-30, 30};
  int agent_num = 0;
  std::vector<agent_t> agent_list;
  std::vector<point2> agent_init_pos;
  std::vector<point2> agent_pos;
  std::vector<point2> agent_previous_pos;
  std::vector<point2> agent_v;
  std::vector<point2> agent_accel;
  std::vector<double> agent_theta;
  std::vector<std::vector<point2>> agent_record;
  std::unordered_map<point2, std::vector<int>> point2wall;

  //   std::vector<point2> agent_record;
  //   bool show_traj = true;
  //   bool draw_obs = true;
  //   bool print_log = false;
  //   bool print_log2 = false;
  //   std::vector<point2> map_object;
  //   std::vector<point2> global_wall_ignore;
  //   std::vector<point2> global_circle_ignore;

  //   //  env hyper
  double tau = 0.1;     // delta t
  double gamma = 0.98;  // v衰减系数
  double wall_restitution = 0.5;
  double circle_restitution = 0.5;

  int step_cnt = 0;
  bool done = false;
  int max_step = 500;
  std::vector<Eigen::MatrixXd> obs_list;

  double energy_recover_rate = 200;
  int speed_cap = 500;

  //   // #for debugg
  //   // # obs_boundary_init = [[80,350], [180,350],[180,450],[80,450]]
  //   // # obs_boundary = obs_boundary_init
  std::vector<std::vector<point2>> obs_boundary_init;
  std::vector<std::vector<point2>> obs_boundary = obs_boundary_init;

  map_t map;
  //   // generate_map(map)
  //   // merge_map()

  view_t view_setting;  //   // map_num = None
  bool display_mode = false;
  obslist_t reset();

  std::vector<point2> get_obs_boundaray(point2 init_position, double r,
                                        double visibility);

  void generate_map(const map_t& map);
  void merge_map();
  void init_state();
  void stepPhysics(std::vector<double> actions_list, int step = 0,
                   bool take_action = true);

 public:
  OlympicsBase();
  obslist_t get_obs();
};

#endif