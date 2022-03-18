#ifndef ENVPOOL_CURLING_CORE_ENV_H_
#define ENVPOOL_CURLING_CORE_ENV_H_
#include <Eigen/Dense>
#include <random>

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
  ignore_t global_wall_ignore;
  ignore_t global_circle_ignore;

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
  std::tuple<float, Target, int, int> bounceable_wall_collision_time(
      std::vector<point2>, std::vector<point2>, float, ignore_t);
  std::tuple<float, Target, int, int> circle_collision_time(std::vector<point2>,
                                                            std::vector<point2>,
                                                            float, ignore_t);

  void generate_map(const map_t& map);
  void merge_map();
  void init_state();
  void stepPhysics(std::vector<std::vector<double>> actions_list, int step,
                   bool take_action);
  std::vector<point2> actions_to_accel(
      std::vector<std::vector<double>> actions_list);
  std::tuple<point2, point2, point2, point2> CCD_circle_collision_f(
      point2 old_pos1, point2 old_pos2, point2 old_v1, point2 old_v2, double r1,
      double r2, double m1, double m2);
  double CCD_circle_collision(point2 old_pos1, point2 old_pos2, point2 old_v1,
                              point2 old_v2, double r1, double r2, double m1,
                              double m2);
  std::tuple<point2, point2> _circle_collision_response(point2 coord1,
                                                        point2 coord2,
                                                        point2 v1, point2 v2,
                                                        double m1, double m2);
  void handle_wall(int target_wall_idx, Target col_target,
                   int current_agent_idx, float col_t,
                   std::vector<point2>& pos_container,
                   std::vector<point2>& v_container, double& remaining_t,
                   ignore_t& ignore_wall_list);
  void handle_circle(int target_circle_idx, Target col_target,
                     int current_circle_idx, float col_t,
                     std::vector<point2>& pos_container,
                     std::vector<point2>& v_container, double& remaining_t,
                     ignore_t& ignore_circle_list);

  std::tuple<point2, point2> wall_response(int target_idx, Target col_target,
                                           point2 pos, point2 v, double r,
                                           double t) {
    auto object = map.objects[target_idx];
    auto [col_pos, col_v] =
        object->collision_response(pos, v, r, col_target, t, wall_restitution);

    return {col_pos, col_v};
  }

  void update_all(std::vector<point2>& pos_container,
                  std::vector<point2>& v_container, double t,
                  std::vector<point2> a) {
    for (size_t agent_idx = 0; agent_idx < agent_num; agent_idx++) {
      auto accel_x = a[agent_idx][0], accel_y = a[agent_idx][1];

      auto pos_old = pos_container[agent_idx];
      auto v_old = v_container[agent_idx];

      auto vx = v_old[0], vy = v_old[1];
      auto x = pos_old[0], y = pos_old[1];
      auto x_new = x + vx * t;  // # update position with t
      auto y_new = y + vy * t;
      auto pos_new = point2(x_new, y_new);
      auto vx_new =
          gamma * vx + accel_x * tau;  //  # update v with acceleration
      auto vy_new = gamma * vy + accel_y * tau;

      // #if vx_new ** 2 + vy_new ** 2 > self.max_speed_square:
      // #    print('speed limited')
      // #    v_new = [vx, vy]
      // #else:
      auto v_new = point2(vx_new, vy_new);

      pos_container[agent_idx] = pos_new;
      v_container[agent_idx] = v_new;
    }
  };

  void update_other(std::vector<point2>& pos_container,
                    std::vector<point2>& v_container, double t,
                    std::unordered_set<int> already_updated) {
    for (size_t agent_idx = 0; agent_idx < agent_num; agent_idx++) {
      if (already_updated.count(agent_idx)) continue;

      auto old_pos = pos_container[agent_idx];
      auto old_v = v_container[agent_idx];

      auto new_x = old_pos[0] + old_v[0] * t;
      auto new_y = old_pos[1] + old_v[1] * t;

      pos_container[agent_idx] = {new_x, new_y};
      v_container[agent_idx] = {old_v[0] * gamma, old_v[1] * gamma};
    }
  }
  void _add_wall_ignore(Target collision_wall_target, int current_agent_idx,
                        int target_wall_idx, ignore_t& ignore_wall,
                        bool if_global = false) {
    if (collision_wall_target == wall || collision_wall_target == arc) {
      if (if_global)
        global_wall_ignore.insert({current_agent_idx, target_wall_idx, 0});
      else
        ignore_wall.insert({current_agent_idx, target_wall_idx, 0});
    } else if (collision_wall_target == l1 or collision_wall_target == l2) {
      if (if_global)
        global_wall_ignore.insert(
            {current_agent_idx,
             map.objects[target_wall_idx]->getattr(collision_wall_target), 0});
      else
        ignore_wall.insert(
            {current_agent_idx,
             map.objects[target_wall_idx]->getattr(collision_wall_target), 0});
    } else {
      exit(233);
    }
  }

 public:
  OlympicsBase();
  obslist_t get_obs();
};

#endif