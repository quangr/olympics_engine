#ifndef ENVPOOL_CURLING_CORE_ENV_H_
#define ENVPOOL_CURLING_CORE_ENV_H_
#include <Eigen/Dense>
#include <random>

#include "objects.hpp"
using point2 = Eigen::Vector2d;
using mat2 = Eigen::Matrix2d;

using obslist_t = Eigen::Matrix<uint8_t, -1, -1>;
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
  // std::vector<std::vector<point2>> agent_record;
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

  bool done = false;
  int max_step = 500;

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
  std::tuple<double, Target, int, int> bounceable_wall_collision_time(
      std::vector<point2>, std::vector<point2>, double, ignore_t);
  std::tuple<double, Target, int, int> circle_collision_time(
      std::vector<point2>, std::vector<point2>, double, ignore_t);

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
                   int current_agent_idx, double col_t,
                   std::vector<point2>& pos_container,
                   std::vector<point2>& v_container, double& remaining_t,
                   ignore_t& ignore_wall_list);
  void handle_circle(int target_circle_idx, Target col_target,
                     int current_circle_idx, double col_t,
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
      throw std::runtime_error("not implemented");
    }
  }

 public:
  int step_cnt = 0;
  obslist_t obs_list;
  OlympicsBase();
  obslist_t get_obs();
};

class curling : public OlympicsBase {
 private:
  int final_winner;
  int round_countdown;
  bool print_log = false;
  bool draw_obs = true;
  bool show_traj = false;
  point2 start_pos{300, 150};
  double start_init_obs = 90;
  int max_n = 4;
  int round_max_step = 100;
  map_t map_copy;
  double vis = 300;
  double vis_clear = 10;
  double top_area_gamma, down_area_gamma;
  int round_step, game_round, num_purple, num_green, purple_game_point,
      green_game_point;
  void cross_detect() {
    for (size_t agent_idx = 0; agent_idx < agent_num; agent_idx++) {
      auto& agent = agent_list[agent_idx];
      if (agent.is_ball) continue;
      for (size_t object_idx = 0; object_idx < map.objects.size();
           object_idx++) {
        auto object = map.objects[object_idx];
        if (!object->can_pass)
          continue;
        else {
          if (object->color == red &&
              object->check_cross(agent_pos[agent_idx], agent.r)) {
            agent.alive = false;
            // #agent.color = 'red'
            gamma = down_area_gamma;  //   #this will change the gamma for the
                                      //   whole env, so need to change if
                                      //   dealing with multi-agent
            release = true;
            round_countdown = round_max_step - round_step;
          }
        }
      }
    }
  }
  bool is_terminal() {
    if (num_green + num_purple == max_n * 2) {
      if ((!release) && round_step > round_max_step) return true;

      if (release) {
        bool L = true;
        for (size_t agent_idx = 0; agent_idx < agent_num; agent_idx++) {
          if ((agent_v[agent_idx][0] * agent_v[agent_idx][0] +
               agent_v[agent_idx][1] * agent_v[agent_idx][1]) < 1e-1)
            L &= true;
          else
            L &= false;
        }

        return L;
      }
    } else
      return false;
  }
  std::tuple<bool, int> _round_terminal() {
    if ((round_step > round_max_step) &&
        (!release))  //      #after maximum round step the agent has not
                     //      released yet
      return {true, -1};

    // #agent_idx = -1
    bool L = true;
    for (size_t agent_idx = 0; agent_idx < agent_num; agent_idx++) {
      if ((!agent_list[agent_idx].alive) &&
          ((agent_v[agent_idx][0] * agent_v[agent_idx][0] +
            agent_v[agent_idx][1] * agent_v[agent_idx][1]) < 1e-1))
        L &= true;
      else
        L &= false;
    }

    return {L, 0};
  }

  int current_winner() {
    auto center = point2(300, 500);
    auto min_dist = 1e4;
    auto win_team = -1;
    for (size_t i = 0; i < agent_list.size(); i++) {
      auto agent = agent_list[i];
      auto pos = agent_pos[i];
      auto distance = (pos - center).norm();
      if (distance < min_dist) {
        win_team = agent.color == purple ? 0 : 1;
        min_dist = distance;
      }
    }

    return win_team;
  }

  obslist_t _reset_round() {
    current_team = 1 - current_team;
    // #convert last agent to ball
    if (agent_list.size() != 0) {
      agent_list[agent_list.size() - 1].to_ball();
      agent_list[agent_list.size() - 1].alive = false;
    }
    Color new_agent_color;
    if (current_team == 0) {
      new_agent_color = purple;
      num_purple += 1;
    } else if (current_team == 1) {
      new_agent_color = green;
      num_green += 1;
    } else {
      throw std::runtime_error("not implemented");
    }
    agent_list.push_back({1, 15, start_pos, new_agent_color, vis, vis_clear});
    agent_init_pos[agent_init_pos.size() - 1] = start_pos;
    auto new_boundary = get_obs_boundaray(start_pos, 15, vis);
    obs_boundary_init.push_back(new_boundary);
    agent_num += 1;
    agent_pos.push_back(start_pos);
    agent_v.push_back({0, 0});
    agent_accel.push_back({0, 0});
    auto init_obs = start_init_obs;
    agent_theta.push_back(init_obs);
    // agent_record.push_back({start_pos});
    release = false;
    gamma = top_area_gamma;
    round_step = 0;
    return _render ? get_obs() : obs_list;
  }
  void cal_game_point() {
    point2 center(300, 500);
    std::vector<double> purple_dis;
    std::vector<double> green_dis;
    auto min_dist = 1e4;
    auto closest_team = -1;
    for (size_t i = 0; i < agent_list.size(); i++) {
      auto agent = agent_list[i];
      auto pos = agent_pos[i];
      auto distance = (pos - center).norm();
      if (agent.color == purple)
        purple_dis.push_back(distance);
      else if (agent.color == green)
        green_dis.push_back(distance);
      else
        throw std::runtime_error("not implemented");

      // raise NotImplementedError
      if (distance < min_dist) {
        closest_team = agent.color == purple ? 0 : 1;
        min_dist = distance;
      }
    }
    std::sort(purple_dis.begin(), purple_dis.end());
    std::sort(green_dis.begin(), green_dis.end());
    if (closest_team == 0) {
      if (green_dis.size() == 0)
        purple_game_point += purple_dis.size();
      else {
        for (auto t : purple_dis) {
          purple_game_point += t < green_dis[0];
        }
      }
    } else if (closest_team == 1)
      if (purple_dis.size() == 0)
        purple_game_point += green_dis.size();
      else {
        for (auto t : purple_dis) {
          purple_game_point += t < purple_dis[0];
        }
      }
  }
  void _clear_agent() {
    if ((round_step > round_max_step) && !release) {
      agent_list.pop_back();
      agent_pos.pop_back();
      agent_v.pop_back();
      agent_theta.pop_back();
      agent_accel.pop_back();
      agent_num -= 1;
    }
  }

 public:
  int temp_winner;
  bool _render = true;
  int cur_ball = 0;
  bool release = false;
  int current_team;
  curling() {
    wall_restitution = 1;
    circle_restitution = 1;
  };
  obslist_t reset(bool reset_game = false) {
    top_area_gamma = 0.98;
    down_area_gamma = 0.95;
    gamma = top_area_gamma;

    agent_num = 0;
    agent_list.clear();
    agent_init_pos.clear();
    agent_pos.clear();
    agent_previous_pos.clear();
    agent_v.clear();
    agent_accel.clear();
    agent_theta.clear();
    temp_winner = -1;
    round_step = 0;

    if (reset_game) {
      assert(game_round == 1);
      current_team = 1;
      num_purple = 0;
      num_green = 1;

      map_copy = map;
      map_copy.agents[0].color = green;
      map_copy.agents[0].original_color = green;
    } else {
      num_purple = 1;
      num_green = 0;
      current_team = 0;
      purple_game_point = 0;
      green_game_point = 0;
      game_round = 0;
      map_copy = map;
    }
    obs_boundary_init.clear();
    obs_boundary = obs_boundary_init;

    generate_map(map_copy);
    merge_map();

    init_state();
    step_cnt = 0;
    done = false;
    release = false;

    // viewer = Viewer(view_setting)
    display_mode = false;
    // view_terminal = false

    auto obs = get_obs();
    return obs;
    // if current_team == 0:
    //     return [obs, np.zeros_like(obs)-1]
    // else:
    //     return [np.zeros_like(obs)-1, obs]
  }
  std::tuple<obslist_t, reward_t, bool, std::string> step(
      std::deque<std::vector<double>> actions_list);
};

#endif