
#include "core.h"

#include <math.h>

#include <optional>
#include <vector>

#include "viewer.h"

namespace helperfunction {
double cross_prod(point2 v1, point2 v2) {
  return v1.x() * v2.y() - v2.x() * v1.y();
}

double point2line(point2 l1, point2 l2, point2 point) {
  // :param l1: coord of line start point
  // :param l2: coord of line end point
  // :param point: coord of circle center
  // :return:
  auto l1l2 = l2 - l1;
  auto l1c = point - l1;
  auto l1l2_length = l1l2.norm();
  return abs(cross_prod(l1c, l1l2)) / l1l2_length;
}

bool line_intersect(mat2 line1, mat2 line2) {
  auto p = line1.row(0);
  auto r = point2(line1(1, 0) - line1(0, 0), line1(1, 1) - line1(0, 1));

  auto q = line2.row(0);
  auto s = point2(line2(1, 0) - line2(0, 0), line2(1, 1) - line2(0, 1));

  auto rs = cross_prod(r, s);
  if (rs == 0) return false;
  auto q_p = point2(q[0] - p[0], q[1] - p[1]);

  auto t = cross_prod(q_p, s) / rs;
  auto u = cross_prod(q_p, r) / rs;
  if (0 <= t && t <= 1 && 0 <= u && u <= 1)
    return true;
  else
    return false;
}

point2 closest_point(point2 l1, point2 l2, point2 point) {
  auto A1 = l2[1] - l1[1];
  auto B1 = l1[0] - l2[0];
  auto C1 = (l2[1] - l1[1]) * l1[0] + (l1[0] - l2[0]) * l1[1];
  auto C2 = -B1 * point[0] + A1 * point[1];
  auto det = A1 * A1 + B1 * B1;
  double cx, cy;
  if (det == 0) {
    cx = point[0];
    cy = point[1];
  } else {
    cx = (A1 * C1 - B1 * C2) / det;
    cy = (A1 * C2 + B1 * C1) / det;
  }
  return point2(cx, cy);
}
point2 rotate(double x, double y, double theta) {
  auto x_n = cos(theta * M_PI / 180) * x + sin(theta * M_PI / 180) * y;
  auto y_n = -sin(theta * M_PI / 180) * x + cos(theta * M_PI / 180) * y;
  return point2(x_n, y_n);
};

point2 rotate2(double x, double y, double theta) {
  auto x_n = cos(theta * M_PI / 180) * x + sin(theta * M_PI / 180) * y;
  auto y_n = -sin(theta * M_PI / 180) * x + cos(theta * M_PI / 180) * y;
  return point2(x_n, y_n);
};
double get_distance(std::vector<point2> AB, point2 vec_OC, double AB_length,
                    bool pixel) {
  auto vec_OA = AB[0];
  auto vec_OB = AB[1];
  auto vec_CA = point2(vec_OA[0] - vec_OC[0], vec_OA[1] - vec_OC[1]);
  auto vec_CB = point2(vec_OB[0] - vec_OC[0], vec_OB[1] - vec_OC[1]);
  auto vec_AB = point2(vec_OB[0] - vec_OA[0], vec_OB[1] - vec_OA[1]);
  auto vec_AC = point2(-vec_OA[0] + vec_OC[0], -vec_OA[1] + vec_OC[1]);
  auto vec_BC = point2(-vec_OB[0] + vec_OC[0], -vec_OB[1] + vec_OC[1]);
  double d;
  if (pixel) {
    if (vec_AB.dot(vec_AC) < 0)
      d = vec_AC.norm();
    else {
      if (vec_AB.dot(vec_BC) > 0)
        d = vec_BC.norm();
      else
        d = ceil(cross_prod(vec_CA, vec_CB) / AB_length);
    }
  } else
    d = ceil(cross_prod(vec_CA, vec_CB) / AB_length);
  return d;
};
bool check_radian(double start_radian, double end_radian, double angle) {
  if (start_radian >= 0) {
    if (end_radian >= 0 && end_radian >= start_radian)
      return start_radian <= angle && angle <= end_radian;
    if (end_radian >= 0 && end_radian < start_radian)
      return !(start_radian <= angle && angle <= end_radian);
    if (end_radian <= 0) {
      if (angle >= 0 && angle >= start_radian)
        return true;
      else
        return (angle < 0 && angle <= end_radian);
    }
  } else {
    if (end_radian >= 0) {
      if (angle >= 0 && angle < end_radian)
        return true;
      else
        return angle < 0 && angle > start_radian;
    } else {
      if (end_radian < 0 && end_radian > start_radian)
        return angle < 0 && start_radian <= angle && angle <= end_radian;
      if (end_radian < 0 && end_radian < start_radian)
        return !(end_radian <= angle && angle <= start_radian);
    }
  }
};

}  // namespace helperfunction
void readjson(std::string filename, map_t& map);

OlympicsBase::OlympicsBase() {
  readjson("/app/envpool/classic_control/testhelper/scenario.json", map);
  generate_map(map);
  merge_map();
  this->view_setting = map.view;
  reset();
}

std::vector<point2> OlympicsBase::get_obs_boundaray(point2 init_position,
                                                    double r,
                                                    double visibility) {
  std::vector<point2> boundary;
  auto x_init = init_position[0];
  auto y_init = init_position[1];
  for (auto unit : std::vector<point2>{{0, 1}, {1, 1}, {1, -1}, {0, -1}}) {
    auto x = x_init + r + visibility * unit[0];
    auto y = y_init - visibility * unit[1] / 2;
    boundary.push_back({x, y});
  }
  return boundary;
}
void OlympicsBase::generate_map(const map_t& map) {
  for (size_t index = 0; index < map.agents.size(); index++) {
    auto item = map.agents[index];
    this->agent_list.push_back(item);
    auto position = item.position_init;
    auto r = item.r;
    this->agent_init_pos.push_back(item.position_init);
    this->agent_num += 1;
    if (!item.is_ball()) {
      auto visibility = item.visibility;
      auto boundary = get_obs_boundaray(position, r, visibility);
      this->obs_boundary_init.push_back(boundary);
    }
  }
}
void OlympicsBase::merge_map() {
  for (size_t idx = 0; idx < map.objects.size(); idx++) {
    auto map_item = map.objects[idx];
    map_item->update_point2wall(point2wall, idx);
  }
}
void OlympicsBase::init_state() {
  agent_pos.clear();
  agent_v.clear();
  agent_accel.clear();
  agent_theta.clear();
  agent_record.clear();
  for (size_t i = 0; i < agent_list.size(); i++) {
    agent_pos.push_back(agent_init_pos[i]);
    agent_previous_pos.push_back(agent_init_pos[i]);
    agent_v.push_back({0, 0});
    agent_accel.push_back({0, 0});
    auto init_obs = view_setting.init_obs[i];
    agent_theta.push_back(init_obs);
    agent_list[i].reset();
    agent_list[i].reset_color();
    agent_record.push_back({agent_init_pos[i]});
  }
}

obslist_t OlympicsBase::reset() {
  // set_seed()
  init_state();
  this->step_cnt = 0;
  this->done = false;
  // this->viewer = Viewer(view_setting);
  this->display_mode = false;
  return get_obs();
}

using namespace helperfunction;

void wall_t::update_cur_pos(double agent_x, double agent_y) {
  auto pos = this->init_pos;
  this->cur_pos.clear();
  for (auto p : pos) {
    auto vec_o_d = point2(p[0], -p[1]);
    auto vec_oo_ = point2(-agent_x, agent_y);
    auto vec_od = point2(vec_o_d[0] + vec_oo_[0], vec_o_d[1] + vec_oo_[1]);
    this->cur_pos.push_back({vec_od[0], vec_od[1]});
  }
}

void arc_t::update_cur_pos(double agent_x, double agent_y) {
  auto pos = this->center;
  this->cur_pos.clear();
  auto vec_o_d = point2(pos[0], -pos[1]);
  auto vec_oo_ = point2(-agent_x, agent_y);
  auto vec_od = point2(vec_o_d[0] + vec_oo_[0], vec_o_d[1] + vec_oo_[1]);
  this->cur_pos.push_back({vec_od[0], vec_od[1]});
}

obslist_t OlympicsBase::get_obs() {
  for (size_t agent_idx = 0; agent_idx < agent_list.size(); agent_idx++) {
    auto agent = agent_list[agent_idx];
    if (agent.is_ball()) continue;
    auto theta_copy = agent_theta[agent_idx];
    auto agent_pos = this->agent_pos;
    auto agent_x = agent_pos[agent_idx][0];
    auto agent_y = agent_pos[agent_idx][1];
    auto theta = theta_copy;
    auto position_init = agent.position_init;
    auto visibility = agent_list[agent_idx].visibility;
    auto v_clear = agent_list[agent_idx].visibility_clear;
    auto obs_size = int(visibility / v_clear);

    std::vector<point2> agent_current_boundary;
    for (auto b : obs_boundary_init[agent_idx]) {
      auto m = b[0];
      auto n = b[1];
      auto vec_oo_ = point2(-agent_x, agent_y);
      auto vec = point2(-position_init[0], position_init[1]);
      auto vec_o_a = point2(m, -n);
      auto vec_oa = point2(vec[0] + vec_o_a[0], vec[1] + vec_o_a[1]);
      auto b_x_ = vec_oa[0];
      auto b_y_ = vec_oa[1];
      auto tempp = rotate2(b_x_, b_y_, theta);
      auto x_new = tempp[0];
      auto y_new = tempp[1];
      auto x_new_ = x_new - vec_oo_[0];
      auto y_new_ = y_new - vec_oo_[1];
      agent_current_boundary.push_back({x_new_, -y_new_});
    }
    obs_boundary.push_back(agent_current_boundary);
    for (auto item : map.objects) item->update_cur_pos(agent_x, agent_y);
    auto vec_oc = point2(agent.r + visibility / 2, 0);
    auto c_x = vec_oc[0];
    auto c_y = vec_oc[1];
    auto tempp = rotate2(c_x, c_y, theta);
    auto c_x_ = tempp[0];
    auto c_y_ = tempp[1];
    auto vec_oc_ = point2(c_x_, c_y_);
    std::vector<component_t*> map_objects;
    map_t map_deduced;
    double distance;
    for (auto c : map.objects) {
      if (c->in_vision(vec_oc_, visibility)) {
        map_deduced.objects.push_back(c);
        map_objects.push_back(c);
      }
    }
    map_deduced.agents.clear();

    auto agent_self = agent_list[agent_idx];
    agent.to_another_agent.clear();
    agent.to_another_agent_rotated.clear();
    int temp_idx = 0;
    for (size_t a_i = 0; a_i < agent_list.size(); a_i++) {
      if (a_i == agent_idx) continue;
      auto a_other = agent_list[a_i];
      auto vec_o_b = point2(agent_pos[a_i][0], -agent_pos[a_i][1]);
      auto vec_oo_ = point2(-agent_x, agent_y);
      auto vec_ob = point2(vec_o_b[0] + vec_oo_[0], vec_o_b[1] + vec_oo_[1]);
      auto vec_bc_ = point2(vec_oc_[0] - vec_ob[0], vec_oc_[1] - vec_ob[1]);
      auto distance = sqrt(vec_bc_[0] * vec_bc_[0] + vec_bc_[1] * vec_bc_[1]);
      auto threshold = agent_list[agent_idx].visibility * 1.415 / 2 + a_other.r;
      if (distance <= threshold) {
        map_deduced.agents.push_back(a_other);
        a_other.temp_idx = temp_idx;
        map_objects.push_back(&a_other);
        agent.to_another_agent.push_back(vec_ob);
        temp_idx += 1;
      }
    }
    obs_list.clear();

    Eigen::MatrixXd obs_map = Eigen::MatrixXd::Zero(obs_size, obs_size);
    for (auto obj : map_deduced.objects) obj->update_cur_pos_rotated(theta);
    for (size_t id = 0; id < map_deduced.agents.size(); id++) {
      auto obj = map_deduced.agents[id];
      auto vec_ob = agent.to_another_agent[id];
      auto theta_obj = -theta;
      agent.to_another_agent_rotated.push_back(
          rotate2(vec_ob[0], vec_ob[1], theta_obj));
    }
    for (auto component = map_objects.rbegin(); component != map_objects.rend();
         component++)
      (*component)
          ->update_obs_map(obs_map, obs_size, visibility, v_clear, theta, agent,
                           agent_self);
    obs_list.push_back(obs_map);
  }
  return obs_list;
}

void OlympicsBase::stepPhysics(std::vector<double> actions_list, int step = 0,
                               bool take_action = true){

};

class curling : OlympicsBase {
 private:
  float tau = 0.1;
  float wall_restitution = 1;
  float circle_restitution = 1;
  bool print_log = false;
  bool draw_obs = true;
  bool show_traj = false;
  bool release;
  point2 start_pos{300, 150};
  float start_init_obs = 90;
  int max_n = 4;
  int round_max_step = 100;
  map_t map_copy;
  float vis = 300;
  float vis_clear = 10;
  float top_area_gamma, down_area_gamma, gamma;
  int agent_num, temp_winner, round_step, game_round, current_team, num_purple,
      num_green, purple_game_point, green_game_point;
  void reset(bool reset_game = false) {
    bool release = false;
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

    // if current_team == 0:
    //     return [obs, np.zeros_like(obs)-1]
    // else:
    //     return [np.zeros_like(obs)-1, obs]
  }

 public:
  curling();
  obslist_t step(std::vector<std::vector<double>> actions_list) {
    auto action = actions_list[current_team];
    stepPhysics(action, step_cnt, release);
  }
};
