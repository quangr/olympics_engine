
#include "core.h"

#include <cassert>
#include <math.h>

#include <optional>
#include <stdexcept>
#include <vector>
namespace helperfunction {
double cross_prod(point2 v1, point2 v2) {
  return v1.x() * v2.y() - v2.x() * v1.y();
}
point2 point_rotate(point2 center, point2 point, double theta) {
  auto px = point[0] - center[0];
  auto py = point[1] - center[1];

  auto nx = point2(cos(theta * M_PI / 180), sin(theta * M_PI / 180));
  auto ny = point2(-sin(theta * M_PI / 180), cos(theta * M_PI / 180));
  auto new_x = px * nx[0] + py * nx[1];
  auto new_y = px * ny[0] + py * ny[1];
  return {new_x, new_y};
}

void DDA_line(rawimage_t &matrix, std::vector<point2> draw_line, double vis,
              double vis_clear, Color value, double view_back) {
  auto size = int(vis / vis_clear);
  assert(matrix.rows() == size);
  if (draw_line.size() == 1) {
    auto point1 = draw_line[0];
    auto x1 = point1[0], y1 = point1[1];
    x1 += view_back;
    y1 += vis / 2;
    x1 /= vis_clear;
    y1 /= vis_clear;
    auto x = x1 - 0.5;
    auto y = y1 - 0.5;
    matrix(size - 1 - int(x), int(y)) = 1;
    return;
  } else {
    if (draw_line.size() == 2) {
      auto p1 = draw_line[0];
      auto p2 = draw_line[1];
      auto x1 = p1[0], y1 = p1[1], x2 = p2[0], y2 = p2[1];
      x1 += view_back;
      y1 += vis / 2;
      x1 /= vis_clear;
      y1 /= vis_clear;
      x2 += view_back;
      y2 += vis / 2;
      x2 /= vis_clear;
      y2 /= vis_clear;

      auto dx = x2 - x1;
      auto dy = y2 - y1;
      auto steps = std::max(abs(dx), abs(dy));
      // std::cout << "steps:" << steps << std::endl;
      if (steps == 0) {
        x1 = p1[0];
        y1 = p1[1];
        x1 += view_back;
        y1 += vis / 2;
        x1 /= vis_clear;
        y1 /= vis_clear;

        auto x = x1 - 0.5;
        auto y = y1 - 0.5;
        matrix(size - 1 - int(x), int(y)) = 1;
        return;
      }
      auto delta_x = dx / steps;
      auto delta_y = dy / steps;
      auto x = x1 - 0.5;
      auto y = y1 - 0.5;

      assert(0 <= int(x) && int(x) <= size - 1);
      assert(0 <= int(y) && int(y) <= size - 1);
      for (int i = 0; i < int(steps + 1); i++) {
        assert((0 <= size - 1 - int(x)) && (size - 1 - int(x) < size) &&
               (0 <= int(y)) && (int(y) < size));
        matrix(size - 1 - int(x), int(y)) = value;
        x += delta_x;
        y += delta_y;
      }
      return;
    } else {
      THROW(std::runtime_error, "not implemented error");
    }
  }
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

bool line_intersect(std::vector<point2> line1, std::vector<point2> line2) {
  auto p = line1[0];
  auto r = point2(line1[1][0] - line1[0][0], line1[1][1] - line1[0][1]);

  auto q = line2[0];
  auto s = point2(line2[1][0] - line2[0][0], line2[1][1] - line2[0][1]);

  auto rs = cross_prod(r, s);
  if (rs == 0)
    return false;
  auto q_p = point2(q[0] - p[0], q[1] - p[1]);

  auto t = cross_prod(q_p, s) / rs;
  auto u = cross_prod(q_p, r) / rs;
  if (0 <= t && t <= 1 && 0 <= u && u <= 1)
    return true;
  else
    return false;
}

std::optional<point2> line_intersect_p(std::vector<point2> line1,
                                       std::vector<point2> line2) {
  auto p = line1[0];
  auto r = point2(line1[1][0] - line1[0][0], line1[1][1] - line1[0][1]);

  auto q = line2[0];
  auto s = point2(line2[1][0] - line2[0][0], line2[1][1] - line2[0][1]);

  auto rs = cross_prod(r, s);
  if (rs == 0)
    return {};
  auto q_p = point2(q[0] - p[0], q[1] - p[1]);

  auto t = cross_prod(q_p, s) / rs;
  auto u = cross_prod(q_p, r) / rs;
  if (0 <= t && t <= 1 && 0 <= u && u <= 1)
    return point2{p[0] + t * r[0], p[1] + t * r[1]};
  else
    return {};
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

bool get_obs_check_radian(double start_radian, double end_radian,
                          double angle) {
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

double distance_to_line(point2 l1, point2 l2, point2 pos) {
  auto closest_p = closest_point(l1, l2, pos);
  return (closest_p - pos).norm();
};

} // namespace helperfunction
void readjson(std::string filename, map_t &map);

OlympicsBase::OlympicsBase(std::string mappath) {
  readjson(mappath, map);
  generate_map();
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
    double x;
    if (VIEW_ITSELF) {
      x = x_init + visibility * unit[0] - VIEW_BACK * visibility;
    } else {
      x = x_init + r + visibility * unit[0];
    }
    auto y = y_init - visibility * unit[1] / 2;
    boundary.push_back({x, y});
  }
  return boundary;
}
void OlympicsBase::_generate_map(const map_t &map) {
  for (size_t index = 0; index < map.agents.size(); index++) {
    auto item = map.agents[index];
    this->agent_list.push_back(item);
    auto position = item.position_init;
    auto r = item.r;
    this->agent_init_pos.push_back(item.position_init);
    this->agent_num += 1;
    if (!item.is_ball) {
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
  // agent_record.clear();
  for (size_t i = 0; i < agent_list.size(); i++) {
    agent_pos.push_back(agent_init_pos[i]);
    agent_previous_pos.push_back(agent_init_pos[i]);
    agent_v.push_back({0, 0});
    agent_accel.push_back({0, 0});
    auto init_obs = view_setting.init_obs[i];
    agent_theta.push_back(init_obs);
    agent_list[i].reset();
    agent_list[i].reset_color();
    // agent_record.push_back({agent_init_pos[i]});
  }
}

obslist_t OlympicsBase::reset() {
  // set_seed()
  init_state();
  this->step_cnt = 0;
  this->done = false;
  // this->viewer = Viewer(view_setting);
  this->display_mode = false;
  auto obs = get_obs();
  return obs;
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
  obslist_t obs_list;
  obs_boundary.clear();
  obsmap_t obs_map;
  for (size_t agent_idx = 0; agent_idx < agent_list.size(); agent_idx++) {
    auto agent = agent_list[agent_idx];
    if (agent.is_ball){
      obs_list.push_back(obs_map);
      continue;
    }
    auto theta_copy = agent_theta[agent_idx];
    auto agent_pos = this->agent_pos;
    auto agent_x = agent_pos[agent_idx][0];
    auto agent_y = agent_pos[agent_idx][1];
    auto theta = theta_copy;
    auto position_init = agent.position_init;
    auto visibility = agent_list[agent_idx].visibility;
    auto v_clear = agent_list[agent_idx].visibility_clear;
    auto obs_size = int(visibility / v_clear);
    auto view_back = visibility * this->VIEW_BACK;
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

    auto view_center_x =
        agent_x + (visibility / 2 - view_back / 2) *
                      cos(theta * M_PI / 180); // start from agent x,y
    auto view_center_y =
        agent_y + (visibility / 2 - view_back / 2) * sin(theta * M_PI / 180);

    auto view_center = point2(view_center_x, view_center_y);
    auto view_R = visibility * sqrt(2) / 2;
    objects_t line_consider;

    for (auto item : map.objects)
      item->update_line_consider(view_R, view_center, line_consider);
    for (auto item : map.objects)
      item->update_cur_pos(agent_x, agent_y);
    auto vec_oc = VIEW_ITSELF ? point2(visibility / 2 - view_back / 2, 0)
                              : point2(agent.r + visibility / 2, 0);
    auto c_x = vec_oc[0];
    auto c_y = vec_oc[1];
    auto tempp = rotate2(c_x, c_y, theta);
    auto c_x_ = tempp[0];
    auto c_y_ = tempp[1];
    auto vec_oc_ = point2(c_x_, c_y_);
    std::vector<object_t *> map_objects;
    map_view_t map_deduced;
    double distance;
    for (auto c : map.objects) {
      if (c->in_vision(vec_oc_, visibility)) {
        map_deduced.objects.push_back(c);
        map_objects.push_back(c);
      }
    }
    map_deduced.agents.clear();

    auto &agent_self = agent_list[agent_idx];
    agent_self.to_another_agent.clear();
    agent_self.to_another_agent_rotated.clear();
    int temp_idx = 0;
    for (size_t a_i = 0; a_i < agent_list.size(); a_i++) {
      if (a_i == agent_idx)
        continue;
      auto &a_other = agent_list[a_i];
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
        agent_self.to_another_agent.push_back(vec_ob);
        temp_idx += 1;
      }
    }
    obs_map = Eigen::Matrix<uint8_t, -1, -1>::Zero(obs_size, obs_size);
    for (auto obj : map_deduced.objects)
      obj->update_cur_pos_rotated(theta);
    for (size_t id = 0; id < map_deduced.agents.size(); id++) {
      auto obj = map_deduced.agents[id];
      auto vec_ob = agent_self.to_another_agent[id];
      auto theta_obj = -theta;
      agent_self.to_another_agent_rotated.push_back(
          rotate2(vec_ob[0], vec_ob[1], theta_obj));
    }

    for (auto obj : line_consider)
      obj->update_line(obs_map, visibility, v_clear, agent_x, agent_y, theta,
                       view_back);

    if (map_objects.size() == 0 && VIEW_ITSELF) {
      for (int i = 0; i < obs_size; i++) {
        auto x = visibility - v_clear * i - v_clear / 2 - view_back;
        for (int j = 0; j < obs_size; j++) {
          auto y = visibility / 2 - v_clear * j - v_clear / 2;
          auto point = point2(x, y);
          auto self_center = point2(0, 0);
          auto dis_to_itself = (point - self_center).norm();
          if (dis_to_itself <= agent_list[agent_idx].r) {
            obs_map(i, j) = agent_list[agent_idx].color;
          }
        }
      }
    }
    for (int i = 0; i < obs_size; i++) {
      auto x = VIEW_ITSELF ? visibility - v_clear * i - v_clear / 2 - view_back
                           : agent.r + visibility - v_clear * i - v_clear / 2;
      for (int j = 0; j < obs_size; j++) {
        auto y = visibility / 2 - v_clear * j - v_clear / 2;
        if (VIEW_ITSELF) {
          point2 self_center{0, 0}, point{x, y};
          auto dis_to_itself = (point - self_center).norm();
          if (dis_to_itself <= agent_list[agent_idx].r)
            obs_map(i, j) = agent_list[agent_idx].color;
        }
        for (auto component = map_objects.rbegin();
             component != map_objects.rend(); component++) {
          (*component)
              ->update_obs_map(obs_map, i, j, x, y, theta, v_clear, agent,
                               agent_self);
        }
      }
    }
    obs_list.push_back(obs_map);
  }
  this->obs_list=obs_list;
  return obs_list;
}

std::vector<point2>
OlympicsBase::actions_to_accel(std::vector<std::vector<double>> actions_list) {
  std::vector<point2> a_container(agent_num);
  for (size_t agent_idx = 0; agent_idx < agent_num; agent_idx++) {
    auto action = actions_list[agent_idx];
    auto mass = agent_list[agent_idx].mass;
    assert(action_f[0] <= action[0] && action[0] <= action_f[1]);
    auto force = action[0] / mass;

    assert(action_theta[0] <= action[1] && action[1] <= action_theta[1]);
    auto theta = action[1];

    auto theta_old = agent_theta[agent_idx];
    auto theta_new = theta_old + theta;
    agent_theta[agent_idx] = theta_new;

    auto accel_x = force * cos(theta_new / 180 * M_PI);
    auto accel_y = force * sin(theta_new / 180 * M_PI);
    a_container[agent_idx] = point2(accel_x, accel_y);
  }
  return a_container;
}

std::tuple<double, Target> wall_t::collision_time(point2 pos, point2 v,
                                                  double radius, int agent_idx,
                                                  int object_idx,
                                                  ignore_t ignore) {
  auto closest_p = closest_point(l1, l2, pos);
  point2 n(pos[0] - closest_p[0], pos[1] - closest_p[1]);
  auto nn_sqrt = n.norm();
  auto cl1 = point2(l1[0] - pos[0], l1[1] - pos[1]);
  auto cl1_n = cl1[0] * n[0] + cl1[1] * n[1];
  auto v_n = (n[0] * v[0] + n[1] * v[1]);
  if (v_n == 0)
    return {-1, None};
  auto r_ = cl1_n < 0 ? radius : -radius;
  auto wall_col_t =
      cl1_n / v_n +
      r_ * nn_sqrt / v_n; // the collision time with the line segment

  // check the collision point is on the line segment
  auto new_pos = point2(pos[0] + wall_col_t * v[0], pos[1] + wall_col_t * v[1]);
  auto collision_point = closest_point(l1, l2, new_pos);
  auto on_the_line = check_on_line(collision_point);
  Target endpoint_target;
  double t_endpoint;
  if (on_the_line)
    return {wall_col_t, wall};
  else
    wall_col_t = -1;

  // #now check the endpoint collision
  auto tl1 = _endpoint_collision_time(pos, v, radius, l1);
  auto tl2 = _endpoint_collision_time(pos, v, radius, l2);

  if (tl1 >= 0 && tl2 >= 0) {
    t_endpoint = std::min(tl1, tl2);
    endpoint_target = tl1 < tl2 ? Target::l1 : Target::l2;
  } else {
    if (tl1 < 0 and tl2 < 0) {
      t_endpoint = -1;
      endpoint_target = None;
    } else {
      t_endpoint = tl1 >= 0 ? tl1 : tl2;
      endpoint_target = tl1 >= 0 ? Target::l1 : Target::l2;
    }
  }

  return {t_endpoint, endpoint_target};
};

std::tuple<double, Target> arc_t::collision_time(point2 pos, point2 v,
                                                 double radius, int agent_idx,
                                                 int object_idx,
                                                 ignore_t ignore) {
  if (circle) {
    bool inflag = ignore.contains({agent_idx, object_idx, 0});
    // #else, compute the collision time and the target
    auto cx = center[0], cy = center[1];
    auto x = pos[0], y = pos[1];
    auto vx = v[0], vy = v[1];
    auto l = vx * x - cx * vx + y * vy - cy * vy;
    auto k = vx * vx + vy * vy;
    auto h = (x * x + y * y) + (cx * cx + cy * cy) - 2 * (cx * x + y * cy);
    auto RHS = (l / k) * (l / k) - h / k + ((R - radius) * (R - radius)) / k;
    double t1;
    if (RHS < 0)
      t1 = -1;
    else {
      auto sqrt = std::sqrt(RHS);

      auto t_inner1 = -(l / k) + sqrt;
      auto t_inner2 = -(l / k) - sqrt;
      if (abs(t_inner1) <= 1e-10)
        t_inner1 = 0;
      if (abs(t_inner2) <= 1e-10)
        t_inner2 = 0;
      auto t1_check = this->check_radian(pos, v, t_inner1);
      auto t2_check = this->check_radian(pos, v, t_inner2);
      if (t1_check && t2_check)
        if (abs(t_inner1) < 1e-10 && inflag)
          t1 = t_inner2;
        else if (abs(t_inner2) < 1e-10 && inflag)
          t1 = t_inner1;
        else

            if (t_inner1 <= 0 && t_inner2 <= 0)
          t1 = std::max(t_inner1, t_inner2);
        else if (t_inner1 >= 0 && t_inner2 >= 0)
          t1 = std::min(t_inner1, t_inner2);
        else if (t_inner1 >= 0 && t_inner2 <= 0)
          t1 = t_inner1;
        else {
          THROW(std::runtime_error, "not implemented error");
        }

      //                            #print('CHECK t1 = {}, t2 =
      //                            {}'.format(t_inner1, t_inner2))

      else if (t1_check && not t2_check)
        t1 = t_inner1;

      else if (not t1_check && t2_check)
        t1 = t_inner2;

      else //    #when both collision is outside the arc angle range
        t1 = -1;
    }
    auto RHS2 = (l / k) * (l / k) - h / k + (R + radius) * (R + radius) / k;
    double t2;
    if (RHS2 < 0)
      // print('outter collision has no solution')
      t2 = -1;
    else {
      auto sqrt2 = std::sqrt(RHS2);
      auto t_outter1 = -(l / k) + sqrt2;
      auto t_outter2 = -(l / k) - sqrt2;

      if (abs(t_outter1) <= 1e-10)
        t_outter1 = 0;
      if (abs(t_outter2) <= 1e-10)
        t_outter2 = 0;

      // check radian, for both t,
      auto t1_check = this->check_radian(pos, v, t_outter1);
      auto t2_check = this->check_radian(pos, v, t_outter2);

      if (t1_check && t2_check) {
        if (abs(t_outter1) < 1e-10 && inflag)
          t2 = t_outter2;
        else if (abs(t_outter2) < 1e-10 && inflag)
          t2 = t_outter1;
        else {
          if (t_outter1 <= 0 && t_outter2 <= 0) // leaving the collision
                                                // point
            t2 = std::max(t_outter1, t_outter2);
          else if (t_outter1 >= 0 &&
                   t_outter2 >= 0) // approaching the collision point
            t2 = std::min(t_outter1, t_outter2);
          else if (t_outter1 >= 0 && t_outter2 <= 0) // inside the circle
            t2 = t_outter1;
          else {
            THROW(std::runtime_error, "not implemented error");
          }

        } // raise NotImplementedError
      } else if (t1_check && not t2_check)
        t2 = t_outter1;

      else if (not t1_check && t2_check)
        t2 = t_outter2;

      else
        t2 = -1;
    }
    double col_t;
    Target col_target;
    if (t1 >= 0 and t2 >= 0) {
      if (t1 > t2) {
        col_target = outter;
        col_t = t2;
      } else if (t1 < t2) {
        col_target = inner;
        col_t = t1;
      } else {
        THROW(std::runtime_error, "not implemented error");
      }

    } // #print('t1 = {}, t2 = {}'.format(t1, t2))
    else if (t1 < 0 and t2 >= 0) {
      col_target = outter;
      col_t = t2;
    } else if (t1 >= 0 and t2 < 0) {
      col_target = inner;
      col_t = t1;
    } else {
      col_target = None;
      col_t = -1;
    }
    return {col_t, col_target == None ? None : arc};

  } else {
    THROW(std::runtime_error, "not implemented error");

    // not implement
  }
}

std::tuple<double, Target, int, int>
OlympicsBase::bounceable_wall_collision_time(std::vector<point2> pos_container,
                                             std::vector<point2> v_container,
                                             double remaining_t,
                                             ignore_t ignore) {
  Target col_target = None;
  int col_target_idx;
  int current_idx;
  auto current_min_t = remaining_t;
  for (size_t agent_idx = 0; agent_idx < agent_num; agent_idx++) {
    auto pos = pos_container[agent_idx];
    auto v = v_container[agent_idx];
    auto r = agent_list[agent_idx].r;
    if (v[0] == 0 && v[1] == 0)
      continue;
    for (size_t object_idx = 0; object_idx < map.objects.size(); object_idx++) {
      auto object = map.objects[object_idx];
      if (object->can_pass)
        continue;
      auto [temp_t, temp_col_target] =
          object->collision_time(pos, v, r, agent_idx, object_idx, ignore);
      // TODO QUESTIONHERE!

      if (abs(temp_t) < 1e-10)
        temp_t = 0;
      if (0 <= temp_t && temp_t < current_min_t) {
        bool check = true;
        {
          if (temp_col_target == wall || temp_col_target == arc)
            check = !ignore.contains({agent_idx, object_idx, temp_t});

          else if (temp_col_target == l1 || temp_col_target == l2)
            check = !ignore.contains({agent_idx, object_idx, temp_t});
          else {
            THROW(std::runtime_error, "not implemented error");
          }
          // raise NotImplementedError('bounceable_wall_collision_time error')
        }
        if (check) {
          current_min_t = temp_t;
          col_target = temp_col_target;
          col_target_idx = object_idx;
          current_idx = agent_idx;
        }
      }
    }
  }
  return {current_min_t, col_target, col_target_idx, current_idx};
};

double OlympicsBase::CCD_circle_collision(point2 old_pos1, point2 old_pos2,
                                          point2 old_v1, point2 old_v2,
                                          double r1, double r2, double m1,
                                          double m2) {
  auto relative_pos =
      point2(old_pos1[0] - old_pos2[0], old_pos1[1] - old_pos2[1]);
  auto relative_v = point2(old_v1[0] - old_v2[0], old_v1[1] - old_v2[1]);
  if (relative_v.norm() == 0)
    return -1;

  auto pos_v =
      relative_pos[0] * relative_v[0] + relative_pos[1] * relative_v[1];
  auto K =
      pos_v / (relative_v[0] * relative_v[0] + relative_v[1] * relative_v[1]);
  auto l = (relative_pos[0] * relative_pos[0] +
            relative_pos[1] * relative_pos[1] - (r1 + r2) * (r1 + r2)) /
           (relative_v[0] * relative_v[0] + relative_v[1] * relative_v[1]);

  auto sqrt = (K * K - l);
  if (sqrt < 0)
    // #print('CCD circle no solution')
    return -1;

  sqrt = std::sqrt(sqrt);
  auto t1 = -K - sqrt;
  auto t2 = -K + sqrt;
  auto t = std::min(t1, t2);
  return t;
};

std::tuple<point2, point2> OlympicsBase::_circle_collision_response(
    point2 coord1, point2 coord2, point2 v1, point2 v2, double m1, double m2) {
  auto n_x = coord1[0] - coord2[0];
  auto n_y = coord1[1] - coord2[1];

  auto vdiff_x = (v1[0] - v2[0]);
  auto vdiff_y = (v1[1] - v2[1]);

  auto n_vdiff = n_x * vdiff_x + n_y * vdiff_y;
  auto nn = n_x * n_x + n_y * n_y;
  auto b = n_vdiff / nn;

  //#object 1
  auto u1_x = v1[0] - 2 * (m2 / (m1 + m2)) * b * n_x;
  auto u1_y = v1[1] - 2 * (m2 / (m1 + m2)) * b * n_y;

  //#object 2
  auto u2_x = v2[0] + 2 * (m1 / (m1 + m2)) * b * n_x;
  auto u2_y = v2[1] + 2 * (m1 / (m1 + m2)) * b * n_y;

  return {{u1_x * circle_restitution, u1_y * circle_restitution},
          {u2_x * circle_restitution, u2_y * circle_restitution}};
}

std::tuple<point2, point2, point2, point2>
OlympicsBase::CCD_circle_collision_f(point2 old_pos1, point2 old_pos2,
                                     point2 old_v1, point2 old_v2, double r1,
                                     double r2, double m1, double m2) {
  auto relative_pos =
      point2(old_pos1[0] - old_pos2[0], old_pos1[1] - old_pos2[1]);
  auto relative_v = point2(old_v1[0] - old_v2[0], old_v1[1] - old_v2[1]);
  if (relative_v.norm() == 0) {
    THROW(std::runtime_error, "not implemented error");
  };

  auto pos_v =
      relative_pos[0] * relative_v[0] + relative_pos[1] * relative_v[1];
  auto K =
      pos_v / (relative_v[0] * relative_v[0] + relative_v[1] * relative_v[1]);
  auto l = (relative_pos[0] * relative_pos[0] +
            relative_pos[1] * relative_pos[1] - (r1 + r2) * (r1 + r2)) /
           (relative_v[0] * relative_v[0] + relative_v[1] * relative_v[1]);

  auto sqrt = (K * K - l);

  sqrt = std::sqrt(sqrt);
  auto t1 = -K - sqrt;
  auto t2 = -K + sqrt;
  auto t = std::min(t1, t2);

  auto x1 = old_pos1[0], y1 = old_pos1[1];
  auto x2 = old_pos2[0], y2 = old_pos2[1];
  auto x1_col = x1 + old_v1[0] * t;
  auto y1_col = y1 + old_v1[1] * t;
  auto x2_col = x2 + old_v2[0] * t;
  auto y2_col = y2 + old_v2[1] * t;
  auto pos_col1 = point2(x1_col, y1_col);
  auto pos_col2 = point2(x2_col, y2_col);

  // #handle collision
  auto [v1_col, v2_col] = _circle_collision_response(
      pos_col1, pos_col2, old_v1, old_v2, m1,
      m2); //   #the position and v at the collision time

  return {pos_col1, v1_col, pos_col2, v2_col};
};

std::tuple<double, Target, int, int>
OlympicsBase::circle_collision_time(std::vector<point2> pos_container,
                                    std::vector<point2> v_container,
                                    double remaining_t, ignore_t ignore) {
  int current_idx;
  int target_idx;
  auto current_min_t = remaining_t;
  Target col_target = None;

  for (size_t agent_idx = 0; agent_idx < agent_num; agent_idx++) {
    auto pos1 = pos_container[agent_idx];
    auto v1 = v_container[agent_idx];
    auto m1 = agent_list[agent_idx].mass;
    auto r1 = agent_list[agent_idx].r;

    for (size_t rest_idx = agent_idx + 1; rest_idx < agent_num; rest_idx++) {
      auto pos2 = pos_container[rest_idx];
      auto v2 = v_container[rest_idx];
      auto m2 = agent_list[rest_idx].mass;
      auto r2 = agent_list[rest_idx].r;

      // compute ccd collision time
      double collision_t =
          CCD_circle_collision(pos1, pos2, v1, v2, r1, r2, m1, m2);
      // # print('agent {}, time on circle {} is  = {}, current_min_t =
      // {}'.format(agent_idx, # rest_idx, # collision_t, # remaining_t)) #
      // print('ignore list = ', ignore)

      if (0 <= collision_t &&
          collision_t < current_min_t) //# and [agent_idx, rest_idx,
                                       // collision_t] not in ignore:
      {
        current_min_t = collision_t;
        current_idx = agent_idx;
        target_idx = rest_idx;
        col_target = circle;
      }
    }
  }

  return {current_min_t, col_target, current_idx, target_idx};
}

void OlympicsBase::handle_wall(int target_wall_idx, Target col_target,
                               int current_agent_idx, double col_t,
                               std::vector<point2> &pos_container,
                               std::vector<point2> &v_container,
                               double &remaining_t,
                               ignore_t &ignore_wall_list) {
  auto [col_pos, col_v] = wall_response(
      target_wall_idx, col_target = col_target,
      pos_container[current_agent_idx], v_container[current_agent_idx],
      agent_list[current_agent_idx].r, col_t);

  pos_container[current_agent_idx] = col_pos;
  v_container[current_agent_idx] = col_v;
  update_other(pos_container, v_container, col_t, {current_agent_idx});
  remaining_t -= col_t;
  if (remaining_t <= 1e-14) {
    if (col_target == wall) {
      _add_wall_ignore(wall, current_agent_idx, target_wall_idx,
                       ignore_wall_list, true);
      _add_wall_ignore(l1, current_agent_idx, target_wall_idx, ignore_wall_list,
                       true);
      _add_wall_ignore(l2, current_agent_idx, target_wall_idx, ignore_wall_list,
                       true);
    } else if (col_target == l2 || col_target == l1) {
      _add_wall_ignore(col_target, current_agent_idx, target_wall_idx,
                       ignore_wall_list, true);
      auto wall_endpoint = map.objects[target_wall_idx]->getattr(col_target);
      auto connected_wall = point2wall[wall_endpoint];
      for (auto idx : connected_wall)
        _add_wall_ignore(wall, current_agent_idx, idx, ignore_wall_list, true);
    } else
      _add_wall_ignore(col_target, current_agent_idx, target_wall_idx,
                       ignore_wall_list,
                       true); //  HERE MODIFYED #collision of arc
  }

  if (col_target == wall) {
    _add_wall_ignore(wall, current_agent_idx, target_wall_idx,
                     ignore_wall_list);
    _add_wall_ignore(l1, current_agent_idx, target_wall_idx, ignore_wall_list);
    _add_wall_ignore(l2, current_agent_idx, target_wall_idx, ignore_wall_list);
  } else if (col_target == l2 or col_target == l1) {
    _add_wall_ignore(col_target, current_agent_idx, target_wall_idx,
                     ignore_wall_list);
    auto wall_endpoint = map.objects[target_wall_idx]->getattr(col_target);
    auto connected_wall = point2wall[wall_endpoint];
    for (auto idx : connected_wall)
      _add_wall_ignore(wall, current_agent_idx, idx, ignore_wall_list);
  } else
    _add_wall_ignore(col_target, current_agent_idx, target_wall_idx,
                     ignore_wall_list);
}

void OlympicsBase::handle_circle(int target_circle_idx, Target col_target,
                                 int current_circle_idx, double col_t,
                                 std::vector<point2> &pos_container,
                                 std::vector<point2> &v_container,
                                 double &remaining_t,
                                 ignore_t &ignore_circle_list) {
  auto [pos_col1, v1_col, pos_col2, v2_col] = CCD_circle_collision_f(
      pos_container[current_circle_idx], pos_container[target_circle_idx],
      v_container[current_circle_idx], v_container[target_circle_idx],
      agent_list[current_circle_idx].r, agent_list[target_circle_idx].r,
      agent_list[current_circle_idx].mass, agent_list[target_circle_idx].mass);

  pos_container[current_circle_idx] = pos_col1;
  pos_container[target_circle_idx] = pos_col2;
  v_container[current_circle_idx] = v1_col;
  v_container[target_circle_idx] = v2_col;

  update_other(pos_container, v_container, col_t,
               {current_circle_idx, target_circle_idx});
  remaining_t -= col_t;
  if (remaining_t <= 1e-14)
    global_circle_ignore.insert({current_circle_idx, target_circle_idx,
                                 0.0}); // # adding instead of defining
  ignore_circle_list.insert({current_circle_idx, target_circle_idx, 0.0});
}

void OlympicsBase::stepPhysics(std::vector<std::vector<double>> actions_list,
                               int step = 0) {
  assert(actions_list.size() == agent_num);
  // print("The number of action needs to match with the number of agents!")
  // actions_list = actions_list
  auto temp_pos_container = agent_pos;
  auto temp_v_container = agent_v;
  // WARNING:ABOVE TWO MAYBE NEED CUTOFF by agent_num
  auto temp_a_container = actions_to_accel(actions_list);
  agent_accel = temp_a_container;

  auto remaining_t = tau;
  auto ignore_wall = global_wall_ignore;
  auto ignore_circle = global_circle_ignore;
  global_wall_ignore.clear();
  global_circle_ignore.clear();
  while (1) {
    auto [earliest_wall_col_t, collision_wall_target, target_wall_idx,
          current_agent_idx] =
        bounceable_wall_collision_time(temp_pos_container, temp_v_container,
                                       remaining_t, ignore_wall);

    auto [earliest_circle_col_t, collision_circle_target, current_circle_idx,
          target_circle_idx] =
        circle_collision_time(temp_pos_container, temp_v_container, remaining_t,
                              ignore_circle);

    if (collision_wall_target != None && collision_circle_target == None)
      // if self.print_log:
      //     print('HIT THE WALL!')
      handle_wall(target_wall_idx, collision_wall_target, current_agent_idx,
                  earliest_wall_col_t, temp_pos_container, temp_v_container,
                  remaining_t, ignore_wall);
    else if (collision_wall_target == None && collision_circle_target == circle)
      //     print('HIT THE BALL!')
      handle_circle(target_circle_idx, collision_circle_target,
                    current_circle_idx, earliest_circle_col_t,
                    temp_pos_container, temp_v_container, remaining_t,
                    ignore_circle);
    else if (collision_wall_target != None &&
             collision_circle_target == circle) {
      // print('HIT BOTH!')
      if (earliest_wall_col_t <
          earliest_circle_col_t) { // print('PROCESS WALL FIRST!')
        handle_wall(target_wall_idx, collision_wall_target, current_agent_idx,
                    earliest_wall_col_t, temp_pos_container, temp_v_container,
                    remaining_t, ignore_wall);
      } else if (earliest_wall_col_t >= earliest_circle_col_t) {
        // print('PROCESS CIRCLE FIRST!')
        handle_circle(target_circle_idx, collision_circle_target,
                      current_circle_idx, earliest_circle_col_t,
                      temp_pos_container, temp_v_container, remaining_t,
                      ignore_circle);
      }
    } else {
      // print('NO COLLISION!')
      update_all(temp_pos_container, temp_v_container, remaining_t,
                 temp_a_container);
      break; // #when no collision occurs, break the collision detection loop
    }
  }
  agent_pos = temp_pos_container;
  agent_v = temp_v_container;
  // std::cout << "agent_pos: " << agent_pos[0][0] << ',' << agent_pos[0][1]
  //           << std::endl;
  // std::cout << "agent_v: " << agent_v[0][0] << ',' << agent_v[0][1]
  //           << std::endl;
};

std::tuple<obslist_t, reward_t, bool, std::string>
curling::step(std::deque<std::vector<double>> actions_list) {
  auto action = std::vector<std::vector<double>>(agent_list.size(),
                                                 std::vector<double>(2));
  if (!release) {
    for (size_t agent_idx = 0; agent_idx < agent_list.size(); agent_idx++) {
      if (!agent_list[agent_idx].is_ball) {
        cur_ball = agent_idx;
        action[agent_idx] = actions_list[0];
        actions_list.pop_front();
      } else {
        action[agent_idx] = {0, 0};
      }
    }
  } else {
    for (size_t agent_idx = 0; agent_idx < agent_list.size(); agent_idx++) {
      action[agent_idx] = {0, 0};
    }
  }
  stepPhysics(action, step_cnt);
  if (!release)
    cross_detect();
  step_cnt += 1;
  round_step += 1;
  // auto obs_next = obs_list;
  // if (_render)
  auto obs_next = get_obs();
  auto done = is_terminal();
  reward_t step_reward;
  if (!done) {
    auto [round_end, end_info] = _round_terminal();
    if (round_end) {
      if (end_info != 0) { // #clean the last agent
        agent_list.pop_back();
        agent_pos.pop_back();
        agent_v.pop_back();
        agent_theta.pop_back();
        agent_accel.pop_back();
        agent_num -= 1;
      }
      temp_winner = current_winner();
      if (temp_winner == -1)
        step_reward = {0., 0.};
      else if (temp_winner == 0)
        step_reward = {1, 0.};
      else if (temp_winner == 1)
        step_reward = {0., 1};
      else {
        THROW(std::runtime_error, "not implemented error");
      }

      // raise NotImplementedError
      obs_next = _reset_round();
    } else {
      step_reward = {0., 0.};
    }
  } else {
    if (game_round == 1) {
      _clear_agent();
      cal_game_point();
      if (purple_game_point > green_game_point) {
        final_winner = 0;
        step_reward = {100., 0};
      } else if (green_game_point > purple_game_point) {
        final_winner = 1;
        step_reward = {0., 100.};
      } else {
        final_winner = -1;
        step_reward = {0., 0.};
      }
      temp_winner = final_winner;
      //                # step_reward = [100., 0] if self.final_winner == 0
      //                else [0., 100]
      // view_terminal = True;
    } else if (game_round == 0) {
      _clear_agent();
      auto game1_winner = current_winner();
      auto step_reward =
          game1_winner == 0 ? std::tuple{10., 0.} : std::tuple{0., 10.};
      cal_game_point();
      game_round += 1;
      auto next_obs = reset(true);
      return {next_obs, step_reward, false, "game1 ends, switch position"};
    } else {
      THROW(std::runtime_error, "not implemented error");
    }
  }
  // if (current_team == 0)
  //     // obs_next = [obs_next, np.zeros_like(obs_next)-1]
  // else
  //     // obs_next = [np.zeros_like(obs_next)-1, obs_next]

  if (release) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-1.0, 1.0);
    // TODO should be random!
    auto h_gamma = down_area_gamma + dis(gen) * 0.001;
    // auto h_gamma = down_area_gamma;
    gamma = h_gamma;
  }
  // #return self.agent_pos, self.agent_v, self.agent_accel, self.agent_theta,
  // obs_next, step_reward, done
  return {obs_next, step_reward, done, ""};
}
