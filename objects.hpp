
#include <math.h>

#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <optional>
#include <vector>
using point2 = Eigen::Vector2d;
using mat2 = Eigen::Matrix2d;
namespace helperfunction {
double cross_prod(point2 v1, point2 v2);
double point2line(point2 l1, point2 l2, point2 point);
bool line_intersect(mat2 line1, mat2 line2);
point2 closest_point(point2 l1, point2 l2, point2 point);
point2 rotate(double x, double y, double theta);

point2 rotate2(double x, double y, double theta);
double get_distance(std::vector<point2> AB, point2 vec_OC, double AB_length,
                    bool pixel);
bool check_radian(double start_radian, double end_radian, double angle);

}  // namespace helperfunction
namespace std {

template <typename Scalar, int Rows, int Cols>
struct hash<Eigen::Matrix<Scalar, Rows, Cols>> {
  // https://wjngkoh.wordpress.com/2015/03/04/c-hash-function-for-eigen-matrix-and-vector/
  size_t operator()(const Eigen::Matrix<Scalar, Rows, Cols>& matrix) const {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i) {
      Scalar elem = *(matrix.data() + i);
      seed ^=
          std::hash<Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};
}  // namespace std

#include "viewer.h"

enum Color {
  light_green,
  green,
  sky_blue,
  yellow,
  grey,
  purple,
  black,
  red,
  blue
};
struct agent_t;
struct component_t {
  virtual ~component_t() = default;
  std::vector<point2> cur_pos;
  std::vector<point2> cur_pos_rotated;
  Color color;
  virtual void print(std::ostream& os) const {};
  virtual void update_cur_pos(double agent_x, double agent_y){};
  virtual bool in_vision(point2 vec_oc_, double visibility){};
  virtual void update_cur_pos_rotated(double theta){};
  virtual void update_obs_map(Eigen::MatrixXd& obs_map, int obs_size,
                              double visibility, double v_clear, double theta,
                              const agent_t& agent,
                              const agent_t& agent_self){};
  virtual void update_point2wall(
      std::unordered_map<point2, std::vector<int>>& point2wall, int idx){};
  virtual component_t* copy() { return (new component_t(*this)); }
};
struct InternalState {
  double mass;
  bool fatigue = false;
  int energy_cap = 1000;
  int energy = 1000;
  double visibility;
  double visibility_clear;
  double r;
  bool alive = true;
  bool finished = false;
  point2 position_init;
  void reset() {
    fatigue = false;
    energy = 1000;
    alive = true;
    finished = false;
  };
};

struct agent_t : component_t, InternalState {
  virtual component_t* copy() { return (new agent_t(*this)); }
  std::vector<point2> to_another_agent;
  std::vector<point2> to_another_agent_rotated;
  Color original_color;
  int temp_idx;
  virtual bool is_ball() { return false; };
  virtual void update_obs_map(Eigen::MatrixXd& obs_map, int obs_size,
                              double visibility, double v_clear, double theta,
                              const agent_t& agent, const agent_t& agent_self) {
    for (size_t i = 0; i < obs_size; i++) {
      auto x = agent.r + visibility - v_clear * i - v_clear / 2;
      for (size_t j = 0; j < obs_size; j++) {
        if (obs_map(i, j) > 0) continue;
        auto y = visibility / 2 - v_clear * j - v_clear / 2;
        auto point = point2(x, y);
        if (obs_map(i, j) > 0) continue;
        auto idx = temp_idx;
        auto vec_bc_ = point2(x - agent_self.to_another_agent_rotated[idx][0],
                              y - agent_self.to_another_agent_rotated[idx][1]);
        auto distance = vec_bc_.norm();
        if (distance <= r) obs_map(i, j) = color;
      }
    }
  };
  void reset_color() { color = original_color; };
};
struct ball_t : agent_t {
  virtual bool is_ball() { return true; };
};

// struct object_t : component_t {
//   point2* center;
//   double length;
//   double R;
//   double start_radian;
//   double end_radian;
// };

// struct Agent_t : component_t, InternalState {
//   std::string original_color;
//   //   Agent_t(std::string color = "purple", double mass = 1, double r = 50,
//   //           std::optional<std::vector<point2>> position = std::nullopt,
//   //           double vis = 300, double vis_clear = 12)
//   //       : component_t("agent", color),
//   //         original_color(color),
//   //         InternalState(mass, r, position, vis, vis_clear) {}
// };
struct wall_t : component_t {
  virtual component_t* copy() { return (new wall_t(*this)); }
  point2 l1;
  point2 l2;
  bool ball_can_pass;
  double width, length;
  double A, B, C;
  std::vector<point2> init_pos;
  virtual void print(std::ostream& os) const;
  virtual void update_cur_pos(double agent_x, double agent_y);
  virtual bool in_vision(point2 vec_oc_, double visibility) {
    return abs(helperfunction::get_distance(this->cur_pos, vec_oc_,
                                            this->length, false)) <=
           visibility / 2 * 1.415;
  };
  virtual void update_cur_pos_rotated(double theta) {
    auto points_pos = cur_pos;
    cur_pos_rotated.clear();
    for (auto pos : points_pos) {
      auto pos_x = pos[0];
      auto pos_y = pos[1];
      auto theta_obj = -theta;
      cur_pos_rotated.push_back(
          helperfunction::rotate2(pos_x, pos_y, theta_obj));
    }
  };
  virtual void update_obs_map(Eigen::MatrixXd& obs_map, int obs_size,
                              double visibility, double v_clear, double theta,
                              const agent_t& agent, const agent_t& agent_self) {
    for (size_t i = 0; i < obs_size; i++) {
      auto x = agent.r + visibility - v_clear * i - v_clear / 2;
      for (size_t j = 0; j < obs_size; j++) {
        auto y = visibility / 2 - v_clear * j - v_clear / 2;
        auto point = point2(x, y);
        if (obs_map(i, j) > 0) continue;

        auto distance = abs(
            helperfunction::get_distance(cur_pos_rotated, point, length, true));
        if (distance <= v_clear) obs_map(i, j) = color;
      }
    }
  };

  virtual void update_point2wall(
      std::unordered_map<point2, std::vector<int>>& point2wall, int idx) {
    if (point2wall.count(l1))
      point2wall[l1].push_back(idx);
    else
      point2wall[l1] = std::vector<int>(idx);
    if (point2wall.count(l2))
      point2wall[l2].push_back(idx);
    else
      point2wall[l2] = std::vector<int>(idx);
  };
};
struct cross_t : wall_t {};

struct arc_t : component_t {
  virtual component_t* copy() { return (new arc_t(*this)); }
  double start_radian, end_radian, color;
  bool passable, ball_can_pass, circle;
  int collision_mode;
  double width, length;
  double R;
  std::vector<double> init_pos;
  point2 center;
  virtual void update_cur_pos(double agent_x, double agent_y);
  virtual bool in_vision(point2 vec_oc_, double visibility) {
    auto distance = point2(this->cur_pos[0][0] - vec_oc_[0],
                           this->cur_pos[0][1] - vec_oc_[1])
                        .norm();
    return (distance <= visibility / 2 * 1.415 + this->R);
  };
  virtual void update_cur_pos_rotated(double theta) {
    auto pos = cur_pos;
    cur_pos_rotated.clear();
    auto pos_x = pos[0][0];
    auto pos_y = pos[0][1];
    auto theta_obj = -theta;
    cur_pos_rotated.push_back(helperfunction::rotate2(pos_x, pos_y, theta_obj));
  };
  virtual void update_obs_map(Eigen::MatrixXd& obs_map, int obs_size,
                              double visibility, double v_clear, double theta,
                              const agent_t& agent, const agent_t& agent_self) {
    for (size_t i = 0; i < obs_size; i++) {
      auto x = agent.r + visibility - v_clear * i - v_clear / 2;
      for (size_t j = 0; j < obs_size; j++) {
        if (obs_map(i, j) > 0) continue;

        auto y = visibility / 2 - v_clear * j - v_clear / 2;
        auto radius = R;
        auto x_2center = x - cur_pos_rotated[0][0];
        auto y_2center = y - cur_pos_rotated[0][1];
        auto theta_pixel = theta;
        auto tempp = helperfunction::rotate2(x_2center, y_2center, theta_pixel);
        auto angle = atan2(tempp[0], tempp[1]);
        if (helperfunction::check_radian(start_radian, end_radian, angle)) {
          auto vec =
              point2(x - cur_pos_rotated[0][0], y - cur_pos_rotated[0][1]);
          auto distance = vec.norm();
          if ((distance <= radius + v_clear) && (distance >= radius - v_clear))
            obs_map(i, j) = color;
        }
      }
    }
  }
};

struct view_t {
  int width;
  int height;
  int edge;
  std::vector<int> init_obs;
};
using objects_t = std::vector<component_t*>;
struct map_t {
  objects_t objects;
  view_t view;
  std::vector<agent_t> agents;
  map_t() {}
  ~map_t() {
    for (auto i : objects) delete i;
  }
  map_t& operator=(const map_t& rhs) {
    for (auto iter : objects) {
      delete iter;
    }
    objects.clear();
    for (auto iter : rhs.objects) {
      objects.push_back(iter->copy());
    }
    view = rhs.view;
    agents = rhs.agents;
  }
  map_t(const map_t&) = delete;
};
