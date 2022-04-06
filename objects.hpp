
#include <math.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>
namespace py = pybind11;
using point2 = Eigen::Vector2d;
using mat2 = Eigen::Matrix2d;
using rawimage_t = Eigen::Matrix<uint8_t, -1, -1>;
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

namespace std {
namespace {

// Code from boost
// Reciprocal of the golden ratio helps spread entropy
//     and handles duplicates.
// See Mike Seymour in magic-numbers-in-boosthash-combine:
//     https://stackoverflow.com/questions/4948780

template <class T>
inline void hash_combine(std::size_t& seed, T const& v) {
  seed ^= hash<T>()(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

// Recursive template code derived from Matthieu M.
template <class Tuple, size_t Index = std::tuple_size<Tuple>::value - 1>
struct HashValueImpl {
  static void apply(size_t& seed, Tuple const& tuple) {
    HashValueImpl<Tuple, Index - 1>::apply(seed, tuple);
    hash_combine(seed, get<Index>(tuple));
  }
};

template <class Tuple>
struct HashValueImpl<Tuple, 0> {
  static void apply(size_t& seed, Tuple const& tuple) {
    hash_combine(seed, get<0>(tuple));
  }
};
}  // namespace

template <typename... TT>
struct hash<std::tuple<TT...>> {
  size_t operator()(std::tuple<TT...> const& tt) const {
    size_t seed = 0;
    HashValueImpl<std::tuple<TT...>>::apply(seed, tt);
    return seed;
  }
};
}  // namespace std

struct ignore_t {
  std::unordered_set<std::tuple<int, int, double>> idx_ignore;
  std::unordered_set<std::tuple<int, point2, double>> point2_ignore;
  ignore_t() = default;
  void clear() {
    idx_ignore.clear();
    point2_ignore.clear();
  }
  bool contains(std::tuple<int, point2, double> obj) {
    return point2_ignore.count(obj);
  }
  bool contains(std::tuple<int, int, double> obj) {
    return idx_ignore.count(obj);
  }
  void insert(std::tuple<int, point2, double> obj) {
    point2_ignore.insert(obj);
  }
  void insert(std::tuple<int, int, double> obj) { idx_ignore.insert(obj); }
};

enum Target { None, wall, l1, l2, outter, inner, arc, circle };

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
struct object_t {
  ~object_t() = default;
  Color color;
  virtual void update_obs_map(rawimage_t& obs_map, int obs_size,
                              double visibility, double v_clear, double theta,
                              const agent_t& agent,
                              const agent_t& agent_self) = 0;
};
struct component_t : object_t {
  component_t() = default;
  virtual ~component_t() = default;
  bool ball_can_pass, can_pass, is_cross = false;
  virtual bool check_cross(point2 pos, double radius) { return false; };
  std::vector<point2> cur_pos;
  std::vector<point2> cur_pos_rotated;
  // virtual void print(std::ostream& os) const {};
  virtual void update_cur_pos(double agent_x, double agent_y){};
  virtual bool in_vision(point2 vec_oc_, double visibility) = 0;
  virtual void update_cur_pos_rotated(double theta){};
  virtual void update_point2wall(
      std::unordered_map<point2, std::vector<int>>& point2wall, int idx){};
  virtual component_t* copy() = 0;
  virtual std::tuple<double, Target> collision_time(point2 pos, point2 v,
                                                    double radius,
                                                    int agent_idx,
                                                    int object_idx,
                                                    ignore_t ignore) = 0;

  virtual std::tuple<point2, point2> collision_response(point2 pos, point2 v,
                                                        double r,
                                                        Target col_target,
                                                        double t,
                                                        double restitution) = 0;
  virtual point2 getattr(Target t) = 0;
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

struct agent_t : object_t, InternalState {
  std::vector<point2> to_another_agent;
  std::vector<point2> to_another_agent_rotated;
  Color original_color;
  bool is_ball = false;
  int temp_idx;
  void to_ball() { is_ball = true; };
  virtual void update_obs_map(rawimage_t& obs_map, int obs_size,
                              double visibility, double v_clear, double theta,
                              const agent_t& agent, const agent_t& agent_self) {
    for (int i = 0; i < obs_size; i++) {
      auto x = agent.r + visibility - v_clear * i - v_clear / 2;
      for (int j = 0; j < obs_size; j++) {
        if (obs_map(i, j) > 0) continue;
        auto y = visibility / 2 - v_clear * j - v_clear / 2;
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
  agent_t() = default;
  agent_t(double mass, double r, point2 position, Color color, double vis,
          double vis_clear) {
    this->mass = mass;
    this->r = r;
    position_init = position;
    this->color = color;
    this->visibility = vis;
    this->visibility_clear = vis_clear;
    original_color = color;
  };
};

struct wall_t : component_t {
  double _endpoint_collision_time(point2 pos, point2 v, double radius,
                                  point2 endpoint) {
    auto deno = v[0] * v[0] + v[1] * v[1];
    if (deno == 0) return -1;
    auto k =
        ((pos[0] - endpoint[0]) * v[0] + (pos[1] - endpoint[1]) * v[1]) / deno;
    auto c =
        (radius * radius - (pos[0] - endpoint[0]) * (pos[0] - endpoint[0]) -
         (pos[1] - endpoint[1]) * (pos[1] - endpoint[1])) /
        deno;
    auto sqrt = c + k * k;
    double tl;
    if (sqrt < 0)
      tl = -1;  // will not collide with this endpoint
    else {
      sqrt = std::sqrt(sqrt);
      auto t1 = -k + sqrt;
      auto t2 = -k - sqrt;

      if (t1 >= 0 && t2 >= 0)
        tl = std::min(t1, t2);
      else if (t1 < 0 && t2 < 0)
        tl = -1;
      else if (t1 >= 0 && t2 < 0)
        tl = t1;
      else {
        std::cout << "not implemented 13";
        abort();
      }

      // std::cout << ("endpoint collision time error");
    }
    return tl;
  };
  point2 getattr(Target t) {
    if (t == Target::l1) return l1;
    if (t == Target::l2) return l2;
    std::cout << "not implemented 14";
    abort();
    // panic
  };

  std::tuple<point2, point2> collision_response(point2 pos, point2 v, double r,
                                                Target col_target, double t,
                                                double restitution = 1) {
    double vx_new, vy_new;
    if (col_target == wall) {
      auto closest_p = helperfunction::closest_point(l1, l2, pos);
      point2 n(pos[0] - closest_p[0],
               pos[1] - closest_p[1]);  //# compute normal
      auto nn = n[0] * n[0] + n[1] * n[1];
      auto v_n = (n[0] * v[0] + n[1] * v[1]);

      auto factor = 2 * v_n / nn;
      vx_new = v[0] - factor * n[0];
      vy_new = v[1] - factor * n[1];
    }

    else if (col_target == Target::l1 or col_target == Target::l2) {
      auto l = col_target == Target::l1 ? l1 : l2;
      auto n = point2(pos[0] - l[0], pos[1] - l[1]);
      auto v_n = v[0] * n[0] + v[1] * n[1];
      auto nn = n[0] * n[0] + n[1] * n[1];
      auto factor = 2 * v_n / nn;
      vx_new = v[0] - factor * n[0];
      vy_new = v[1] - factor * n[1];
    } else {
      std::cout << "not implemented 15";
      abort();
      //  raise NotImplementedError("collision response error")
    }
    auto col_x = pos[0] + v[0] * t;
    auto col_y = pos[1] + v[1] * t;
    return {{col_x, col_y}, {vx_new * restitution, vy_new * restitution}};
  }
  bool check_on_line(point2 point) {
    auto temp = A * point[0] + B * point[1];
    if (abs(temp - C) <= 1e-6)
      return (((std::min(l1[0], l2[0]) <= point[0] <= std::max(l1[0], l2[0])) &&
               (std::min(l1[1], l2[1]) <= point[1] <= std::max(l1[1], l2[1]))));
    return false;
  };
  virtual component_t* copy() { return (new wall_t(*this)); }
  point2 l1;
  point2 l2;
  double width, length;
  double A, B, C;
  std::vector<point2> init_pos;
  std::tuple<double, Target> collision_time(point2 pos, point2 v, double radius,
                                            int agent_idx, int object_idx,
                                            ignore_t ignore) override;
  // virtual void print(std::ostream& os) const;
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
  virtual void update_obs_map(rawimage_t& obs_map, int obs_size,
                              double visibility, double v_clear, double theta,
                              const agent_t& agent, const agent_t& agent_self) {
    for (int i = 0; i < obs_size; i++) {
      auto x = agent.r + visibility - v_clear * i - v_clear / 2;
      for (int j = 0; j < obs_size; j++) {
        if (obs_map(i, j) > 0) continue;
        auto y = visibility / 2 - v_clear * j - v_clear / 2;
        auto point = point2(x, y);
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
struct cross_t : wall_t {
  virtual bool check_cross(point2 pos, double radius) {
    auto l1 = init_pos[0], l2 = init_pos[1];
    auto closest_p = helperfunction::closest_point(l1, l2, pos);
    if (!((std::min(l1[0], l2[0]) <= closest_p[0] &&
           closest_p[0] <= std::max(l1[0], l2[0])) &&
          (std::min(l1[1], l2[1]) <= closest_p[1] &&
           closest_p[1] <= std::max(l1[1], l2[1]))))
      // #print('THE CROSSING POINT IS NOT ON THE CROSS LINE SEGMENT')
      return false;

    point2 n(pos[0] - closest_p[0], pos[1] - closest_p[1]);  //# compute normal
    auto nn_sqrt = n.norm();
    point2 cl1(l1[0] - pos[0], l1[1] - pos[1]);
    auto cl1_n = (cl1[0] * n[0] + cl1[1] * n[1]) / nn_sqrt;

    return abs(cl1_n) - radius <= 0;
  };
};

struct arc_t : component_t {
  virtual component_t* copy() { return (new arc_t(*this)); }
  double start_radian, end_radian, color;
  bool passable, circle;
  int collision_mode;
  double width, length;
  double R;
  std::vector<double> init_pos;
  point2 center;
  point2 getattr(Target t) {
    std::cout << "not implemented 16";
    abort();
  };

  std::tuple<point2, point2> collision_response(point2 pos, point2 v, double r,
                                                Target col_target, double t,
                                                double restitution = 1) {
    auto x_old = pos[0], y_old = pos[1];
    auto x_new = x_old + v[0] * t;
    auto y_new = y_old + v[1] * t;
    auto n = point2(x_new - center[0], y_new - center[1]);

    auto v_n = v[0] * n[0] + v[1] * n[1];
    auto nn = n[0] * n[0] + n[1] * n[1];
    auto factor = 2 * v_n / nn;
    auto vx_new = v[0] - factor * n[0];
    auto vy_new = v[1] - factor * n[1];

    auto col_x = pos[0] + v[0] * t;
    auto col_y = pos[1] + v[1] * t;

    return {{col_x, col_y}, {vx_new * restitution, vy_new * restitution}};
  }
  bool check_radian(point2 pos, point2 v, double t) {
    auto x_old = pos[0], y_old = pos[1];
    auto x_new = x_old + v[0] * t;
    auto y_new = y_old + v[1] * t;
    auto angle =
        atan2(center[1] - y_new,
              x_new - center[0]);  //   compute the angle of the circle, which
                                   //   is also the angle of the collision point
    if (collision_mode == 0 && angle == start_radian) return true;
    if (collision_mode == 1 && angle == end_radian) return true;
    if (collision_mode == 2 && (angle == start_radian or angle == end_radian))
      return true;

    if (start_radian >= 0) {
      if (end_radian >= 0 && end_radian >= start_radian)
        return start_radian < angle && angle < end_radian;

      else if (end_radian >= 0 && end_radian < start_radian)

        return !(start_radian < angle && angle < end_radian);
      else if (end_radian <= 0) {
        if (angle >= 0 && angle > start_radian)
          return true;
        else
          return (angle < 0 && angle < end_radian);
      }
    } else {
      if (end_radian >= 0)

        if (angle >= 0 and angle < end_radian)
          return true;
        else
          return (angle < 0 and angle > start_radian);

      else if (end_radian < 0 and end_radian > start_radian)

        return (angle < 0 and start_radian < angle < end_radian);

      else if (end_radian < 0 and end_radian < start_radian)

        return !(end_radian < angle < start_radian);
    }
  }

  std::tuple<double, Target> collision_time(point2 pos, point2 v, double radius,
                                            int agent_idx, int object_idx,
                                            ignore_t ignore) override;
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
  virtual void update_obs_map(rawimage_t& obs_map, int obs_size,
                              double visibility, double v_clear, double theta,
                              const agent_t& agent, const agent_t& agent_self) {
    for (int i = 0; i < obs_size; i++) {
      auto x = agent.r + visibility - v_clear * i - v_clear / 2;
      for (int j = 0; j < obs_size; j++) {
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
struct map_view_t {
  objects_t objects;
  view_t view;
  std::vector<agent_t> agents;

  ~map_view_t() = default;
};
struct map_t : map_view_t {
  map_t(){};
  ~map_t() {
    for (auto i : objects) delete i;
  };
  map_t& operator=(const map_t& rhs) {
    if (this != &rhs) {
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
    return (*this);
  }
  map_t(const map_t&) = delete;
};
using reward_t = std::tuple<double, double>;