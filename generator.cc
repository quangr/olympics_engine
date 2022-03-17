#include "json.hpp"
using json = nlohmann::json;
#include <fstream>
#include <iomanip>
#include <sstream>

#include "objects.hpp"
using json = nlohmann::json;
void wall_t::print(std::ostream& os) const {
  // os << "(" << this->A << ", " << this->B << ", " << this->C << ", "
  //    << this->width << ", " << this->ball_can_pass << ", " << this->l1 <<
  //    "\n "
  //    << this->l2 << "\n " << this->length << ")";
  os << "l1:" << this->l1 << "\n";
}
std::ostream& operator<<(std::ostream& os, const component_t& dt) {
  dt.print(os);
  return os;
}

std::unordered_map<std::string, Color> COLORSMAP{
    {"red", red},          {"green", green},
    {"blue", blue},        {"yellow", yellow},
    {"grey", grey},        {"purple", purple},
    {"black", black},      {"light green", light_green},
    {"sky blue", sky_blue}};

void from_json(const json& j, wall_t& obj) {
  std::vector<std::vector<double>> tmp;
  j.at("initial_position").get_to(tmp);
  for (auto i : tmp) obj.init_pos.push_back(point2(i.data()));
  obj.color = COLORSMAP[j.at("color")];
  obj.ball_can_pass = j.value("ball_pass", false);
  auto l1 = obj.init_pos[0];
  auto l2 = obj.init_pos[1];
  obj.A = l1[1] - l2[1];
  obj.B = l2[0] - l1[0];
  obj.C = (l1[1] - l2[1]) * l1[0] + (l2[0] - l1[0]) * l1[1];
  obj.l1 = l1;
  obj.l2 = l2;
  obj.length = j.value("length", (l1 - l2).norm());
  obj.width = j.value("width", 2);
}

void from_json(const json& j, cross_t& obj) {
  std::vector<std::vector<double>> tmp;
  j.at("initial_position").get_to(tmp);
  for (auto i : tmp) obj.init_pos.push_back(point2(i.data()));
  obj.color = COLORSMAP[j.at("color")];
  obj.ball_can_pass = j.value("ball_pass", true);
  auto l1 = obj.init_pos[0];
  auto l2 = obj.init_pos[1];
  obj.A = l1[1] - l2[1];
  obj.B = l2[0] - l1[0];
  obj.C = (l1[1] - l2[1]) * l1[0] + (l2[0] - l1[0]) * l1[1];
  obj.l1 = point2(l1);
  obj.l2 = point2(l2);
  obj.length = j.value("length", (l1 - l2).norm());
  obj.width = j.value("width", obj.color == red ? 5 : 2);
}

void from_json(const json& j, arc_t& obj) {
  j.at("initial_position").get_to(obj.init_pos);
  auto init_pos = obj.init_pos;
  double tmp;
  j.at("start_radian").get_to(tmp);
  obj.start_radian = tmp * M_PI / 180;
  j.at("end_radian").get_to(tmp);
  obj.end_radian = tmp * M_PI / 180;
  obj.color = COLORSMAP[j.at("color")];
  j.at("passable").get_to(obj.passable);
  j.at("collision_mode").get_to(obj.collision_mode);
  auto l1 = obj.init_pos[0];
  auto l2 = obj.init_pos[1];
  obj.center =
      point2(init_pos[0] + init_pos[2] * .5, init_pos[1] + init_pos[3] * .5);
  if (init_pos[2] == init_pos[3]) {
    obj.circle = true;
    obj.R = init_pos[2] * .5;
  } else {
    obj.circle = false;
  }
  obj.ball_can_pass = false;
  obj.width = j.value("width", 2);
}

void from_json(const json& j, agent_t& obj) {
  j.at("mass").get_to(obj.mass);
  j.at("radius").get_to(obj.r);
  std::vector<double> tmp;
  j.at("initial_position").get_to(tmp);
  obj.position_init = point2(tmp.data());
  obj.color = COLORSMAP[j.at("color")];
  obj.visibility = j.value("vis", 300);
  obj.visibility_clear = j.value("vis_clear", 12);
  obj.original_color = obj.color;
}

void from_json(const json& j, view_t& obj) {
  j.at("edge").get_to(obj.edge);
  j.at("height").get_to(obj.height);
  j.at("init_obs").get_to(obj.init_obs);
  j.at("width").get_to(obj.width);
}

void readjson(std::string filename, map_t& map) {
  std::ifstream file(filename);
  // a JSON text

  if (!file) {
    std::cout << "no such file";
    exit(1);
    // operations on the buffer...
  }
  std::stringstream ss;

  ss << file.rdbuf();

  file.close();
  // fill a stream with JSON text
  // parse and serialize JSON
  for (auto iter : map.objects) delete iter;
  map.objects.clear();
  json j_complete = json::parse(ss);
  json j_map = j_complete["curling"];
  map.view = j_map["view"].get<view_t>();
  json it;
  it = j_map["wall"].at("objects");
  for (auto& ell : it.items()) {
    wall_t* temp = new wall_t();
    ell.value().get_to(*temp);
    map.objects.push_back(temp);
  }
  it = j_map["arc"].at("objects");
  for (auto& ell : it.items()) {
    arc_t* temp = new arc_t();
    ell.value().get_to(*temp);
    map.objects.push_back(temp);
  }
  it = j_map["cross"].at("objects");
  for (auto& ell : it.items()) {
    cross_t* temp = new cross_t();
    ell.value().get_to(*temp);
    map.objects.push_back(temp);
  }
  it = j_map["agent"].at("objects");
  for (auto& ell : it.items()) {
    agent_t* temp = new agent_t();
    ell.value().get_to(*temp);
    map.agents.push_back(*temp);
  }
}