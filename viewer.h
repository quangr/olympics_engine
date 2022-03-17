#ifndef ENVPOOL_CURLING_VIEWER_ENV_H_
#define ENVPOOL_CURLING_VIEWER_ENV_H_

#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <string>
#include <unordered_map>
#include <vector>
using namespace pybind11::literals;
namespace py = pybind11;
class Viewer {
 private:
  int width;
  int height;
  int edge;
  int WIN_SIZE[2];
  std::vector<std::vector<int>> color_list = {
      {255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {0, 0, 0}, {160, 32, 240}};

 public:
  Viewer(int width, int height, int edge);
};
extern std::unordered_map<std::string, std::vector<int>> COLORS;
extern std::unordered_map<std::string, int> COLOR_TO_IDX;
extern std::vector<std::string> IDX_TO_COLOR;
#endif