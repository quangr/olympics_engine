#include "core.h"

float inline cross_prod(point2f v1, point2f v2) {
  return v1.x() * v2.y() - v2.x() * v1.y();
}

float inline point2line(point2f l1, point2f l2, point2f point) {
  // :param l1: coord of line start point
  // :param l2: coord of line end point
  // :param point: coord of circle center
  // :return:
  auto l1l2 = l2 - l1;
  auto l1c = point - l1;
  auto l1l2_length = l1l2.norm();
  return abs(cross_prod(l1c, l1l2)) / l1l2_length;
}
