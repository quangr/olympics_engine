#ifndef ENVPOOL_CURLING_CORE_ENV_H_
#define ENVPOOL_CURLING_CORE_ENV_H_
#include <Eigen/Dense>
using point2f = Eigen::Vector2f;
float cross_prod(point2f v1, point2f v2);
float point2line(point2f l1, point2f l2, point2f point);
#endif