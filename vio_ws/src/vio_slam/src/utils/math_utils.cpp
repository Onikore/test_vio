#include "vio_slam/utils/math_utils.h"

namespace vio_slam {
namespace utils {

double rad2deg(double rad) {
  return rad * 180.0 / M_PI;
}

double deg2rad(double deg) {
  return deg * M_PI / 180.0;
}

Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v) {
  Eigen::Matrix3d m;
  m << 0, -v.z(), v.y(),
       v.z(), 0, -v.x(),
       -v.y(), v.x(), 0;
  return m;
}

}  // namespace utils
}  // namespace vio_slam
