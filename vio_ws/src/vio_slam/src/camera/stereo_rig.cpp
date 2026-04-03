#include "vio_slam/camera/stereo_rig.h"

namespace vio_slam {
namespace camera {

StereoRig::StereoRig(std::shared_ptr<CameraModel> left,
                     std::shared_ptr<CameraModel> right,
                     const Sophus::SE3d& T_right_left)
    : left_(left), right_(right), T_right_left_(T_right_left) {}

std::optional<double> StereoRig::computeDisparity(
    const Eigen::Vector2d& px_left,
    const Eigen::Vector2d& px_right) const {
  // Simple horizontal disparity (assumes rectified stereo)
  double disparity = px_left.x() - px_right.x();
  if (disparity > 0.1) return disparity;
  return std::nullopt;
}

std::optional<Eigen::Vector3d> StereoRig::triangulate(
    const Eigen::Vector2d& px_left,
    const Eigen::Vector2d& px_right) const {
  
  auto disparity = computeDisparity(px_left, px_right);
  if (!disparity.has_value()) return std::nullopt;
  
  // Unproject to normalized coordinates
  Eigen::Vector3d ray_left = left_->unproject(px_left);
  Eigen::Vector3d ray_right = right_->unproject(px_right);
  
  // Transform right ray to left frame
  Eigen::Vector3d t = T_right_left_.translation();
  double baseline = t.norm();
  
  // Simple triangulation (midpoint method)
  double depth = baseline / (*disparity + 1e-10);
  
  Eigen::Vector3d point_left = ray_left * depth;
  return point_left;
}

}  // namespace camera
}  // namespace vio_slam
