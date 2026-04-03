#include "vio_slam/imu/imu_initializer.hpp"

namespace vio_slam {
namespace imu {

ImuInitializer::ImuInitializer(double gravity_magnitude)
    : gravity_(gravity_magnitude), is_initialized_(false) {}

bool ImuInitializer::processMeasurement(const Eigen::Vector3d& accel,
                                         const Eigen::Vector3d& gyro) {
  measurements_.push_back({accel, gyro});
  
  if (measurements_.size() < 100) return false;
  
  // Compute variance to check if stationary
  Eigen::Vector3d mean_accel = Eigen::Vector3d::Zero();
  for (const auto& m : measurements_) {
    mean_accel += m.accel;
  }
  mean_accel /= measurements_.size();
  
  double variance = 0;
  for (const auto& m : measurements_) {
    variance += (m.accel - mean_accel).squaredNorm();
  }
  variance /= measurements_.size();
  
  // If stationary, estimate gravity direction and gyro bias
  if (variance < 0.01) {
    gravity_direction_ = -mean_accel.normalized();
    gyro_bias_ = Eigen::Vector3d::Zero();
    for (const auto& m : measurements_) {
      gyro_bias_ += m.gyro;
    }
    gyro_bias_ /= measurements_.size();
    
    is_initialized_ = true;
    return true;
  }
  
  return false;
}

Eigen::Vector3d ImuInitializer::getGravityDirection() const {
  return gravity_direction_;
}

Eigen::Vector3d ImuInitializer::getGyroBias() const {
  return gyro_bias_;
}

}  // namespace imu
}  // namespace vio_slam
