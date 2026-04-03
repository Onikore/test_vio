#include "vio_slam/imu/imu_preintegration.h"

namespace vio_slam {
namespace imu {

ImuPreintegration::ImuPreintegration(const ImuConfig& config)
    : config_(config) {
  reset();
}

void ImuPreintegration::reset() {
  delta_p_.setZero();
  delta_v_.setZero();
  delta_q_ = Eigen::Quaterniond(1, 0, 0, 0);
  covariance_.setZero();
}

void ImuPreintegration::integrate(const Eigen::Vector3d& accel,
                                   const Eigen::Vector3d& gyro,
                                   double dt) {
  // Simplified preintegration (on-manifold would be more accurate)
  Eigen::Vector3d omega = gyro - config_.gyro_bias;
  Eigen::Vector3d a = accel - config_.accel_bias;
  
  // Update delta rotation
  Eigen::Vector3d angle = omega * dt;
  Eigen::Quaterniond dq(1, angle.x()/2, angle.y()/2, angle.z()/2);
  dq.normalize();
  delta_q_ = delta_q_ * dq;
  
  // Update delta velocity and position
  delta_v_ += delta_q_ * a * dt;
  delta_p_ += delta_v_ * dt + 0.5 * delta_q_ * a * dt * dt;
}

}  // namespace imu
}  // namespace vio_slam
