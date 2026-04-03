#include "vio_slam/imu/ekf_propagator.hpp"

namespace vio_slam {
namespace imu {

EkfPropagator::EkfPropagator() {
  state_.setZero();
  state_.segment<4>(6) << 1, 0, 0, 0;  // Identity quaternion
  covariance_.setIdentity() * 0.01;
}

void EkfPropagator::propagate(const Eigen::Vector3d& accel,
                               const Eigen::Vector3d& gyro,
                               double dt) {
  // Extract state
  Eigen::Vector3d pos = state_.segment<3>(0);
  Eigen::Vector3d vel = state_.segment<3>(3);
  Eigen::Quaterniond q(state_(9), state_(10), state_(11), state_(12));
  Eigen::Vector3d accel_bias = state_.segment<3>(13);
  Eigen::Vector3d gyro_bias = state_.segment<3>(16);
  
  // Correct IMU measurements
  Eigen::Vector3d a = accel - accel_bias;
  Eigen::Vector3d omega = gyro - gyro_bias;
  
  // Propagate orientation
  Eigen::Vector3d angle = omega * dt;
  Eigen::Quaterniond dq(1, angle.x()/2, angle.y()/2, angle.z()/2);
  dq.normalize();
  q = q * dq;
  
  // Propagate velocity and position
  vel += q * a * dt;
  pos += vel * dt + 0.5 * q * a * dt * dt;
  
  // Update state
  state_.segment<3>(0) = pos;
  state_.segment<3>(3) = vel;
  state_.segment<4>(6) << q.w(), q.x(), q.y(), q.z();
  
  // Covariance propagation (simplified)
  covariance_ *= (1.0 + dt * 0.01);
}

Eigen::VectorXd EkfPropagator::getState() const {
  return state_;
}

void EkfPropagator::setState(const Eigen::VectorXd& state) {
  state_ = state;
}

}  // namespace imu
}  // namespace vio_slam
