#include "vio_slam/frontend/exposure_controller.hpp"

namespace vio_slam {
namespace frontend {

ExposureController::ExposureController(double target_brightness, double kp, double ki, double kd)
    : target_(target_brightness), kp_(kp), ki_(ki), kd_(kd),
      integral_(0.0), prev_error_(0.0) {}

double ExposureController::computeExposure(double current_brightness) {
  double error = target_ - current_brightness;
  
  integral_ += error;
  double derivative = error - prev_error_;
  prev_error_ = error;
  
  double adjustment = kp_*error + ki_*integral_ + kd_*derivative;
  return adjustment;
}

}  // namespace frontend
}  // namespace vio_slam
