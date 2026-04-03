#include "vio_slam/backend/sliding_window_optimizer.hpp"

namespace vio_slam {
namespace backend {

SlidingWindowOptimizer::SlidingWindowOptimizer(int window_size)
    : window_size_(window_size) {}

bool SlidingWindowOptimizer::optimize() {
  // Placeholder - actual implementation would use Ceres
  return true;
}

void SlidingWindowOptimizer::addKeyframe(const Keyframe& kf) {
  keyframes_.push_back(kf);
  if (keyframes_.size() > static_cast<size_t>(window_size_)) {
    keyframes_.erase(keyframes_.begin());
  }
}

}  // namespace backend
}  // namespace vio_slam
