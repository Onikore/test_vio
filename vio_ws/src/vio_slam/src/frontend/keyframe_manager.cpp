#include "vio_slam/frontend/keyframe_manager.h"

namespace vio_slam {
namespace frontend {

KeyframeManager::KeyframeManager(double min_parallax, int min_features, double min_time)
    : min_parallax_(min_parallax), min_features_(min_features), min_time_(min_time) {}

bool KeyframeManager::shouldCreateKeyframe(
    double current_parallax,
    int tracked_features,
    double time_since_last) const {
  
  return (current_parallax > min_parallax_) ||
         (tracked_features < min_features_) ||
         (time_since_last > min_time_);
}

}  // namespace frontend
}  // namespace vio_slam
