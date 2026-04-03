#include "vio_slam/loop_closure/loop_detector.hpp"

namespace vio_slam {
namespace loop_closure {

LoopDetector::LoopDetector(double score_threshold)
    : score_threshold_(score_threshold) {}

std::optional<int> LoopDetector::detectLoop(const std::vector<cv::Mat>& descriptors) {
  // Placeholder - actual implementation would use DBoW3
  return std::nullopt;
}

void LoopDetector::addKeyframe(int id, const std::vector<cv::Mat>& descriptors) {
  keyframes_.push_back(id);
}

}  // namespace loop_closure
}  // namespace vio_slam
