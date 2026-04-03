#include "vio_slam/backend/pose_graph_optimizer.hpp"

namespace vio_slam {
namespace backend {

PoseGraphOptimizer::PoseGraphOptimizer() {}

bool PoseGraphOptimizer::optimize() {
  // Placeholder - actual implementation would use Ceres
  return true;
}

void PoseGraphOptimizer::addNode(int id, const Sophus::SE3d& pose) {
  nodes_[id] = pose;
}

void PoseGraphOptimizer::addEdge(int from, int to, const Sophus::SE3d& relative_pose) {
  edges_.push_back({from, to, relative_pose});
}

}  // namespace backend
}  // namespace vio_slam
