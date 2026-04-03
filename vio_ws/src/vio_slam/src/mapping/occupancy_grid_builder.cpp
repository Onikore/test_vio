#include "vio_slam/mapping/occupancy_grid_builder.h"

namespace vio_slam {
namespace mapping {

OccupancyGridBuilder::OccupancyGridBuilder(double resolution)
    : resolution_(resolution), octree_(resolution) {}

void OccupancyGridBuilder::insertPoint(const Eigen::Vector3d& point) {
  octree_.updateNode(point.x(), point.y(), point.z(), true);
}

void OccupancyGridBuilder::insertPoints(
    const std::vector<Eigen::Vector3d>& points) {
  for (const auto& p : points) {
    insertPoint(p);
  }
}

octomap::OcTree& OccupancyGridBuilder::getOctree() {
  return octree_;
}

}  // namespace mapping
}  // namespace vio_slam
