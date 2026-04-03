#include "vio_slam/mapping/sparse_map.h"

namespace vio_slam {
namespace mapping {

SparseMap::SparseMap() {}

void SparseMap::addPoint(int id, const Eigen::Vector3d& position) {
  points_[id] = position;
}

std::optional<Eigen::Vector3d> SparseMap::getPoint(int id) const {
  auto it = points_.find(id);
  if (it != points_.end()) {
    return it->second;
  }
  return std::nullopt;
}

std::vector<Eigen::Vector3d> SparseMap::getAllPoints() const {
  std::vector<Eigen::Vector3d> result;
  for (const auto& kv : points_) {
    result.push_back(kv.second);
  }
  return result;
}

}  // namespace mapping
}  // namespace vio_slam
