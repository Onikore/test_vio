#ifndef VIO_SLAM_OCCUPANCY_GRID_BUILDER_HPP
#define VIO_SLAM_OCCUPANCY_GRID_BUILDER_HPP

#include <memory>
#include <octomap/octomap.h>
#include "vio_slam/mapping/sparse_map.hpp"

namespace vio_slam {
namespace mapping {

/**
 * @brief 3D occupancy grid builder using OctoMap
 */
class OccupancyGridBuilder {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    using Ptr = std::shared_ptr<OccupancyGridBuilder>;
    
    struct Config {
        double resolution = 0.1;         // Octree resolution in meters
        double prob_hit = 0.7;           // Probability for hit
        double prob_miss = 0.4;          // Probability for miss
        double clamp_min = 0.12;         // Minimum clamping value
        double clamp_max = 0.97;         // Maximum clamping value
        int max_range = 10;              // Maximum sensor range in meters
    };
    
    explicit OccupancyGridBuilder(const Config& config = Config());
    
    /**
     * @brief Insert pointcloud from keyframe into octree
     */
    void insertPointcloud(const std::vector<Eigen::Vector3d>& points,
                          const Eigen::Isometry3d& sensor_origin);
    
    /**
     * @brief Insert sparse map points
     */
    void insertSparseMap(const SparseMap& map);
    
    /**
     * @brief Get the underlying octree
     */
    const octomap::OcTree& getOctree() const { return *octree_; }
    octomap::OcTree& getOctree() { return *octree_; }
    
    /**
     * @brief Clear octree
     */
    void clear();
    
    /**
     * @brief Save octree to file
     */
    bool saveToFile(const std::string& path) const;
    
    /**
     * @brief Load octree from file
     */
    bool loadFromFile(const std::string& path);
    
    /**
     * @brief Get statistics
     */
    size_t getOccupiedNodeCount() const;

private:
    Config config_;
    std::unique_ptr<octomap::OcTree> octree_;
};

} // namespace mapping
} // namespace vio_slam

#endif // VIO_SLAM_OCCUPANCY_GRID_BUILDER_HPP
