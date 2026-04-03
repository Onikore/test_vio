#ifndef VIO_SLAM_SPARSE_MAP_HPP
#define VIO_SLAM_SPARSE_MAP_HPP

#include <Eigen/Core>
#include <memory>
#include <vector>
#include <unordered_map>

namespace vio_slam {
namespace mapping {

/**
 * @brief 3D map point
 */
struct MapPoint {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    using Ptr = std::shared_ptr<MapPoint>;
    
    int id;                                    // Unique ID
    Eigen::Vector3d position;                  // 3D position in world frame
    Eigen::Vector3d color = Eigen::Vector3d::Zero(); // RGB color (optional)
    std::unordered_map<int, Eigen::Vector2d> observations; // kf_id -> pixel
    int num_observations = 0;
    bool is_outlier = false;
};

/**
 * @brief Sparse feature map manager
 */
class SparseMap {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    using Ptr = std::shared_ptr<SparseMap>;
    
    struct Config {
        int min_observations = 3;      // Minimum observations before keeping point
        double max_reprojection_error = 5.0; // Max reprojection error for outliers
        int max_points = 10000;        // Maximum points in map
    };
    
    explicit SparseMap(const Config& config = Config());
    
    /**
     * @brief Add or update map point from triangulation
     */
    void addMapPoint(const MapPoint::Ptr& point);
    
    /**
     * @brief Add observation to existing point
     */
    void addObservation(int point_id, int keyframe_id, const Eigen::Vector2d& px);
    
    /**
     * @brief Get all active map points
     */
    std::vector<MapPoint::Ptr> getAllPoints() const;
    
    /**
     * @brief Get points observed by specific keyframe
     */
    std::vector<MapPoint::Ptr> getPointsObservedBy(int keyframe_id) const;
    
    /**
     * @brief Remove outlier points
     */
    void removeOutliers();
    
    /**
     * @brief Apply rigid transform to all points (after loop closure)
     */
    void applyTransform(const Eigen::Isometry3d& T);
    
    /**
     * @brief Clear map
     */
    void clear();
    
    /**
     * @brief Get statistics
     */
    size_t getPointCount() const { return points_.size(); }

private:
    Config config_;
    std::unordered_map<int, MapPoint::Ptr> points_;
    int next_id_ = 0;
};

} // namespace mapping
} // namespace vio_slam

#endif // VIO_SLAM_SPARSE_MAP_HPP
