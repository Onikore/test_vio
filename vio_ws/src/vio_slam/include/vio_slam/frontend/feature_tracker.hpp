#ifndef VIO_SLAM_FEATURE_TRACKER_HPP
#define VIO_SLAM_FEATURE_TRACKER_HPP

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <opencv2/video/tracking.hpp>
#include <vector>
#include <memory>
#include "vio_slam/camera/camera_model.hpp"

namespace vio_slam {
namespace frontend {

/**
 * @brief Tracked feature with 2D position and optional 3D position
 */
struct TrackedFeature {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    int id;                          // Unique feature ID
    Eigen::Vector2d px;              // Current pixel position
    Eigen::Vector3d p3d;             // 3D position in world frame (if triangulated)
    bool has_3d = false;             // Whether 3D position is known
    std::vector<Eigen::Vector2d> track_history;  // Positions in previous frames
    int track_length = 0;            // Number of frames tracked
};

/**
 * @brief KLT optical flow feature tracker
 * 
 * Tracks Shi-Tomasi corners across consecutive frames using pyramidal Lucas-Kanade
 */
class FeatureTracker {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    using Ptr = std::shared_ptr<FeatureTracker>;
    
    struct Config {
        int max_features = 200;              // Maximum number of features to track
        int min_features_per_cell = 3;       // Minimum features per grid cell
        int grid_rows = 8;                   // Grid rows for distribution
        int grid_cols = 8;                   // Grid columns for distribution
        double quality_level = 0.01;         // Shi-Tomasi quality threshold
        double min_distance = 20.0;          // Minimum distance between features
        int pyramid_levels = 4;              // Optical flow pyramid levels
        int window_size = 21;                // LK window size
        int max_track_length = 50;           // Maximum track length before reset
        double epipolar_threshold = 3.0;     // RANSAC threshold in pixels
    };
    
    explicit FeatureTracker(const Config& config = Config());
    
    /**
     * @brief Process new image and track features
     * @param image New grayscale image
     * @param timestamp Image timestamp
     * @return Vector of tracked features
     */
    std::vector<TrackedFeature> track(const cv::Mat& image, double timestamp);
    
    /**
     * @brief Set camera model for geometric verification
     */
    void setCameraModel(camera::CameraModel::ConstPtr camera);
    
    /**
     * @brief Get current feature count
     */
    size_t getFeatureCount() const { return features_.size(); }
    
    /**
     * @brief Get average track length
     */
    double getAverageTrackLength() const;
    
    /**
     * @brief Reset tracker state
     */
    void reset();

private:
    /**
     * @brief Detect new features in cells with insufficient coverage
     */
    void detectFeatures(const cv::Mat& image);
    
    /**
     * @brief Reject outliers using fundamental matrix RANSAC
     */
    std::vector<bool> rejectOutliers(const std::vector<cv::Point2f>& prev_pts,
                                     const std::vector<cv::Point2f>& curr_pts);
    
    /**
     * @brief Assign features to grid cells
     */
    void updateGrid();
    
    Config config_;
    camera::CameraModel::ConstPtr camera_;
    
    cv::Mat prev_image_;
    double prev_timestamp_ = 0.0;
    
    std::vector<TrackedFeature> features_;
    std::vector<std::vector<int>> grid_;  // Grid cell -> feature indices
    
    int next_feature_id_ = 0;
};

} // namespace frontend
} // namespace vio_slam

#endif // VIO_SLAM_FEATURE_TRACKER_HPP
