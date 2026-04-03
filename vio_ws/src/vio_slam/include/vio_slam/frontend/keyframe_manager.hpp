#ifndef VIO_SLAM_KEYFRAME_MANAGER_HPP
#define VIO_SLAM_KEYFRAME_MANAGER_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <vector>
#include <memory>
#include "vio_slam/frontend/feature_tracker.hpp"

namespace vio_slam {
namespace frontend {

/**
 * @brief Keyframe data structure
 */
struct KeyFrame {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    using Ptr = std::shared_ptr<KeyFrame>;
    using ConstPtr = std::shared_ptr<const KeyFrame>;
    
    int id;                                    // Unique keyframe ID
    double timestamp;                          // Timestamp
    Eigen::Isometry3d T_world_cam;             // Pose: world from camera
    cv::Mat image;                             // Grayscale image (for ORB extraction)
    std::vector<cv::KeyPoint> keypoints;       // Keypoint locations
    cv::Mat descriptors;                       // ORB descriptors
    std::vector<TrackedFeature> features;      // Tracked features at this keyframe
    
    bool is_initialized = false;               // Whether pose is initialized
};

/**
 * @brief Manages keyframe selection and storage
 */
class KeyframeManager {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    using Ptr = std::shared_ptr<KeyframeManager>;
    
    struct Config {
        // Keyframe selection criteria
        double min_parallax_px = 15.0;         // Minimum average parallax to trigger keyframe
        int min_features = 80;                 // Minimum features before triggering keyframe
        double min_time_interval = 0.5;        // Minimum time between keyframes (seconds)
        
        // ORB extraction
        int orb_max_keypoints = 1000;          // Maximum ORB keypoints
        float orb_scale_factor = 1.2f;         // ORB scale factor
        int orb_n_levels = 8;                  // ORB pyramid levels
        int orb_fast_threshold = 20;           // FAST threshold for ORB
        
        // Management
        int max_keyframes = 200;               // Maximum keyframes in memory
        int min_obs_per_feature = 3;           // Minimum observations before pruning
    };
    
    explicit KeyframeManager(const Config& config = Config());
    
    /**
     * @brief Evaluate if current frame should be a keyframe
     * @param tracked_features Current tracked features
     * @param last_kf Last keyframe
     * @param current_time Current timestamp
     * @return True if should create keyframe
     */
    bool shouldCreateKeyframe(const std::vector<TrackedFeature>& tracked_features,
                              const KeyFrame::ConstPtr& last_kf,
                              double current_time) const;
    
    /**
     * @brief Create a new keyframe
     * @param image Current image
     * @param timestamp Image timestamp
     * @param T_world_cam Initial camera pose estimate
     * @param features Current tracked features
     * @return New keyframe shared pointer
     */
    KeyFrame::Ptr createKeyframe(const cv::Mat& image,
                                 double timestamp,
                                 const Eigen::Isometry3d& T_world_cam,
                                 const std::vector<TrackedFeature>& features);
    
    /**
     * @brief Extract ORB descriptors for a keyframe
     * @param kf Keyframe to process
     */
    void extractOrbDescriptors(KeyFrame::Ptr kf);
    
    /**
     * @brief Get recent keyframes for backend optimization
     * @param n Number of recent keyframes
     * @return Vector of keyframe pointers
     */
    std::vector<KeyFrame::Ptr> getRecentKeyframes(size_t n) const;
    
    /**
     * @brief Get all keyframes
     */
    const std::vector<KeyFrame::Ptr>& getAllKeyframes() const { return keyframes_; }
    
    /**
     * @brief Remove old keyframes to bound memory usage
     */
    void pruneOldKeyframes();
    
    /**
     * @brief Get the latest keyframe
     */
    KeyFrame::Ptr getLatestKeyframe() const;
    
    /**
     * @brief Get keyframe by ID
     */
    KeyFrame::Ptr getKeyframeById(int id) const;
    
    /**
     * @brief Reset manager state
     */
    void reset();
    
    /**
     * @brief Get statistics
     */
    size_t getKeyframeCount() const { return keyframes_.size(); }
    int getNextId() const { return next_id_; }

private:
    /**
     * @brief Compute average parallax between frames
     */
    double computeAverageParallax(const std::vector<TrackedFeature>& features,
                                  const KeyFrame::ConstPtr& last_kf) const;
    
    Config config_;
    std::vector<KeyFrame::Ptr> keyframes_;
    int next_id_ = 0;
};

} // namespace frontend
} // namespace vio_slam

#endif // VIO_SLAM_KEYFRAME_MANAGER_HPP
