#ifndef VIO_SLAM_LOOP_DETECTOR_HPP
#define VIO_SLAM_LOOP_DETECTOR_HPP

#include <memory>
#include <vector>
#include "vio_slam/frontend/keyframe_manager.hpp"

namespace vio_slam {
namespace loop_closure {

/**
 * @brief Loop closure detection result
 */
struct LoopClosureResult {
    bool success = false;
    int query_kf_id;           // Query keyframe ID
    int match_kf_id;           // Matched (loop) keyframe ID
    double score;              // DBoW3 similarity score
    Eigen::Isometry3d T_query_match;  // Relative transform
    int inlier_count;          // Number of geometric inliers
};

/**
 * @brief Loop detector using DBoW3 vocabulary
 */
class LoopDetector {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    using Ptr = std::shared_ptr<LoopDetector>;
    
    struct Config {
        std::string vocabulary_path;     // Path to DBoW3 vocabulary file
        
        // Detection thresholds
        double min_score = 0.15;         // Minimum BoW similarity score
        int temporal_gap = 20;           // Minimum keyframes between candidates
        int consecutive_confirmations = 3; // Confirmations needed
        
        // Geometric verification
        int min_inliers = 30;            // Minimum PnP inliers
        double ransac_threshold = 3.0;   // RANSAC threshold in pixels
    };
    
    explicit LoopDetector(const Config& config = Config());
    
    /**
     * @brief Add keyframe to database
     */
    void addKeyframe(const frontend::KeyFrame::Ptr& kf);
    
    /**
     * @brief Query for loop closures
     * @param query_kf Query keyframe
     * @return Loop closure result (success=false if no loop)
     */
    LoopClosureResult detect(const frontend::KeyFrame::Ptr& query_kf);
    
    /**
     * @brief Load vocabulary from file
     */
    bool loadVocabulary(const std::string& path);
    
    /**
     * @brief Save database to disk
     */
    bool saveDatabase(const std::string& path) const;
    
    /**
     * @brief Load database from disk
     */
    bool loadDatabase(const std::string& path);
    
    /**
     * @brief Clear database
     */
    void clear();

private:
    /**
     * @brief Geometric verification using PnP RANSAC
     */
    bool verifyGeometrically(const frontend::KeyFrame::Ptr& query,
                             const frontend::KeyFrame::Ptr& match,
                             Eigen::Isometry3d& T_rel,
                             int& inlier_count);
    
    Config config_;
    // DBoW3 database would be stored here
    std::vector<frontend::KeyFrame::Ptr> keyframes_;
    std::vector<int> confirmation_counts_;
};

} // namespace loop_closure
} // namespace vio_slam

#endif // VIO_SLAM_LOOP_DETECTOR_HPP
