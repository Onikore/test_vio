#ifndef VIO_SLAM_SLIDING_WINDOW_OPTIMIZER_HPP
#define VIO_SLAM_SLIDING_WINDOW_OPTIMIZER_HPP

#include <Eigen/Core>
#include <ceres/ceres.h>
#include <memory>
#include <vector>
#include "vio_slam/frontend/keyframe_manager.hpp"
#include "vio_slam/imu/imu_preintegration.hpp"

namespace vio_slam {
namespace backend {

/**
 * @brief Sliding window optimizer using Ceres Solver
 * 
 * Optimizes poses, velocities, and biases over last N keyframes
 */
class SlidingWindowOptimizer {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    using Ptr = std::shared_ptr<SlidingWindowOptimizer>;
    
    struct Config {
        int window_size = 10;              // Number of keyframes in window
        int max_iterations = 10;           // Maximum Ceres iterations
        double solver_tolerance = 1e-6;    // Convergence tolerance
        
        // Loss functions
        double visual_huber_loss = 1.0;    // Huber loss for visual residuals
        double imu_huber_loss = 1.0;       // Huber loss for IMU residuals
    };
    
    explicit SlidingWindowOptimizer(const Config& config = Config());
    
    /**
     * @brief Add keyframe to optimization window
     */
    void addKeyframe(const frontend::KeyFrame::Ptr& kf);
    
    /**
     * @brief Add IMU preintegration between keyframes
     */
    void addImuFactor(int keyframe_id_i, int keyframe_id_j,
                      const imu::PreintegratedImu& pim);
    
    /**
     * @brief Run optimization
     * @return True if optimization converged
     */
    bool optimize();
    
    /**
     * @brief Marginalize oldest keyframe from window
     */
    void marginalizeOldest();
    
    /**
     * @brief Get optimized keyframes
     */
    std::vector<frontend::KeyFrame::Ptr> getOptimizedKeyframes() const;
    
    /**
     * @brief Clear all data
     */
    void clear();

private:
    Config config_;
    std::vector<frontend::KeyFrame::Ptr> keyframes_;
    // Additional internal state for marginals would go here
};

} // namespace backend
} // namespace vio_slam

#endif // VIO_SLAM_SLIDING_WINDOW_OPTIMIZER_HPP
