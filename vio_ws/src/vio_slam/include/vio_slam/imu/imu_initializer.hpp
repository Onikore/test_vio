#ifndef VIO_SLAM_IMU_INITIALIZER_HPP
#define VIO_SLAM_IMU_INITIALIZER_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <memory>
#include "vio_slam/imu/imu_preintegration.hpp"
#include "vio_slam/frontend/keyframe_manager.hpp"

namespace vio_slam {
namespace imu {

/**
 * @brief IMU initialization results
 */
struct InitializationResult {
    bool success = false;
    Eigen::Vector3d gravity;           // Estimated gravity direction
    Eigen::Vector3d gyro_bias;         // Estimated gyroscope bias
    double scale = 1.0;                // Metric scale factor (for monocular)
    std::string message;
};

/**
 * @brief IMU initializer for visual-inertial alignment
 * 
 * Supports two modes:
 * 1. Static initialization: Detect stationary state, estimate gravity and biases
 * 2. Dynamic initialization: Solve for gravity, velocity, scale using first N keyframes
 */
class ImuInitializer {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    using Ptr = std::shared_ptr<ImuInitializer>;
    
    struct Config {
        // Static initialization
        double static_accel_variance_threshold = 0.05;  // Variance threshold for static detection
        int static_window_size = 200;                    // Number of samples for variance check
        double static_gyro_threshold = 0.01;             // Gyro magnitude threshold
        
        // Dynamic initialization
        int min_keyframes_for_init = 10;                 // Minimum keyframes for VI alignment
        double max_initialization_time = 5.0;            // Timeout in seconds
        
        // Gravity
        double gravity_magnitude = 9.81;
    };
    
    explicit ImuInitializer(const Config& config = Config());
    
    /**
     * @brief Add IMU measurement for processing
     */
    void addImuMeasurement(const ImuMeasurement& meas);
    
    /**
     * @brief Add keyframe for dynamic initialization
     */
    void addKeyframe(const frontend::KeyFrame::Ptr& kf);
    
    /**
     * @brief Check if system is stationary (for static initialization)
     */
    bool isStatic() const;
    
    /**
     * @brief Attempt static initialization
     * @return Result with estimated gravity and biases
     */
    InitializationResult initializeStatic();
    
    /**
     * @brief Attempt dynamic initialization using visual-inertial alignment
     * @return Result with full VI initialization parameters
     */
    InitializationResult initializeDynamic();
    
    /**
     * @brief Check if initialization is complete
     */
    bool isInitialized() const { return initialized_; }
    
    /**
     * @brief Reset initializer state
     */
    void reset();
    
    /**
     * @brief Get current IMU buffer
     */
    const std::vector<ImuMeasurement>& getImuBuffer() const { return imu_buffer_; }

private:
    /**
     * @brief Compute acceleration variance over window
     */
    double computeAccelVariance() const;
    
    /**
     * @brief Estimate gravity direction from static accelerometer data
     */
    Eigen::Vector3d estimateGravityDirection() const;
    
    /**
     * @brief Solve for scale, velocity, and gravity using VI alignment
     */
    bool solveViAlignment(InitializationResult& result);
    
    Config config_;
    std::vector<ImuMeasurement> imu_buffer_;
    std::vector<frontend::KeyFrame::Ptr> keyframes_;
    
    bool initialized_ = false;
    InitializationResult last_result_;
};

} // namespace imu
} // namespace vio_slam

#endif // VIO_SLAM_IMU_INITIALIZER_HPP
