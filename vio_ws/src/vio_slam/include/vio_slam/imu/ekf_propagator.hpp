#ifndef VIO_SLAM_EKF_PROPAGATOR_HPP
#define VIO_SLAM_EKF_PROPAGATOR_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "vio_slam/imu/imu_preintegration.hpp"
#include <memory>

namespace vio_slam {
namespace imu {

/**
 * @brief EKF state vector for IMU propagation
 * 
 * State: [position(3), velocity(3), orientation_quat(4), accel_bias(3), gyro_bias(3)] = 16D
 * Error state: 15D (orientation error as minimal 3D rotation vector)
 */
struct EkfState {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    Eigen::Vector3d position = Eigen::Vector3d::Zero();      // Position in world frame
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();      // Velocity in world frame
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity(); // World from body
    Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();    // Accelerometer bias
    Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();     // Gyroscope bias
    
    /**
     * @brief Get state as vector (for covariance)
     */
    Eigen::Matrix<double, 15, 1> getErrorState() const;
    
    /**
     * @brief Set state from error vector
     */
    void applyErrorState(const Eigen::Matrix<double, 15, 1>& error);
};

/**
 * @brief EKF propagator for high-rate IMU integration
 * 
 * Runs at IMU rate (200-400Hz), provides odometry between visual updates
 */
class EkfPropagator {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    using Ptr = std::shared_ptr<EkfPropagator>;
    
    struct Config {
        // Process noise (standard deviations)
        double position_noise_std = 0.01;       // m/sqrt(Hz)
        double velocity_noise_std = 0.1;        // m/s/sqrt(Hz)
        double orientation_noise_std = 0.01;    // rad/sqrt(Hz)
        double accel_bias_noise_std = 1e-4;     // m/s^2/sqrt(Hz)
        double gyro_bias_noise_std = 1e-5;      // rad/s/sqrt(Hz)
        
        // Initial covariance
        double initial_position_cov = 0.1;
        double initial_velocity_cov = 0.1;
        double initial_orientation_cov = 0.01;
        double initial_bias_cov = 0.01;
    };
    
    explicit EkfPropagator(const Config& config = Config());
    
    /**
     * @brief Initialize state
     */
    void initialize(const Eigen::Vector3d& position,
                    const Eigen::Vector3d& velocity,
                    const Eigen::Quaterniond& orientation,
                    const Eigen::Vector3d& gravity);
    
    /**
     * @brief Propagate state with IMU measurement
     * @param imu IMU measurement
     * @param dt Time since last propagation
     */
    void propagate(const ImuMeasurement& imu, double dt);
    
    /**
     * @brief Reset state to optimized values from backend
     */
    void resetToOptimized(const EkfState& state);
    
    /**
     * @brief Get current state estimate
     */
    const EkfState& getState() const { return state_; }
    
    /**
     * @brief Get current covariance
     */
    const Eigen::Matrix<double, 15, 15>& getCovariance() const { return P_; }
    
    /**
     * @brief Check if initialized
     */
    bool isInitialized() const { return initialized_; }
    
    /**
     * @brief Set gravity direction
     */
    void setGravity(const Eigen::Vector3d& gravity) { gravity_ = gravity; }

private:
    /**
     * @brief Compute state transition Jacobian
     */
    Eigen::Matrix<double, 15, 15> computeF(const Eigen::Vector3d& accel,
                                            const Eigen::Vector3d& gyro) const;
    
    /**
     * @brief Compute process noise Jacobian
     */
    Eigen::Matrix<double, 15, 6> computeG() const;
    
    Config config_;
    EkfState state_;
    Eigen::Matrix<double, 15, 15> P_ = Eigen::Matrix<double, 15, 15>::Zero();
    Eigen::Vector3d gravity_ = Eigen::Vector3d(0, 0, -9.81);
    bool initialized_ = false;
};

} // namespace imu
} // namespace vio_slam

#endif // VIO_SLAM_EKF_PROPAGATOR_HPP
