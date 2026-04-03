#ifndef VIO_SLAM_IMU_PREINTEGRATION_HPP
#define VIO_SLAM_IMU_PREINTEGRATION_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <memory>

namespace vio_slam {
namespace imu {

/**
 * @brief IMU measurement
 */
struct ImuMeasurement {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    double timestamp;                    // Timestamp in seconds
    Eigen::Vector3d accel;               // Accelerometer measurement (m/s^2)
    Eigen::Vector3d gyro;                // Gyroscope measurement (rad/s)
};

/**
 * @brief Preintegrated IMU measurements between two keyframes
 * 
 * Implements on-manifold preintegration (Forster et al., 2017)
 */
struct PreintegratedImu {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    // Preintegrated measurements
    Eigen::Vector3d delta_p;             // Delta position
    Eigen::Vector3d delta_v;             // Delta velocity
    Eigen::Quaterniond delta_q;          // Delta rotation
    
    // Covariance of preintegrated measurements (9x9: position, velocity, orientation)
    Eigen::Matrix<double, 9, 9> covariance;
    
    // Jacobians w.r.t. biases (for first-order correction)
    Eigen::Matrix<double, 3, 6> J_dp_dbias;   // Position jacobian w.r.t. [ba, bg]
    Eigen::Matrix<double, 3, 6> J_dv_dbias;   // Velocity jacobian w.r.t. [ba, bg]
    Eigen::Matrix<double, 3, 6> J_dq_dbias;   // Orientation jacobian w.r.t. [ba, bg]
    
    // Timing
    double dt_total = 0.0;               // Total integration time
    int num_measurements = 0;            // Number of IMU measurements
    
    /**
     * @brief Reset to identity
     */
    void reset();
    
    /**
     * @brief Get error residual for optimization
     * @param Ti_pose Pose at start of integration
     * @param Tj_pose Pose at end of integration
     * @param vi Velocity at start
     * @param vj Velocity at end
     * @param gravity Gravity vector
     * @return 9D residual vector [pos_err, vel_err, orient_err]
     */
    Eigen::Matrix<double, 9, 1> computeResidual(
        const Eigen::Isometry3d& Ti_pose,
        const Eigen::Isometry3d& Tj_pose,
        const Eigen::Vector3d& vi,
        const Eigen::Vector3d& vj,
        const Eigen::Vector3d& gravity) const;
    
    /**
     * @brief Apply first-order bias correction
     * @param dba Change in accelerometer bias
     * @param dbg Change in gyroscope bias
     */
    void applyBiasCorrection(const Eigen::Vector3d& dba, const Eigen::Vector3d& dbg);
};

/**
 * @brief IMU preintegration integrator
 */
class ImuPreintegration {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    using Ptr = std::shared_ptr<ImuPreintegration>;
    
    struct Config {
        // Noise parameters (standard deviations)
        double gyro_noise_std = 0.01;         // Gyroscope noise (rad/s/sqrt(Hz))
        double accel_noise_std = 0.1;         // Accelerometer noise (m/s^2/sqrt(Hz))
        double gyro_bias_random_walk = 1e-5;  // Gyro bias random walk
        double accel_bias_random_walk = 1e-4; // Accel bias random walk
        
        // Integration
        double max_dt = 0.05;                 // Maximum time between IMU measurements
        
        // Gravity
        Eigen::Vector3d gravity = Eigen::Vector3d(0, 0, -9.81);
    };
    
    explicit ImuPreintegration(const Config& config = Config());
    
    /**
     * @brief Integrate IMU measurements between two timestamps
     * @param measurements IMU measurements (sorted by time)
     * @param ba Initial accelerometer bias estimate
     * @param bg Initial gyroscope bias estimate
     * @return Preintegrated measurement
     */
    PreintegratedImu integrate(const std::vector<ImuMeasurement>& measurements,
                               const Eigen::Vector3d& ba = Eigen::Vector3d::Zero(),
                               const Eigen::Vector3d& bg = Eigen::Vector3d::Zero()) const;
    
    /**
     * @brief Set gravity direction and magnitude
     */
    void setGravity(const Eigen::Vector3d& gravity) { config_.gravity = gravity; }
    
    /**
     * @brief Get gravity
     */
    const Eigen::Vector3d& getGravity() const { return config_.gravity; }

private:
    /**
     * @brief Single-step integration with midpoint method
     */
    void integrateStep(PreintegratedImu& pim,
                       const Eigen::Vector3d& accel,
                       const Eigen::Vector3d& gyro,
                       double dt,
                       const Eigen::Vector3d& ba,
                       const Eigen::Vector3d& bg) const;
    
    Config config_;
};

} // namespace imu
} // namespace vio_slam

#endif // VIO_SLAM_IMU_PREINTEGRATION_HPP
