#ifndef VIO_SLAM_PINHOLE_CAMERA_HPP
#define VIO_SLAM_PINHOLE_CAMERA_HPP

#include "vio_slam/camera/camera_model.hpp"

namespace vio_slam {
namespace camera {

/**
 * @brief Standard pinhole camera model (4 parameters)
 * 
 * Parameters: [fx, fy, cx, cy]
 */
class PinholeCamera : public CameraModel {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PinholeCamera() = default;
    
    /**
     * @brief Construct from individual parameters
     */
    PinholeCamera(double fx, double fy, double cx, double cy);

    /**
     * @brief Construct from parameter vector
     */
    explicit PinholeCamera(const Eigen::Vector4d& params);

    // CameraModel interface
    Eigen::Vector2d project(const Eigen::Vector3d& p3d) const override;
    Eigen::Vector3d unproject(const Eigen::Vector2d& px) const override;
    Eigen::Matrix<double, 2, 3> dproject(const Eigen::Vector3d& p3d) const override;
    
    std::string getType() const override { return "pinhole"; }
    size_t getNumParams() const override { return 4; }
    Eigen::VectorXd getParams() const override;
    void setParams(const Eigen::VectorXd& params) override;

    // Accessors
    double fx() const { return params_(0); }
    double fy() const { return params_(1); }
    double cx() const { return params_(2); }
    double cy() const { return params_(3); }

protected:
    Eigen::Vector4d params_;
};

} // namespace camera
} // namespace vio_slam

#endif // VIO_SLAM_PINHOLE_CAMERA_HPP
