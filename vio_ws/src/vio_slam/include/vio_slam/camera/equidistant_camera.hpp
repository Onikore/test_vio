#ifndef VIO_SLAM_EQUIDISTANT_CAMERA_HPP
#define VIO_SLAM_EQUIDISTANT_CAMERA_HPP

#include "vio_slam/camera/camera_model.hpp"

namespace vio_slam {
namespace camera {

/**
 * @brief Kannala-Brandt equidistant fisheye camera model (8 parameters)
 * 
 * Parameters: [fx, fy, cx, cy, k1, k2, k3, k4]
 * Uses the Kannala-Brandt projection model for wide-angle/fisheye lenses
 */
class EquidistantCamera : public CameraModel {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EquidistantCamera() = default;
    
    /**
     * @brief Construct from individual parameters
     */
    EquidistantCamera(double fx, double fy, double cx, double cy,
                      double k1, double k2, double k3, double k4);

    /**
     * @brief Construct from parameter vector
     */
    explicit EquidistantCamera(const Eigen::Matrix<double, 8, 1>& params);

    // CameraModel interface
    Eigen::Vector2d project(const Eigen::Vector3d& p3d) const override;
    Eigen::Vector3d unproject(const Eigen::Vector2d& px) const override;
    Eigen::Matrix<double, 2, 3> dproject(const Eigen::Vector3d& p3d) const override;
    
    std::string getType() const override { return "equidistant"; }
    size_t getNumParams() const override { return 8; }
    Eigen::VectorXd getParams() const override;
    void setParams(const Eigen::VectorXd& params) override;

    // Accessors
    double fx() const { return params_(0); }
    double fy() const { return params_(1); }
    double cx() const { return params_(2); }
    double cy() const { return params_(3); }
    double k1() const { return params_(4); }
    double k2() const { return params_(5); }
    double k3() const { return params_(6); }
    double k4() const { return params_(7); }

protected:
    Eigen::Matrix<double, 8, 1> params_;
};

} // namespace camera
} // namespace vio_slam

#endif // VIO_SLAM_EQUIDISTANT_CAMERA_HPP
