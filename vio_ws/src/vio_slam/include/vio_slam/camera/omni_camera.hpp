#ifndef VIO_SLAM_OMNI_CAMERA_HPP
#define VIO_SLAM_OMNI_CAMERA_HPP

#include "vio_slam/camera/camera_model.hpp"

namespace vio_slam {
namespace camera {

/**
 * @brief Mei unified catadioptric camera model (9 parameters)
 * 
 * Parameters: [fx, fy, cx, cy, xi, k1, k2, p1, p2]
 * - xi: mirror parameter for unified spherical projection
 * - k1, k2: radial distortion
 * - p1, p2: tangential distortion
 * 
 * Suitable for catadioptric (mirror-lens) systems
 */
class OmniCamera : public CameraModel {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    OmniCamera() = default;
    
    /**
     * @brief Construct from individual parameters
     */
    OmniCamera(double fx, double fy, double cx, double cy,
               double xi, double k1, double k2, double p1, double p2);

    /**
     * @brief Construct from parameter vector
     */
    explicit OmniCamera(const Eigen::Matrix<double, 9, 1>& params);

    // CameraModel interface
    Eigen::Vector2d project(const Eigen::Vector3d& p3d) const override;
    Eigen::Vector3d unproject(const Eigen::Vector2d& px) const override;
    Eigen::Matrix<double, 2, 3> dproject(const Eigen::Vector3d& p3d) const override;
    
    std::string getType() const override { return "omni"; }
    size_t getNumParams() const override { return 9; }
    Eigen::VectorXd getParams() const override;
    void setParams(const Eigen::VectorXd& params) override;

    // Accessors
    double fx() const { return params_(0); }
    double fy() const { return params_(1); }
    double cx() const { return params_(2); }
    double cy() const { return params_(3); }
    double xi() const { return params_(4); }
    double k1() const { return params_(5); }
    double k2() const { return params_(6); }
    double p1() const { return params_(7); }
    double p2() const { return params_(8); }

protected:
    Eigen::Matrix<double, 9, 1> params_;
};

} // namespace camera
} // namespace vio_slam

#endif // VIO_SLAM_OMNI_CAMERA_HPP
