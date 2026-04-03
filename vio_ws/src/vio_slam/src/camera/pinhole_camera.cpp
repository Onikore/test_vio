#include "vio_slam/camera/pinhole_camera.hpp"

namespace vio_slam {
namespace camera {

PinholeCamera::PinholeCamera(double fx, double fy, double cx, double cy)
    : params_(fx, fy, cx, cy) {}

PinholeCamera::PinholeCamera(const Eigen::Vector4d& params)
    : params_(params) {}

Eigen::Vector2d PinholeCamera::project(const Eigen::Vector3d& p3d) const {
    // Check for points behind camera
    if (p3d.z() <= 0.0) {
        return Eigen::Vector2d(NAN, NAN);
    }
    
    // Standard pinhole projection
    const double inv_z = 1.0 / p3d.z();
    return Eigen::Vector2d(
        params_(0) * p3d.x() * inv_z + params_(2),  // fx * x/z + cx
        params_(1) * p3d.y() * inv_z + params_(3)   // fy * y/z + cy
    );
}

Eigen::Vector3d PinholeCamera::unproject(const Eigen::Vector2d& px) const {
    // Inverse pinhole projection
    const double x = (px.x() - params_(2)) / params_(0);
    const double y = (px.y() - params_(3)) / params_(1);
    
    // Return unit vector (z=1 normalized)
    const double norm = std::sqrt(x * x + y * y + 1.0);
    return Eigen::Vector3d(x / norm, y / norm, 1.0 / norm);
}

Eigen::Matrix<double, 2, 3> PinholeCamera::dproject(const Eigen::Vector3d& p3d) const {
    Eigen::Matrix<double, 2, 3> J;
    const double z = p3d.z();
    const double z2 = z * z;
    const double inv_z = 1.0 / z;
    
    // Derivatives of pinhole projection
    J << params_(0) * inv_z, 0.0, -params_(0) * p3d.x() / z2,
         0.0, params_(1) * inv_z, -params_(1) * p3d.y() / z2;
    
    return J;
}

Eigen::VectorXd PinholeCamera::getParams() const {
    return params_;
}

void PinholeCamera::setParams(const Eigen::VectorXd& params) {
    if (params.size() != 4) {
        throw std::runtime_error("PinholeCamera requires 4 parameters");
    }
    params_ = params;
}

} // namespace camera
} // namespace vio_slam
