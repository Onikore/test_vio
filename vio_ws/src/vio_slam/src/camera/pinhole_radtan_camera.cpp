#include "vio_slam/camera/pinhole_radtan_camera.hpp"

namespace vio_slam {
namespace camera {

PinholeRadtanCamera::PinholeRadtanCamera(double fx, double fy, double cx, double cy,
                                          double k1, double k2, double p1, double p2)
    : params_(fx, fy, cx, cy, k1, k2, p1, p2) {}

PinholeRadtanCamera::PinholeRadtanCamera(const Eigen::Matrix<double, 8, 1>& params)
    : params_(params) {}

Eigen::Vector2d PinholeRadtanCamera::project(const Eigen::Vector3d& p3d) const {
    if (p3d.z() <= 0.0) {
        return Eigen::Vector2d(NAN, NAN);
    }
    
    // Normalize coordinates
    const double x = p3d.x() / p3d.z();
    const double y = p3d.y() / p3d.z();
    const double r2 = x * x + y * y;
    const double r4 = r2 * r2;
    
    // Radial distortion
    const double k1 = params_(4);
    const double k2 = params_(5);
    const double radial = 1.0 + k1 * r2 + k2 * r4;
    
    // Tangential distortion
    const double p1 = params_(6);
    const double p2 = params_(7);
    const double tx = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
    const double ty = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
    
    // Apply distortion
    const double xd = x * radial + tx;
    const double yd = y * radial + ty;
    
    // Apply intrinsics
    return Eigen::Vector2d(
        params_(0) * xd + params_(2),  // fx * xd + cx
        params_(1) * yd + params_(3)   // fy * yd + cy
    );
}

Eigen::Vector3d PinholeRadtanCamera::unproject(const Eigen::Vector2d& px) const {
    // Remove intrinsics to get distorted normalized coordinates
    double x = (px.x() - params_(2)) / params_(0);
    double y = (px.y() - params_(3)) / params_(1);
    
    // Iterative undistortion (Newton-Raphson)
    const double k1 = params_(4);
    const double k2 = params_(5);
    const double p1 = params_(6);
    const double p2 = params_(7);
    
    for (int i = 0; i < 10; ++i) {
        const double r2 = x * x + y * y;
        const double r4 = r2 * r2;
        const double radial = 1.0 + k1 * r2 + k2 * r4;
        
        const double tx = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
        const double ty = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
        
        const double x_dist = x * radial + tx;
        const double y_dist = y * radial + ty;
        
        const double dx = x_dist - (px.x() - params_(2)) / params_(0);
        const double dy = y_dist - (px.y() - params_(3)) / params_(1);
        
        // Simple gradient descent step
        x -= 0.5 * dx;
        y -= 0.5 * dy;
        
        if (std::abs(dx) < 1e-6 && std::abs(dy) < 1e-6) {
            break;
        }
    }
    
    // Return unit vector
    const double norm = std::sqrt(x * x + y * y + 1.0);
    return Eigen::Vector3d(x / norm, y / norm, 1.0 / norm);
}

Eigen::Matrix<double, 2, 3> PinholeRadtanCamera::dproject(const Eigen::Vector3d& p3d) const {
    Eigen::Matrix<double, 2, 3> J;
    const double z = p3d.z();
    const double x = p3d.x() / z;
    const double y = p3d.y() / z;
    const double r2 = x * x + y * y;
    const double r4 = r2 * r2;
    
    const double k1 = params_(4);
    const double k2 = params_(5);
    const double p1 = params_(6);
    const double p2 = params_(7);
    
    // Derivatives of distorted coordinates w.r.t. normalized coordinates
    const double d_radial_dx = 2.0 * k1 * x + 4.0 * k2 * r2 * x;
    const double d_radial_dy = 2.0 * k1 * y + 4.0 * k2 * r2 * y;
    
    const double d_tx_dx = 2.0 * p1 * y + p2 * (2.0 * x + 4.0 * x);
    const double d_tx_dy = 2.0 * p1 * x + p2 * (2.0 * y);
    const double d_ty_dx = p1 * (2.0 * x) + 2.0 * p2 * (y + 2.0 * y);
    const double d_ty_dy = p1 * (2.0 * y + 4.0 * y) + 2.0 * p2 * x;
    
    // Full Jacobian chain rule
    const double inv_z = 1.0 / z;
    J(0, 0) = params_(0) * ((1.0 + k1 * r2 + k2 * r4) * inv_z + x * d_radial_dx * inv_z + d_tx_dx * inv_z);
    J(0, 1) = params_(0) * (x * d_radial_dy * inv_z + d_tx_dy * inv_z);
    J(0, 2) = -params_(0) * (x * (1.0 + k1 * r2 + k2 * r4) + x * x * d_radial_dx + x * d_tx_dx) / z;
    
    J(1, 0) = params_(1) * (y * d_radial_dx * inv_z + d_ty_dx * inv_z);
    J(1, 1) = params_(1) * ((1.0 + k1 * r2 + k2 * r4) * inv_z + y * d_radial_dy * inv_z + d_ty_dy * inv_z);
    J(1, 2) = -params_(1) * (y * (1.0 + k1 * r2 + k2 * r4) + y * y * d_radial_dy + y * d_ty_dy) / z;
    
    return J;
}

Eigen::VectorXd PinholeRadtanCamera::getParams() const {
    return params_;
}

void PinholeRadtanCamera::setParams(const Eigen::VectorXd& params) {
    if (params.size() != 8) {
        throw std::runtime_error("PinholeRadtanCamera requires 8 parameters");
    }
    params_ = params;
}

} // namespace camera
} // namespace vio_slam
