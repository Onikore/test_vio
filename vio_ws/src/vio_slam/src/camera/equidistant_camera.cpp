#include "vio_slam/camera/equidistant_camera.hpp"
#include <cmath>

namespace vio_slam {
namespace camera {

EquidistantCamera::EquidistantCamera(double fx, double fy, double cx, double cy,
                                      double k1, double k2, double k3, double k4)
    : params_(fx, fy, cx, cy, k1, k2, k3, k4) {}

EquidistantCamera::EquidistantCamera(const Eigen::Matrix<double, 8, 1>& params)
    : params_(params) {}

Eigen::Vector2d EquidistantCamera::project(const Eigen::Vector3d& p3d) const {
    if (p3d.z() <= 0.0) {
        return Eigen::Vector2d(NAN, NAN);
    }
    
    // Normalize coordinates
    const double x = p3d.x() / p3d.z();
    const double y = p3d.y() / p3d.z();
    
    // Angle from optical axis
    const double r = std::sqrt(x * x + y * y);
    
    // Kannala-Brandt distortion model
    // theta_d = k1*theta + k2*theta^3 + k3*theta^5 + k4*theta^7
    const double k1 = params_(4);
    const double k2 = params_(5);
    const double k3 = params_(6);
    const double k4 = params_(7);
    
    double theta_d;
    if (r > 1e-10) {
        const double theta = std::atan(r);
        const double theta2 = theta * theta;
        const double theta3 = theta2 * theta;
        const double theta5 = theta3 * theta2;
        const double theta7 = theta5 * theta2;
        theta_d = k1 * theta + k2 * theta3 + k3 * theta5 + k4 * theta7;
    } else {
        theta_d = 0.0;
    }
    
    // Apply distortion
    double xd, yd;
    if (r > 1e-10) {
        const double scale = theta_d / r;
        xd = x * scale;
        yd = y * scale;
    } else {
        xd = 0.0;
        yd = 0.0;
    }
    
    // Apply intrinsics
    return Eigen::Vector2d(
        params_(0) * xd + params_(2),
        params_(1) * yd + params_(3)
    );
}

Eigen::Vector3d EquidistantCamera::unproject(const Eigen::Vector2d& px) const {
    // Remove intrinsics
    double xd = (px.x() - params_(2)) / params_(0);
    double yd = (px.y() - params_(3)) / params_(1);
    
    const double k1 = params_(4);
    const double k2 = params_(5);
    const double k3 = params_(6);
    const double k4 = params_(7);
    
    // Iterative inverse distortion
    double r = std::sqrt(xd * xd + yd * yd);
    
    if (r < 1e-10) {
        return Eigen::Vector3d(0.0, 0.0, 1.0);
    }
    
    // Newton-Raphson iteration to find undistorted angle
    double theta = r;  // Initial guess
    for (int i = 0; i < 10; ++i) {
        const double theta2 = theta * theta;
        const double theta3 = theta2 * theta;
        const double theta5 = theta3 * theta2;
        const double theta7 = theta5 * theta2;
        
        const double f = k1 * theta + k2 * theta3 + k3 * theta5 + k4 * theta7 - r;
        const double df = k1 + 3.0 * k2 * theta2 + 5.0 * k3 * theta2 * theta2 + 
                          7.0 * k4 * theta2 * theta2 * theta2;
        
        if (std::abs(df) < 1e-10) break;
        
        const double dtheta = f / df;
        theta -= dtheta;
        
        if (std::abs(dtheta) < 1e-10) break;
    }
    
    // Compute undistorted normalized coordinates
    const double scale = std::tan(theta) / r;
    const double x = xd * scale;
    const double y = yd * scale;
    
    // Return unit vector
    const double norm = std::sqrt(x * x + y * y + 1.0);
    return Eigen::Vector3d(x / norm, y / norm, 1.0 / norm);
}

Eigen::Matrix<double, 2, 3> EquidistantCamera::dproject(const Eigen::Vector3d& p3d) const {
    Eigen::Matrix<double, 2, 3> J;
    const double z = p3d.z();
    const double x = p3d.x() / z;
    const double y = p3d.y() / z;
    const double r = std::sqrt(x * x + y * y);
    
    const double k1 = params_(4);
    const double k2 = params_(5);
    const double k3 = params_(6);
    const double k4 = params_(7);
    
    const double theta = std::atan(r);
    const double theta2 = theta * theta;
    const double dtheta_dr = 1.0 / (1.0 + r * r);
    
    // Derivative of distorted angle w.r.t. theta
    const double d_theta_d_dtheta = k1 + 3.0 * k2 * theta2 + 5.0 * k3 * theta2 * theta2 + 
                                     7.0 * k4 * theta2 * theta2 * theta2;
    
    // Derivative of scale factor
    double d_scale_dr;
    double scale;
    if (r > 1e-10) {
        const double theta_d = k1 * theta + k2 * theta * theta2 + k3 * theta * theta2 * theta2 + 
                               k4 * theta * theta2 * theta2 * theta2;
        scale = theta_d / r;
        d_scale_dr = (d_theta_d_dtheta * dtheta_dr * r - theta_d) / (r * r);
    } else {
        scale = k1;
        d_scale_dr = 0.0;
    }
    
    const double inv_z = 1.0 / z;
    const double dr_dx = x / (r * z);
    const double dr_dy = y / (r * z);
    
    // Distorted coordinates
    const double theta_d = k1 * theta + k2 * theta * theta2 + k3 * theta * theta2 * theta2 + 
                           k4 * theta * theta2 * theta2 * theta2;
    const double xd = x * theta_d / r;
    const double yd = y * theta_d / r;
    
    // Jacobian of distorted coordinates
    double d_xd_dx, d_xd_dy, d_yd_dx, d_yd_dy;
    if (r > 1e-10) {
        d_xd_dx = scale * inv_z + xd * d_scale_dr * dr_dx;
        d_xd_dy = xd * d_scale_dr * dr_dy;
        d_yd_dx = yd * d_scale_dr * dr_dx;
        d_yd_dy = scale * inv_z + yd * d_scale_dr * dr_dy;
    } else {
        d_xd_dx = k1 * inv_z;
        d_xd_dy = 0.0;
        d_yd_dx = 0.0;
        d_yd_dy = k1 * inv_z;
    }
    
    // Apply intrinsics
    J(0, 0) = params_(0) * d_xd_dx;
    J(0, 1) = params_(0) * d_xd_dy;
    J(0, 2) = -params_(0) * (xd + x * d_scale_dr * r) / z;
    
    J(1, 0) = params_(1) * d_yd_dx;
    J(1, 1) = params_(1) * d_yd_dy;
    J(1, 2) = -params_(1) * (yd + y * d_scale_dr * r) / z;
    
    return J;
}

Eigen::VectorXd EquidistantCamera::getParams() const {
    return params_;
}

void EquidistantCamera::setParams(const Eigen::VectorXd& params) {
    if (params.size() != 8) {
        throw std::runtime_error("EquidistantCamera requires 8 parameters");
    }
    params_ = params;
}

} // namespace camera
} // namespace vio_slam
