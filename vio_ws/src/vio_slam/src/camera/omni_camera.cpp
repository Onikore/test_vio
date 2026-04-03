#include "vio_slam/camera/omni_camera.h"

namespace vio_slam {
namespace camera {

OmniCamera::OmniCamera(double xi, double fx, double fy, double cx, double cy,
                       double k1, double k2, double p1, double p2)
    : xi_(xi), fx_(fx), fy_(fy), cx_(cx), cy_(cy),
      k1_(k1), k2_(k2), p1_(p1), p2_(p2) {}

Eigen::Vector2d OmniCamera::project(const Eigen::Vector3d& p3d) const {
  // Unified spherical model projection
  double norm = p3d.norm();
  if (norm < 1e-10) return Eigen::Vector2d(0, 0);
  
  double x = p3d.x() / norm;
  double y = p3d.y() / norm;
  double z = p3d.z() / norm;
  
  // Projection with xi parameter
  double denom = 1.0 + xi_ * (z + 1e-10);
  double u = x / denom;
  double v = y / denom;
  
  // Apply distortion
  double r2 = u*u + v*v;
  double r4 = r2*r2;
  double dr = 1.0 + k1_*r2 + k2_*r4;
  double du = 2.0*p1_*u*v + p2_*(r2 + 2.0*u*u);
  double dv = p1_*(r2 + 2.0*v*v) + 2.0*p2_*u*v;
  
  u = u*dr + du;
  v = v*dr + dv;
  
  // Convert to pixel coordinates
  return Eigen::Vector2d(fx_*u + cx_, fy_*v + cy_);
}

Eigen::Vector3d OmniCamera::unproject(const Eigen::Vector2d& px) const {
  // Normalize to [-1, 1] range approximately
  double u = (px.x() - cx_) / fx_;
  double v = (px.y() - cy_) / fy_;
  
  // Inverse distortion (simplified)
  double r2 = u*u + v*v;
  double r4 = r2*r2;
  double dr = 1.0 + k1_*r2 + k2_*r4;
  double du = 2.0*p1_*u*v + p2_*(r2 + 2.0*u*u);
  double dv = p1_*(r2 + 2.0*v*v) + 2.0*p2_*u*v;
  
  u = (u - du) / (dr + 1e-10);
  v = (v - dv) / (dr + 1e-10);
  
  // Back-project to sphere
  double denom = 1.0 + xi_*xi_ - (1.0 + 2.0*xi_)*u*u - (1.0 + 2.0*xi_)*v*v;
  if (denom < 0) denom = 0;
  double w = (xi_ + std::sqrt(denom)) / (1.0 + 2.0*xi_ + 1e-10);
  
  double norm = std::sqrt(u*u + v*v + w*w);
  return Eigen::Vector3d(u/norm, v/norm, w/norm);
}

Eigen::Matrix<double,2,3> OmniCamera::dproject(const Eigen::Vector3d& p3d) const {
  // Numerical Jacobian approximation
  Eigen::Matrix<double,2,3> J;
  double eps = 1e-8;
  
  for (int i = 0; i < 3; ++i) {
    Eigen::Vector3d p_plus = p3d;
    Eigen::Vector3d p_minus = p3d;
    p_plus(i) += eps;
    p_minus(i) -= eps;
    
    J.col(i) = (project(p_plus) - project(p_minus)) / (2.0*eps);
  }
  
  return J;
}

}  // namespace camera
}  // namespace vio_slam
