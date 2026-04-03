#ifndef VIO_SLAM_CAMERA_MODEL_HPP
#define VIO_SLAM_CAMERA_MODEL_HPP

#include <Eigen/Core>
#include <memory>

namespace vio_slam {
namespace camera {

/**
 * @brief Abstract base class for all camera models
 * 
 * Supports diverse lens types: pinhole, fisheye, catadioptric
 */
class CameraModel {
public:
    using Ptr = std::shared_ptr<CameraModel>;
    using ConstPtr = std::shared_ptr<const CameraModel>;

    virtual ~CameraModel() = default;

    /**
     * @brief Project 3D point to 2D pixel coordinates
     * @param p3d 3D point in camera frame
     * @return 2D pixel coordinates
     */
    virtual Eigen::Vector2d project(const Eigen::Vector3d& p3d) const = 0;

    /**
     * @brief Unproject 2D pixel to 3D ray (unit vector)
     * @param px 2D pixel coordinates
     * @return Unit vector in camera frame
     */
    virtual Eigen::Vector3d unproject(const Eigen::Vector2d& px) const = 0;

    /**
     * @brief Jacobian of projection w.r.t. 3D point
     * @param p3d 3D point in camera frame
     * @return 2x3 Jacobian matrix
     */
    virtual Eigen::Matrix<double, 2, 3> dproject(const Eigen::Vector3d& p3d) const = 0;

    /**
     * @brief Get camera type name
     */
    virtual std::string getType() const = 0;

    /**
     * @brief Get number of intrinsic parameters
     */
    virtual size_t getNumParams() const = 0;

    /**
     * @brief Get intrinsic parameters as vector
     */
    virtual Eigen::VectorXd getParams() const = 0;

    /**
     * @brief Set intrinsic parameters from vector
     */
    virtual void setParams(const Eigen::VectorXd& params) = 0;
};

} // namespace camera
} // namespace vio_slam

#endif // VIO_SLAM_CAMERA_MODEL_HPP
