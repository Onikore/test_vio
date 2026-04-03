#ifndef VIO_SLAM_STEREO_RIG_HPP
#define VIO_SLAM_STEREO_RIG_HPP

#include "vio_slam/camera/camera_model.hpp"
#include <Eigen/Geometry>

namespace vio_slam {
namespace camera {

/**
 * @brief Stereo rig holding two cameras with known extrinsic transform
 * 
 * Supports asymmetric stereo configurations (different intrinsics per camera)
 */
class StereoRig {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    using Ptr = std::shared_ptr<StereoRig>;
    using ConstPtr = std::shared_ptr<const StereoRig>;

    StereoRig() = default;

    /**
     * @brief Construct from two camera models and relative transform
     * @param left Left camera model
     * @param right Right camera model
     * @param T_right_left Transform from left to right camera frame
     */
    StereoRig(CameraModel::Ptr left, CameraModel::Ptr right,
              const Eigen::Isometry3d& T_right_left = Eigen::Isometry3d::Identity());

    /**
     * @brief Get left camera
     */
    const CameraModel::Ptr& leftCamera() const { return left_; }
    CameraModel::Ptr& leftCamera() { return left_; }

    /**
     * @brief Get right camera
     */
    const CameraModel::Ptr& rightCamera() const { return right_; }
    CameraModel::Ptr& rightCamera() { return right_; }

    /**
     * @brief Get transform from left to right camera
     */
    const Eigen::Isometry3d& T_right_left() const { return T_right_left_; }

    /**
     * @brief Get transform from right to left camera
     */
    Eigen::Isometry3d T_left_right() const { return T_right_left_.inverse(); }

    /**
     * @brief Get baseline distance (norm of translation component)
     */
    double baseline() const;

    /**
     * @brief Project 3D point to both cameras
     * @param p3d_world 3D point in world frame
     * @param T_world_cam Transform from camera to world
     * @return Pair of pixel coordinates (left, right)
     */
    std::pair<Eigen::Vector2d, Eigen::Vector2d> 
    projectStereo(const Eigen::Vector3d& p3d_world, 
                  const Eigen::Isometry3d& T_world_cam) const;

    /**
     * @brief Triangulate 3D point from stereo correspondence
     * @param px_left Pixel in left image
     * @param px_right Pixel in right image
     * @param T_world_cam Transform from camera to world
     * @return Triangulated 3D point in world frame
     */
    Eigen::Vector3d triangulate(const Eigen::Vector2d& px_left,
                                const Eigen::Vector2d& px_right,
                                const Eigen::Isometry3d& T_world_cam) const;

private:
    CameraModel::Ptr left_;
    CameraModel::Ptr right_;
    Eigen::Isometry3d T_right_left_ = Eigen::Isometry3d::Identity();
};

} // namespace camera
} // namespace vio_slam

#endif // VIO_SLAM_STEREO_RIG_HPP
