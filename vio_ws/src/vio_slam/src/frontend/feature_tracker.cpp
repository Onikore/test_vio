#include "vio_slam/frontend/feature_tracker.h"

namespace vio_slam {
namespace frontend {

FeatureTracker::FeatureTracker(int max_features, int min_features)
    : max_features_(max_features), min_features_(min_features) {}

std::vector<cv::Point2f> FeatureTracker::track(
    const cv::Mat& img_prev,
    const cv::Mat& img_curr,
    const std::vector<cv::Point2f>& pts_prev) {
  
  if (pts_prev.empty()) return {};
  
  std::vector<cv::Point2f> pts_curr;
  std::vector<uchar> status;
  std::vector<float> err;
  
  cv::calcOpticalFlowPyrLK(
      img_prev, img_curr, pts_prev, pts_curr,
      status, err, cv::Size(21, 21), 4);
  
  // Remove outliers
  std::vector<cv::Point2f> tracked;
  for (size_t i = 0; i < pts_curr.size(); ++i) {
    if (status[i] && err[i] < 1.0) {
      tracked.push_back(pts_curr[i]);
    }
  }
  
  return tracked;
}

std::vector<cv::Point2f> FeatureTracker::detect(const cv::Mat& img) {
  std::vector<cv::Point2f> corners;
  cv::goodFeaturesToTrack(img, corners, max_features_, 0.01, 10);
  return corners;
}

}  // namespace frontend
}  // namespace vio_slam
