#ifndef VIO_SLAM_EXPOSURE_CONTROLLER_HPP
#define VIO_SLAM_EXPOSURE_CONTROLLER_HPP

#include <opencv2/core.hpp>
#include <memory>
#include <functional>

namespace vio_slam {
namespace frontend {

/**
 * @brief Active exposure control using PID controller
 * 
 * Adjusts camera exposure to maintain optimal brightness in feature-rich regions
 */
class ExposureController {
public:
    using Ptr = std::shared_ptr<ExposureController>;
    
    struct Config {
        double target_brightness = 120.0;      // Target mean brightness (0-255)
        double hysteresis = 10.0;              // Deadband to prevent oscillation
        
        // PID gains
        double kp = 0.5;                       // Proportional gain
        double ki = 0.01;                      // Integral gain
        double kd = 0.1;                       // Derivative gain
        
        // Constraints
        int min_exposure = 100;                // Minimum exposure (microseconds)
        int max_exposure = 30000;              // Maximum exposure (microseconds)
        int initial_exposure = 5000;           // Initial exposure value
        
        // Gradient weighting
        bool use_gradient_weighting = true;    // Weight by image gradient magnitude
        double gradient_threshold = 5.0;       // Minimum gradient for feature-rich regions
    };
    
    /**
     * @brief Callback type for setting exposure
     * @param exposure_us New exposure value in microseconds
     */
    using ExposureCallback = std::function<void(int exposure_us)>;
    
    explicit ExposureController(const Config& config = Config());
    
    /**
     * @brief Set the callback for applying exposure changes
     */
    void setExposureCallback(ExposureCallback callback);
    
    /**
     * @brief Process frame and update exposure if needed
     * @param image Current grayscale image
     * @return True if exposure was adjusted
     */
    bool processFrame(const cv::Mat& image);
    
    /**
     * @brief Manually set exposure value
     */
    void setExposure(int exposure_us);
    
    /**
     * @brief Get current exposure value
     */
    int getCurrentExposure() const { return current_exposure_; }
    
    /**
     * @brief Reset controller state
     */
    void reset();
    
    /**
     * @brief Enable/disable automatic exposure control
     */
    void setEnabled(bool enabled) { enabled_ = enabled; }
    bool isEnabled() const { return enabled_; }

private:
    /**
     * @brief Compute brightness metric from image
     * @param image Grayscale image
     * @return Mean brightness value (0-255)
     */
    double computeBrightness(const cv::Mat& image) const;
    
    /**
     * @brief Compute gradient-weighted brightness
     * @param image Grayscale image
     * @return Gradient-weighted mean brightness
     */
    double computeGradientWeightedBrightness(const cv::Mat& image) const;
    
    /**
     * @brief PID controller update
     * @param error Brightness error (target - current)
     * @return Exposure adjustment
     */
    int pidUpdate(double error);
    
    Config config_;
    ExposureCallback callback_;
    
    int current_exposure_;
    bool enabled_ = true;
    
    // PID state
    double integral_ = 0.0;
    double prev_error_ = 0.0;
    double prev_brightness_ = 0.0;
};

} // namespace frontend
} // namespace vio_slam

#endif // VIO_SLAM_EXPOSURE_CONTROLLER_HPP
