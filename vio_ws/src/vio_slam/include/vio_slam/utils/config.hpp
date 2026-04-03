#ifndef VIO_SLAM_CONFIG_HPP
#define VIO_SLAM_CONFIG_HPP

#include <string>
#include <unordered_map>
#include <variant>
#include <fstream>
#include <sstream>

namespace vio_slam {
namespace utils {

/**
 * @brief Simple YAML-like configuration loader
 */
class Config {
public:
    using Value = std::variant<int, double, std::string, bool>;
    
    /**
     * @brief Load configuration from YAML file
     */
    static Config loadFromFile(const std::string& path);
    
    /**
     * @brief Get value by key with type conversion
     */
    template<typename T>
    T get(const std::string& key, const T& default_value) const {
        auto it = values_.find(key);
        if (it == values_.end()) {
            return default_value;
        }
        try {
            return std::get<T>(it->second);
        } catch (...) {
            return default_value;
        }
    }
    
    /**
     * @brief Set value by key
     */
    template<typename T>
    void set(const std::string& key, const T& value) {
        values_[key] = value;
    }
    
    /**
     * @brief Check if key exists
     */
    bool hasKey(const std::string& key) const {
        return values_.find(key) != values_.end();
    }

private:
    std::unordered_map<std::string, Value> values_;
};

} // namespace utils
} // namespace vio_slam

#endif // VIO_SLAM_CONFIG_HPP
