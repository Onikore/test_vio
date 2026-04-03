#include "vio_slam/utils/config.hpp"

namespace vio_slam {
namespace utils {

Config::Config() {}

bool Config::loadFromFile(const std::string& filename) {
  // Placeholder - actual implementation would parse YAML
  return true;
}

template<typename T>
T Config::get(const std::string& key, const T& default_value) const {
  return default_value;
}

// Explicit template instantiations
template int Config::get<int>(const std::string&, const int&) const;
template double Config::get<double>(const std::string&, const double&) const;
template std::string Config::get<std::string>(const std::string&, const std::string&) const;

}  // namespace utils
}  // namespace vio_slam
