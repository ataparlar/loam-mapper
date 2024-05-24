
#ifndef BUILD_UTILS_HPP
#define BUILD_UTILS_HPP

#include <cmath>
#include <cstdint>
#include <string>
#include <type_traits>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/imu.hpp"
#include <Eigen/Geometry>

namespace loam_mapper::utils
{

class Utils
{
public:
  static std::string byte_hex_to_string(uint8_t byte_hex);
  static std::string bytes_hexes_to_string(const std::vector<uint8_t> & bytes_hexes);
  static std::vector<std::string> string_to_vec_split_by(const std::string & input, char splitter);

  template <typename T>
  static T deg_to_rad(T deg)
  {
    constexpr double multiplier = M_PI / 180.0;
    return static_cast<T>(deg * multiplier);
  }

  template<typename T>
  static double stamp2Sec(const T& stamp)
  {
    return rclcpp::Time(stamp).seconds();
  }

  static rclcpp::Time get_time()
  {
    return rclcpp::Clock().now();
  }

  static Eigen::Matrix3d ned2enu_converter_for_matrices(const Eigen::Matrix3d & matrix3d);

  struct CloudInfo
  {
    std::vector<float> point_range;
    std::vector<uint32_t> start_ring_index;
    std::vector<uint32_t> point_col_index;
    std::vector<int32_t> end_ring_index;
  };
};

}  // namespace loam_mapper::utils

#endif  // BUILD_UTILS_HPP
