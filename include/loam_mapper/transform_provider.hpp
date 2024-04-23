
#ifndef BUILD_TRANSFORM_PROVIDER_HPP
#define BUILD_TRANSFORM_PROVIDER_HPP

#include "csv.hpp"

#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <boost/filesystem.hpp>
#include <string>
#include <memory>

namespace loam_mapper::transform_provider
{
namespace fs = boost::filesystem;
class TransformProvider
{
public:
  using SharedPtr = std::shared_ptr<TransformProvider>;
  using ConstSharedPtr = const SharedPtr;

  explicit TransformProvider(const std::string & path_file_ascii_output);

  void process(double origin_x, double origin_y, double origin_z);

  struct Pose
  {
    uint32_t stamp_unix_seconds{0U};
    uint32_t stamp_nanoseconds{0U};
    geometry_msgs::msg::PoseWithCovariance pose_with_covariance;
  };

  std::vector<Pose> poses_;

  Pose get_pose_at(
    uint32_t stamp_unix_seconds,
    uint32_t stamp_nanoseconds);

private:
  fs::path path_file_ascii_output_;
  std::string header_line_string;
  std::string time_string;
  int data_line_number;
  std::string mission_date;
};
}  // loam_mapper::transform_provider

#endif  // BUILD_TRANSFORM_PROVIDER_HPP
