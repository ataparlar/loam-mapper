
#ifndef BUILD_TRANSFORM_PROVIDER_HPP
#define BUILD_TRANSFORM_PROVIDER_HPP

#include "csv.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/MGRS.hpp>

#include <boost/filesystem.hpp>

#include <memory>
#include <string>

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

  bool origin_init = false;
  GeographicLib::LocalCartesian local_cartesian_;

  struct Velocity
  {
    double x{0U};
    double y{0U};
    double z{0U};
  };
  struct Pose
  {
    uint32_t stamp_unix_seconds{0U};
    uint32_t stamp_nanoseconds{0U};
    Velocity velocity;
    geometry_msgs::msg::PoseWithCovariance pose_with_covariance;
  };
  struct Imu
  {
    uint32_t stamp_unix_seconds{0U};
    uint32_t stamp_nanoseconds{0U};
    sensor_msgs::msg::Imu imu;
  };

  std::vector<Pose> poses_;
  std::vector<Imu> imu_rotations_;

  Pose get_pose_at(uint32_t stamp_unix_seconds, uint32_t stamp_nanoseconds);

  Imu get_imu_at(uint32_t stamp_unix_seconds, uint32_t stamp_nanoseconds);

  std::vector<double> parse_mgrs_coordinates(const std::string & mgrs_string);
  std::string parse_mgrs_zone(const std::string & mgrs_string);

private:
  fs::path path_file_ascii_output_;
  std::string header_line_string;
  std::string time_string;
  int data_line_number;
  std::string mission_date;

  double x, y, z,
    origin_x_, origin_y_, origin_z_,
    x_current, y_current, z_current;
  int zone; bool northp; int prec=8;

};
}  // namespace loam_mapper::transform_provider

#endif  // BUILD_TRANSFORM_PROVIDER_HPP
