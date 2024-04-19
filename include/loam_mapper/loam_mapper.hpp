#include "loam_mapper/date.h"
#include "loam_mapper/points_provider.hpp"
#include "loam_mapper/transform_provider.hpp"
#include "rclcpp/rclcpp.hpp"

#include <boost/filesystem.hpp>
#include <boost/math/special_functions/relative_difference.hpp>

#include <pcapplusplus/RawPacket.h>

#include <cstdint>
#include <cstdlib>
#include <limits>
#include <map>

namespace loam_mapper
{
class LoamMapper : public rclcpp::Node
{
public:
  explicit LoamMapper();

  // Params
  std::string pcap_dir_path_;
  std::string pose_txt_path_;
  std::string pcd_export_dir_;

  double map_origin_x_;
  double map_origin_y_;
  double map_origin_z_;

  double imu2lidar_roll_;
  double imu2lidar_pitch_;
  double imu2lidar_yaw_;

  bool enable_ned2enu_;

  double voxel_resolution_;

  TransformProvider::SharedPtr transform_provider;
  PointsProvider::SharedPtr points_provider;

private:
};
}  // namespace loam_mapper