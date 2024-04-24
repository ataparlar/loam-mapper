
#include "loam_mapper/points_provider.hpp"
#include "loam_mapper/transform_provider.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_array.hpp>


namespace loam_mapper
{
class LoamMapper : public rclcpp::Node
{
public:
  using SharedPtr = std::shared_ptr<LoamMapper>;
  using ConstSharedPtr = const std::shared_ptr<LoamMapper>;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using Points = points_provider::PointsProviderBase::Points;



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
  bool save_pcd_;

  void process();

  std::vector<points_provider::PointsProvider::Points> clouds;

private:
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_ptr_basic_cloud_current_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_ptr_loam_cloud_current_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_ptr_path_;

  transform_provider::TransformProvider::SharedPtr transform_provider;
  points_provider::PointsProvider::SharedPtr points_provider;

  PointCloud2::SharedPtr points_to_cloud(const Points & points_bad, const std::string & frame_id);

  void callback_cloud_surround_out(const Points & points_surround);

};

}  // namespace loam_mapper
