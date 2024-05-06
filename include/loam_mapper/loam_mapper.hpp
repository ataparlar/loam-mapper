
#include "loam_mapper/points_provider.hpp"
#include "loam_mapper/transform_provider.hpp"
#include "loam_mapper/image_projection.hpp"
#include "loam_mapper/feature_extraction.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>


namespace loam_mapper
{
class LoamMapper : public rclcpp::Node
{
public:
  using SharedPtr = std::shared_ptr<LoamMapper>;
  using ConstSharedPtr = const std::shared_ptr<LoamMapper>;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using Points = points_provider::PointsProviderBase::Points;
  using Point = points_provider::PointsProviderBase::Point;



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
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_ptr_corner_cloud_current_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_ptr_corner_instant_cloud_current_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_ptr_surface_cloud_current_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_ptr_surface_instant_cloud_current_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_ptr_ext_cloud_current_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_ptr_occ_not_cloud_current_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_ptr_occ_cloud_current_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_ptr_path_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_ptr_cloud_path_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_ptr_image_;

  transform_provider::TransformProvider::SharedPtr transform_provider;
  points_provider::PointsProvider::SharedPtr points_provider;
  image_projection::ImageProjection::SharedPtr image_projection;
  feature_extraction::FeatureExtraction::SharedPtr feature_extraction;

  Eigen::Affine3f transStartInverse;
  
  PointCloud2::SharedPtr points_to_cloud(const Points & points_bad, const std::string & frame_id);

  void callback_cloud_surround_out(const Points & points_surround);
  sensor_msgs::msg::Image createImageFromRangeMat(const cv::Mat & rangeMat, int counter);
  void clear_cloudInfo(utils::Utils::CloudInfo & cloudInfo);
  Points transform_points(Points & cloud);

  struct by_ring_and_angle
  {
    inline bool operator() (const Point& point1, const Point& point2)
    {
      if (point1.ring == point2.ring){
        return (point1.horizontal_angle < point2.horizontal_angle);
      } else {
        return (point1.ring < point2.ring);
      }
    }
  };
  struct by_angle
  {
    inline bool operator() (const Point& point1, const Point& point2)
    {
      return (point1.horizontal_angle < point2.horizontal_angle);
    }
  };

};

}  // namespace loam_mapper
