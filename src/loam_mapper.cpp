
#include "loam_mapper/loam_mapper.hpp"

#include "loam_mapper/Occtree.h"

#include <Eigen/Geometry>
#include <loam_mapper/point_types.hpp>
#include <loam_mapper/utils.hpp>
#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/path.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cstdint>
#include <execution>
#include <memory>
#include <string>
#include <vector>

namespace
{
const std::uint32_t QOS_HISTORY_DEPTH = 10;
}

namespace loam_mapper
{
LoamMapper::LoamMapper() : Node("loam_mapper")
{
  this->declare_parameter("pcap_dir_path", "");
  this->declare_parameter("pose_txt_path", "");
  this->declare_parameter("pcd_export_directory", "");
  this->declare_parameter("map_origin_x", 0.0);
  this->declare_parameter("map_origin_y", 0.0);
  this->declare_parameter("map_origin_z", 0.0);
  this->declare_parameter("imu2lidar_roll", 0.0);
  this->declare_parameter("imu2lidar_pitch", 0.0);
  this->declare_parameter("imu2lidar_yaw", 0.0);
  this->declare_parameter("enable_ned2enu", true);
  this->declare_parameter("voxel_resolution", 0.4);
  this->declare_parameter("save_pcd", true);

  pcap_dir_path_ = this->get_parameter("pcap_dir_path").as_string();
  pose_txt_path_ = this->get_parameter("pose_txt_path").as_string();
  pcd_export_dir_ = this->get_parameter("pcd_export_directory").as_string();
  map_origin_x_ = this->get_parameter("map_origin_x").as_double();
  map_origin_y_ = this->get_parameter("map_origin_y").as_double();
  map_origin_z_ = this->get_parameter("map_origin_z").as_double();
  imu2lidar_roll_ = this->get_parameter("imu2lidar_roll").as_double();
  imu2lidar_pitch_ = this->get_parameter("imu2lidar_pitch").as_double();
  imu2lidar_yaw_ = this->get_parameter("imu2lidar_yaw").as_double();
  enable_ned2enu_ = this->get_parameter("enable_ned2enu").as_bool();
  voxel_resolution_ = this->get_parameter("voxel_resolution").as_double();
  save_pcd_ = this->get_parameter("save_pcd").as_bool();

  pub_ptr_basic_cloud_current_ = this->create_publisher<PointCloud2>("basic_cloud_current", 10);
  pub_ptr_corner_cloud_current_ = this->create_publisher<PointCloud2>("corner_cloud_current", 10);
  pub_ptr_surface_cloud_current_ = this->create_publisher<PointCloud2>("surface_cloud_current", 10);
  pub_ptr_path_ = this->create_publisher<nav_msgs::msg::Path>("vehicle_path", 10);
  pub_ptr_image_ = this->create_publisher<sensor_msgs::msg::Image>("rangeMat", 10);
  pub_ptr_marker_corner_ =
    this->create_publisher<visualization_msgs::msg::Marker>("marker_array_corner", 10);

  transform_provider = std::make_shared<transform_provider::TransformProvider>(
    "/home/ataparlar/data/task_spesific/loam_based_localization/mapping/pcap_and_poses/"
    "ytu_campus_080423_ground_truth.txt");

  transform_provider->process(map_origin_x_, map_origin_y_, map_origin_z_);

  points_provider = std::make_shared<points_provider::PointsProvider>(std::string(
    "/home/ataparlar/data/task_spesific/loam_based_localization/mapping/pcap_and_poses/pcaps/"));
  points_provider->process();

  image_projection = std::make_shared<image_projection::ImageProjection>();
  feature_extraction = std::make_shared<feature_extraction::FeatureExtraction>();

  std::function<void(const Points &)> callback =
    std::bind(&LoamMapper::callback_cloud_surround_out, this, std::placeholders::_1);
  points_provider->process_pcaps_into_clouds(callback, 0, 2);
  std::cout << "process_pcaps_into_clouds done" << std::endl;

  process();
}

void LoamMapper::process()
{
  auto thing_to_cloud =
    [](const points_provider::PointsProvider::Points & points_bad, const std::string & frame_id) {
      using CloudModifier = point_cloud_msg_wrapper::PointCloud2Modifier<point_types::PointXYZI>;
      PointCloud2::SharedPtr cloud_ptr_current = std::make_shared<PointCloud2>();
      CloudModifier cloud_modifier_current(*cloud_ptr_current, frame_id);
      cloud_modifier_current.resize(points_bad.size());
      std::transform(
        std::execution::par, points_bad.cbegin(), points_bad.cend(), cloud_modifier_current.begin(),
        [](const points_provider::PointsProvider::Point & point_bad) {
          return point_types::PointXYZI{
            point_bad.x, point_bad.y, point_bad.z, static_cast<float>(point_bad.intensity)};
        });
      return cloud_ptr_current;
    };

  nav_msgs::msg::Path path_;
  path_.header.frame_id = "map";
  path_.poses.resize(transform_provider->poses_.size());

  points_provider::PointsProvider::Points cloud_all;
  points_provider::PointsProvider::Points cloud_all_corner_;
  points_provider::PointsProvider::Points cloud_all_surface_;

  for (auto & cloud : clouds) {
    //    utils::Utils::CloudInfo cloudInfo;

    points_provider::PointsProvider::Points cloud_trans;
    cloud_trans.resize(cloud.size());

    std::transform(
      std::execution::par, cloud.cbegin(), cloud.cend(), cloud_trans.begin(),
      [this, &path_](const points_provider::PointsProvider::Point & point) {
        points_provider::PointsProvider::Point point_trans;
        loam_mapper::transform_provider::TransformProvider::Pose pose =
          this->transform_provider->get_pose_at(point.stamp_unix_seconds, point.stamp_nanoseconds);

        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.pose = pose.pose_with_covariance.pose;
        pose_stamped.header.frame_id = path_.header.frame_id;

        const auto & pose_ori = pose.pose_with_covariance.pose.orientation;
        Eigen::Quaterniond quat(pose_ori.w, pose_ori.x, pose_ori.y, pose_ori.z);

        Eigen::Affine3d affine_sensor2map(Eigen::Affine3d::Identity());

        Eigen::Affine3d affine_imu2lidar(Eigen::Affine3d::Identity());
        affine_imu2lidar.matrix().topLeftCorner<3, 3>() =
          Eigen::AngleAxisd(utils::Utils::deg_to_rad(imu2lidar_yaw_), Eigen::Vector3d::UnitZ())
            .toRotationMatrix() *
          Eigen::AngleAxisd(utils::Utils::deg_to_rad(imu2lidar_pitch_), Eigen::Vector3d::UnitY())
            .toRotationMatrix() *
          Eigen::AngleAxisd(utils::Utils::deg_to_rad(imu2lidar_roll_), Eigen::Vector3d::UnitX())
            .toRotationMatrix();

        if (enable_ned2enu_) {
          Eigen::Affine3d ned2enu(Eigen::Affine3d::Identity());
          ned2enu.matrix().topLeftCorner<3, 3>() =
            Eigen::AngleAxisd(utils::Utils::deg_to_rad(-90.0), Eigen::Vector3d::UnitZ())
              .toRotationMatrix() *
            Eigen::AngleAxisd(utils::Utils::deg_to_rad(0.0), Eigen::Vector3d::UnitY())
              .toRotationMatrix() *
            Eigen::AngleAxisd(utils::Utils::deg_to_rad(180.0), Eigen::Vector3d::UnitX())
              .toRotationMatrix();

          Eigen::Affine3d affine_imu2lidar_enu(Eigen::Affine3d::Identity());
          affine_imu2lidar_enu = affine_imu2lidar.matrix() * ned2enu.matrix();

          affine_sensor2map.matrix().topLeftCorner<3, 3>() =
            quat.toRotationMatrix() * affine_imu2lidar_enu.rotation();

        } else {
          affine_sensor2map.matrix().topLeftCorner<3, 3>() =
            quat.toRotationMatrix() * affine_imu2lidar.rotation();
        }

        auto & pose_pos = pose_stamped.pose.position;
        affine_sensor2map.matrix().topRightCorner<3, 1>() << pose_pos.x, pose_pos.y, pose_pos.z;

        Eigen::Vector4d vec_point_in(point.x, point.y, point.z, 1.0);
        Eigen::Vector4d vec_point_trans = affine_sensor2map.matrix() * vec_point_in;

        point_trans.x = static_cast<float>(vec_point_trans(0));
        point_trans.y = static_cast<float>(vec_point_trans(1));
        point_trans.z = static_cast<float>(vec_point_trans(2));
        point_trans.ring = point.ring;
        point_trans.horizontal_angle = point.horizontal_angle;
        point_trans.intensity = point.intensity;

        return point_trans;
      });

    //    image_projection->setLaserCloudIn(cloud_trans);
    image_projection->cloudHandler(cloud_trans);
    //    sensor_msgs::msg::Image image = createImageFromRangeMat(image_projection->rangeMat);

    feature_extraction->laserCloudInfoHandler(cloud_trans, image_projection->cloudInfo);

    auto corner_cloud_ptr_current = thing_to_cloud(feature_extraction->cornerCloud, "map");
    pub_ptr_corner_cloud_current_->publish(*corner_cloud_ptr_current);

    for (auto & point1 : feature_extraction->cornerCloud) {
      for (auto & point2 : feature_extraction->cornerCloud) {
        if (
          std::sqrt(
            std::pow(point1.x - point2.x, 2) + std::pow(point1.y - point2.y, 2) +
            std::pow(point1.z - point2.z, 2)) < 1.0) {
          feature_extraction->marker_corner.header.stamp = utils::Utils::get_time();
          feature_extraction->marker_corner.header.frame_id = "map";
          feature_extraction->marker_corner.color.r = 1.0;
          feature_extraction->marker_corner.color.g = 0.0;
          feature_extraction->marker_corner.color.b = 0.0;
          feature_extraction->marker_corner.color.a = 1.0;
          feature_extraction->marker_corner.action = visualization_msgs::msg::Marker::ADD;
          feature_extraction->marker_corner.ns = "corner_lines";
          feature_extraction->marker_corner.scale.x = 0.02;
          feature_extraction->marker_corner.scale.y = 0.02;
          feature_extraction->marker_corner.type = visualization_msgs::msg::Marker::LINE_LIST;
          feature_extraction->marker_corner.id = feature_extraction->marker_counter;
          geometry_msgs::msg::Point p1;
          p1.x = point1.x;
          p1.y = point1.y;
          p1.z = point1.z;
          feature_extraction->marker_corner.points.push_back(p1);
          geometry_msgs::msg::Point p2;
          p2.x = point2.x;
          p2.y = point2.y;
          p2.z = point2.z;
          feature_extraction->marker_corner.points.push_back(p2);
          feature_extraction->marker_counter++;
        }
      }
    }

    if (feature_extraction->marker_corner.points.size() % 2 == 1) {
      feature_extraction->marker_corner.points.pop_back();
    }
    //    feature_extraction->markerArray_corner.markers.push_back(feature_extraction->marker_corner);
    //    if (feature_extraction->markerArray_corner.markers.size() > 10) {
    //      feature_extraction->markerArray_corner.markers.pop;
    //    }
    //    std::cout << "feature_extraction->markerArray_corner.markers.size(): "
    //              << feature_extraction->markerArray_corner.markers.size() << std::endl;
    pub_ptr_marker_corner_->publish(feature_extraction->marker_corner);

    auto surface_cloud_ptr_current = thing_to_cloud(feature_extraction->surfaceCloud, "map");
    pub_ptr_surface_cloud_current_->publish(*surface_cloud_ptr_current);

    cloud_all.insert(cloud_all.end(), cloud_trans.begin(), cloud_trans.end());
    cloud_all_corner_.insert(
      cloud_all_corner_.end(), feature_extraction->cornerCloud.begin(),
      feature_extraction->cornerCloud.end());
    cloud_all_surface_.insert(
      cloud_all_surface_.end(), feature_extraction->surfaceCloud.begin(),
      feature_extraction->surfaceCloud.end());
    //    pub_ptr_image_->publish(image);

    image_projection->resetParameters();
  }

  if (save_pcd_) {
    Occtree occ_cloud(voxel_resolution_);
    Occtree occ_cloud_corner(voxel_resolution_);
    Occtree occ_cloud_surface(voxel_resolution_);

    for (const auto & point : cloud_all) {
      occ_cloud.addPointIfVoxelEmpty(pcl::PointXYZI(point.x, point.y, point.z, point.intensity));
    }
    for (const auto & point : cloud_all_corner_) {
      occ_cloud_corner.addPointIfVoxelEmpty(
        pcl::PointXYZI(point.x, point.y, point.z, point.intensity));
    }
    for (const auto & point : cloud_all_surface_) {
      occ_cloud_surface.addPointIfVoxelEmpty(
        pcl::PointXYZI(point.x, point.y, point.z, point.intensity));
    }

    pcl::PointCloud<pcl::PointXYZI> new_cloud;
    for (auto & point : *occ_cloud.cloud) {
      new_cloud.push_back(point);
    }
    pcl::PointCloud<pcl::PointXYZI> corner_cloud_pcl;
    for (auto & point : *occ_cloud_corner.cloud) {
      corner_cloud_pcl.push_back(point);
    }
    pcl::PointCloud<pcl::PointXYZI> surface_cloud_pcl;
    for (auto & point : *occ_cloud_surface.cloud) {
      surface_cloud_pcl.push_back(point);
    }
    pcl::io::savePCDFileASCII(pcd_export_dir_ + "ytu_campus.pcd", new_cloud);
    pcl::io::savePCDFileASCII(pcd_export_dir_ + "ytu_campus_corner.pcd", corner_cloud_pcl);
    pcl::io::savePCDFileASCII(pcd_export_dir_ + "ytu_campus_surface.pcd", surface_cloud_pcl);
    std::cout << "PCDs saved." << std::endl;
  }

  std::cout << "LoamMapper is done." << std::endl;
}

void LoamMapper::callback_cloud_surround_out(const LoamMapper::Points & points_surround)
{
  clouds.push_back(points_surround);
}

sensor_msgs::msg::PointCloud2::SharedPtr LoamMapper::points_to_cloud(
  const LoamMapper::Points & points_bad, const std::string & frame_id)
{
  using CloudModifier = point_cloud_msg_wrapper::PointCloud2Modifier<point_types::PointXYZI>;
  PointCloud2::SharedPtr cloud_ptr_current = std::make_shared<PointCloud2>();
  CloudModifier cloud_modifier_current(*cloud_ptr_current, frame_id);
  cloud_modifier_current.resize(points_bad.size());
  std::transform(
    std::execution::par, points_bad.cbegin(), points_bad.cend(), cloud_modifier_current.begin(),
    [](const points_provider::PointsProvider::Point & point_bad) {
      return point_types::PointXYZI{
        point_bad.x, point_bad.y, point_bad.z, static_cast<float>(point_bad.intensity)};
    });
  return cloud_ptr_current;
}

sensor_msgs::msg::Image LoamMapper::createImageFromRangeMat(const cv::Mat & rangeMat)
{
  sensor_msgs::msg::Image image;
  image.header.stamp = this->get_clock()->now();
  image.header.frame_id = "map";
  image.height = 16;
  image.width = 1800;
  image.step = rangeMat.step;
  image.encoding = "mono8";
  for (int i = 0; i < 16; i++) {
    for (int j = 0; j < 1800; j++) {
      image.data.push_back(rangeMat.at<char>(i, j));
    }
  }
  return image;
}

void LoamMapper::clear_cloudInfo(utils::Utils::CloudInfo & cloudInfo)
{
  cloudInfo.point_range.clear();
  cloudInfo.start_ring_index.clear();
  cloudInfo.end_ring_index.clear();
  cloudInfo.point_col_index.clear();
}

}  // namespace loam_mapper

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<loam_mapper::LoamMapper>());
  rclcpp::shutdown();

  return 0;
}