
#include "loam_mapper/loam_mapper.hpp"

#include "loam_mapper/Occtree.h"
#include "pcl/common/transforms.h"

#include <Eigen/Geometry>
#include <loam_mapper/point_types.hpp>
#include <loam_mapper/utils.hpp>
#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/path.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

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
  pub_ptr_ext_cloud_current_ = this->create_publisher<PointCloud2>("ext_cloud_current", 10);
  pub_ptr_occ_not_cloud_current_ = this->create_publisher<PointCloud2>("occ_not_cloud_current", 10);
  pub_ptr_occ_cloud_current_ = this->create_publisher<PointCloud2>("occ_cloud_current", 10);
  pub_ptr_path_ = this->create_publisher<nav_msgs::msg::Path>("vehicle_path", 10);
  pub_ptr_cloud_path_ = this->create_publisher<nav_msgs::msg::Path>("cloud_path", 10);
  pub_ptr_image_ = this->create_publisher<sensor_msgs::msg::Image>("rangeMat", 10);

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
  points_provider->process_pcaps_into_clouds(callback, 0, 3);
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
  path_.header.stamp = this->get_clock()->now();
  path_.poses.resize(transform_provider->poses_.size());

  for (auto & pose : transform_provider->poses_) {
    geometry_msgs::msg::PoseStamped poseStamped;
    poseStamped.header.frame_id = "map";
    poseStamped.header.stamp = this->get_clock()->now();
    poseStamped.pose = pose.pose_with_covariance.pose;
    path_.poses.push_back(poseStamped);
  }
  pub_ptr_path_->publish(path_);

  points_provider::PointsProvider::Points cloud_all;
  points_provider::PointsProvider::Points cloud_all_corner_;
  points_provider::PointsProvider::Points cloud_all_surface_;



  for (auto & cloud : clouds) {
//    std::sort(cloud.begin(), cloud.end(), by_ring_and_angle());
//    std::sort(cloud.begin(), cloud.end(), by_ring_and_angle());

    //    image_projection->setLaserCloudIn(cloud_trans);

    image_projection->cloudHandler(cloud);

    sensor_msgs::msg::Image hsv_image = prepareVisImage(cloud);


    feature_extraction->laserCloudInfoHandler(cloud, image_projection->cloudInfo);

    pub_ptr_cloud_path_->publish(feature_extraction->cloudPath);
    feature_extraction->cloudPath.poses.clear();


    image_projection->resetParameters();



    Points cloud_trans = transform_points(cloud);
    auto cloud_ptr_current = thing_to_cloud(cloud_trans, "map");
    pub_ptr_basic_cloud_current_->publish(*cloud_ptr_current);
    pub_ptr_image_->publish(hsv_image);
    cloud_all.insert(cloud_all.end(), cloud_trans.begin(), cloud_trans.end());



    Points occ_cloud_not = transform_points(feature_extraction->cloudOccludedNot);
    auto occ_not_cloud_ptr_current = thing_to_cloud(occ_cloud_not, "map");
    pub_ptr_occ_not_cloud_current_->publish(*occ_not_cloud_ptr_current);
    feature_extraction->cloudOccludedNot.clear();

    Points occ_cloud_trans = transform_points(feature_extraction->cloudOccluded);
    auto occ_cloud_ptr_current = thing_to_cloud(occ_cloud_trans, "map");
    pub_ptr_occ_cloud_current_->publish(*occ_cloud_ptr_current);
    feature_extraction->cloudOccluded.clear();

    Points ext_trans = transform_points(feature_extraction->extractedCloud);
    auto ext_cloud_ptr_current = thing_to_cloud(ext_trans, "map");
    pub_ptr_ext_cloud_current_->publish(*ext_cloud_ptr_current);





    Points cloud_corner_trans = transform_points(feature_extraction->cornerCloud);
    auto corner_cloud_ptr_current = thing_to_cloud(cloud_corner_trans, "map");
    pub_ptr_corner_cloud_current_->publish(*corner_cloud_ptr_current);
    cloud_all_corner_.insert(
      cloud_all_corner_.end(), cloud_corner_trans.begin(),
      cloud_corner_trans.end());


    Points cloud_surface_trans = transform_points(feature_extraction->surfaceCloud);
    auto surface_cloud_ptr_current = thing_to_cloud(cloud_surface_trans, "map");
    pub_ptr_surface_cloud_current_->publish(*surface_cloud_ptr_current);
    cloud_all_surface_.insert(
      cloud_all_surface_.end(), cloud_surface_trans.begin(),
      cloud_surface_trans.end());

    std::this_thread::sleep_for(std::chrono::milliseconds(300));
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
//  sensor_msgs::msg::PointCloud2::SharedPtr cloud_surround = points_to_cloud(points_surround, "map");
//  pub_ptr_basic_cloud_current_->publish(*cloud_surround);
  clouds.push_back(points_surround);
//  std::this_thread::sleep_for(std::chrono::milliseconds(500));
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

void LoamMapper::clear_cloudInfo(utils::Utils::CloudInfo & cloudInfo)
{
  cloudInfo.point_range.clear();
  cloudInfo.start_ring_index.clear();
  cloudInfo.end_ring_index.clear();
  cloudInfo.point_col_index.clear();
}

LoamMapper::Points LoamMapper::transform_points(LoamMapper::Points & cloud) {
  Points cloud_trans;
  cloud_trans.resize(cloud.size());

  std::transform(
    std::execution::par, cloud.cbegin(), cloud.cend(), cloud_trans.begin(),
    //      std::execution::par, feature_extraction->cornerCloud.cbegin(),
    //      feature_extraction->cornerCloud.cend(), cloud_trans.begin(),
    [this](const points_provider::PointsProvider::Point & point) {
      points_provider::PointsProvider::Point point_trans;
      loam_mapper::transform_provider::TransformProvider::Pose pose =
        this->transform_provider->get_pose_at(point.stamp_unix_seconds, point.stamp_nanoseconds);

      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.pose = pose.pose_with_covariance.pose;
      pose_stamped.header.frame_id = "map";

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
        Eigen::Affine3d affine_imu2lidar_enu;
        affine_imu2lidar_enu.matrix().topLeftCorner<3, 3>() =
          utils::Utils::ned2enu_converter_for_matrices(
            affine_imu2lidar.matrix().topLeftCorner<3, 3>());

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
      point_trans.stamp_unix_seconds = point.stamp_unix_seconds;
      point_trans.stamp_nanoseconds = point.stamp_nanoseconds;

      return point_trans;
    });
  return cloud_trans;
}

sensor_msgs::msg::Image LoamMapper::prepareVisImage(Points laserCloudMsg) {

  cv::Mat HSV(16, 1800, CV_32FC1, 0.0);

  int cloudSize = laserCloudMsg.size();

  for (int i = 0; i < cloudSize; ++i) {
      Point thisPoint;
      thisPoint.x = laserCloudMsg[i].x;
      thisPoint.y = laserCloudMsg[i].y;
      thisPoint.z = laserCloudMsg[i].z;
      thisPoint.intensity = laserCloudMsg[i].intensity;
      thisPoint.horizontal_angle = laserCloudMsg[i].horizontal_angle;
      thisPoint.ring = laserCloudMsg[i].ring;
      thisPoint.stamp_unix_seconds = laserCloudMsg[i].stamp_unix_seconds;
      thisPoint.stamp_nanoseconds = laserCloudMsg[i].stamp_nanoseconds;

      float range =
        sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);

      int rowIdn = thisPoint.ring;

      if (rowIdn < 0 || rowIdn >= 16) continue;

      int columnIdn = -1;
      float horizonAngle = thisPoint.horizontal_angle;
//              float horizonAngle = (atan2(thisPoint.x, thisPoint.y) * 180 / M_PI);
      static float ang_res_x = 360.0 / float(1800);
      columnIdn = round((horizonAngle) / ang_res_x);
//          columnIdn = -round((horizonAngle-90) / ang_res_x) + 1800.0 / 2;
      if (columnIdn >= 1800) columnIdn -= 1800;
      if (columnIdn < 0 || columnIdn >= 1800) continue;

//      uchar hue = static_cast<uchar>((range * 180.0) / 60);
//      uchar hue = static_cast<uchar>((horizonAngle * 180.0) / 360);
//      HSV.at<cv::Vec3b>(rowIdn, columnIdn) = cv::Vec3b(hue, 255.0, 255.0);
      HSV.at<float>(rowIdn, columnIdn) = range;
  }

//  cv::Mat normalized_image(HSV.rows, HSV.cols, CV_8UC1, 0.0);
////  HSV.convertTo(normalized_image, CV_8UC1, 180.0);
//  for (int col = 0; col < HSV.cols; ++col) {
//    for (int row = 0; row < HSV.rows; ++row) {
//      normalized_image.at<uchar>(row, col) = static_cast<uchar>((HSV.at<float>(row, col) * 180.0) / 60);
//    }
//  }

  cv::Mat normalized_image(image_projection->rangeMat.rows, image_projection->rangeMat.cols, CV_8UC1, 0.0);
  for (int col = 0; col < image_projection->rangeMat.cols; ++col) {
    for (int row = 0; row < image_projection->rangeMat.rows; ++row) {
      normalized_image.at<uchar>(row, col) = static_cast<uchar>((image_projection->rangeMat.at<float>(row, col) * 180.0) / 60);
    }
  }

  cv::Mat hsv_image(normalized_image.size(), CV_8UC3, cv::Vec3b(0.0, 0.0, 0.0));
  for (int col = 0; col < normalized_image.cols; ++col) {
    for (int row = 0; row < normalized_image.rows; ++row) {
      uchar hue = normalized_image.at<uchar>(row, col);
      hsv_image.at<cv::Vec3b>(row, col) = cv::Vec3b(hue, 255, 255); // Full saturation and value
    }
  }

  cv::Mat BGR;
  cv::cvtColor(hsv_image, BGR, cv::COLOR_HSV2BGR);

  cv::Mat bgr_resized;
  cv::resize(BGR, bgr_resized, cv::Size(), 1.0, 20.0);

    cv_bridge::CvImage cv_image;
    cv_image.header.frame_id = "map";
    cv_image.header.stamp = this->get_clock()->now();
    cv_image.encoding = "bgr8";
    cv_image.image = bgr_resized;


    sensor_msgs::msg::Image image;
    cv_image.toImageMsg(image);

    return image;
}



}  // namespace loam_mapper

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<loam_mapper::LoamMapper>());
  rclcpp::shutdown();

  return 0;
}