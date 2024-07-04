/*
* Copyright 2024 LeoDrive.ai, Inc. All rights reserved.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
 */

#include "loam_mapper/loam_mapper.hpp"

#include "loam_mapper/Occtree.h"

#include <Eigen/Geometry>
#include <loam_mapper/point_types.hpp>
#include <loam_mapper/utils.hpp>
#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <cv_bridge/cv_bridge.h>
#include <pcl/io/pcd_io.h>

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
  this->declare_parameter("project_namespace", "");
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
  project_namespace_ = this->get_parameter("project_namespace").as_string();
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

  transform_provider = std::make_shared<transform_provider::TransformProvider>(pose_txt_path_);

  transform_provider->process(map_origin_x_, map_origin_y_, map_origin_z_);

  points_provider = std::make_shared<points_provider::PointsProvider>(std::string(pcap_dir_path_));
  points_provider->process();

  image_projection = std::make_shared<image_projection::ImageProjection>();
  feature_extraction = std::make_shared<feature_extraction::FeatureExtraction>();

  for (int i = 0; i < points_provider->paths_pcaps_.size(); i++) {
    std::function<void(const Points &)> callback =
      std::bind(&LoamMapper::callback_cloud_surround_out, this, std::placeholders::_1);
    points_provider->process_pcaps_into_clouds(callback, i, 1);
    RCLCPP_INFO(
      this->get_logger(), "PCAP number %s is converted to clouds.", std::to_string(i).c_str());
    process(i);
    clouds.clear();
    RCLCPP_INFO(this->get_logger(), "-----------------------------------------------");
  }
  RCLCPP_INFO(this->get_logger(), "All PCAP files are converted into .pcd point clouds.");
  RCLCPP_INFO(this->get_logger(), "Destination:  %s", pcd_export_dir_.c_str());

  rclcpp::shutdown();
}

void LoamMapper::process(int file_counter)
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
    Points filtered_points;
//    filtered_points.resize(cloud.size());

    auto last_pose = transform_provider->poses_.back();
    std::copy_if(
      cloud.cbegin(), cloud.cend(), std::back_inserter(filtered_points),
      [&last_pose](const points_provider::PointsProvider::Point & point) {
        return !(
          point.stamp_unix_seconds > last_pose.stamp_unix_seconds ||
          (point.stamp_unix_seconds == last_pose.stamp_unix_seconds &&
           point.stamp_nanoseconds > last_pose.stamp_nanoseconds));
      });


    Points cloud_trans_undistorted;
    cloud_trans_undistorted.resize(filtered_points.size());


    bool first_point_flag = true;
    std::transform(
      std::execution::par, filtered_points.cbegin(), filtered_points.cend(), cloud_trans_undistorted.begin(),
      [this, &first_point_flag](const points_provider::PointsProvider::Point & point) {
        auto imu_ =
          transform_provider->get_imu_at(point.stamp_unix_seconds, point.stamp_nanoseconds);
        auto pose_ =
          transform_provider->get_pose_at(point.stamp_unix_seconds, point.stamp_nanoseconds);

        Eigen::Quaternion imu_ori(
          imu_.imu.orientation.w, imu_.imu.orientation.x, imu_.imu.orientation.y,
          imu_.imu.orientation.z);
        auto imu_euler = imu_ori.toRotationMatrix().eulerAngles(0, 1, 2);

        double first_point_time;
        float first_point_x_vel, first_point_y_vel, first_point_z_vel;

        if (first_point_flag) {
          first_point_time = point.stamp_unix_seconds / 10e-6 + point.stamp_nanoseconds / 10e-9;
          first_point_x_vel = pose_.velocity.x;
          first_point_y_vel = pose_.velocity.y;
          first_point_z_vel = pose_.velocity.z;
          first_point_flag = false;
          return point;
        } else {
          double point_time = point.stamp_unix_seconds / 10e-6 + point.stamp_nanoseconds / 10e-9;

          double time_dif = point_time - first_point_time;

          double omega_x = imu_.imu.angular_velocity.x;
          double omega_y = imu_.imu.angular_velocity.y;
          double omega_z = imu_.imu.angular_velocity.z;

          double omega_magnitude = sqrt(omega_x * omega_x + omega_y * omega_y + omega_z * omega_z);

          double theta = omega_magnitude * time_dif;
          double cosTheta = cos(theta);
          double sinTheta = sin(theta);

          double magnitude = sqrt(omega_x * omega_x + omega_y * omega_y + omega_z * omega_z);
          if (magnitude != 0) {
            omega_x /= magnitude;
            omega_y /= magnitude;
            omega_z /= magnitude;
          }

          double new_x = (cosTheta + (1 - cosTheta) * omega_x * omega_x) * point.x +
                         ((1 - cosTheta) * omega_x * omega_y - omega_z * sinTheta) * point.y +
                         ((1 - cosTheta) * omega_x * omega_z + omega_y * sinTheta) * point.z;
          double new_y = ((1 - cosTheta) * omega_y * omega_x + omega_z * sinTheta) * point.x +
                         (cosTheta + (1 - cosTheta) * omega_y * omega_y) * point.y +
                         ((1 - cosTheta) * omega_y * omega_z - omega_x * sinTheta) * point.z;
          double new_z = ((1 - cosTheta) * omega_z * omega_x - omega_y * sinTheta) * point.x +
                         ((1 - cosTheta) * omega_z * omega_y + omega_x * sinTheta) * point.y +
                         (cosTheta + (1 - cosTheta) * omega_z * omega_z) * point.z;

          Point new_point;
          new_point.x = new_x;
          new_point.y = new_y;
          new_point.z = new_z;

          float x_linear_dif = time_dif * (pose_.velocity.x * first_point_x_vel);
          float y_linear_dif = time_dif * (pose_.velocity.y * first_point_y_vel);
          float z_linear_dif = time_dif * (pose_.velocity.z * first_point_z_vel);

          new_point.x = x_linear_dif + point.x;
          new_point.y = y_linear_dif + point.y;
          new_point.z = z_linear_dif + point.z;
          new_point.intensity = point.intensity;
          new_point.stamp_unix_seconds = point.stamp_unix_seconds;
          new_point.stamp_nanoseconds = point.stamp_nanoseconds;
          new_point.horizontal_angle = point.horizontal_angle;
          new_point.ring = point.ring;

          return new_point;
        }
      });

    image_projection->cloudHandler(cloud_trans_undistorted);
    sensor_msgs::msg::Image hsv_image = prepareVisImage(image_projection->rangeMat);
    feature_extraction->laserCloudInfoHandler(
      image_projection->extractedCloud, image_projection->cloudInfo);

    image_projection->resetParameters();
    clear_cloudInfo(image_projection->cloudInfo);

    Points cloud_trans = transform_points(cloud);
    auto cloud_ptr_current = thing_to_cloud(cloud_trans, "map");
    pub_ptr_basic_cloud_current_->publish(*cloud_ptr_current);
    pub_ptr_image_->publish(hsv_image);
    cloud_all.insert(cloud_all.end(), cloud_trans.begin(), cloud_trans.end());

    Points cloud_corner_trans =
      transform_points(feature_extraction->cornerCloud);
    auto corner_cloud_ptr_current = thing_to_cloud(cloud_corner_trans, "map");
    pub_ptr_corner_cloud_current_->publish(*corner_cloud_ptr_current);
    cloud_all_corner_.insert(
      cloud_all_corner_.end(), cloud_corner_trans.begin(), cloud_corner_trans.end());

    Points cloud_surface_trans =
      transform_points(feature_extraction->surfaceCloud);
    auto surface_cloud_ptr_current = thing_to_cloud(cloud_surface_trans, "map");
    pub_ptr_surface_cloud_current_->publish(*surface_cloud_ptr_current);
    cloud_all_surface_.insert(
      cloud_all_surface_.end(), cloud_surface_trans.begin(), cloud_surface_trans.end());

    //    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  if (save_pcd_ && !cloud_all.empty()) {
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
    pcl::io::savePCDFileASCII(
      pcd_export_dir_ + project_namespace_ + "_" + std::to_string(file_counter) + ".pcd",
      new_cloud);
    pcl::io::savePCDFileASCII(
      pcd_export_dir_ + project_namespace_ + "_corner_" + std::to_string(file_counter) + ".pcd",
      corner_cloud_pcl);
    pcl::io::savePCDFileASCII(
      pcd_export_dir_ + project_namespace_ + "_surface_" + std::to_string(file_counter) + ".pcd",
      surface_cloud_pcl);
    RCLCPP_INFO(
      this->get_logger(), "PCDs saved for PCAP number: %s.", std::to_string(file_counter).c_str());
  }

  cloud_all.clear();
  cloud_all_corner_.clear();
  cloud_all_surface_.clear();

  image_projection->fullCloud.clear();
  image_projection->extractedCloud.clear();
  feature_extraction->extractedCloud.clear();
  feature_extraction->cornerCloud.clear();
  feature_extraction->surfaceCloud.clear();
}

void LoamMapper::callback_cloud_surround_out(const LoamMapper::Points & points_surround)
{
  //  pub_ptr_basic_cloud_current_->publish(*points_to_cloud(points_surround, "map"));
  //  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  clouds.push_back(points_surround);
}

sensor_msgs::msg::PointCloud2::SharedPtr LoamMapper::points_to_cloud(
  const LoamMapper::Points & points_bad, const std::string & frame_id)
{
  using CloudModifier = point_cloud_msg_wrapper::PointCloud2Modifier<point_types::PointXYZIR>;
  PointCloud2::SharedPtr cloud_ptr_current = std::make_shared<PointCloud2>();
  CloudModifier cloud_modifier_current(*cloud_ptr_current, frame_id);
  cloud_modifier_current.resize(points_bad.size());
  std::transform(
    std::execution::par, points_bad.cbegin(), points_bad.cend(), cloud_modifier_current.begin(),
    [](const points_provider::PointsProvider::Point & point_bad) {
      return point_types::PointXYZIR{
        point_bad.x, point_bad.y, point_bad.z, static_cast<uint32_t>(point_bad.intensity),
        //        static_cast<float>(point_bad.stamp_nanoseconds),
        //        static_cast<float>(point_bad.horizontal_angle),
        static_cast<uint32_t>(point_bad.ring)};
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

LoamMapper::Points LoamMapper::transform_points(LoamMapper::Points & cloud)
{
  Points filtered_points;
//  filtered_points.resize(cloud.size());

  auto last_pose = transform_provider->poses_.back();
  std::copy_if(
    cloud.cbegin(), cloud.cend(), std::back_inserter(filtered_points),
    [&last_pose](const points_provider::PointsProvider::Point & point) {
      return !(
        point.stamp_unix_seconds > last_pose.stamp_unix_seconds ||
        (point.stamp_unix_seconds == last_pose.stamp_unix_seconds &&
         point.stamp_nanoseconds > last_pose.stamp_nanoseconds));
    });

  Points cloud_trans;
  cloud_trans.resize(filtered_points.size());

  std::transform(
    std::execution::par, filtered_points.cbegin(), filtered_points.cend(), cloud_trans.begin(),
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
        Eigen::AngleAxisd(
          utils::Utils::deg_to_rad(-(imu2lidar_yaw_ - pose.meridian_convergence)), Eigen::Vector3d::UnitZ())
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

sensor_msgs::msg::Image LoamMapper::prepareVisImage(cv::Mat & rangeMat)
{
  cv::Mat normalized_image(rangeMat.rows, rangeMat.cols, CV_8UC1, 0.0);
  for (int col = 0; col < rangeMat.cols; ++col) {
    for (int row = 0; row < rangeMat.rows; ++row) {
      normalized_image.at<uchar>(row, col) =
        static_cast<uchar>((rangeMat.at<float>(row, col) * 180.0) / 60);
    }
  }

  cv::Mat hsv_image(normalized_image.size(), CV_8UC3, cv::Vec3b(0.0, 255.0, 255.0));
  for (int col = 0; col < normalized_image.cols; ++col) {
    for (int row = 0; row < normalized_image.rows; ++row) {
      uchar hue = normalized_image.at<uchar>(row, col);
      if (hue == 0) {
        hsv_image.at<cv::Vec3b>(row, col) = cv::Vec3b(hue, 0, 0);  // Full saturation and value
      } else {
        hsv_image.at<cv::Vec3b>(row, col) = cv::Vec3b(hue, 255, 255);  // Full saturation and value
      }
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