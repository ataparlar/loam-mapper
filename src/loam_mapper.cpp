
#include "loam_mapper/loam_mapper.hpp"

#include <iostream>
#include <numeric>
#include <execution>
#include "loam_mapper/utils.hpp"
#include "pcapplusplus/PcapFileDevice.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/range/iterator_range.hpp>

namespace loam_mapper
{
LoamMapper::LoamMapper(const loam_mapper::TransformProvider::ConstSharedPtr & transform_provider,
                       const loam_mapper::PointsProvider::ConstSharedPtr & points_provider)
{

  // params
  double imu2lidar_roll;
  double imu2lidar_pitch;
  double imu2lidar_yaw;


  for (int i = 0; i < points_provider->paths_pcaps_.size(); i++)
  {

    points_provider->process_pcap(points_provider->paths_pcaps_.at(i));

    auto process_cloud_single = [&](const std::vector<PointsProvider::PointXYZIT> cloud)
    {
      std::vector<PointsProvider::PointXYZIT> cloud_trans;
      cloud_trans.resize(cloud.size());


      std::transform(
        std::execution::par,
        cloud.cbegin(),
        cloud.cend(),
        cloud_trans.begin(),
        [this, &transform_provider](const PointsProvider::PointXYZIT & point) {
          PointsProvider::PointXYZIT point_trans;

          // position from applanix data is taken into pose below according to the stamps .
          TransformProvider::Pose pose = transform_provider->get_pose_at(point.stamp_unix_seconds, point.stamp_nanoseconds);

          Eigen::Quaterniond quat_ins_to_map(
            pose.pose_with_covariance.pose.orientation.w,
            pose.pose_with_covariance.pose.orientation.x,
            pose.pose_with_covariance.pose.orientation.y,
            pose.pose_with_covariance.pose.orientation.z);
          Eigen::Affine3d affine_imu2lidar(Eigen::Affine3d::Identity());
          affine_imu2lidar.matrix().topLeftCorner<3, 3>() =
            Eigen::AngleAxisd(utils::Utils::deg_to_rad(179.59), Eigen::Vector3d::UnitZ())
              .toRotationMatrix() *
            Eigen::AngleAxisd(utils::Utils::deg_to_rad(-0.42), Eigen::Vector3d::UnitY())
              .toRotationMatrix() *
            Eigen::AngleAxisd(utils::Utils::deg_to_rad(0.39), Eigen::Vector3d::UnitX())
              .toRotationMatrix();

          // sensor to map rotation is created to add translations and get the right rotation.
          Eigen::Affine3d affine_sensor2map(Eigen::Affine3d::Identity());

          affine_sensor2map.matrix().topLeftCorner<3, 3>() = quat_ins_to_map.toRotationMatrix() * affine_imu2lidar.rotation().inverse();

          // pose is added to the transformation matrix. - these were completed for every point in the pointclouds.
          affine_sensor2map.matrix().topRightCorner<3, 1>() << pose.pose_with_covariance.pose.position.x,
                pose.pose_with_covariance.pose.position.y, pose.pose_with_covariance.pose.position.z;

          // get the point's position w.r.t. the point cloud origin.
          Eigen::Vector4d vec_point_in_first(point.x,
                                             point.y,
                                             point.z,
                                             1.0);
          // create a 3D vector for transformed point to the map position and rotation.
          Eigen::Vector4d vec_point_trans = affine_sensor2map.matrix() * vec_point_in_first;

          point_trans.x = static_cast<double>(vec_point_trans(0));
          point_trans.y = static_cast<double>(vec_point_trans(1));
          point_trans.z = static_cast<double>(vec_point_trans(2));
          point_trans.intensity = point.intensity;

          return point_trans;
        });

      std::string point_cloud_name = "/home/ataparlar/data/task_spesific/loam_based_localization/mapping/pcap_and_poses/output/";
      point_cloud_name += "ytu_campus_" + std::to_string(i) + ".pcd";


      pcl::PointCloud<pcl::PointXYZI> new_cloud;
      for (auto & point : cloud_trans) {
        pcl::PointXYZI pcl_point;
        pcl_point.x = point.x;
        pcl_point.y = point.y;
        pcl_point.z = point.z;
        pcl_point.intensity = point.intensity;
        new_cloud.push_back(pcl_point);
      }
      pcl::io::savePCDFileASCII(point_cloud_name, new_cloud);

    };


    process_cloud_single(points_provider->cloud_);

    points_provider->cloud_.clear();
  }


}





}  // namespace loam_mapper

int main()
{
  boost::filesystem::path pose_path("/home/ataparlar/data/task_spesific/loam_based_localization/mapping/pcap_and_poses/ytu_campus_080423_ground_truth.txt");
  boost::filesystem::path pcap_path("/home/ataparlar/data/task_spesific/loam_based_localization/mapping/pcap_and_poses/pcaps/");
  auto transform_provider = std::make_shared<loam_mapper::TransformProvider>(pose_path);
  auto points_provider = std::make_shared<loam_mapper::PointsProvider>(pcap_path);

  loam_mapper::LoamMapper(transform_provider, points_provider);

  return 0;
}