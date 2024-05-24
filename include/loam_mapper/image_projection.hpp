#ifndef BUILD_IMAGE_PROJECTION_HPP
#define BUILD_IMAGE_PROJECTION_HPP

#include "points_provider_base.hpp"
#include "transform_provider.hpp"
#include "utils.hpp"

#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <boost/filesystem.hpp>

#include <deque>
#include <memory>
#include <string>
//
namespace loam_mapper::image_projection
{
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using Point = points_provider::PointsProviderBase::Point;
using Points = points_provider::PointsProviderBase::Points;

const int queueLength = 2000;

class ImageProjection
{
public:
  using SharedPtr = std::shared_ptr<ImageProjection>;
  using ConstSharedPtr = const SharedPtr;

  explicit ImageProjection();

  utils::Utils::CloudInfo cloudInfo;

  std::deque<sensor_msgs::msg::Imu> imuQueue;
  std::deque<nav_msgs::msg::Odometry> odomQueue;
  std::deque<Points> cloudQueue;

  Points currentCloudMsg;

    bool firstPointFlag{};

  Eigen::Affine3f transStartInverse;

  Points fullCloud;
  Points extractedCloud;

  cv::Mat rangeMat;
  std_msgs::msg::Header cloudHeader;

  std::vector<int> columnIdnCountVec;

  void allocateMemory();

  void cloudHandler(Points & laserCloudMsg);
  void cachePointCloud(Points & laserCloudMsg);
  void projectPointCloud(Points & laserCloudMsg);
  void cloudExtraction();
  void resetParameters();
};

}  // namespace loam_mapper::image_projection

#endif  // BUILD_IMAGE_PROJECTION_HPP
