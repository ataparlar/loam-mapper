#ifndef BUILD_IMAGE_PROJECTION_HPP
#define BUILD_IMAGE_PROJECTION_HPP

#include "csv.hpp"
#include "point_types.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <Eigen/Geometry>

#include <boost/filesystem.hpp>

#include <deque>
#include <memory>
#include <string>

namespace loam_mapper::image_projection
{
namespace fs = boost::filesystem;
using Point = point_types::PointXYZITR;
using Points = std::vector<Point>;

const int queueLength = 2000;

class ImageProjection {
  using SharedPtr = std::shared_ptr<ImageProjection>;
  using ConstSharedPtr = const SharedPtr;

  explicit ImageProjection();

  std::mutex imuLock;
  std::mutex odoLock;

  std::deque<sensor_msgs::msg::Imu> imuQueue;
  std::deque<nav_msgs::msg::Odometry> odomQueue;
  std::deque<sensor_msgs::msg::PointCloud2> cloudQueue;

  sensor_msgs::msg::PointCloud2 currentCloudMsg;


  double *imuTime = new double[queueLength];
  double *imuRotX = new double[queueLength];
  double *imuRotY = new double[queueLength];
  double *imuRotZ = new double[queueLength];

  int imuPointerCur;
  bool firstPointFlag;
  Eigen::Affine3f transStartInverse;

  Points laserCloudIn;
  Points tmpOusterCloudIn;
  Points fullCloud;
  Points extractedCloud;

  int deskewFlag;
//  cv::Mat rangeMat;

  bool odomDeskewFlag;
  float odomIncreX;
  float odomIncreY;
  float odomIncreZ;

//  lio_sam::msg::CloudInfo cloudInfo;
  double timeScanCur;
  double timeScanEnd;
  std_msgs::msg::Header cloudHeader;

  std::vector<int> columnIdnCountVec;

};


}  // namespace loam_mapper::image_projection


#endif  // BUILD_IMAGE_PROJECTION_HPP
