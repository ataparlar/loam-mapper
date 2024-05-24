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

  std::mutex imuLock;
  std::mutex odoLock;

  std::deque<sensor_msgs::msg::Imu> imuQueue;
  std::deque<nav_msgs::msg::Odometry> odomQueue;
  std::deque<Points> cloudQueue;

  Points currentCloudMsg;

  //  double * imuTime = new double[queueLength];
  //  double * imuRotX = new double[queueLength];
  //  double * imuRotY = new double[queueLength];
  //  double * imuRotZ = new double[queueLength];

  //  int imuPointerCur{};
    bool firstPointFlag{};

  Eigen::Affine3f transStartInverse;

  Points fullCloud;
  Points extractedCloud;

  std::ofstream file_point_range;
  std::ofstream file_start_ring_ind;
  std::ofstream file_end_ring_ind;
  std::ofstream file_point_col_ind;


  //  int deskewFlag{};
  cv::Mat rangeMat;
//  cv::Mat image_vis;

  //  bool odomDeskewFlag{};
  //  float odomIncreX{};
  //  float odomIncreY{};
  //  float odomIncreZ{};

  //  lio_sam::msg::CloudInfo cloudInfo;
  //  double timeScanCur{};
  //  double timeScanEnd{};
  std_msgs::msg::Header cloudHeader;

  std::vector<int> columnIdnCountVec;

  //  void setLaserCloudIn(const Points & cloud);
  void allocateMemory();

  void imuHandler(const transform_provider::TransformProvider::Imu & imuMsg);
  void odomHandler(const nav_msgs::msg::Odometry odometryMsg);
  void cloudHandler(Points & laserCloudMsg);

  void cachePointCloud(Points & laserCloudMsg);
  //  bool deskewInfo();
  //  void imuDeskewInfo();
  //  void odomDeskewInfo();
  //  void findRotation(double pointTime, float * rotXCur, float * rotYCur, float * rotZCur);
  //  void findPosition(double relTime, float * posXCur, float * posYCur, float * posZCur);
  Point deskewPoint(
    Point & point,
    loam_mapper::transform_provider::TransformProvider::SharedPtr & transform_provider);
  void projectPointCloud(Points & laserCloudMsg);
  void cloudExtraction(Points & laserCloudMsg);
  //  void publishClouds();
  void resetParameters();
};

}  // namespace loam_mapper::image_projection

#endif  // BUILD_IMAGE_PROJECTION_HPP
