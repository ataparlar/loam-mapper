
#include "loam_mapper/image_projection.hpp"

#include "loam_mapper/utils.hpp"

#include "pcl/common/transforms.h"

namespace loam_mapper::image_projection
{
ImageProjection::ImageProjection()
{
  allocateMemory();
  resetParameters();
}

// void ImageProjection::setLaserCloudIn(const Points & cloud) {
//   laserCloudIn.resize(cloud.size());
//   laserCloudIn = cloud;
// }

void ImageProjection::allocateMemory()
{
  fullCloud.resize(16*1800);

  cloudInfo.start_ring_index.assign(16, 0);
  cloudInfo.end_ring_index.assign(16, 0);

  cloudInfo.point_col_index.assign(16*1800, 0);
  cloudInfo.point_range.assign(16*1800, 0);

  resetParameters();
}

void ImageProjection::imuHandler(const transform_provider::TransformProvider::Imu & imuMsg)
{
  sensor_msgs::msg::Imu thisImu;
  thisImu = imuMsg.imu;

  std::lock_guard<std::mutex> lock1(imuLock);
  imuQueue.push_back(thisImu);
}

void ImageProjection::odomHandler(const nav_msgs::msg::Odometry odometryMsg)
{
  std::lock_guard<std::mutex> lock2(odoLock);
  odomQueue.push_back(odometryMsg);
}

void ImageProjection::cloudHandler(Points & laserCloudMsg, loam_mapper::transform_provider::TransformProvider::SharedPtr & transform_provider)
{
  cachePointCloud(laserCloudMsg);

  projectPointCloud(laserCloudMsg, transform_provider);

  cloudExtraction(laserCloudMsg);

}

void ImageProjection::cachePointCloud(Points & laserCloudMsg)
{
  cloudQueue.push_back(laserCloudMsg);
  if (cloudQueue.size() > 2)
    // convert cloud
    currentCloudMsg = std::move(cloudQueue.front());
    cloudQueue.pop_front();




}

void ImageProjection::projectPointCloud(Points & laserCloudMsg, loam_mapper::transform_provider::TransformProvider::SharedPtr & transform_provider)
{
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



//    double pointTime = laserCloudMsg[i].stamp_unix_seconds + laserCloudMsg[i].stamp_nanoseconds*10e-9;
//
//    std::cout << "laserCloudMsg[i].stamp_unix_seconds: " << laserCloudMsg[i].stamp_unix_seconds << std::endl;
//    std::cout << "laserCloudMsg[i].stamp_nanoseconds: " << laserCloudMsg[i].stamp_nanoseconds << std::endl;
//    std::cout << "pointTime: " << std::setprecision(18) << pointTime << std::endl;
//    std::cout << "\n" << std::endl;

    float range =
      sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);

    int rowIdn = laserCloudMsg[i].ring;
    if (rowIdn < 0 || rowIdn >= 16) continue;

    //    if (rowIdn % downsampleRate != 0)
    //      continue;

    int columnIdn = -1;
    float horizonAngle = laserCloudMsg[i].horizontal_angle;
    static float ang_res_x = 360.0 / float(1800);
    columnIdn = round((horizonAngle) / ang_res_x);
    if (columnIdn >= 1800) columnIdn -= 1800;

    if (columnIdn < 0 || columnIdn >= 1800) continue;

    // project the point cloud into 2d projection. make a depth map from it.
    if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX) continue;

//    auto new_point = deskewPoint(thisPoint, transform_provider);

    rangeMat.at<float>(rowIdn, columnIdn) = range;

    fullCloud.push_back(thisPoint);
  }
}

void ImageProjection::cloudExtraction(Points & laserCloudMsg)
{
  int count = 0;
  // extract segmented cloud for lidar odometry
  //  std::cout << laserCloudMsg.size() << std::endl;
  for (int i = 0; i < 16; ++i) {
    cloudInfo.start_ring_index[i] = count - 1 + 5;
    for (int j = 0; j < 1800; ++j) {
      if (rangeMat.at<float>(i, j) != FLT_MAX) {
        // mark the points' column index for marking occlusion later
        cloudInfo.point_col_index[count] = j;
        // save range info
        cloudInfo.point_range[count] = rangeMat.at<float>(i, j);
        // save extracted cloud
        extractedCloud.push_back(laserCloudMsg[j + i * 1800]);
        // size of extracted cloud
        ++count;
      }
    }
    cloudInfo.end_ring_index[i] = count - 1 - 5;
  }
}

void ImageProjection::resetParameters()
{
  //  laserCloudIn.clear();
  extractedCloud.clear();
  // reset range matrix for range image projection
  rangeMat = cv::Mat(16, 1800, CV_32F, cv::Scalar::all(FLT_MAX));

  //  imuPointerCur = 0;
    firstPointFlag = true;
  //  odomDeskewFlag = false;
  //
  //  for (int i = 0; i < queueLength; ++i)
  //  {
  //    imuTime[i] = 0;
  //    imuRotX[i] = 0;
  //    imuRotY[i] = 0;
  //    imuRotZ[i] = 0;
  //  }
}

Point ImageProjection::deskewPoint(Point & point, loam_mapper::transform_provider::TransformProvider::SharedPtr & transform_provider)
{
//  double pointTime = point.stamp_unix_seconds + point.stamp_nanoseconds*10e-9;

//  auto imu_ = transform_provider->get_imu_at(point.stamp_unix_seconds, point.stamp_nanoseconds);
  auto pose_ = transform_provider->get_pose_at(point.stamp_unix_seconds, point.stamp_nanoseconds);

  Eigen::Quaternion q(
    pose_.pose_with_covariance.pose.orientation.w, pose_.pose_with_covariance.pose.orientation.x,
    pose_.pose_with_covariance.pose.orientation.y, pose_.pose_with_covariance.pose.orientation.z);

  auto euler  = q.toRotationMatrix().eulerAngles(0, 1, 2);
  double rotXCur = euler[0];
  double rotYCur = euler[1];
  double rotZCur = euler[2];

  double posXCur = pose_.pose_with_covariance.pose.position.x;
  double posYCur = pose_.pose_with_covariance.pose.position.y;
  double posZCur = pose_.pose_with_covariance.pose.position.z;

  if (firstPointFlag == true)
  {
    transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur)).inverse();
    firstPointFlag = false;
  }

  // transform points to start
  Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
  Eigen::Affine3f transBt = transStartInverse * transFinal;


  Point point_;
  point_.x = transBt(0, 0) * point.x + transBt(0, 1) * point.y + transBt(0, 2) * point.z + transBt(0, 3);
  point_.y = transBt(1, 0) * point.x + transBt(1, 1) * point.y + transBt(1, 2) * point.z + transBt(1, 3);
  point_.z = transBt(2, 0) * point.x + transBt(2, 1) * point.y + transBt(2, 2) * point.z + transBt(2, 3);
  point_.intensity = point.intensity;
  point_.stamp_unix_seconds = point.stamp_unix_seconds;
  point_.stamp_nanoseconds = point.stamp_nanoseconds;
  point_.ring = point.ring;
  point_.horizontal_angle = point.horizontal_angle;

  return point_;
}


}  // namespace loam_mapper::image_projection