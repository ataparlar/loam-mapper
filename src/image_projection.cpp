
#include "loam_mapper/image_projection.hpp"

#include "loam_mapper/utils.hpp"

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

void ImageProjection::imuHandler(const sensor_msgs::msg::Imu imuMsg)
{
  std::lock_guard<std::mutex> lock1(imuLock);
  imuQueue.push_back(imuMsg);
}

void ImageProjection::odomHandler(const nav_msgs::msg::Odometry odometryMsg)
{
  std::lock_guard<std::mutex> lock2(odoLock);
  odomQueue.push_back(odometryMsg);
}

void ImageProjection::cloudHandler(Points & laserCloudMsg)
{
  cachePointCloud(laserCloudMsg);

  projectPointCloud(laserCloudMsg);

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

void ImageProjection::projectPointCloud(Points & laserCloudMsg)
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

    //    thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time);

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
  //  firstPointFlag = true;
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

}  // namespace loam_mapper::image_projection