// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following papers:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.
//   T. Shan and B. Englot. LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain
//      IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). October 2018.

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

void ImageProjection::allocateMemory()
{
  fullCloud.resize(16 * 1800);

  cloudInfo.start_ring_index.assign(16, 0);
  cloudInfo.end_ring_index.assign(16, 0);

  cloudInfo.point_col_index.assign(16 * 1800, 0);
  cloudInfo.point_range.assign(16 * 1800, 0);

  resetParameters();
}

void ImageProjection::cloudHandler(Points & laserCloudMsg) {

  cachePointCloud(laserCloudMsg);

  projectPointCloud(laserCloudMsg);

  cloudExtraction();

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
    thisPoint.stamp_unix_seconds = laserCloudMsg[i].stamp_unix_seconds;
    thisPoint.stamp_nanoseconds = laserCloudMsg[i].stamp_nanoseconds;

    float range =
      sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);

    int rowIdn = thisPoint.ring;

    if (rowIdn < 0 || rowIdn >= 16) continue;

    int columnIdn = -1;
    float horizonAngle = thisPoint.horizontal_angle;
    static float ang_res_x = 360.0 / float(1800);
    columnIdn = 1800 - round((horizonAngle) / ang_res_x);
    if (columnIdn >= 1800) columnIdn -= 1800;
    if (columnIdn < 0 || columnIdn >= 1800) continue;

    // project the point cloud into 2d projection. make a depth map from it.
    if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX) continue;

    rangeMat.at<float>(rowIdn, columnIdn) = range;

    int index = columnIdn + rowIdn * 1800;
    fullCloud[index] = thisPoint;
  }
}


void ImageProjection::cloudExtraction()
{
  int count = 0;
  // extract segmented cloud for lidar odometry

  for (int i = 0; i < 16; ++i) {
    cloudInfo.start_ring_index[i] = count - 1 + 5;
    for (int j = 0; j < 1800; ++j) {
      if (rangeMat.at<float>(i, j) != FLT_MAX) {
        // mark the points' column index for marking occlusion later
        cloudInfo.point_col_index[count] = j;
        // save range info
        cloudInfo.point_range[count] = rangeMat.at<float>(i, j);
        // save extracted cloud
        extractedCloud.push_back(fullCloud[j + i * 1800]);
        // size of extracted cloud
        ++count;
      }
    }
    cloudInfo.end_ring_index[i] = count - 1 - 5;
  }
}

void ImageProjection::resetParameters()
{
  extractedCloud.clear();
  // reset range matrix for range image projection
  rangeMat = cv::Mat(16, 1800, CV_32F, FLT_MAX);
  firstPointFlag = true;
}

}  // namespace loam_mapper::image_projection