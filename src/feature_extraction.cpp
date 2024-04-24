#include "loam_mapper/feature_extraction.hpp"

namespace loam_mapper::feature_extraction
{
FeatureExtraction::FeatureExtraction()
{
  initializationValue();
}

void FeatureExtraction::initializationValue()
{
  cloudSmoothness.resize(28800);  // 16 * 1800

  //  downSizeFilter.setLeafSize(odometrySurfLeafSize, odometrySurfLeafSize, odometrySurfLeafSize);

  //  extractedCloud.reset(new pcl::PointCloud<PointType>());
  //  cornerCloud.reset(new pcl::PointCloud<PointType>());
  //  surfaceCloud.reset(new pcl::PointCloud<PointType>());

  cloudCurvature = new float[28800];     // 16 * 1800
  cloudNeighborPicked = new int[28800];  // 16 * 1800
  cloudLabel = new int[28800];           // 16 * 1800
}

void FeatureExtraction::laserCloudInfoHandler(
  const Points & deskewed_cloud, utils::Utils::CloudInfo & cloudInfo)
{
  extractedCloud = deskewed_cloud;

  calculateSmoothness(cloudInfo);

  markOccludedPoints(cloudInfo);

  extractFeatures(cloudInfo, 1.0, 0.1);

  freeCloudInfoMemory(cloudInfo);
  //    publishFeatureCloud();
}

void FeatureExtraction::calculateSmoothness(utils::Utils::CloudInfo & cloudInfo)
{
  int cloudSize = extractedCloud.size();
  for (int i = 5; i < cloudSize - 5; i++) {
    float diffRange =
      cloudInfo.point_range[i - 5] + cloudInfo.point_range[i - 4] + cloudInfo.point_range[i - 3] +
      cloudInfo.point_range[i - 2] + cloudInfo.point_range[i - 1] - cloudInfo.point_range[i] * 10 +
      cloudInfo.point_range[i + 1] + cloudInfo.point_range[i + 2] + cloudInfo.point_range[i + 3] +
      cloudInfo.point_range[i + 4] + cloudInfo.point_range[i + 5];

//      if (diffRange != 0) {
//        std::cout << cloudInfo.point_range[i - 5] << std::endl;
//      }

    cloudCurvature[i] = diffRange * diffRange;  // diffX * diffX + diffY * diffY + diffZ * diffZ;

    cloudNeighborPicked[i] = 0;
    cloudLabel[i] = 0;
    // cloudSmoothness for sorting
    cloudSmoothness[i].value = cloudCurvature[i];
    cloudSmoothness[i].ind = i;
  }
}

void FeatureExtraction::markOccludedPoints(utils::Utils::CloudInfo & cloudInfo)
{
  int cloudSize = extractedCloud.size();
  // mark occluded points and parallel beam points
  for (int i = 5; i < cloudSize - 6; ++i) {
    // occluded points
    float depth1 = cloudInfo.point_range[i];
    float depth2 = cloudInfo.point_range[i + 1];
    int columnDiff = std::abs(int(cloudInfo.point_col_index[i + 1] - cloudInfo.point_col_index[i]));
    if (columnDiff < 10) {
      // 10 pixel diff in range image
      if (depth1 - depth2 > 0.3) {
        cloudNeighborPicked[i - 5] = 1;
        cloudNeighborPicked[i - 4] = 1;
        cloudNeighborPicked[i - 3] = 1;
        cloudNeighborPicked[i - 2] = 1;
        cloudNeighborPicked[i - 1] = 1;
        cloudNeighborPicked[i] = 1;
      } else if (depth2 - depth1 > 0.3) {
        cloudNeighborPicked[i + 1] = 1;
        cloudNeighborPicked[i + 2] = 1;
        cloudNeighborPicked[i + 3] = 1;
        cloudNeighborPicked[i + 4] = 1;
        cloudNeighborPicked[i + 5] = 1;
        cloudNeighborPicked[i + 6] = 1;
      }
    }
    // parallel beam
    float diff1 = std::abs(float(cloudInfo.point_range[i - 1] - cloudInfo.point_range[i]));
    float diff2 = std::abs(float(cloudInfo.point_range[i + 1] - cloudInfo.point_range[i]));

    if (diff1 > 0.02 * cloudInfo.point_range[i] && diff2 > 0.02 * cloudInfo.point_range[i])
      cloudNeighborPicked[i] = 1;
  }
}

void FeatureExtraction::extractFeatures(
  utils::Utils::CloudInfo & cloudInfo, float edgeThreshold, float surfaceThreshold)
{
  cornerCloud.clear();
  surfaceCloud.clear();

  Points surfaceCloudScan;
  Points surfaceCloudScanDS;

  for (int i = 0; i < 16; i++) {
    surfaceCloudScan.clear();

    for (int j = 0; j < 6; j++) {
      int sp = (cloudInfo.start_ring_index[i] * (6 - j) + cloudInfo.end_ring_index[i] * j) / 6;
      int ep =
        (cloudInfo.start_ring_index[i] * (5 - j) + cloudInfo.end_ring_index[i] * (j + 1)) / 6 - 1;

      if (sp >= ep) continue;

//      for (auto smooth : cloudSmoothness) {
//        if (smooth.value != 0) {
//          std::cout << smooth.value << std::endl;
//        }
//      }

      std::sort(cloudSmoothness.begin() + sp, cloudSmoothness.begin() + ep, by_value());

      int largestPickedNum = 0;
      for (int k = ep; k >= sp; k--) {
        int ind = cloudSmoothness[k].ind;
        if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > edgeThreshold) {
          largestPickedNum++;
          if (largestPickedNum <= 20) {
            cloudLabel[ind] = 1;
            cornerCloud.push_back(extractedCloud[ind]);
          } else {
            break;
          }

          cloudNeighborPicked[ind] = 1;
          for (int l = 1; l <= 5; l++) {
            int columnDiff = std::abs(
              int(cloudInfo.point_col_index[ind + l] - cloudInfo.point_col_index[ind + l - 1]));
            if (columnDiff > 10) break;
            cloudNeighborPicked[ind + l] = 1;
          }
          for (int l = -1; l >= -5; l--) {
            int columnDiff = std::abs(
              int(cloudInfo.point_col_index[ind + l] - cloudInfo.point_col_index[ind + l + 1]));
            if (columnDiff > 10) break;
            cloudNeighborPicked[ind + l] = 1;
          }
        }
      }

      for (int k = sp; k <= ep; k++) {
        int ind = cloudSmoothness[k].ind;
        if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < surfaceThreshold) {
          cloudLabel[ind] = -1;
          cloudNeighborPicked[ind] = 1;

          for (int l = 1; l <= 5; l++) {
            int columnDiff = std::abs(
              int(cloudInfo.point_col_index[ind + l] - cloudInfo.point_col_index[ind + l - 1]));
            if (columnDiff > 10) break;

            cloudNeighborPicked[ind + l] = 1;
          }
          for (int l = -1; l >= -5; l--) {
            int columnDiff = std::abs(
              int(cloudInfo.point_col_index[ind + l] - cloudInfo.point_col_index[ind + l + 1]));
            if (columnDiff > 10) break;

            cloudNeighborPicked[ind + l] = 1;
          }
        }
      }

      for (int k = sp; k <= ep; k++) {
        if (cloudLabel[k] <= 0) {
          surfaceCloudScan.push_back(extractedCloud[k]);
        }
      }
    }

    surfaceCloudScanDS.clear();
    //    downSizeFilter.setInputCloud(surfaceCloudScan);
    //    downSizeFilter.filter(*surfaceCloudScanDS);

    surfaceCloud.insert(surfaceCloud.end(), surfaceCloudScanDS.begin(), surfaceCloudScanDS.end());
  }
}

void FeatureExtraction::freeCloudInfoMemory(utils::Utils::CloudInfo & cloudInfo)
{
  cloudInfo.start_ring_index.clear();
  cloudInfo.end_ring_index.clear();
  cloudInfo.point_col_index.clear();
  cloudInfo.point_range.clear();
}

}  // namespace loam_mapper::feature_extraction