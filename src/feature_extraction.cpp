#include "loam_mapper/feature_extraction.hpp"
#include "loam_mapper/utils.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

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

  extractedCloud.clear();
  cornerCloud.clear();
  surfaceCloud.clear();

  cloudCurvature = new float[28800];     // 16 * 1800
  cloudNeighborPicked = new int[28800];  // 16 * 1800
  cloudLabel = new int[28800];           // 16 * 1800
}

void FeatureExtraction::laserCloudInfoHandler(
  const Points & deskewed_cloud, utils::Utils::CloudInfo & cloudInfo)
{
//      std::cout << "image_projection->cloudInfo.start_ring_index: " << cloudInfo.start_ring_index.size() << std::endl;
//  for (auto a : cloudInfo.point_col_index){
//    std::cout << "a: " << a << std::endl;
//  }
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
//  std::cout << "cloudSize: " << cloudSize << std::endl;
//  int a_counter = 0;
  for (int i = 5; i < cloudSize - 5; i++) {
    float diffRange =
      cloudInfo.point_range[i - 5] + cloudInfo.point_range[i - 4] + cloudInfo.point_range[i - 3] +
      cloudInfo.point_range[i - 2] + cloudInfo.point_range[i - 1] - cloudInfo.point_range[i] * 10 +
      cloudInfo.point_range[i + 1] + cloudInfo.point_range[i + 2] + cloudInfo.point_range[i + 3] +
      cloudInfo.point_range[i + 4] + cloudInfo.point_range[i + 5];


//    std::cout << "cloudInfo.point_range[i - 5]: " << cloudInfo.point_range[i - 5] << std::endl;
//    std::cout << "cloudInfo.point_range[i - 2]: " << cloudInfo.point_range[i - 2] << std::endl;
//    std::cout << "cloudInfo.point_range[i + 1]: " << cloudInfo.point_range[i + 1] << std::endl;
//    std::cout << "cloudInfo.point_range[i + 4]: " << cloudInfo.point_range[i + 4] << std::endl;
//
//    std::cout << "cloudInfo.point_range[i - 4]: " << cloudInfo.point_range[i - 4] << std::endl;
//    std::cout << "cloudInfo.point_range[i - 1]: " << cloudInfo.point_range[i - 1] << std::endl;
//    std::cout << "cloudInfo.point_range[i + 2]: " << cloudInfo.point_range[i + 2] << std::endl;
//    std::cout << "cloudInfo.point_range[i + 5]: " << cloudInfo.point_range[i + 5] << std::endl;
//
//    std::cout << "cloudInfo.point_range[i - 3]: " << cloudInfo.point_range[i - 3] << std::endl;
//    std::cout << "cloudInfo.point_range[i] * 10: " << cloudInfo.point_range[i] * 10 << std::endl;
//    std::cout << "cloudInfo.point_range[i + 3]: " << cloudInfo.point_range[i + 3] << std::endl;
//    std::cout << "cloudInfo.point_range[i]: " << cloudInfo.point_range[i] << std::endl;
//    std::cout << "\n " << std::endl;

//    if (cloudInfo.point_range[i] != 0) {
//      a_counter++;
//    }



    //      if (diffRange != 0) {
    //        std::cout << cloudInfo.point_range[i - 5] << std::endl;
    //      }

    cloudCurvature[i] = diffRange * diffRange;  // diffX * diffX + diffY * diffY + diffZ * diffZ;

    cloudNeighborPicked[i] = 0;
    cloudLabel[i] = 0;
    // cloudSmoothness for sorting
    cloudSmoothness[i].value = cloudCurvature[i];
    cloudSmoothness[i].ind = i;

//    std::cout << "cloudSmoothness[i].value:  " << cloudSmoothness[i].value << std::endl;


  }
  //  std::cout << "a_counter: " << a_counter << std::endl;
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
//    std::cout << "cloudInfo.start_ring_index[i]: " << cloudInfo.start_ring_index[i] << std::endl;
//    std::cout << "cloudInfo.point_col_index[i+1]: " << cloudInfo.point_col_index[i+1] << std::endl;
//    std::cout << "cloudInfo.point_col_index[i]: " << cloudInfo.point_col_index[i] << std::endl;
//    if (columnDiff > 1) {
//      std::cout << "columnDiff: " << columnDiff << std::endl;
//    }
//    std::cout << "\n\n" << std::endl;


    if (columnDiff < 10) {
      // 10 pixel diff in range image
      if (depth1 - depth2 > 0.3) {
//        std::cout << "cloudOccludedNot Left: " << depth1 << " - " << depth2 << " = " << depth1 - depth2 << std::endl;
//        std::cout << "\ncloudInfo.point_col_index[i]: " << cloudInfo.point_col_index[i] << std::endl;
//        std::cout << "cloudInfo.start_ring_index[i]: " << cloudInfo.start_ring_index[i] << std::endl;
//        std::cout << "\ncloudInfo.point_col_index[i+1]: " << cloudInfo.point_col_index[i+1] << std::endl;
//        std::cout << "cloudInfo.start_ring_index[i+1]: " << cloudInfo.start_ring_index[i+1] << std::endl;
//        std::cout << "\ncolumnDiff: " << columnDiff << std::endl;
//        std::cout << "\n\n" << std::endl;

        cloudNeighborPicked[i - 5] = 1;
        cloudNeighborPicked[i - 4] = 1;
        cloudNeighborPicked[i - 3] = 1;
        cloudNeighborPicked[i - 2] = 1;
        cloudNeighborPicked[i - 1] = 1;
        cloudNeighborPicked[i] = 1;
      } else if (depth2 - depth1 > 0.3) {
//        std::cout << "cloudOccludedNot Right: " << depth2 << " - " << depth1 << " = " << depth2 - depth1 << std::endl;
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


  cloudPath.header.stamp = loam_mapper::utils::Utils::get_time();
  cloudPath.header.frame_id = "map";



  for (int i = 0; i < 16; i++) {
    surfaceCloudScan.clear();

    for (int j = 0; j < 6; j++) {




      int sp = (cloudInfo.start_ring_index[i] * (6 - j) + cloudInfo.end_ring_index[i] * j) / 6;
      int ep =
        (cloudInfo.start_ring_index[i] * (5 - j) + cloudInfo.end_ring_index[i] * (j + 1)) / 6 - 1;

//      std::cout << "j: " << j << "\ti: " << i << std::endl;
//      std::cout << "sp: " << sp << "\tep: " << ep << std::endl;
//      std::cout << "cloudInfo.start_ring_index[i]: " << cloudInfo.start_ring_index[i] <<
//        "\tcloudInfo.end_ring_index[i]: " << cloudInfo.end_ring_index[i] << std::endl;


      if (sp >= ep) continue;

      std::sort(cloudSmoothness.begin() + sp, cloudSmoothness.begin() + ep, by_value());

      int largestPickedNum = 0;
      for (int k = ep; k >= sp; k--) {  // find edge points

        int ind = cloudSmoothness[k].ind;


        geometry_msgs::msg::PoseStamped pose_;
        pose_.header = cloudPath.header;
        pose_.pose.position.x = extractedCloud[ind].x;
        pose_.pose.position.y = extractedCloud[ind].y;
        pose_.pose.position.z = extractedCloud[ind].z;
        cloudPath.poses.push_back(pose_);

          //        std::cout << "cloudNeighborPicked[ind]: " << cloudNeighborPicked[ind] << "\tcloudCurvature[ind]" << cloudCurvature[ind] << std::endl;




        if (cloudNeighborPicked[ind] == 0) {
          cloudOccludedNot.push_back(extractedCloud[ind]);
        } else {
          cloudOccluded.push_back(extractedCloud[ind]);
        }




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



      for (int k = sp; k <= ep; k++) {  // find surface points
        int ind = cloudSmoothness[k].ind;


//        if (cloudNeighborPicked[ind] == 0) {
//          cloudOccludedNot.push_back(extractedCloud[ind]);
//        } else {
//          cloudOccluded.push_back(extractedCloud[ind]);
//        }



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

    //    downSizeFilter.setInputCloud(surfaceCloudScan);
    //    downSizeFilter.filter(*surfaceCloudScanDS);

    surfaceCloud.insert(surfaceCloud.end(), surfaceCloudScan.begin(), surfaceCloudScan.end());
    surfaceCloudScanDS.clear();

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