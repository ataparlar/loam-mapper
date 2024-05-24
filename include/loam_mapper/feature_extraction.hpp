#ifndef BUILD_FEATURE_EXTRACTION_HPP
#define BUILD_FEATURE_EXTRACTION_HPP

#include "points_provider_base.hpp"
#include "rclcpp/rclcpp.hpp"
#include "utils.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/path.hpp>

#include <boost/filesystem.hpp>

#include <deque>
#include <memory>
#include <string>
#include <iostream>
#include <fstream>

namespace loam_mapper::feature_extraction
{
namespace fs = boost::filesystem;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using Point = points_provider::PointsProviderBase::Point;
using Points = points_provider::PointsProviderBase::Points;

struct smoothness_t
{
  float value;
  size_t ind;
};

struct by_value
{
  bool operator()(smoothness_t const & left, smoothness_t const & right)
  {
    return left.value < right.value;
  }
};

class FeatureExtraction
{
public:
  using SharedPtr = std::shared_ptr<FeatureExtraction>;
  using ConstSharedPtr = const SharedPtr;

  explicit FeatureExtraction();

  Points cloudOccluded;
  Points cloudOccludedNot;
  Points extractedCloud;
  Points cornerCloud;
  Points surfaceCloud;
  nav_msgs::msg::Path cloudPath;

  int counter = 0;
  std::ofstream file_point_range;
  std::ofstream file_start_ring_ind;
  std::ofstream file_end_ring_ind;
  std::ofstream file_point_col_ind;
  std::ofstream file_cloud_smoothness;
  std::ofstream file_cloud_neighbor;

  std::vector<smoothness_t> cloudSmoothness;
  float * cloudCurvature;
  int * cloudNeighborPicked;
  int * cloudLabel;

  void initializationValue();
  void laserCloudInfoHandler(const Points & deskewed_cloud, utils::Utils::CloudInfo & cloudInfo);
  void calculateSmoothness(utils::Utils::CloudInfo & cloudInfo);
  void markOccludedPoints(utils::Utils::CloudInfo & cloudInfo);
  void extractFeatures(
    utils::Utils::CloudInfo & cloudInfo, float edgeThreshold, float surfaceThreshold);
  void freeCloudInfoMemory(utils::Utils::CloudInfo & cloudInfo);
};
}  // namespace loam_mapper::feature_extraction

#endif  // BUILD_FEATURE_EXTRACTION_HPP
