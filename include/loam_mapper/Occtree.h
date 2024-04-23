
#ifndef BUILD_OCCTREE_H
#define BUILD_OCCTREE_H

#include "loam_mapper/point_types.hpp"

#include <pcl/octree/octree_search.h>

#include <vector>

class Occtree
{
private:
  typedef pcl::PointXYZI PointType;
  typedef pcl::PointCloud<PointType> CloudType;
  typedef CloudType::Ptr CloudPtrType;
  typedef CloudType::ConstPtr CloudConstPtrType;
  typedef pcl::octree::OctreePointCloudSearch<PointType> OctreeType;
  typedef OctreeType::Ptr OctreePtrType;

public:
  CloudPtrType cloud;
  OctreePtrType octree;
  float resolution;
  typedef std::shared_ptr<Occtree> Ptr;

  explicit Occtree(float res)
  {
    resolution = res;
    cloud.reset(new CloudType());
    octree.reset(new OctreeType(res));
    octree->setInputCloud(cloud);
    octree->addPointsFromInputCloud();
  }

  void addPointsFromCloud(CloudConstPtrType cloudIn)
  {
    for (const auto & point : cloudIn->points) {
      addPoint(point);
    }
  }

  void addPointsFromCloudIfVoxelEmpty(const CloudConstPtrType & cloudIn)
  {
    for (const auto & point : cloudIn->points) {
      addPointIfVoxelEmpty(point);
    }
  }

  void addPoint(const PointType & point) { octree->addPointToCloud(point, cloud); }

  void addPointIfVoxelEmpty(const PointType & point)
  {
    if (!octree->isVoxelOccupiedAtPoint(point)) {
      addPoint(point);
    }
  }

  CloudPtrType box(PointType pt_search, float width)
  {
    float box_radius = width / 2;
    Eigen::Vector3f pt_min(
      pt_search.x - box_radius, pt_search.y - box_radius, pt_search.z - box_radius);
    Eigen::Vector3f pt_max(
      pt_search.x + box_radius, pt_search.y + box_radius, pt_search.z + box_radius);

    CloudPtrType cloud_box(new CloudType());
    std::vector<int> k_indices;
    if (octree->boxSearch(pt_min, pt_max, k_indices) > 0) {
      for (const int & k_indice : k_indices) cloud_box->points.push_back(cloud->points[k_indice]);
    }
    return cloud_box;
  }
};

#endif  // BUILD_OCCTREE_H
