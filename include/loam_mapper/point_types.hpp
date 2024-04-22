#ifndef BUILD_POINT_TYPES_HPP
#define BUILD_POINT_TYPES_HPP


#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

typedef pcl::PointXYZI PointType;
//typedef pcl::PointXYZRGBNormal PointType;
typedef pcl::PointCloud<PointType> CloudType;
typedef CloudType::Ptr CloudPtrType;
typedef CloudType::ConstPtr CloudConstPtrType;
typedef pcl::KdTreeFLANN<PointType> KdTreeType;
typedef KdTreeType::Ptr KdTreePtrType;

typedef pcl::PointXYZHSV PointColorType;
typedef pcl::PointCloud<PointColorType> CloudColorType;
typedef CloudColorType::Ptr CloudColorPtrType;


#endif  // BUILD_POINT_TYPES_HPP
