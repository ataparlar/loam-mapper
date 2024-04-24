//
// Created by ataparlar on 22.04.2024.
//

#ifndef BUILD_POINTS_PROVIDER_BASE_HPP
#define BUILD_POINTS_PROVIDER_BASE_HPP

#include <cstdint>
#include <vector>
#include <string>
#include "point_types.hpp"

namespace loam_mapper::points_provider
{
class PointsProviderBase
{
public:
  using Point = point_types::PointXYZITRH;
  using Points = std::vector<Point>;
  virtual void process() = 0;
  //  virtual bool get_next_cloud(std::vector<Points> & cloud_out) = 0;
  virtual std::string info() = 0;
};
}  // namespace loam_mapper::points_provider


#endif  // BUILD_POINTS_PROVIDER_BASE_HPP
