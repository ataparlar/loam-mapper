#include "loam_mapper/date.h"
#include "loam_mapper/transform_provider.hpp"
#include "loam_mapper/points_provider.hpp"

#include <boost/math/special_functions/relative_difference.hpp>
#include <boost/filesystem.hpp>

#include <pcapplusplus/RawPacket.h>

#include <cstdint>
#include <cstdlib>
#include <limits>
#include <map>

namespace loam_mapper
{
class LoamMapper
{
public:

  explicit LoamMapper(const loam_mapper::TransformProvider::ConstSharedPtr & transform_provider,
                      const loam_mapper::PointsProvider::ConstSharedPtr & points_provider);

private:


};
}  // namespace loam_mapper