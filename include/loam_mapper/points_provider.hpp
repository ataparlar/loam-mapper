

#include <boost/filesystem.hpp>
#include <string>
#include <memory>
#include <vector>
#include <map>
#include <functional>
#include "points_provider_base.hpp"
#include "point_types.hpp"
#include "date.h"
#include "continuous_packet_parser.hpp"
#include "continuous_packet_parser_xt32.hpp"

namespace loam_mapper::points_provider
{
namespace fs = boost::filesystem;
class PointsProvider : public virtual PointsProviderBase
{
public:
  using SharedPtr = std::shared_ptr<PointsProvider>;
  using ConstSharedPtr = const SharedPtr;

  explicit PointsProvider( std::string  path_folder_pcaps);

  void process() override;

  template <typename parser_type>
  void process_pcaps_into_clouds(
    std::function<void(const Points &)> & callback_cloud_surround_out,
    size_t index_start,
    size_t count);
  std::string info() override;

  template <typename parser_type>
  void process_pcap_into_clouds(
    const fs::path & path_pcap,
    const std::function<void(const Points &)>& callback_cloud_surround_out,
    parser_type& parser);

  std::vector<fs::path> paths_pcaps_;

private:

  fs::path path_folder_pcaps_;
};
}  // namespace loam_mapper::points_provider
