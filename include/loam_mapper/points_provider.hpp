
#ifndef BUILD_POINTS_PROVIDER_HPP
#define BUILD_POINTS_PROVIDER_HPP

#include "loam_mapper/date.h"

#include <boost/math/special_functions/relative_difference.hpp>
#include <boost/filesystem.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcapplusplus/RawPacket.h>

#include <cstdint>
#include <cstdlib>
#include <limits>
#include <map>


namespace loam_mapper
{
class PointsProvider
{
public:
  using SharedPtr = std::shared_ptr<PointsProvider>;
  using ConstSharedPtr = const SharedPtr;

  explicit PointsProvider(const boost::filesystem::path& pcap_dir);

  std::vector<boost::filesystem::path> paths_pcaps_;

  struct PointXYZIT
  {
    double x;
    double y;
    double z;
    uint32_t intensity {0U};
    uint32_t stamp_unix_seconds{0U};
    uint32_t stamp_nanoseconds{0U};
    friend bool operator==(
      const PointXYZIT & p1,
      const PointXYZIT & p2)
    {
      using boost::math::epsilon_difference;
      return epsilon_difference(p1.x, p2.x) == 0.0F &&
             epsilon_difference(p1.y, p2.y) == 0.0F &&
             epsilon_difference(p1.z, p2.z) == 0.0F &&
             p1.intensity == p2.intensity &&
             p1.stamp_unix_seconds == p2.stamp_unix_seconds &&
             p1.stamp_nanoseconds == p2.stamp_nanoseconds;
    }
  } __attribute__((packed));

  std::vector<PointXYZIT> instant_cloud_;
  std::vector<PointXYZIT> cloud_;

  void process_pcap(const boost::filesystem::path & pcap_path);
  void process_packet(const pcpp::RawPacket & rawPacket);

private:



  void process_pcaps(const std::vector<boost::filesystem::path> & paths_pcaps);

  enum class ReturnMode { Strongest, LastReturn, DualReturn, DualReturnWithConfidence };

  enum class VelodyneModel {
    HDL32E,
    VLP16orPuckLITE,
    PuckHiRes,
    VLP32CorVLP32MR,
    Velarray,
    VLS128
  };



  struct DataPoint
  {
    uint16_t distance_divided_by_2mm;
    uint8_t reflectivity;
  } __attribute__((packed));

  struct DataBlock
  {
    uint8_t flag_1;
    uint8_t flag_2;
    uint16_t azimuth_multiplied_by_100_deg;
    DataPoint data_points[32];
    size_t get_size_data_points() const { return sizeof(data_points) / sizeof(data_points[0]); }
  } __attribute__((packed));

  struct DataPacket
  {
    uint8_t udp_header[42];
    DataBlock data_blocks[12];
    uint32_t microseconds_toh;
    uint8_t factory_byte_return_mode;
    uint8_t factory_byte_product_id;
    [[nodiscard]] size_t get_size_data_blocks() const
    {
      return sizeof(data_blocks) / sizeof(data_blocks[0]);
    }
  } __attribute__((packed));

  struct PositionPacket
  {
    uint8_t udp_header[42];
    uint8_t reserved_01[187];
    uint8_t temp_top_board;
    uint8_t temp_bot_board;
    uint8_t reserved_02[9];
    uint32_t timestamp_microseconds_since_hour;
    uint8_t status_pps;
    uint8_t status_thermal;
    uint8_t temp_when_last_shut_from_overheat;
    uint8_t temp_when_booted;
    char nmea_sentence[128];
    uint8_t reserved_03[178];
  } __attribute__((packed));

  date::sys_time<std::chrono::hours> tp_hours_since_epoch;
  bool factory_bytes_are_read_at_least_once_;
  bool has_received_valid_position_package_;

  VelodyneModel velodyne_model_;
  ReturnMode return_mode_;

  std::map<uint8_t, ReturnMode> map_byte_to_return_mode_;
  std::map<ReturnMode, std::string> map_return_mode_to_string_;
  std::map<uint8_t, VelodyneModel> map_byte_to_velodyne_model_;
  std::map<VelodyneModel, std::string> map_velodyne_model_to_string_;

  std::vector<float> channel_to_angle_vertical_;
  std::vector<float> channel_mod_8_to_azimuth_offsets_;
  std::vector<size_t> ind_block_to_first_channel_;

  bool has_processed_a_packet_;
  float angle_deg_azimuth_last_packet_;
  uint32_t microseconds_last_packet_;
};
}  // namespace loam_mapper




#endif  // BUILD_POINTS_PROVIDER_HPP
