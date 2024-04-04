#include <cstdint>
#include <cstdlib>

namespace loam_mapper
{
class LoamMapper
{
public:
  explicit LoamMapper();

private:
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
};
}  // namespace loam_mapper