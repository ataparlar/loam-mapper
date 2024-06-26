#include "loam_mapper/continuous_packet_parser_xt32.hpp"

#include "loam_mapper/utils.hpp"

#include <iostream>

namespace loam_mapper::points_provider::continuous_packet_parser
{
ContinuousPacketParserXt32::ContinuousPacketParserXt32()
: factory_bytes_are_read_at_least_once_{false},
  has_received_valid_position_package_{false},
  has_processed_a_packet_{false},
  angle_deg_azimuth_last_packet_{0.0f},
  microseconds_last_packet_{0U},
  can_publish_again_{true},
  angle_deg_cut_{90.0f}
{
  map_byte_to_return_mode_.insert(std::make_pair(55, ReturnMode::Strongest));
  map_byte_to_return_mode_.insert(std::make_pair(56, ReturnMode::LastReturn));
  map_byte_to_return_mode_.insert(std::make_pair(57, ReturnMode::DualReturn));
  //  map_byte_to_return_mode_.insert(std::make_pair(59, ReturnMode::DualReturnWithConfidence));

  map_return_mode_to_string_.insert(std::make_pair(ReturnMode::Strongest, "Strongest"));
  map_return_mode_to_string_.insert(std::make_pair(ReturnMode::LastReturn, "LastReturn"));
  map_return_mode_to_string_.insert(std::make_pair(ReturnMode::DualReturn, "DualReturn"));
  //  map_return_mode_to_string_.insert(
  //    std::make_pair(ReturnMode::DualReturnWithConfidence, "DualReturnWithConfidence"));

  map_byte_to_hesai_model_.insert(std::make_pair(1, HesaiModel::Pandar128));
  map_byte_to_hesai_model_.insert(std::make_pair(2, HesaiModel::ET));
  map_byte_to_hesai_model_.insert(std::make_pair(3, HesaiModel::QT));
  map_byte_to_hesai_model_.insert(std::make_pair(4, HesaiModel::AT128));
  map_byte_to_hesai_model_.insert(std::make_pair(6, HesaiModel::PandarXT));
  map_byte_to_hesai_model_.insert(std::make_pair(7, HesaiModel::PandarFT));

  map_hesai_model_to_string_.insert(std::make_pair(HesaiModel::Pandar128, "Pandar128"));
  map_hesai_model_to_string_.insert(std::make_pair(HesaiModel::ET, "ET"));
  map_hesai_model_to_string_.insert(std::make_pair(HesaiModel::QT, "QT"));
  map_hesai_model_to_string_.insert(std::make_pair(HesaiModel::AT128, "AT128"));
  map_hesai_model_to_string_.insert(std::make_pair(HesaiModel::PandarXT, "PandarXT"));
  map_hesai_model_to_string_.insert(std::make_pair(HesaiModel::PandarFT, "PandarFT"));

  channel_to_angle_vertical_ = std::vector<float>{
    15.0F, 14.0F, 13.0F, 12.0F,  11.0F,  10.0F,  9.0F,   8.0F,   7.0F,  6.0F,  5.0F,
    4.0F,  3.0F,  2.0F,  1.0F,   0.0F,   -1.0F,  -2.0F,  -3.0F,  -4.0F, -5.0F, -6.0F,
    -7.0F, -8.0F, -9.0F, -10.0F, -11.0F, -12.0F, -13.0F, -14.0F, -15.0F};
  assert(channel_to_angle_vertical_.size() == 32);
}

void ContinuousPacketParserXt32::process_packet_into_cloud(
  const pcpp::RawPacket & rawPacket,
  const std::function<void(const Points &)> & callback_cloud_surround_out)
{
  switch (rawPacket.getFrameLength()) {
    case 554: {
      if (has_received_valid_position_package_) {
        break;
      }

      auto * position_packet = reinterpret_cast<const PositionPacket *>(rawPacket.getRawData());

      std::string nmea_sentence(position_packet->nmea_sentence);
      auto segments_with_nullstuff = utils::Utils::string_to_vec_split_by(nmea_sentence, '\r');

      auto segments_with_crc =
        utils::Utils::string_to_vec_split_by(segments_with_nullstuff.front(), '*');
      auto segments = utils::Utils::string_to_vec_split_by(segments_with_crc.front(), ',');

      if (13 > segments.size() || 14 < segments.size()) {
        throw std::length_error(
          "nmea sentence should have 13 elements, it has " + std::to_string(segments.size()));
      }

      // Receiver status: A= Active, V= Void
      if (segments.at(2) != "A") {
        std::cout << "Receiver Status != Active" << std::endl;
        break;
      }
      const auto & str_time = segments.at(1);
      int hours_raw = std::stoi(str_time.substr(0, 2));

      const auto & str_date = segments.at(9);
      int days_raw = std::stoi(str_date.substr(0, 2));
      int months_raw = std::stoi(str_date.substr(2, 2));
      int years_raw = 2000 + std::stoi(str_date.substr(4, 2));

      date::year_month_day date_current_ = date::year{years_raw} / months_raw / days_raw;
      tp_hours_since_epoch = date::sys_days(date_current_) + std::chrono::hours(hours_raw);
      uint32_t seconds_epoch_precision_of_hour =
        std::chrono::seconds(tp_hours_since_epoch.time_since_epoch()).count();

      for (auto segment : segments) {
        std::cout << segment << std::endl;
      }

      has_received_valid_position_package_ = true;
      break;
    }
    case 1122: {
      if (!has_received_valid_position_package_) {
        // Ignore until first valid Position Packet is received
        break;
      }

      const auto * data_packet_with_header =
        reinterpret_cast<const DataPacket *>(rawPacket.getRawData());


      std::cout
        << "distance: "
        << static_cast<float>(data_packet_with_header->body.data_block[5].data_points[20].distance_divided_by_4mm * 4)  / 1000.0f
        << std::endl;
      std::cout
        << "timestamp: "
        << data_packet_with_header->tail.timestamp
        << std::endl;

      auto year = data_packet_with_header->tail.year;
      auto month = data_packet_with_header->tail.month;
      auto day = data_packet_with_header->tail.day;
      std::cout
        << "date: "
        << utils::Utils::byte_hex_to_string(year) << "/"
        << utils::Utils::byte_hex_to_string(month) << "/"
        << utils::Utils::byte_hex_to_string(day)
        << std::endl;


    }
  }
}

}  // namespace loam_mapper::points_provider::continuous_packet_parser