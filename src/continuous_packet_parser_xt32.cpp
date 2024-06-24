#include "loam_mapper/continuous_packet_parser_xt32.hpp"

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
  map_byte_to_return_mode_.insert(std::make_pair(59, ReturnMode::DualReturnWithConfidence));

  map_return_mode_to_string_.insert(std::make_pair(ReturnMode::Strongest, "Strongest"));
  map_return_mode_to_string_.insert(std::make_pair(ReturnMode::LastReturn, "LastReturn"));
  map_return_mode_to_string_.insert(std::make_pair(ReturnMode::DualReturn, "DualReturn"));
  map_return_mode_to_string_.insert(
    std::make_pair(ReturnMode::DualReturnWithConfidence, "DualReturnWithConfidence"));

  map_byte_to_hesai_model_.insert(std::make_pair(1, HesaiModel::Pandar128));
  map_byte_to_hesai_model_.insert(std::make_pair(2, HesaiModel::ET));
  map_byte_to_hesai_model_.insert(std::make_pair(3, HesaiModel::QT));
  map_byte_to_hesai_model_.insert(std::make_pair(4, HesaiModel::AT128));
  map_byte_to_hesai_model_.insert(std::make_pair(6, HesaiModel::PandarXT));
  map_byte_to_hesai_model_.insert(std::make_pair(7, HesaiModel::PandarFT));

  map_hesai_model_to_string_.insert(std::make_pair(HesaiModel::Pandar128, "Pandar128"));
  map_hesai_model_to_string_.insert(
    std::make_pair(HesaiModel::ET, "ET"));
  map_hesai_model_to_string_.insert(std::make_pair(HesaiModel::QT, "QT"));
  map_hesai_model_to_string_.insert(
    std::make_pair(HesaiModel::AT128, "AT128"));
  map_hesai_model_to_string_.insert(std::make_pair(HesaiModel::PandarXT, "PandarXT"));
  map_hesai_model_to_string_.insert(std::make_pair(HesaiModel::PandarFT, "PandarFT"));


  channel_to_angle_vertical_ = std::vector<float>{

    // filllll

  };
  assert(channel_to_angle_vertical_.size() == 32);



}
}