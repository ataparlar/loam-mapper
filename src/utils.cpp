
#include "loam_mapper/utils.hpp"

#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace loam_mapper::utils
{

std::string Utils::byte_hex_to_string(uint8_t byte_hex)
{
  std::stringstream ss;
  ss << std::hex << std::setfill('0');
  ss << "data_packet_with_header: " << std::setw(2) << (int)byte_hex;
  return ss.str();
}

std::string Utils::bytes_hexes_to_string(const std::vector<uint8_t> & bytes_hexes)
{
  std::string output;
  for (const auto & byte_hex : bytes_hexes) {
    output += byte_hex_to_string(byte_hex) + " ";
  }
  if (output.empty()) {
    throw std::length_error("output.empty()");
  }
  output.erase(output.end() - 1, output.end());
  return output;
}
std::vector<std::string> Utils::string_to_vec_split_by(const std::string & input, char splitter)
{
  std::stringstream ss_input(input);
  std::string segment;
  std::vector<std::string> seglist;
  while (std::getline(ss_input, segment, splitter)) {
    seglist.push_back(segment);
  }
  return seglist;
}
}  // namespace loam_mapper::utils