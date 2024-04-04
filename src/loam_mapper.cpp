
#include "loam_mapper/loam_mapper.hpp"

#include "iostream"
#include "pcapplusplus/PcapFileDevice.h"

#include <pcapplusplus/Packet.h>

namespace loam_mapper
{
LoamMapper::LoamMapper()
{
  pcpp::IFileReaderDevice * reader = pcpp::IFileReaderDevice::getReader(
    "/home/ataparlar/data/task_spesific/loam_based_localization/mapping/pcap_and_poses/pcaps/ytu_campus_00014_20230407211955.pcap");
  if (reader == nullptr) {
    printf("Cannot determine reader for file type\n");
    exit(1);
  }
  if (!reader->open()) {
    printf("Cannot open input.pcap for reading\n");
    exit(1);
  }

  pcpp::RawPacket rawPacket;
  while (reader->getNextPacket(rawPacket)) {

    switch (rawPacket.getFrameLength()) {
      case 554: {  // position packet
        std::cout << "inside 554" << std::endl;
      }
      case 1248: {  // data packet
        std::cout << "inside 1248" << std::endl;
      }
    }

  }

  reader->close();
}
}  // namespace loam_mapper


int main()
{
  loam_mapper::LoamMapper();
  return 0;
}