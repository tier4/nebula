// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <nebula_core_common/point_types.hpp>
#include <nebula_seyond_common/seyond_calibration_data.hpp>
#include <nebula_seyond_common/seyond_common.hpp>
#include <nebula_seyond_common/seyond_configuration.hpp>
#include <nebula_seyond_decoders/seyond_decoder.hpp>

#include <netinet/in.h>
#include <netinet/ip.h>
#include <netinet/udp.h>
#include <pcap.h>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace
{
using nebula::drivers::ReturnMode;
using nebula::drivers::seyond_sensor_model_from_string;
using nebula::drivers::SeyondCalibrationData;
using nebula::drivers::SeyondConnectionConfiguration;
using nebula::drivers::SeyondDecoder;
using nebula::drivers::SeyondReflectanceMode;
using nebula::drivers::SeyondSensorConfiguration;
using nebula::drivers::SeyondSensorModel;
using nebula::drivers::SeyondSyncMode;

struct Stats
{
  size_t ip_packets{0};
  size_t udp_datagrams{0};
  size_t decoded_packets{0};
  size_t decoded_clouds{0};
  size_t decoded_points{0};
  bool saw_first_packet{false};
  uint16_t first_magic{0};
  uint32_t first_size{0};
  uint8_t first_lidar_type{0};
  uint8_t first_item_type{0};
  uint32_t first_item_number{0};
  uint16_t first_item_size{0};
  float min_x{std::numeric_limits<float>::max()};
  float min_y{std::numeric_limits<float>::max()};
  float min_z{std::numeric_limits<float>::max()};
  float max_x{std::numeric_limits<float>::lowest()};
  float max_y{std::numeric_limits<float>::lowest()};
  float max_z{std::numeric_limits<float>::lowest()};
};

struct ReassemblyKey
{
  uint32_t src;
  uint32_t dst;
  uint16_t id;
  uint8_t protocol;

  bool operator<(const ReassemblyKey & other) const
  {
    return std::tie(src, dst, id, protocol) <
           std::tie(other.src, other.dst, other.id, other.protocol);
  }
};

struct FragmentAssembly
{
  bool saw_first_fragment{false};
  bool saw_last_fragment{false};
  uint16_t source_port{0};
  uint16_t dest_port{0};
  size_t expected_udp_payload_size{0};
  size_t expected_total_ip_payload_size{0};
  std::vector<uint8_t> udp_payload{};
  std::vector<bool> received{};

  void ensure_capacity(size_t size)
  {
    if (udp_payload.size() < size) {
      udp_payload.resize(size);
      received.resize(size, false);
    }
  }

  bool is_complete() const
  {
    return saw_first_fragment && saw_last_fragment && expected_udp_payload_size > 0 &&
           udp_payload.size() >= expected_udp_payload_size &&
           std::all_of(
             received.begin(),
             received.begin() + static_cast<std::ptrdiff_t>(expected_udp_payload_size),
             [](bool v) { return v; });
  }
};

SeyondSensorConfiguration make_config(const std::string & sensor_model)
{
  SeyondSensorConfiguration config{};
  config.sensor_model = seyond_sensor_model_from_string(sensor_model);
  if (config.sensor_model == SeyondSensorModel::UNKNOWN) {
    throw std::runtime_error("Unknown Seyond sensor model: " + sensor_model);
  }
  config.connection = SeyondConnectionConfiguration{"0.0.0.0", "0.0.0.0", "", "", 0, 0, 0};
  config.frame_id = "seyond";
  config.setup_sensor = false;
  config.return_mode = ReturnMode::STRONGEST;
  config.reflectance_mode = SeyondReflectanceMode::REFLECTIVITY;
  config.sync_mode = SeyondSyncMode::HOST;
  return config;
}

void update_cloud_stats(const nebula::drivers::NebulaPointCloudPtr & cloud, Stats & stats)
{
  stats.decoded_clouds++;
  stats.decoded_points += cloud->size();
  for (const auto & point : *cloud) {
    stats.min_x = std::min(stats.min_x, point.x);
    stats.min_y = std::min(stats.min_y, point.y);
    stats.min_z = std::min(stats.min_z, point.z);
    stats.max_x = std::max(stats.max_x, point.x);
    stats.max_y = std::max(stats.max_y, point.y);
    stats.max_z = std::max(stats.max_z, point.z);
  }
}

bool process_ipv4_packet(
  const uint8_t * data, size_t size, std::map<ReassemblyKey, FragmentAssembly> & assemblies,
  SeyondDecoder & decoder, Stats & stats)
{
  constexpr size_t kEthernetHeaderSize = 14;
  if (size <= kEthernetHeaderSize) {
    return false;
  }

  const auto * ip_header = reinterpret_cast<const ip *>(data + kEthernetHeaderSize);
  if (ip_header->ip_v != 4 || ip_header->ip_p != IPPROTO_UDP) {
    return false;
  }

  const size_t ip_header_size = static_cast<size_t>(ip_header->ip_hl) * 4;
  const size_t total_ip_size = ntohs(ip_header->ip_len);
  if (
    ip_header_size < sizeof(ip) || kEthernetHeaderSize + total_ip_size > size ||
    total_ip_size < ip_header_size) {
    return false;
  }

  const size_t ip_payload_size = total_ip_size - ip_header_size;
  const uint8_t * ip_payload = reinterpret_cast<const uint8_t *>(ip_header) + ip_header_size;
  const uint16_t ip_off = ntohs(ip_header->ip_off);
  const bool more_fragments = (ip_off & IP_MF) != 0;
  const size_t fragment_offset = static_cast<size_t>(ip_off & IP_OFFMASK) * 8;

  ReassemblyKey key{
    ip_header->ip_src.s_addr, ip_header->ip_dst.s_addr, ntohs(ip_header->ip_id), ip_header->ip_p};
  auto & assembly = assemblies[key];

  if (fragment_offset == 0) {
    if (ip_payload_size < sizeof(udphdr)) {
      return false;
    }

    const auto * udp_header = reinterpret_cast<const udphdr *>(ip_payload);
    assembly.saw_first_fragment = true;
    assembly.source_port = ntohs(udp_header->uh_sport);
    assembly.dest_port = ntohs(udp_header->uh_dport);
    assembly.expected_udp_payload_size = ntohs(udp_header->uh_ulen) - sizeof(udphdr);
    assembly.expected_total_ip_payload_size = ntohs(udp_header->uh_ulen);
    assembly.ensure_capacity(assembly.expected_udp_payload_size);

    const size_t fragment_udp_data_size = ip_payload_size - sizeof(udphdr);
    const uint8_t * fragment_udp_data = ip_payload + sizeof(udphdr);
    std::copy(
      fragment_udp_data, fragment_udp_data + fragment_udp_data_size, assembly.udp_payload.begin());
    std::fill(
      assembly.received.begin(),
      assembly.received.begin() + static_cast<std::ptrdiff_t>(fragment_udp_data_size), true);
  } else {
    if (fragment_offset < sizeof(udphdr)) {
      return false;
    }
    const size_t udp_data_offset = fragment_offset - sizeof(udphdr);
    const size_t fragment_udp_data_size = ip_payload_size;
    assembly.ensure_capacity(udp_data_offset + fragment_udp_data_size);
    std::copy(
      ip_payload, ip_payload + fragment_udp_data_size,
      assembly.udp_payload.begin() + static_cast<std::ptrdiff_t>(udp_data_offset));
    std::fill(
      assembly.received.begin() + static_cast<std::ptrdiff_t>(udp_data_offset),
      assembly.received.begin() +
        static_cast<std::ptrdiff_t>(udp_data_offset + fragment_udp_data_size),
      true);
  }

  if (!more_fragments) {
    assembly.saw_last_fragment = true;
  }

  if (!assembly.is_complete()) {
    return false;
  }

  stats.udp_datagrams++;
  if (
    !stats.saw_first_packet &&
    assembly.udp_payload.size() >= sizeof(nebula::drivers::SeyondDataPacket)) {
    const auto * packet =
      reinterpret_cast<const nebula::drivers::SeyondDataPacket *>(assembly.udp_payload.data());
    stats.saw_first_packet = true;
    stats.first_magic = packet->common.magic_number;
    stats.first_size = packet->common.size;
    stats.first_lidar_type = packet->common.lidar_type;
    stats.first_item_type = packet->type;
    stats.first_item_number = packet->item_number;
    stats.first_item_size = packet->item_size;
  }
  if (assembly.dest_port == 2372) {
    auto result = decoder.unpack(assembly.udp_payload);
    if (result.points_unpacked > 0) {
      stats.decoded_packets++;
    }
  }
  assemblies.erase(key);
  return true;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 4) {
    std::cerr << "Usage: " << argv[0] << " <sensor_model> <pcap_file> <anglehv_table.bin>"
              << std::endl;
    return 1;
  }

  const std::string sensor_model = argv[1];
  const std::string pcap_path = argv[2];
  const std::string anglehv_path = argv[3];

  auto calibration_or_error = SeyondCalibrationData::load_from_file(anglehv_path);
  if (!calibration_or_error.has_value()) {
    throw std::runtime_error(calibration_or_error.error().message);
  }
  SeyondCalibrationData calibration = calibration_or_error.value();

  Stats stats{};
  auto config = make_config(sensor_model);
  SeyondDecoder decoder(
    config,
    [&stats](nebula::drivers::NebulaPointCloudPtr cloud, uint64_t) {
      update_cloud_stats(cloud, stats);
    },
    calibration);

  char errbuf[PCAP_ERRBUF_SIZE]{};
  pcap_t * pcap = pcap_open_offline(pcap_path.c_str(), errbuf);
  if (pcap == nullptr) {
    std::cerr << "Error opening pcap: " << errbuf << std::endl;
    return 1;
  }

  std::map<ReassemblyKey, FragmentAssembly> assemblies{};
  struct pcap_pkthdr * header = nullptr;
  const u_char * data = nullptr;

  while (true) {
    const int rc = pcap_next_ex(pcap, &header, &data);
    if (rc == -2) {
      break;
    }
    if (rc < 0) {
      std::cerr << "pcap_next_ex failed: " << pcap_geterr(pcap) << std::endl;
      pcap_close(pcap);
      return 1;
    }
    if (rc == 0) {
      continue;
    }

    stats.ip_packets++;
    process_ipv4_packet(data, header->caplen, assemblies, decoder, stats);
  }

  pcap_close(pcap);

  std::cout << "sensor_model=" << sensor_model << '\n';
  std::cout << "pcap=" << pcap_path << '\n';
  std::cout << "anglehv_table_bytes=" << calibration.angle_hv_table.size() << '\n';
  std::cout << "ip_packets=" << stats.ip_packets << '\n';
  std::cout << "udp_datagrams_reassembled=" << stats.udp_datagrams << '\n';
  if (stats.saw_first_packet) {
    std::cout << "first_packet.magic=0x" << std::hex << stats.first_magic << std::dec << '\n';
    std::cout << "first_packet.size=" << stats.first_size << '\n';
    std::cout << "first_packet.lidar_type=" << static_cast<unsigned>(stats.first_lidar_type)
              << '\n';
    std::cout << "first_packet.item_type=" << static_cast<unsigned>(stats.first_item_type) << '\n';
    std::cout << "first_packet.item_number=" << stats.first_item_number << '\n';
    std::cout << "first_packet.item_size=" << stats.first_item_size << '\n';
  }
  std::cout << "decoded_packets_with_points=" << stats.decoded_packets << '\n';
  std::cout << "decoded_clouds=" << stats.decoded_clouds << '\n';
  std::cout << "decoded_points=" << stats.decoded_points << '\n';
  if (stats.decoded_points > 0) {
    std::cout << std::fixed << std::setprecision(3) << "xyz_bounds=[(" << stats.min_x << ", "
              << stats.min_y << ", " << stats.min_z << ") .. (" << stats.max_x << ", "
              << stats.max_y << ", " << stats.max_z << ")]" << '\n';
  }

  if (stats.udp_datagrams == 0 || stats.decoded_points == 0 || stats.decoded_clouds == 0) {
    std::cerr << "Validation failed: no Robin E1X point clouds were decoded from the pcap."
              << std::endl;
    return 2;
  }

  return 0;
}
