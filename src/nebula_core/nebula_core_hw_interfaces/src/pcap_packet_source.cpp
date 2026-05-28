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

#include <nebula_core_hw_interfaces/pcap_packet_source.hpp>

#include <arpa/inet.h>
#include <net/ethernet.h>
#include <netinet/ip.h>
#include <netinet/udp.h>
#include <pcap.h>

#include <algorithm>
#include <cstring>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace nebula::drivers
{
namespace
{
uint16_t read_be16(const u_char * data)
{
  uint16_t value{};
  std::memcpy(&value, data, sizeof(value));
  return ntohs(value);
}

// Maximum number of in-flight fragment reassemblies. Bounds memory in pathological
// PCAPs where last fragments are missing (truncated capture, IP-id reuse, etc.).
constexpr size_t kMaxPendingAssemblies = 1024;
}  // namespace

PcapPacketSource::PcapPacketSource()
{
}

PcapPacketSource::~PcapPacketSource()
{
  stop();
}

void PcapPacketSource::open(const std::string & pcap_file)
{
  if (running_ && running_->load()) {
    throw std::runtime_error("Cannot open a PCAP file while replay is running");
  }

  char errbuf[PCAP_ERRBUF_SIZE];
  pcap_t * handle = pcap_open_offline(pcap_file.c_str(), errbuf);
  if (!handle) {
    throw std::runtime_error(std::string("Could not open PCAP file: ") + errbuf);
  }
  const int link_type = pcap_datalink(handle);
  pcap_close(handle);
  if (link_type != DLT_EN10MB) {
    throw std::runtime_error(
      "Unsupported PCAP link type: " + std::to_string(link_type) + ". Only Ethernet is supported.");
  }
  pcap_file_ = pcap_file;
}

void PcapPacketSource::set_packet_callback(SensorPacketCallback callback)
{
  callback_ = callback;
}

void PcapPacketSource::set_error_callback(SensorErrorCallback callback)
{
  error_callback_ = callback;
}

void PcapPacketSource::start()
{
  if (running_ && running_->load()) return;
  if (thread_.joinable()) {
    if (thread_.get_id() == std::this_thread::get_id()) return;
    thread_.join();
  }

  running_ = std::make_shared<std::atomic<bool>>(true);
  thread_ = std::thread(&PcapPacketSource::run, running_, pcap_file_, callback_, error_callback_);
}

void PcapPacketSource::stop()
{
  if (running_) {
    running_->store(false);
  }
  if (thread_.joinable()) {
    if (thread_.get_id() == std::this_thread::get_id()) {
      thread_.detach();
      return;
    }
    thread_.join();
  }
}

void PcapPacketSource::wait_until_finished()
{
  if (thread_.joinable()) {
    if (thread_.get_id() == std::this_thread::get_id()) {
      thread_.detach();
      return;
    }
    thread_.join();
  }
}

void PcapPacketSource::run(
  std::shared_ptr<std::atomic<bool>> running, std::string pcap_file, SensorPacketCallback callback,
  SensorErrorCallback error_callback)
{
  std::map<ReassemblyKey, FragmentAssembly> assemblies;

  char errbuf[PCAP_ERRBUF_SIZE];
  pcap_t * handle = pcap_open_offline(pcap_file.c_str(), errbuf);
  if (!handle) {
    if (error_callback) {
      SensorError error;
      error.type = SensorErrorType::TransportError;
      error.message = std::string("Could not open PCAP file: ") + errbuf;
      error_callback(error);
    }
    running->store(false);
    return;
  }

  int link_type = pcap_datalink(handle);
  if (link_type != DLT_EN10MB) {
    if (error_callback) {
      SensorError error;
      error.type = SensorErrorType::TransportError;
      error.message = "Unsupported PCAP link type: " + std::to_string(link_type);
      error_callback(error);
    }
    pcap_close(handle);
    running->store(false);
    return;
  }

  struct pcap_pkthdr * header;
  const u_char * pkt_data;

  int next_status = 0;
  while (running->load() && (next_status = pcap_next_ex(handle, &header, &pkt_data)) >= 0) {
    if (next_status == 0) {
      continue;
    }

    constexpr size_t ethernet_header_size = 14;
    constexpr size_t ethernet_type_offset = 12;
    if (header->caplen < ethernet_header_size) continue;

    uint16_t eth_type = read_be16(pkt_data + ethernet_type_offset);
    size_t eth_hdr_len = ethernet_header_size;

    bool truncated_vlan = false;
    while (eth_type == 0x8100 || eth_type == 0x88a8) {
      if (header->caplen < eth_hdr_len + 4) {
        truncated_vlan = true;
        break;
      }
      eth_type = read_be16(pkt_data + eth_hdr_len + 2);
      eth_hdr_len += 4;
    }
    if (truncated_vlan) continue;

    if (eth_type != ETHERTYPE_IP) continue;
    if (header->caplen < eth_hdr_len + sizeof(struct ip)) continue;

    struct ip ip_hdr = {};
    std::memcpy(&ip_hdr, pkt_data + eth_hdr_len, sizeof(ip_hdr));
    size_t ip_hdr_len = ip_hdr.ip_hl * 4;
    if (ip_hdr_len < sizeof(struct ip)) continue;
    if (header->caplen < eth_hdr_len + ip_hdr_len) continue;

    const size_t ip_total_len = ntohs(ip_hdr.ip_len);
    if (ip_total_len < ip_hdr_len) {
      continue;
    }

    uint16_t ip_off = ntohs(ip_hdr.ip_off);
    uint16_t frag_offset = (ip_off & IP_OFFMASK) * 8;
    bool more_frags = (ip_off & IP_MF) != 0;

    char src_ip_str[INET_ADDRSTRLEN];
    char dst_ip_str[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &(ip_hdr.ip_src), src_ip_str, INET_ADDRSTRLEN);
    inet_ntop(AF_INET, &(ip_hdr.ip_dst), dst_ip_str, INET_ADDRSTRLEN);

    if (ip_hdr.ip_p == IPPROTO_UDP) {
      const u_char * ip_payload = pkt_data + eth_hdr_len + ip_hdr_len;
      size_t ip_payload_len = ip_total_len - ip_hdr_len;

      // Truncated packet check
      if (eth_hdr_len + ip_hdr_len + ip_payload_len > header->caplen) {
        continue;
      }

      ReassemblyKey key{
        ip_hdr.ip_src.s_addr, ip_hdr.ip_dst.s_addr, ntohs(ip_hdr.ip_id), ip_hdr.ip_p};

      if (frag_offset == 0) {
        if (ip_payload_len < sizeof(struct udphdr)) continue;
        struct udphdr udp_hdr = {};
        std::memcpy(&udp_hdr, ip_payload, sizeof(udp_hdr));
        uint16_t src_port = ntohs(udp_hdr.uh_sport);
        uint16_t dst_port = ntohs(udp_hdr.uh_dport);
        uint16_t udp_len = ntohs(udp_hdr.uh_ulen);

        if (udp_len < sizeof(struct udphdr)) {
          continue;
        }

        if (!more_frags) {
          // Not fragmented
          if (callback) {
            SensorPacket sp;
            sp.transport = SensorTransportKind::UDP;
            sp.from_replay = true;
            sp.timestamp_ns = static_cast<uint64_t>(header->ts.tv_sec) * 1000000000ULL +
                              static_cast<uint64_t>(header->ts.tv_usec) * 1000ULL;
            sp.source = {src_ip_str, src_port};
            sp.destination = {dst_ip_str, dst_port};
            sp.payload.assign(
              ip_payload + sizeof(struct udphdr),
              ip_payload + std::min(static_cast<size_t>(udp_len), ip_payload_len));
            try {
              callback(sp);
            } catch (const std::exception & e) {
              if (error_callback) {
                SensorError err;
                err.type = SensorErrorType::DecoderError;
                err.message = std::string("PcapPacketSource: callback threw: ") + e.what();
                error_callback(err);
              }
            } catch (...) {
              if (error_callback) {
                SensorError err;
                err.type = SensorErrorType::DecoderError;
                err.message = "PcapPacketSource: callback threw a non-std::exception";
                error_callback(err);
              }
            }
          }
        } else {
          // First fragment of many
          const size_t total_size = udp_len - sizeof(struct udphdr);
          if (total_size == 0) {
            continue;
          }

          // If the map is at capacity and this is a brand-new key, evict the
          // oldest pending assembly (by pcap timestamp) to keep memory bounded.
          if (
            assemblies.size() >= kMaxPendingAssemblies &&
            assemblies.find(key) == assemblies.end()) {
            auto oldest = std::min_element(
              assemblies.begin(), assemblies.end(), [](const auto & a, const auto & b) {
                return a.second.timestamp_ns < b.second.timestamp_ns;
              });
            if (oldest != assemblies.end()) assemblies.erase(oldest);
          }

          // Always reset the assembly for this key. A new first-fragment with the
          // same (src,dst,id,proto) means the previous datagram never completed
          // (or its IP-id was reused); stale received[] / saw_last would otherwise
          // bleed into the new assembly.
          auto & ass = assemblies[key];
          ass = FragmentAssembly{};
          ass.src_ip = src_ip_str;
          ass.dst_ip = dst_ip_str;
          ass.src_port = src_port;
          ass.dst_port = dst_port;
          ass.timestamp_ns = static_cast<uint64_t>(header->ts.tv_sec) * 1000000000ULL +
                             static_cast<uint64_t>(header->ts.tv_usec) * 1000ULL;
          ass.total_size = total_size;
          ass.data.resize(ass.total_size);
          ass.received.resize(ass.total_size, false);

          size_t frag_data_len = ip_payload_len - sizeof(struct udphdr);
          if (frag_data_len > ass.total_size) frag_data_len = ass.total_size;
          std::copy(
            ip_payload + sizeof(struct udphdr), ip_payload + sizeof(struct udphdr) + frag_data_len,
            ass.data.begin());
          std::fill(ass.received.begin(), ass.received.begin() + frag_data_len, true);
        }
      } else {
        // Subsequent fragment
        if (assemblies.count(key)) {
          auto & ass = assemblies[key];
          if (frag_offset < sizeof(struct udphdr)) {
            continue;
          }
          size_t udp_data_offset = frag_offset - sizeof(struct udphdr);
          if (udp_data_offset >= ass.data.size()) {
            continue;
          }
          if (udp_data_offset + ip_payload_len > ass.data.size()) {
            ip_payload_len = ass.data.size() - udp_data_offset;
          }
          std::copy(ip_payload, ip_payload + ip_payload_len, ass.data.begin() + udp_data_offset);
          std::fill(
            ass.received.begin() + udp_data_offset,
            ass.received.begin() + udp_data_offset + ip_payload_len, true);
          if (!more_frags) ass.saw_last = true;

          if (ass.is_complete()) {
            if (callback) {
              SensorPacket sp;
              sp.transport = SensorTransportKind::UDP;
              sp.from_replay = true;
              sp.timestamp_ns = ass.timestamp_ns;
              sp.source = {ass.src_ip, ass.src_port};
              sp.destination = {ass.dst_ip, ass.dst_port};
              sp.payload = std::move(ass.data);
              try {
                callback(sp);
              } catch (const std::exception & e) {
                if (error_callback) {
                  SensorError err;
                  err.type = SensorErrorType::DecoderError;
                  err.message = std::string("PcapPacketSource: callback threw: ") + e.what();
                  error_callback(err);
                }
              } catch (...) {
                if (error_callback) {
                  SensorError err;
                  err.type = SensorErrorType::DecoderError;
                  err.message = "PcapPacketSource: callback threw a non-std::exception";
                  error_callback(err);
                }
              }
            }
            assemblies.erase(key);
          }
        }
      }
    }
  }

  if (next_status == -1 && error_callback) {
    SensorError error;
    error.type = SensorErrorType::TransportError;
    error.message = std::string("Failed while reading PCAP file: ") + pcap_geterr(handle);
    error_callback(error);
  }

  pcap_close(handle);
  running->store(false);
}

}  // namespace nebula::drivers
