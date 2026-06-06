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

#ifndef NEBULA_PCAP_PACKET_SOURCE_HPP
#define NEBULA_PCAP_PACKET_SOURCE_HPP

#include <nebula_core_hw_interfaces/packet_source.hpp>

#include <algorithm>
#include <atomic>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace nebula::drivers
{
class PcapPacketSource : public PacketSource
{
public:
  PcapPacketSource();
  ~PcapPacketSource() override;

  void open(const std::string & pcap_file);
  void set_packet_callback(SensorPacketCallback callback) override;
  void set_error_callback(SensorErrorCallback callback) override;
  void start() override;
  void stop() override;
  void wait_until_finished();
  bool is_running() const override { return running_ && running_->load(); }

private:
  static void run(
    std::shared_ptr<std::atomic<bool>> running, std::string pcap_file,
    SensorPacketCallback callback, SensorErrorCallback error_callback);

  struct ReassemblyKey
  {
    uint32_t src{};
    uint32_t dst{};
    uint16_t id{};
    uint8_t protocol{};

    bool operator<(const ReassemblyKey & other) const
    {
      if (src != other.src) return src < other.src;
      if (dst != other.dst) return dst < other.dst;
      if (id != other.id) return id < other.id;
      return protocol < other.protocol;
    }
  };

  struct FragmentAssembly
  {
    std::vector<uint8_t> data;
    std::vector<std::pair<size_t, size_t>> received_ranges;
    bool saw_last{false};
    uint64_t timestamp_ns{0};
    uint16_t src_port{0};
    uint16_t dst_port{0};
    std::string src_ip;
    std::string dst_ip;

    bool is_complete() const
    {
      return saw_last && !data.empty() && received_ranges.size() == 1 &&
             received_ranges.front().first == 0 && received_ranges.front().second >= data.size();
    }

    void mark_received(size_t offset, size_t length)
    {
      if (length == 0) {
        return;
      }

      std::pair<size_t, size_t> next_range{offset, offset + length};
      auto it = received_ranges.begin();
      while (it != received_ranges.end()) {
        if (next_range.second < it->first) {
          break;
        }
        if (it->second < next_range.first) {
          ++it;
          continue;
        }

        next_range.first = std::min(next_range.first, it->first);
        next_range.second = std::max(next_range.second, it->second);
        it = received_ranges.erase(it);
      }
      received_ranges.insert(it, next_range);
    }
  };

  std::string pcap_file_;
  SensorPacketCallback callback_;
  SensorErrorCallback error_callback_;
  std::thread thread_;
  std::shared_ptr<std::atomic<bool>> running_{std::make_shared<std::atomic<bool>>(false)};
};

}  // namespace nebula::drivers

#endif  // NEBULA_PCAP_PACKET_SOURCE_HPP
