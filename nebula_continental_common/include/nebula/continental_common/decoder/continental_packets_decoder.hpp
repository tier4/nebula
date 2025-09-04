// Copyright 2024 TIER IV, Inc.
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

#ifndef NEBULA_WS_CONTINENTAL_PACKETS_DECODER_HPP
#define NEBULA_WS_CONTINENTAL_PACKETS_DECODER_HPP

#include <nebula_common/nebula_status.hpp>
#include <nebula_common/point_types.hpp>

#include <nebula_msgs/msg/nebula_packet.hpp>

#include <memory>
#include <tuple>

namespace nebula::drivers
{
/// @brief Base class for Continental Radar decoder
class ContinentalPacketsDecoder
{
public:
  ContinentalPacketsDecoder(ContinentalPacketsDecoder && c) = delete;
  ContinentalPacketsDecoder & operator=(ContinentalPacketsDecoder && c) = delete;
  ContinentalPacketsDecoder(const ContinentalPacketsDecoder & c) = delete;
  ContinentalPacketsDecoder & operator=(const ContinentalPacketsDecoder & c) = delete;

  virtual ~ContinentalPacketsDecoder() = default;
  ContinentalPacketsDecoder() = default;

  /// @brief Get current status of this driver
  /// @return Current status
  virtual Status get_status() = 0;

  /// @brief Virtual function for parsing a NebulaPacket
  /// @param packet_msg
  /// @return Resulting flag
  virtual bool process_packet(std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg) = 0;
};
}  // namespace nebula::drivers
#endif  // NEBULA_WS_CONTINENTAL_PACKETS_DECODER_HPP
