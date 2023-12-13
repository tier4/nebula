// Copyright 2023 Tier IV, Inc.
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

#include "nebula_common/continental/continental_common.hpp"
#include "nebula_common/point_types.hpp"

#include "nebula_msgs/msg/nebula_packet.hpp"
#include "nebula_msgs/msg/nebula_packets.hpp"

#include <memory>
#include <tuple>

namespace nebula
{
namespace drivers
{
/// @brief Base class for Continental Radar decoder
class ContinentalPacketsDecoder
{
protected:
  /// @brief SensorConfiguration for this decoder
  std::shared_ptr<drivers::ContinentalRadarEthernetSensorConfiguration> sensor_configuration_;

public:
  ContinentalPacketsDecoder(ContinentalPacketsDecoder && c) = delete;
  ContinentalPacketsDecoder & operator=(ContinentalPacketsDecoder && c) = delete;
  ContinentalPacketsDecoder(const ContinentalPacketsDecoder & c) = delete;
  ContinentalPacketsDecoder & operator=(const ContinentalPacketsDecoder & c) = delete;

  virtual ~ContinentalPacketsDecoder() = default;
  ContinentalPacketsDecoder() = default;

  /// @brief Virtual function for parsing NebulaPackets based on packet structure
  /// @param nebula_packets
  /// @return Resulting flag
  virtual bool ProcessPackets(const nebula_msgs::msg::NebulaPackets & nebula_packets) = 0;
};
}  // namespace drivers
}  // namespace nebula
#endif  // NEBULA_WS_CONTINENTAL_PACKETS_DECODER_HPP
