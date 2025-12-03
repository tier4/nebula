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

#ifndef NEBULA_SAMPLE_DECODER_HPP
#define NEBULA_SAMPLE_DECODER_HPP

#include "nebula_sample_common/sample_common.hpp"
#include "nebula_sample_decoders/decoders/sample_scan_decoder.hpp"

#include <memory>
#include <vector>

namespace nebula::drivers
{

class SampleDecoder : public SampleScanDecoder
{
public:
  SampleDecoder(
    const std::shared_ptr<const SampleSensorConfiguration> & sensor_configuration,
    const std::shared_ptr<const SampleCalibrationConfiguration> & calibration_configuration)
  {
    // Initialize
    (void)sensor_configuration;
    (void)calibration_configuration;
  }

  PacketDecodeResult unpack(const std::vector<uint8_t> & packet) override
  {
    // Sample implementation - does nothing, just returns an error
    (void)packet;
    return {{}, DecodeError::PACKET_PARSE_FAILED};
  }

  void set_pointcloud_callback(pointcloud_callback_t callback) override
  {
    pointcloud_callback_ = callback;
  }

private:
  pointcloud_callback_t pointcloud_callback_;
};

}  // namespace nebula::drivers

#endif  // NEBULA_SAMPLE_DECODER_HPP
