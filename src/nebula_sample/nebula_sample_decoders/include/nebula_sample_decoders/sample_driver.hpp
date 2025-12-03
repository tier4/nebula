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

#ifndef NEBULA_SAMPLE_DRIVER_HPP
#define NEBULA_SAMPLE_DRIVER_HPP

#include "nebula_core_common/nebula_status.hpp"
#include "nebula_sample_common/sample_common.hpp"
#include "nebula_sample_decoders/decoders/sample_scan_decoder.hpp"

#include <memory>
#include <vector>

namespace nebula::drivers
{

class SampleDriver
{
public:
  SampleDriver(
    const std::shared_ptr<const SampleSensorConfiguration> & sensor_configuration,
    const std::shared_ptr<const SampleCalibrationConfiguration> & calibration_configuration);

  Status get_status();

  PacketDecodeResult parse_cloud_packet(const std::vector<uint8_t> & packet);

  void set_pointcloud_callback(SampleScanDecoder::pointcloud_callback_t pointcloud_cb);

private:
  std::shared_ptr<SampleScanDecoder> scan_decoder_;
  Status driver_status_;
};

}  // namespace nebula::drivers

#endif  // NEBULA_SAMPLE_DRIVER_HPP
