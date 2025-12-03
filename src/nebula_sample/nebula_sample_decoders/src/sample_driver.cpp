// Copyright 2025 TIER IV, Inc.
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

#include "nebula_sample_decoders/sample_driver.hpp"

#include "nebula_sample_decoders/decoders/sample_decoder.hpp"

#include <memory>
#include <vector>

namespace nebula::drivers
{

SampleDriver::SampleDriver(
  const std::shared_ptr<const SampleSensorConfiguration> & sensor_configuration,
  const std::shared_ptr<const SampleCalibrationConfiguration> & calibration_configuration)
{
  driver_status_ = Status::OK;
  scan_decoder_ = std::make_shared<SampleDecoder>(sensor_configuration, calibration_configuration);
}

Status SampleDriver::get_status()
{
  return driver_status_;
}

PacketDecodeResult SampleDriver::parse_cloud_packet(const std::vector<uint8_t> & packet)
{
  return scan_decoder_->unpack(packet);
}

void SampleDriver::set_pointcloud_callback(SampleScanDecoder::pointcloud_callback_t pointcloud_cb)
{
  scan_decoder_->set_pointcloud_callback(pointcloud_cb);
}

}  // namespace nebula::drivers
