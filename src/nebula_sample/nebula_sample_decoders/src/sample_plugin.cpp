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

#include <nebula_core_decoders/sensor_plugin_export.hpp>
#include <nebula_sample_common/sample_configuration.hpp>
#include <nebula_sample_decoders/sample_plugin.hpp>

#include <iostream>
#include <memory>
#include <vector>

namespace nebula::drivers
{

SampleSensorDecoderRuntime::SampleSensorDecoderRuntime()
{
}

void SampleSensorDecoderRuntime::configure(const SensorConfiguration & config)
{
  SampleSensorConfiguration sample_config;
  sample_config.connection.host_ip = config.host_ip;
  sample_config.connection.sensor_ip = config.sensor_ip;
  sample_config.connection.data_port = config.data_port;
  sample_config.fov.azimuth.start = config.fov.azimuth.start;
  sample_config.fov.azimuth.end = config.fov.azimuth.end;
  sample_config.fov.elevation.start = config.fov.elevation.start;
  sample_config.fov.elevation.end = config.fov.elevation.end;

  decoder_ = std::make_unique<SampleDecoder>(
    sample_config.fov, std::bind(
                         &SampleSensorDecoderRuntime::on_pointcloud, this, std::placeholders::_1,
                         std::placeholders::_2));
}

void SampleSensorDecoderRuntime::set_output_callback(SensorOutputCallback callback)
{
  output_callback_ = callback;
}

void SampleSensorDecoderRuntime::set_error_callback(SensorErrorCallback callback)
{
  error_callback_ = callback;
}

void SampleSensorDecoderRuntime::set_progress_callback(SensorProgressCallback callback)
{
  progress_callback_ = callback;
}

SensorPacketResult SampleSensorDecoderRuntime::process_packet(const SensorPacketView & packet)
{
  if (!decoder_) {
    return SensorPacketResult::Error;
  }

  progress_.processed_packets++;

  // For sample sensor, we only care about UDP Data channel
  if (packet.transport == SensorTransportKind::UDP && packet.channel == SensorPacketChannel::Data) {
    progress_.matched_packets++;
    if (packet.payload_size == 0) {
      progress_.dropped_packets++;
      if (progress_callback_) progress_callback_(progress_);
      return SensorPacketResult::Ignored;
    }
    // Adapter copy: SampleDecoder::unpack() requires a vector; real decoders should use the view
    // directly.
    const std::vector<uint8_t> payload(
      packet.payload_data, packet.payload_data + packet.payload_size);
    auto result = decoder_->unpack(payload);

    if (result.metadata_or_error.has_value()) {
      progress_.decoded_packets++;
      if (progress_callback_) progress_callback_(progress_);
      return SensorPacketResult::Success;
    } else {
      progress_.error_count++;
      if (error_callback_) {
        SensorError error;
        error.type = SensorErrorType::DecoderError;
        error.message = to_cstr(result.metadata_or_error.error());
        error.timestamp_ns = packet.timestamp_ns;
        error_callback_(error);
      }
      if (progress_callback_) progress_callback_(progress_);
      return SensorPacketResult::Error;
    }
  }

  progress_.dropped_packets++;
  if (progress_callback_) progress_callback_(progress_);
  return SensorPacketResult::Ignored;
}

void SampleSensorDecoderRuntime::flush()
{
  // SampleDecoder doesn't have a flush, but if it did, we'd call it here.
}

void SampleSensorDecoderRuntime::on_pointcloud(
  const NebulaPointCloudPtr & pointcloud, double timestamp_s)
{
  progress_.output_count++;
  if (output_callback_) {
    SensorDecodedOutput output;
    output.kind = SensorOutputKind::PointCloud;
    const auto seconds = static_cast<uint64_t>(timestamp_s);
    const double fractional_seconds = timestamp_s - static_cast<double>(seconds);
    auto fractional_ns = static_cast<uint64_t>(fractional_seconds * 1000000000.0 + 0.5);
    output.timestamp_ns = seconds * 1000000000ULL;
    if (fractional_ns >= 1000000000ULL) {
      output.timestamp_ns += 1000000000ULL;
    } else {
      output.timestamp_ns += fractional_ns;
    }
    output.sensor_id = "sample_sensor";
    output.payload = pointcloud;
    output_callback_(output);
  }
}

SensorPluginMetadata SampleSensorPlugin::metadata() const
{
  SensorPluginMetadata md;
  md.vendor = "nebula";
  md.package_name = "nebula_sample_decoders";
  md.library_path = "libnebula_sample_decoders_plugin.so";
  md.factory_symbol = "create_nebula_sensor_plugin";
  md.supported_models = {SensorModel::SAMPLE};
  return md;
}

std::vector<SensorModelInfo> SampleSensorPlugin::supported_models() const
{
  return {{SensorModel::SAMPLE, "Sample", "Nebula Sample LiDAR Sensor"}};
}

std::vector<PacketChannelRequirement> SampleSensorPlugin::packet_requirements(
  const SensorConfiguration & config) const
{
  PacketChannelRequirement req;
  req.transport = SensorTransportKind::UDP;
  req.channel = SensorPacketChannel::Data;
  req.required = true;
  req.destination_port = config.data_port;
  return {req};
}

std::vector<LiveTransportRequirement> SampleSensorPlugin::live_transport_requirements(
  const SensorConfiguration & config) const
{
  LiveTransportRequirement req;
  req.transport = SensorTransportKind::UDP;
  req.channel = SensorPacketChannel::Data;
  req.required = true;
  req.name = "sample_udp_data";
  req.port = config.data_port;
  return {req};
}

std::unique_ptr<SensorDecoderRuntime> SampleSensorPlugin::create_decoder_runtime() const
{
  return std::make_unique<SampleSensorDecoderRuntime>();
}

}  // namespace nebula::drivers

extern "C" {
nebula::drivers::SensorPlugin * create_nebula_sensor_plugin()
{
  return new nebula::drivers::SampleSensorPlugin();
}

void destroy_nebula_sensor_plugin(nebula::drivers::SensorPlugin * plugin)
{
  delete plugin;
}

uint32_t nebula_plugin_abi_version()
{
  return nebula::drivers::kNebulaPluginAbiVersion;
}
}
