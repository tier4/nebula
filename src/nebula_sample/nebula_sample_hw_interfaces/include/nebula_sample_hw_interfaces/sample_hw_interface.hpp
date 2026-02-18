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

#ifndef NEBULA_SAMPLE_HW_INTERFACE_HPP
#define NEBULA_SAMPLE_HW_INTERFACE_HPP

#include <nebula_core_common/util/expected.hpp>
#include <nebula_core_hw_interfaces/connections/udp.hpp>
#include <nebula_sample_common/sample_configuration.hpp>

#include <cstdint>
#include <optional>
#include <variant>

namespace nebula::drivers
{

/// @brief Receives raw sensor packets and forwards them to a registered callback.
/// @details This class owns the transport-facing state used by the sample driver.
class SampleHwInterface
{
public:
  enum class Error : uint8_t {
    CALLBACK_NOT_REGISTERED,  ///< Start requested before a callback was registered.
    INVALID_CALLBACK,         ///< Empty callback passed to register_scan_callback.
  };

  static const char * to_cstr(Error error);

  /// @brief Construct the hardware interface with connection settings.
  /// @param connection_configuration Network addresses and ports used by the sensor stream.
  explicit SampleHwInterface(ConnectionConfiguration connection_configuration);

  /// @brief Start packet reception.
  /// @return std::monostate on success, Error on failure.
  /// @post On success, incoming packets are delivered through the registered callback.
  util::expected<std::monostate, Error> sensor_interface_start();

  /// @brief Stop packet reception.
  /// @return std::monostate on success, Error on failure.
  util::expected<std::monostate, Error> sensor_interface_stop();

  /// @brief Register or replace the callback invoked for each incoming packet.
  /// @param scan_callback Callback receiving packet bytes and transport metadata.
  /// @return std::monostate if callback is accepted, Error otherwise.
  util::expected<std::monostate, Error> register_scan_callback(
    connections::UdpSocket::callback_t scan_callback);

private:
  ConnectionConfiguration connection_configuration_;
  std::optional<connections::UdpSocket::callback_t> packet_callback_;
};

}  // namespace nebula::drivers

#endif  // NEBULA_SAMPLE_HW_INTERFACE_HPP
