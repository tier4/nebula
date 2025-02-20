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

#include "nebula_hw_interfaces/nebula_hw_interfaces_continental/continental_ars548_hw_interface.hpp"

#include "nebula_hw_interfaces/nebula_hw_interfaces_common/connections/udp.hpp"

#include <nebula_common/continental/continental_ars548.hpp>

#include <algorithm>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace nebula::drivers::continental_ars548
{
ContinentalARS548HwInterface::ContinentalARS548HwInterface(
  const std::shared_ptr<loggers::Logger> & logger)
: logger_(logger)
{
}

Status ContinentalARS548HwInterface::set_sensor_configuration(
  std::shared_ptr<const ContinentalARS548SensorConfiguration> new_config_ptr)
{
  config_ptr_ = new_config_ptr;
  return Status::OK;
}

Status ContinentalARS548HwInterface::sensor_interface_start()
{
  try {
    using UdpSocket = connections::UdpSocket;
    auto udp_socket =
      UdpSocket::Builder(config_ptr_->host_ip, config_ptr_->data_port)
        .join_multicast_group(config_ptr_->multicast_ip)
        .set_send_destination(config_ptr_->sensor_ip, config_ptr_->configuration_sensor_port)
        .bind();

    udp_socket_.emplace(std::move(udp_socket));
    udp_socket_->subscribe([this](const auto & payload, const auto & metadata) {
      receive_sensor_packet_callback(payload, metadata);
    });
  } catch (const std::exception & ex) {
    Status status = Status::UDP_CONNECTION_ERROR;
    logger_->error(
      "Failed to start sensor interface: " + std::string(ex.what()) + " (" +
      config_ptr_->sensor_ip + ":" + std::to_string(config_ptr_->data_port) + ")");
    return status;
  }
  return Status::OK;
}

Status ContinentalARS548HwInterface::register_packet_callback(
  std::function<void(std::unique_ptr<nebula_msgs::msg::NebulaPacket>)> packet_callback)
{
  packet_callback_ = std::move(packet_callback);
  return Status::OK;
}

void ContinentalARS548HwInterface::receive_sensor_packet_callback(
  const std::vector<uint8_t> & buffer, const connections::UdpSocket::RxMetadata & metadata)
{
  if (buffer.size() < sizeof(HeaderPacket)) {
    logger_->error("Unrecognized packet. Too short");
    return;
  }

  auto rx_timestamp_ns = metadata.timestamp_ns;
  if (!rx_timestamp_ns) {
    auto now = std::chrono::high_resolution_clock::now();
    rx_timestamp_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
  }

  auto msg_ptr = std::make_unique<nebula_msgs::msg::NebulaPacket>();
  msg_ptr->stamp.sec = static_cast<int>(rx_timestamp_ns.value() / 1'000'000'000);
  msg_ptr->stamp.nanosec = static_cast<int>(rx_timestamp_ns.value() % 1'000'000'000);
  msg_ptr->data.resize(buffer.size());
  std::copy(buffer.begin(), buffer.end(), msg_ptr->data.begin());

  packet_callback_(std::move(msg_ptr));
}

Status ContinentalARS548HwInterface::sensor_interface_stop()
{
  if (udp_socket_) {
    udp_socket_->unsubscribe();
  }
  return Status::OK;
}

Status ContinentalARS548HwInterface::set_sensor_mounting(
  float longitudinal_autosar, float lateral_autosar, float vertical_autosar, float yaw_autosar,
  float pitch_autosar, uint8_t plug_orientation)
{
  if (
    longitudinal_autosar < -100.f || longitudinal_autosar > 100.f || lateral_autosar < -100.f ||
    lateral_autosar > 100.f || vertical_autosar < 0.01f || vertical_autosar > 10.f ||
    yaw_autosar < -M_PI || yaw_autosar > M_PI || pitch_autosar < -M_PI_2 ||
    pitch_autosar > M_PI_2) {
    print_error("Invalid SetSensorMounting values");
    return Status::SENSOR_CONFIG_ERROR;
  }

  ConfigurationPacket configuration_packet{};
  configuration_packet.header.service_id = configuration_service_id;
  configuration_packet.header.method_id = configuration_method_id;
  configuration_packet.header.length = configuration_payload_length;
  configuration_packet.configuration.longitudinal = longitudinal_autosar;
  configuration_packet.configuration.lateral = lateral_autosar;
  configuration_packet.configuration.vertical = vertical_autosar;
  configuration_packet.configuration.yaw = yaw_autosar;
  configuration_packet.configuration.pitch = pitch_autosar;
  configuration_packet.configuration.plug_orientation = plug_orientation;
  configuration_packet.new_sensor_mounting = 1;

  std::vector<uint8_t> send_vector(sizeof(ConfigurationPacket));
  std::memcpy(send_vector.data(), &configuration_packet, sizeof(ConfigurationPacket));

  print_info("longitudinal_autosar = " + std::to_string(longitudinal_autosar));
  print_info("lateral_autosar = " + std::to_string(lateral_autosar));
  print_info("vertical_autosar = " + std::to_string(vertical_autosar));
  print_info("yaw_autosar = " + std::to_string(yaw_autosar));
  print_info("pitch_autosar = " + std::to_string(pitch_autosar));
  print_info("plug_orientation = " + std::to_string(plug_orientation));

  return safe_send(send_vector);
}

Status ContinentalARS548HwInterface::set_vehicle_parameters(
  float length_autosar, float width_autosar, float height_autosar, float wheel_base_autosar)
{
  if (
    length_autosar < 0.01f || length_autosar > 100.f || width_autosar < 0.01f ||
    width_autosar > 100.f || height_autosar < 0.01f || height_autosar > 100.f ||
    wheel_base_autosar < 0.01f || wheel_base_autosar > 100.f) {
    print_error("Invalid SetVehicleParameters values");
    return Status::SENSOR_CONFIG_ERROR;
  }

  ConfigurationPacket configuration_packet{};
  static_assert(sizeof(ConfigurationPacket) == configuration_udp_length);
  configuration_packet.header.service_id = configuration_service_id;
  configuration_packet.header.method_id = configuration_method_id;
  configuration_packet.header.length = configuration_payload_length;
  configuration_packet.configuration.length = length_autosar;
  configuration_packet.configuration.width = width_autosar;
  configuration_packet.configuration.height = height_autosar;
  configuration_packet.configuration.wheelbase = wheel_base_autosar;
  configuration_packet.new_vehicle_parameters = 1;

  std::vector<uint8_t> send_vector(sizeof(ConfigurationPacket));
  std::memcpy(send_vector.data(), &configuration_packet, sizeof(ConfigurationPacket));

  print_info("length_autosar = " + std::to_string(length_autosar));
  print_info("width_autosar = " + std::to_string(width_autosar));
  print_info("height_autosar = " + std::to_string(height_autosar));
  print_info("wheel_base_autosar = " + std::to_string(wheel_base_autosar));

  return safe_send(send_vector);
}

Status ContinentalARS548HwInterface::set_radar_parameters(
  uint16_t maximum_distance, uint8_t frequency_slot, uint8_t cycle_time_ms, uint8_t time_slot_ms,
  uint8_t hcc, uint8_t power_save_standstill)
{
  if (
    maximum_distance < maximum_distance_min_value ||
    maximum_distance > maximum_distance_max_value) {
    print_error("Invalid maximum_distance value");
    return Status::SENSOR_CONFIG_ERROR;
  }

  if (cycle_time_ms < min_cycle_time_ms || cycle_time_ms > max_cycle_time_ms) {
    print_error("Invalid cycle_time_ms value");
    return Status::SENSOR_CONFIG_ERROR;
  }

  if (
    time_slot_ms < min_time_slot_ms || time_slot_ms > cycle_time_ms - 1 ||
    time_slot_ms > max_time_slot_ms) {
    print_error("Invalid time_slot_ms value");
    return Status::SENSOR_CONFIG_ERROR;
  }

  ConfigurationPacket configuration_packet{};
  static_assert(sizeof(ConfigurationPacket) == configuration_udp_length);
  configuration_packet.header.service_id = configuration_service_id;
  configuration_packet.header.method_id = configuration_method_id;
  configuration_packet.header.length = configuration_payload_length;
  configuration_packet.configuration.maximum_distance = maximum_distance;
  configuration_packet.configuration.frequency_slot = frequency_slot;
  configuration_packet.configuration.cycle_time = cycle_time_ms;
  configuration_packet.configuration.time_slot = time_slot_ms;
  configuration_packet.configuration.hcc = hcc;
  configuration_packet.configuration.powersave_standstill = power_save_standstill;
  configuration_packet.new_radar_parameters = 1;

  std::vector<uint8_t> send_vector(sizeof(ConfigurationPacket));
  std::memcpy(send_vector.data(), &configuration_packet, sizeof(ConfigurationPacket));

  print_info("maximum_distance = " + std::to_string(maximum_distance));
  print_info("frequency_slot = " + std::to_string(frequency_slot) + " ms");
  print_info("cycle_time = " + std::to_string(cycle_time_ms) + " ms");
  print_info("time_slot = " + std::to_string(time_slot_ms) + " ms");
  print_info("hcc = " + std::to_string(hcc));
  print_info("power_save_standstill = " + std::to_string(power_save_standstill));

  return safe_send(send_vector);
}

Status ContinentalARS548HwInterface::set_sensor_ip_address(const std::string & sensor_ip_address)
{
  std::array<uint8_t, 4> ip_bytes;

  try {
    auto sensor_ip = boost::asio::ip::address::from_string(sensor_ip_address);
    ip_bytes = sensor_ip.to_v4().to_bytes();
  } catch (const std::exception & ex) {
    print_error("Setting invalid IP=" + sensor_ip_address);
    return Status::SENSOR_CONFIG_ERROR;
  }

  print_info("New sensor IP = " + sensor_ip_address);

  ConfigurationPacket configuration{};
  static_assert(sizeof(ConfigurationPacket) == configuration_udp_length);
  configuration.header.service_id = configuration_service_id;
  configuration.header.method_id = configuration_method_id;
  configuration.header.length = configuration_payload_length;
  configuration.configuration.sensor_ip_address00 = ip_bytes[0];
  configuration.configuration.sensor_ip_address01 = ip_bytes[1];
  configuration.configuration.sensor_ip_address02 = ip_bytes[2];
  configuration.configuration.sensor_ip_address03 = ip_bytes[3];
  configuration.configuration.sensor_ip_address10 = 169;
  configuration.configuration.sensor_ip_address11 = 254;
  configuration.configuration.sensor_ip_address12 = 116;
  configuration.configuration.sensor_ip_address13 = 113;
  configuration.new_network_configuration = 1;

  std::vector<uint8_t> send_vector(sizeof(ConfigurationPacket));
  std::memcpy(send_vector.data(), &configuration, sizeof(ConfigurationPacket));

  return safe_send(send_vector);
}

Status ContinentalARS548HwInterface::set_acceleration_lateral_cog(float lateral_acceleration)
{
  constexpr uint16_t acceleration_lateral_cog_service_id = 0;
  constexpr uint16_t acceleration_lateral_cog_method_id = 321;
  constexpr uint8_t acceleration_lateral_cog_length = 32;
  const int acceleration_lateral_cog_udp_length = 40;

  if (lateral_acceleration < -65.f || lateral_acceleration > 65.f) {
    print_error("Invalid lateral_acceleration value");
    return Status::ERROR_1;
  }

  AccelerationLateralCoGPacket acceleration_lateral_cog{};
  static_assert(sizeof(AccelerationLateralCoGPacket) == acceleration_lateral_cog_udp_length);
  acceleration_lateral_cog.header.service_id = acceleration_lateral_cog_service_id;
  acceleration_lateral_cog.header.method_id = acceleration_lateral_cog_method_id;
  acceleration_lateral_cog.header.length = acceleration_lateral_cog_length;
  acceleration_lateral_cog.acceleration_lateral = lateral_acceleration;

  std::vector<uint8_t> send_vector(sizeof(AccelerationLateralCoGPacket));
  std::memcpy(send_vector.data(), &acceleration_lateral_cog, sizeof(AccelerationLateralCoGPacket));

  udp_socket_->send(send_vector);

  return Status::OK;
}

Status ContinentalARS548HwInterface::set_acceleration_longitudinal_cog(
  float longitudinal_acceleration)
{
  constexpr uint16_t acceleration_longitudinal_cog_service_id = 0;
  constexpr uint16_t acceleration_longitudinal_cog_method_id = 322;
  constexpr uint8_t acceleration_longitudinal_cog_length = 32;
  const int acceleration_longitudinal_cog_udp_length = 40;

  if (longitudinal_acceleration < -65.f || longitudinal_acceleration > 65.f) {
    print_error("Invalid longitudinal_acceleration value");
    return Status::ERROR_1;
  }

  AccelerationLongitudinalCoGPacket acceleration_longitudinal_cog{};
  static_assert(
    sizeof(AccelerationLongitudinalCoGPacket) == acceleration_longitudinal_cog_udp_length);
  acceleration_longitudinal_cog.header.service_id = acceleration_longitudinal_cog_service_id;
  acceleration_longitudinal_cog.header.method_id = acceleration_longitudinal_cog_method_id;
  acceleration_longitudinal_cog.header.length = acceleration_longitudinal_cog_length;
  acceleration_longitudinal_cog.acceleration_lateral = longitudinal_acceleration;

  std::vector<uint8_t> send_vector(sizeof(AccelerationLongitudinalCoGPacket));
  std::memcpy(
    send_vector.data(), &acceleration_longitudinal_cog, sizeof(AccelerationLongitudinalCoGPacket));

  return safe_send(send_vector);
}

Status ContinentalARS548HwInterface::set_characteristic_speed(float characteristic_speed)
{
  constexpr uint16_t characteristic_speed_service_id = 0;
  constexpr uint16_t characteristic_speed_method_id = 328;
  constexpr uint8_t characteristic_speed_length = 11;
  const int characteristic_speed_udp_length = 19;

  if (characteristic_speed < 0.f || characteristic_speed > 255.f) {
    print_error("Invalid characteristic_speed value");
    return Status::ERROR_1;
  }

  CharacteristicSpeedPacket characteristic_speed_packet{};
  static_assert(sizeof(CharacteristicSpeedPacket) == characteristic_speed_udp_length);
  characteristic_speed_packet.header.service_id = characteristic_speed_service_id;
  characteristic_speed_packet.header.method_id = characteristic_speed_method_id;
  characteristic_speed_packet.header.length = characteristic_speed_length;
  characteristic_speed_packet.characteristic_speed = characteristic_speed;

  std::vector<uint8_t> send_vector(sizeof(CharacteristicSpeedPacket));
  std::memcpy(send_vector.data(), &characteristic_speed_packet, sizeof(CharacteristicSpeedPacket));

  return safe_send(send_vector);
}

Status ContinentalARS548HwInterface::set_driving_direction(int direction)
{
  constexpr uint16_t driving_direction_service_id = 0;
  constexpr uint16_t driving_direction_method_id = 325;
  constexpr uint8_t driving_direction_length = 22;
  constexpr uint8_t driving_direction_standstill = 0;
  constexpr uint8_t driving_direction_forward = 1;
  constexpr uint8_t driving_direction_backwards = 2;
  const int driving_direction_udp_length = 30;

  DrivingDirectionPacket driving_direction_packet{};
  static_assert(sizeof(DrivingDirectionPacket) == driving_direction_udp_length);
  driving_direction_packet.header.service_id = driving_direction_service_id;
  driving_direction_packet.header.method_id = driving_direction_method_id;
  driving_direction_packet.header.length = driving_direction_length;

  if (direction == 0) {
    driving_direction_packet.driving_direction = driving_direction_standstill;
  } else if (direction > 0) {
    driving_direction_packet.driving_direction = driving_direction_forward;
  } else {
    driving_direction_packet.driving_direction = driving_direction_backwards;
  }

  std::vector<uint8_t> send_vector(sizeof(DrivingDirectionPacket));
  std::memcpy(send_vector.data(), &driving_direction_packet, sizeof(DrivingDirectionPacket));

  return safe_send(send_vector);
}

Status ContinentalARS548HwInterface::set_steering_angle_front_axle(float angle_rad)
{
  constexpr uint16_t steering_angle_service_id = 0;
  constexpr uint16_t steering_angle_method_id = 327;
  constexpr uint8_t steering_angle_length = 32;
  const int steering_angle_udp_length = 40;

  if (angle_rad < -90.f || angle_rad > 90.f) {
    print_error("Invalid angle_rad value");
    return Status::ERROR_1;
  }

  SteeringAngleFrontAxlePacket steering_angle_front_axle_packet{};
  static_assert(sizeof(SteeringAngleFrontAxlePacket) == steering_angle_udp_length);
  steering_angle_front_axle_packet.header.service_id = steering_angle_service_id;
  steering_angle_front_axle_packet.header.method_id = steering_angle_method_id;
  steering_angle_front_axle_packet.header.length = steering_angle_length;
  steering_angle_front_axle_packet.steering_angle_front_axle = angle_rad;

  std::vector<uint8_t> send_vector(sizeof(SteeringAngleFrontAxlePacket));
  std::memcpy(
    send_vector.data(), &steering_angle_front_axle_packet, sizeof(SteeringAngleFrontAxlePacket));

  return safe_send(send_vector);
}

Status ContinentalARS548HwInterface::set_velocity_vehicle(float velocity_kmh)
{
  if (velocity_kmh < 0.f || velocity_kmh > 350.f) {
    print_error("Invalid velocity value");
    return Status::ERROR_1;
  }

  constexpr uint16_t velocity_vehicle_service_id = 0;
  constexpr uint16_t velocity_vehicle_method_id = 323;
  constexpr uint8_t velocity_vehicle_length = 28;
  const int velocity_vehicle_udp_size = 36;

  VelocityVehiclePacket velocity_vehicle_packet{};
  static_assert(sizeof(VelocityVehiclePacket) == velocity_vehicle_udp_size);
  velocity_vehicle_packet.header.service_id = velocity_vehicle_service_id;
  velocity_vehicle_packet.header.method_id = velocity_vehicle_method_id;
  velocity_vehicle_packet.header.length = velocity_vehicle_length;
  velocity_vehicle_packet.velocity_vehicle = velocity_kmh;

  std::vector<uint8_t> send_vector(sizeof(VelocityVehiclePacket));
  std::memcpy(send_vector.data(), &velocity_vehicle_packet, sizeof(VelocityVehiclePacket));

  return safe_send(send_vector);
}

Status ContinentalARS548HwInterface::set_yaw_rate(float yaw_rate)
{
  if (yaw_rate < -163.83 || yaw_rate > 163.83) {
    print_error("Invalid yaw rate value");
    return Status::ERROR_1;
  }

  constexpr uint16_t yaw_rate_service_id = 0;
  constexpr uint16_t yaw_rate_method_id = 326;
  constexpr uint8_t yaw_rate_length = 32;
  const int yaw_rate_udp_size = 40;

  YawRatePacket yaw_rate_packet{};
  static_assert(sizeof(YawRatePacket) == yaw_rate_udp_size);
  yaw_rate_packet.header.service_id = yaw_rate_service_id;
  yaw_rate_packet.header.method_id = yaw_rate_method_id;
  yaw_rate_packet.header.length = yaw_rate_length;
  yaw_rate_packet.yaw_rate = yaw_rate;

  std::vector<uint8_t> send_vector(sizeof(YawRatePacket));
  std::memcpy(send_vector.data(), &yaw_rate_packet, sizeof(YawRatePacket));

  return safe_send(send_vector);
}

Status ContinentalARS548HwInterface::safe_send(const std::vector<uint8_t> & buffer)
{
  if (!udp_socket_) {
    logger_->error("UDP socket not initialized");
    return Status::UDP_CONNECTION_ERROR;
  }

  try {
    udp_socket_->send(buffer);
  } catch (const connections::SocketError & e) {
    logger_->error("Failed to send packet: " + std::string(e.what()));
    return Status::UDP_CONNECTION_ERROR;
  }

  return Status::OK;
}
}  // namespace nebula::drivers::continental_ars548
