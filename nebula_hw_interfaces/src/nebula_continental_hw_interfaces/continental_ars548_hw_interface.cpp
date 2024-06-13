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

#include <nebula_common/continental/continental_ars548.hpp>

namespace nebula
{
namespace drivers
{
namespace continental_ars548
{
ContinentalARS548HwInterface::ContinentalARS548HwInterface()
: sensor_io_context_ptr_{new ::drivers::common::IoContext(1)},
  sensor_udp_driver_ptr_{new ::drivers::udp_driver::UdpDriver(*sensor_io_context_ptr_)}
{
}

Status ContinentalARS548HwInterface::SetSensorConfiguration(
  std::shared_ptr<const ContinentalARS548SensorConfiguration> new_config_ptr)
{
  config_ptr_ = new_config_ptr;
  return Status::OK;
}

Status ContinentalARS548HwInterface::SensorInterfaceStart()
{
  try {
    sensor_udp_driver_ptr_->init_receiver(
      config_ptr_->multicast_ip, config_ptr_->data_port, config_ptr_->host_ip,
      config_ptr_->data_port, 2 << 16);
    sensor_udp_driver_ptr_->receiver()->setMulticast(true);
    sensor_udp_driver_ptr_->receiver()->open();
    sensor_udp_driver_ptr_->receiver()->bind();
    sensor_udp_driver_ptr_->receiver()->asyncReceiveWithSender(std::bind(
      &ContinentalARS548HwInterface::ReceiveSensorPacketCallbackWithSender, this,
      std::placeholders::_1, std::placeholders::_2));

    sensor_udp_driver_ptr_->init_sender(
      config_ptr_->sensor_ip, config_ptr_->configuration_sensor_port, config_ptr_->host_ip,
      config_ptr_->configuration_host_port);

    sensor_udp_driver_ptr_->sender()->open();
    sensor_udp_driver_ptr_->sender()->bind();

    if (!sensor_udp_driver_ptr_->sender()->isOpen()) {
      return Status::ERROR_1;
    }
  } catch (const std::exception & ex) {
    Status status = Status::UDP_CONNECTION_ERROR;
    std::cerr << status << config_ptr_->sensor_ip << "," << config_ptr_->data_port << std::endl;
    return status;
  }
  return Status::OK;
}

Status ContinentalARS548HwInterface::RegisterPacketCallback(
  std::function<void(std::unique_ptr<nebula_msgs::msg::NebulaPacket>)> packet_callback)
{
  packet_callback_ = std::move(packet_callback);
  return Status::OK;
}

void ContinentalARS548HwInterface::ReceiveSensorPacketCallbackWithSender(
  std::vector<uint8_t> & buffer, const std::string & sender_ip)
{
  if (sender_ip == config_ptr_->sensor_ip) {
    ReceiveSensorPacketCallback(buffer);
  }
}
void ContinentalARS548HwInterface::ReceiveSensorPacketCallback(std::vector<uint8_t> & buffer)
{
  if (buffer.size() < sizeof(HeaderPacket)) {
    PrintError("Unrecognized packet. Too short");
    return;
  }

  const auto now = std::chrono::high_resolution_clock::now();
  const auto timestamp_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();

  auto msg_ptr = std::make_unique<nebula_msgs::msg::NebulaPacket>();
  msg_ptr->stamp.sec = static_cast<int>(timestamp_ns / 1'000'000'000);
  msg_ptr->stamp.nanosec = static_cast<int>(timestamp_ns % 1'000'000'000);
  msg_ptr->data.swap(buffer);

  packet_callback_(std::move(msg_ptr));
}

Status ContinentalARS548HwInterface::SensorInterfaceStop()
{
  return Status::ERROR_1;
}

Status ContinentalARS548HwInterface::SetSensorMounting(
  float longitudinal_autosar, float lateral_autosar, float vertical_autosar, float yaw_autosar,
  float pitch_autosar, uint8_t plug_orientation)
{
  if (
    longitudinal_autosar < -100.f || longitudinal_autosar > 100.f || lateral_autosar < -100.f ||
    lateral_autosar > 100.f || vertical_autosar < 0.01f || vertical_autosar > 10.f ||
    yaw_autosar < -M_PI || yaw_autosar > M_PI || pitch_autosar < -M_PI_2 ||
    pitch_autosar > M_PI_2) {
    PrintError("Invalid SetSensorMounting values");
    return Status::SENSOR_CONFIG_ERROR;
  }

  ConfigurationPacket configuration_packet{};
  configuration_packet.header.service_id = CONFIGURATION_SERVICE_ID;
  configuration_packet.header.method_id = CONFIGURATION_METHOD_ID;
  configuration_packet.header.length = CONFIGURATION_PAYLOAD_LENGTH;
  configuration_packet.configuration.longitudinal = longitudinal_autosar;
  configuration_packet.configuration.lateral = lateral_autosar;
  configuration_packet.configuration.vertical = vertical_autosar;
  configuration_packet.configuration.yaw = yaw_autosar;
  configuration_packet.configuration.pitch = pitch_autosar;
  configuration_packet.configuration.plug_orientation = plug_orientation;
  configuration_packet.new_sensor_mounting = 1;

  std::vector<uint8_t> send_vector(sizeof(ConfigurationPacket));
  std::memcpy(send_vector.data(), &configuration_packet, sizeof(ConfigurationPacket));

  PrintInfo("longitudinal_autosar = " + std::to_string(longitudinal_autosar));
  PrintInfo("lateral_autosar = " + std::to_string(lateral_autosar));
  PrintInfo("vertical_autosar = " + std::to_string(vertical_autosar));
  PrintInfo("yaw_autosar = " + std::to_string(yaw_autosar));
  PrintInfo("pitch_autosar = " + std::to_string(pitch_autosar));
  PrintInfo("plug_orientation = " + std::to_string(plug_orientation));

  if (!sensor_udp_driver_ptr_->sender()->isOpen()) {
    return Status::ERROR_1;
  }

  sensor_udp_driver_ptr_->sender()->asyncSend(send_vector);

  return Status::OK;
}

Status ContinentalARS548HwInterface::SetVehicleParameters(
  float length_autosar, float width_autosar, float height_autosar, float wheel_base_autosar)
{
  if (
    length_autosar < 0.01f || length_autosar > 100.f || width_autosar < 0.01f ||
    width_autosar > 100.f || height_autosar < 0.01f || height_autosar > 100.f ||
    wheel_base_autosar < 0.01f || wheel_base_autosar > 100.f) {
    PrintError("Invalid SetVehicleParameters values");
    return Status::SENSOR_CONFIG_ERROR;
  }

  ConfigurationPacket configuration_packet{};
  static_assert(sizeof(ConfigurationPacket) == CONFIGURATION_UDP_LENGTH);
  configuration_packet.header.service_id = CONFIGURATION_SERVICE_ID;
  configuration_packet.header.method_id = CONFIGURATION_METHOD_ID;
  configuration_packet.header.length = CONFIGURATION_PAYLOAD_LENGTH;
  configuration_packet.configuration.length = length_autosar;
  configuration_packet.configuration.width = width_autosar;
  configuration_packet.configuration.height = height_autosar;
  configuration_packet.configuration.wheelbase = wheel_base_autosar;
  configuration_packet.new_vehicle_parameters = 1;

  std::vector<uint8_t> send_vector(sizeof(ConfigurationPacket));
  std::memcpy(send_vector.data(), &configuration_packet, sizeof(ConfigurationPacket));

  PrintInfo("length_autosar = " + std::to_string(length_autosar));
  PrintInfo("width_autosar = " + std::to_string(width_autosar));
  PrintInfo("height_autosar = " + std::to_string(height_autosar));
  PrintInfo("wheel_base_autosar = " + std::to_string(wheel_base_autosar));

  if (!sensor_udp_driver_ptr_->sender()->isOpen()) {
    return Status::ERROR_1;
  }

  sensor_udp_driver_ptr_->sender()->asyncSend(send_vector);

  return Status::OK;
}

Status ContinentalARS548HwInterface::SetRadarParameters(
  uint16_t maximum_distance, uint8_t frequency_slot, uint8_t cycle_time, uint8_t time_slot,
  uint8_t hcc, uint8_t power_save_standstill)
{
  if (
    maximum_distance < 93 || maximum_distance > 1514 || frequency_slot > 2 || cycle_time < 50 ||
    cycle_time > 100 || time_slot < 10 || time_slot > 90 || hcc < 1 || hcc > 2 ||
    power_save_standstill > 1) {
    PrintError("Invalid SetRadarParameters values");
    return Status::SENSOR_CONFIG_ERROR;
  }

  ConfigurationPacket configuration_packet{};
  static_assert(sizeof(ConfigurationPacket) == CONFIGURATION_UDP_LENGTH);
  configuration_packet.header.service_id = CONFIGURATION_SERVICE_ID;
  configuration_packet.header.method_id = CONFIGURATION_METHOD_ID;
  configuration_packet.header.length = CONFIGURATION_PAYLOAD_LENGTH;
  configuration_packet.configuration.maximum_distance = maximum_distance;
  configuration_packet.configuration.frequency_slot = frequency_slot;
  configuration_packet.configuration.cycle_time = cycle_time;
  configuration_packet.configuration.time_slot = time_slot;
  configuration_packet.configuration.hcc = hcc;
  configuration_packet.configuration.powersave_standstill = power_save_standstill;
  configuration_packet.new_radar_parameters = 1;

  std::vector<uint8_t> send_vector(sizeof(ConfigurationPacket));
  std::memcpy(send_vector.data(), &configuration_packet, sizeof(ConfigurationPacket));

  PrintInfo("maximum_distance = " + std::to_string(maximum_distance));
  PrintInfo("frequency_slot = " + std::to_string(frequency_slot));
  PrintInfo("cycle_time = " + std::to_string(cycle_time));
  PrintInfo("time_slot = " + std::to_string(time_slot));
  PrintInfo("hcc = " + std::to_string(hcc));
  PrintInfo("power_save_standstill = " + std::to_string(power_save_standstill));

  if (!sensor_udp_driver_ptr_->sender()->isOpen()) {
    return Status::ERROR_1;
  }

  sensor_udp_driver_ptr_->sender()->asyncSend(send_vector);

  return Status::OK;
}

Status ContinentalARS548HwInterface::SetSensorIPAddress(const std::string & sensor_ip_address)
{
  std::array<uint8_t, 4> ip_bytes;

  try {
    auto sensor_ip = boost::asio::ip::address::from_string(sensor_ip_address);
    ip_bytes = sensor_ip.to_v4().to_bytes();
  } catch (const std::exception & ex) {
    PrintError("Setting invalid IP=" + sensor_ip_address);
    return Status::SENSOR_CONFIG_ERROR;
  }

  PrintInfo("New sensor IP = " + sensor_ip_address);

  ConfigurationPacket configuration{};
  static_assert(sizeof(ConfigurationPacket) == CONFIGURATION_UDP_LENGTH);
  configuration.header.service_id = CONFIGURATION_SERVICE_ID;
  configuration.header.method_id = CONFIGURATION_METHOD_ID;
  configuration.header.length = CONFIGURATION_PAYLOAD_LENGTH;
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

  if (!sensor_udp_driver_ptr_->sender()->isOpen()) {
    return Status::ERROR_1;
  }

  sensor_udp_driver_ptr_->sender()->asyncSend(send_vector);

  return Status::OK;
}

Status ContinentalARS548HwInterface::SetAccelerationLateralCog(float lateral_acceleration)
{
  constexpr uint16_t ACCELERATION_LATERAL_COG_SERVICE_ID = 0;
  constexpr uint16_t ACCELERATION_LATERAL_COG_METHOD_ID = 321;
  constexpr uint8_t ACCELERATION_LATERAL_COG_LENGTH = 32;
  const int ACCELERATION_LATERAL_COG_UDP_LENGTH = 40;

  if (lateral_acceleration < -65.f || lateral_acceleration > 65.f) {
    PrintError("Invalid lateral_acceleration value");
    return Status::ERROR_1;
  }

  AccelerationLateralCoGPacket acceleration_lateral_cog{};
  static_assert(sizeof(AccelerationLateralCoGPacket) == ACCELERATION_LATERAL_COG_UDP_LENGTH);
  acceleration_lateral_cog.header.service_id = ACCELERATION_LATERAL_COG_SERVICE_ID;
  acceleration_lateral_cog.header.method_id = ACCELERATION_LATERAL_COG_METHOD_ID;
  acceleration_lateral_cog.header.length = ACCELERATION_LATERAL_COG_LENGTH;
  acceleration_lateral_cog.acceleration_lateral = lateral_acceleration;

  std::vector<uint8_t> send_vector(sizeof(AccelerationLateralCoGPacket));
  std::memcpy(send_vector.data(), &acceleration_lateral_cog, sizeof(AccelerationLateralCoGPacket));

  if (!sensor_udp_driver_ptr_->sender()->isOpen()) {
    return Status::ERROR_1;
  }

  sensor_udp_driver_ptr_->sender()->asyncSend(send_vector);

  return Status::OK;
}

Status ContinentalARS548HwInterface::SetAccelerationLongitudinalCog(float longitudinal_acceleration)
{
  constexpr uint16_t ACCELERATION_LONGITUDINAL_COG_SERVICE_ID = 0;
  constexpr uint16_t ACCELERATION_LONGITUDINAL_COG_METHOD_ID = 322;
  constexpr uint8_t ACCELERATION_LONGITUDINAL_COG_LENGTH = 32;
  const int ACCELERATION_LONGITUDINAL_COG_UDP_LENGTH = 40;

  if (longitudinal_acceleration < -65.f || longitudinal_acceleration > 65.f) {
    PrintError("Invalid longitudinal_acceleration value");
    return Status::ERROR_1;
  }

  AccelerationLongitudinalCoGPacket acceleration_longitudinal_cog{};
  static_assert(
    sizeof(AccelerationLongitudinalCoGPacket) == ACCELERATION_LONGITUDINAL_COG_UDP_LENGTH);
  acceleration_longitudinal_cog.header.service_id = ACCELERATION_LONGITUDINAL_COG_SERVICE_ID;
  acceleration_longitudinal_cog.header.method_id = ACCELERATION_LONGITUDINAL_COG_METHOD_ID;
  acceleration_longitudinal_cog.header.length = ACCELERATION_LONGITUDINAL_COG_LENGTH;
  acceleration_longitudinal_cog.acceleration_lateral = longitudinal_acceleration;

  std::vector<uint8_t> send_vector(sizeof(AccelerationLongitudinalCoGPacket));
  std::memcpy(
    send_vector.data(), &acceleration_longitudinal_cog, sizeof(AccelerationLongitudinalCoGPacket));

  if (!sensor_udp_driver_ptr_->sender()->isOpen()) {
    return Status::ERROR_1;
  }

  sensor_udp_driver_ptr_->sender()->asyncSend(send_vector);

  return Status::OK;
}

Status ContinentalARS548HwInterface::SetCharacteristicSpeed(float characteristic_speed)
{
  constexpr uint16_t CHARACTERISTIC_SPEED_SERVICE_ID = 0;
  constexpr uint16_t CHARACTERISTIC_SPEED_METHOD_ID = 328;
  constexpr uint8_t CHARACTERISTIC_SPEED_LENGTH = 11;
  const int CHARACTERISTIC_SPEED_UDP_LENGTH = 19;

  if (characteristic_speed < 0.f || characteristic_speed > 255.f) {
    PrintError("Invalid characteristic_speed value");
    return Status::ERROR_1;
  }

  CharacteristicSpeedPacket characteristic_speed_packet{};
  static_assert(sizeof(CharacteristicSpeedPacket) == CHARACTERISTIC_SPEED_UDP_LENGTH);
  characteristic_speed_packet.header.service_id = CHARACTERISTIC_SPEED_SERVICE_ID;
  characteristic_speed_packet.header.method_id = CHARACTERISTIC_SPEED_METHOD_ID;
  characteristic_speed_packet.header.length = CHARACTERISTIC_SPEED_LENGTH;
  characteristic_speed_packet.characteristic_speed = characteristic_speed;

  std::vector<uint8_t> send_vector(sizeof(CharacteristicSpeedPacket));
  std::memcpy(send_vector.data(), &characteristic_speed_packet, sizeof(CharacteristicSpeedPacket));

  if (!sensor_udp_driver_ptr_->sender()->isOpen()) {
    return Status::ERROR_1;
  }

  sensor_udp_driver_ptr_->sender()->asyncSend(send_vector);

  return Status::OK;
}

Status ContinentalARS548HwInterface::SetDrivingDirection(int direction)
{
  constexpr uint16_t DRIVING_DIRECTION_SERVICE_ID = 0;
  constexpr uint16_t DRIVING_DIRECTION_METHOD_ID = 325;
  constexpr uint8_t DRIVING_DIRECTION_LENGTH = 22;
  constexpr uint8_t DRIVING_DIRECTION_STANDSTILL = 0;
  constexpr uint8_t DRIVING_DIRECTION_FORWARD = 1;
  constexpr uint8_t DRIVING_DIRECTION_BACKWARDS = 2;
  const int DRIVING_DIRECTION_UDP_LENGTH = 30;

  DrivingDirectionPacket driving_direction_packet{};
  static_assert(sizeof(DrivingDirectionPacket) == DRIVING_DIRECTION_UDP_LENGTH);
  driving_direction_packet.header.service_id = DRIVING_DIRECTION_SERVICE_ID;
  driving_direction_packet.header.method_id = DRIVING_DIRECTION_METHOD_ID;
  driving_direction_packet.header.length = DRIVING_DIRECTION_LENGTH;

  if (direction == 0) {
    driving_direction_packet.driving_direction = DRIVING_DIRECTION_STANDSTILL;
  } else if (direction > 0) {
    driving_direction_packet.driving_direction = DRIVING_DIRECTION_FORWARD;
  } else {
    driving_direction_packet.driving_direction = DRIVING_DIRECTION_BACKWARDS;
  }

  std::vector<uint8_t> send_vector(sizeof(DrivingDirectionPacket));
  std::memcpy(send_vector.data(), &driving_direction_packet, sizeof(DrivingDirectionPacket));

  if (!sensor_udp_driver_ptr_->sender()->isOpen()) {
    return Status::ERROR_1;
  }

  sensor_udp_driver_ptr_->sender()->asyncSend(send_vector);

  return Status::OK;
}

Status ContinentalARS548HwInterface::SetSteeringAngleFrontAxle(float angle_rad)
{
  constexpr uint16_t STEERING_ANGLE_SERVICE_ID = 0;
  constexpr uint16_t STEERING_ANGLE_METHOD_ID = 327;
  constexpr uint8_t STEERING_ANGLE_LENGTH = 32;
  const int STEERING_ANGLE_UDP_LENGTH = 40;

  if (angle_rad < -90.f || angle_rad > 90.f) {
    PrintError("Invalid angle_rad value");
    return Status::ERROR_1;
  }

  SteeringAngleFrontAxlePacket steering_angle_front_axle_packet{};
  static_assert(sizeof(SteeringAngleFrontAxlePacket) == STEERING_ANGLE_UDP_LENGTH);
  steering_angle_front_axle_packet.header.service_id = STEERING_ANGLE_SERVICE_ID;
  steering_angle_front_axle_packet.header.method_id = STEERING_ANGLE_METHOD_ID;
  steering_angle_front_axle_packet.header.length = STEERING_ANGLE_LENGTH;
  steering_angle_front_axle_packet.steering_angle_front_axle = angle_rad;

  std::vector<uint8_t> send_vector(sizeof(SteeringAngleFrontAxlePacket));
  std::memcpy(
    send_vector.data(), &steering_angle_front_axle_packet, sizeof(SteeringAngleFrontAxlePacket));

  if (!sensor_udp_driver_ptr_->sender()->isOpen()) {
    return Status::ERROR_1;
  }

  sensor_udp_driver_ptr_->sender()->asyncSend(send_vector);

  return Status::OK;
}

Status ContinentalARS548HwInterface::SetVelocityVehicle(float velocity_kmh)
{
  if (velocity_kmh < 0.f || velocity_kmh > 350.f) {
    PrintError("Invalid velocity value");
    return Status::ERROR_1;
  }

  constexpr uint16_t VELOCITY_VEHICLE_SERVICE_ID = 0;
  constexpr uint16_t VELOCITY_VEHICLE_METHOD_ID = 323;
  constexpr uint8_t VELOCITY_VEHICLE_LENGTH = 28;
  const int VELOCITY_VEHICLE_UDP_SIZE = 36;

  VelocityVehiclePacket steering_angle_front_axle_packet{};
  static_assert(sizeof(VelocityVehiclePacket) == VELOCITY_VEHICLE_UDP_SIZE);
  steering_angle_front_axle_packet.header.service_id = VELOCITY_VEHICLE_SERVICE_ID;
  steering_angle_front_axle_packet.header.method_id = VELOCITY_VEHICLE_METHOD_ID;
  steering_angle_front_axle_packet.header.length = VELOCITY_VEHICLE_LENGTH;
  steering_angle_front_axle_packet.velocity_vehicle = velocity_kmh;

  std::vector<uint8_t> send_vector(sizeof(VelocityVehiclePacket));
  std::memcpy(send_vector.data(), &steering_angle_front_axle_packet, sizeof(VelocityVehiclePacket));

  if (!sensor_udp_driver_ptr_->sender()->isOpen()) {
    return Status::ERROR_1;
  }

  sensor_udp_driver_ptr_->sender()->asyncSend(send_vector);

  return Status::OK;
}

Status ContinentalARS548HwInterface::SetYawRate(float yaw_rate)
{
  if (yaw_rate < -163.83 || yaw_rate > 163.83) {
    PrintError("Invalid yaw rate value");
    return Status::ERROR_1;
  }

  constexpr uint16_t YAW_RATE_SERVICE_ID = 0;
  constexpr uint16_t YAW_RATE_METHOD_ID = 326;
  constexpr uint8_t YAW_RATE_LENGTH = 32;
  const int YAW_RATE_UDP_SIZE = 40;

  YawRatePacket yaw_rate_packet{};
  static_assert(sizeof(YawRatePacket) == YAW_RATE_UDP_SIZE);
  yaw_rate_packet.header.service_id = YAW_RATE_SERVICE_ID;
  yaw_rate_packet.header.method_id = YAW_RATE_METHOD_ID;
  yaw_rate_packet.header.length = YAW_RATE_LENGTH;
  yaw_rate_packet.yaw_rate = yaw_rate;

  std::vector<uint8_t> send_vector(sizeof(YawRatePacket));
  std::memcpy(send_vector.data(), &yaw_rate_packet, sizeof(YawRatePacket));

  if (!sensor_udp_driver_ptr_->sender()->isOpen()) {
    return Status::ERROR_1;
  }

  sensor_udp_driver_ptr_->sender()->asyncSend(send_vector);

  return Status::OK;
}

void ContinentalARS548HwInterface::SetLogger(std::shared_ptr<rclcpp::Logger> logger)
{
  parent_node_logger_ptr_ = logger;
}

void ContinentalARS548HwInterface::PrintInfo(std::string info)
{
  if (parent_node_logger_ptr_) {
    RCLCPP_INFO_STREAM((*parent_node_logger_ptr_), info);
  } else {
    std::cout << info << std::endl;
  }
}

void ContinentalARS548HwInterface::PrintError(std::string error)
{
  if (parent_node_logger_ptr_) {
    RCLCPP_ERROR_STREAM((*parent_node_logger_ptr_), error);
  } else {
    std::cerr << error << std::endl;
  }
}

void ContinentalARS548HwInterface::PrintDebug(std::string debug)
{
  if (parent_node_logger_ptr_) {
    RCLCPP_DEBUG_STREAM((*parent_node_logger_ptr_), debug);
  } else {
    std::cout << debug << std::endl;
  }
}

}  // namespace continental_ars548
}  // namespace drivers
}  // namespace nebula
