// Copyright 2024 Tier IV, Inc.
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

#include <nebula_common/continental/continental_ars548.hpp>
#include <nebula_hw_interfaces/nebula_hw_interfaces_continental/multi_continental_ars548_hw_interface.hpp>

#include <limits>
#include <sstream>
namespace nebula
{
namespace drivers
{
namespace continental_ars548
{
MultiContinentalARS548HwInterface::MultiContinentalARS548HwInterface()
: sensor_io_context_{new ::drivers::common::IoContext(1)},
  nebula_packets_ptr_{std::make_unique<nebula_msgs::msg::NebulaPackets>()}
{
}

Status MultiContinentalARS548HwInterface::SetSensorConfiguration(
  std::shared_ptr<SensorConfigurationBase> sensor_configuration)
{
  Status status = Status::OK;

  try {
    sensor_configuration_ =
      std::static_pointer_cast<MultiContinentalARS548SensorConfiguration>(sensor_configuration);
  } catch (const std::exception & ex) {
    status = Status::SENSOR_CONFIG_ERROR;
    std::cerr << status << std::endl;
    return status;
  }

  return Status::OK;
}

Status MultiContinentalARS548HwInterface::SensorInterfaceStart()
{
  for (std::size_t sensor_id = 0; sensor_id < sensor_configuration_->sensor_ips.size();
       sensor_id++) {
    auto udp_driver = std::make_unique<::drivers::udp_driver::UdpDriver>(*sensor_io_context_);
    sensor_ip_to_frame_[sensor_configuration_->sensor_ips[sensor_id]] =
      sensor_configuration_->frame_ids[sensor_id];

    try {
      if (sensor_id == 0) {
        udp_driver->init_receiver(
          sensor_configuration_->multicast_ip, sensor_configuration_->data_port,
          sensor_configuration_->host_ip, sensor_configuration_->data_port, 2 << 16);
        udp_driver->receiver()->setMulticast(true);
        udp_driver->receiver()->open();
        udp_driver->receiver()->bind();
        udp_driver->receiver()->asyncReceiveWithSender(std::bind(
          &MultiContinentalARS548HwInterface::ReceiveSensorPacketCallback, this,
          std::placeholders::_1, std::placeholders::_2));
      }

      udp_driver->init_sender(
        sensor_configuration_->sensor_ips[sensor_id],
        sensor_configuration_->configuration_sensor_port, sensor_configuration_->host_ip,
        sensor_configuration_->configuration_host_port);

      udp_driver->sender()->open();
      udp_driver->sender()->bind();

      if (!udp_driver->sender()->isOpen()) {
        return Status::ERROR_1;
      }
    } catch (const std::exception & ex) {
      Status status = Status::UDP_CONNECTION_ERROR;
      std::cerr << status << sensor_configuration_->sensor_ip << ","
                << sensor_configuration_->data_port << std::endl;
      return status;
    }

    sensor_udp_drivers_.emplace_back(std::move(udp_driver));
  }

  return Status::OK;
}

Status MultiContinentalARS548HwInterface::RegisterScanCallback(
  std::function<void(std::unique_ptr<nebula_msgs::msg::NebulaPackets>, const std::string &)>
    callback)
{
  nebula_packets_reception_callback_ = std::move(callback);
  return Status::OK;
}

void MultiContinentalARS548HwInterface::ReceiveSensorPacketCallback(
  const std::vector<uint8_t> & buffer, const std::string & sender_ip)
{
  if (buffer.size() < sizeof(HeaderPacket)) {
    PrintError("Unrecognized packet. Too short");
    return;
  }

  HeaderPacket header_packet{};
  std::memcpy(&header_packet, buffer.data(), sizeof(HeaderPacket));

  if (header_packet.service_id.value() != 0) {
    PrintError("Invalid service id");
    return;
  } else if (header_packet.method_id.value() == SENSOR_STATUS_METHOD_ID) {
    if (
      buffer.size() != SENSOR_STATUS_UDP_PAYLOAD ||
      header_packet.length.value() != SENSOR_STATUS_PDU_LENGTH) {
      PrintError("SensorStatus message with invalid size");
      return;
    }
    ProcessDataPacket(buffer, sender_ip);
  } else if (header_packet.method_id.value() == FILTER_STATUS_METHOD_ID) {
    if (
      buffer.size() != FILTER_STATUS_UDP_PAYLOAD ||
      header_packet.length.value() != FILTER_STATUS_PDU_LENGTH) {
      PrintError("FilterStatus message with invalid size");
      return;
    }

    ProcessFilterStatusPacket(buffer);
  } else if (header_packet.method_id.value() == DETECTION_LIST_METHOD_ID) {
    if (
      buffer.size() != DETECTION_LIST_UDP_PAYLOAD ||
      header_packet.length.value() != DETECTION_LIST_PDU_LENGTH) {
      PrintError("DetectionList message with invalid size");
      return;
    }

    ProcessDataPacket(buffer, sender_ip);
  } else if (header_packet.method_id.value() == OBJECT_LIST_METHOD_ID) {
    if (
      buffer.size() != OBJECT_LIST_UDP_PAYLOAD ||
      header_packet.length.value() != OBJECT_LIST_PDU_LENGTH) {
      PrintError("ObjectList message with invalid size");
      return;
    }

    ProcessDataPacket(buffer, sender_ip);
  }
}

void MultiContinentalARS548HwInterface::ProcessFilterStatusPacket(
  const std::vector<uint8_t> & buffer)
{
  assert(buffer.size() == sizeof(FilterStatusPacket));
  std::memcpy(&filter_status_, buffer.data(), sizeof(FilterStatusPacket));
}

void MultiContinentalARS548HwInterface::ProcessDataPacket(
  const std::vector<uint8_t> & buffer, const std::string & sensor_ip)
{
  nebula_msgs::msg::NebulaPacket nebula_packet;
  nebula_packet.data = buffer;
  auto now = std::chrono::system_clock::now();
  auto now_secs = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
  auto now_nanosecs =
    std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
  nebula_packet.stamp.sec = static_cast<int>(now_secs);
  nebula_packet.stamp.nanosec =
    static_cast<int>((now_nanosecs / 1000000000.0 - static_cast<double>(now_secs)) * 1000000000);
  nebula_packets_ptr_->packets.emplace_back(nebula_packet);

  nebula_packets_ptr_->header.stamp = nebula_packets_ptr_->packets.front().stamp;
  nebula_packets_ptr_->header.frame_id = sensor_configuration_->frame_id;

  nebula_packets_reception_callback_(std::move(nebula_packets_ptr_), sensor_ip);
  nebula_packets_ptr_ = std::make_unique<nebula_msgs::msg::NebulaPackets>();
}

Status MultiContinentalARS548HwInterface::SensorInterfaceStop()
{
  return Status::ERROR_1;
}

Status MultiContinentalARS548HwInterface::GetSensorConfiguration(
  SensorConfigurationBase & sensor_configuration)
{
  std::stringstream ss;
  ss << sensor_configuration;
  PrintDebug(ss.str());
  return Status::ERROR_1;
}

Status MultiContinentalARS548HwInterface::SetAccelerationLateralCog(float lateral_acceleration)
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

  for (auto & udp_driver : sensor_udp_drivers_) {
    udp_driver->sender()->asyncSend(send_vector);
  }

  return Status::OK;
}

Status MultiContinentalARS548HwInterface::SetAccelerationLongitudinalCog(
  float longitudinal_acceleration)
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

  for (auto & udp_driver : sensor_udp_drivers_) {
    udp_driver->sender()->asyncSend(send_vector);
  }

  return Status::OK;
}

Status MultiContinentalARS548HwInterface::SetCharacteristicSpeed(float characteristic_speed)
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

  for (auto & udp_driver : sensor_udp_drivers_) {
    udp_driver->sender()->asyncSend(send_vector);
  }

  return Status::OK;
}

Status MultiContinentalARS548HwInterface::SetDrivingDirection(int direction)
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

  for (auto & udp_driver : sensor_udp_drivers_) {
    udp_driver->sender()->asyncSend(send_vector);
  }

  return Status::OK;
}

Status MultiContinentalARS548HwInterface::SetSteeringAngleFrontAxle(float angle_rad)
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

  for (auto & udp_driver : sensor_udp_drivers_) {
    udp_driver->sender()->asyncSend(send_vector);
  }

  return Status::OK;
}

Status MultiContinentalARS548HwInterface::SetVelocityVehicle(float velocity_kmh)
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

  for (auto & udp_driver : sensor_udp_drivers_) {
    udp_driver->sender()->asyncSend(send_vector);
  }

  return Status::OK;
}

Status MultiContinentalARS548HwInterface::SetYawRate(float yaw_rate)
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

  for (auto & udp_driver : sensor_udp_drivers_) {
    udp_driver->sender()->asyncSend(send_vector);
  }

  return Status::OK;
}

Status MultiContinentalARS548HwInterface::CheckAndSetConfig()
{
  RCLCPP_ERROR(
    *parent_node_logger,
    "This functionality is not yet implemented. Sensor is probably out of sync with config now.");
  return Status::ERROR_1;
}

void MultiContinentalARS548HwInterface::SetLogger(std::shared_ptr<rclcpp::Logger> logger)
{
  parent_node_logger = logger;
}

void MultiContinentalARS548HwInterface::PrintInfo(std::string info)
{
  if (parent_node_logger) {
    RCLCPP_INFO_STREAM((*parent_node_logger), info);
  } else {
    std::cout << info << std::endl;
  }
}

void MultiContinentalARS548HwInterface::PrintError(std::string error)
{
  if (parent_node_logger) {
    RCLCPP_ERROR_STREAM((*parent_node_logger), error);
  } else {
    std::cerr << error << std::endl;
  }
}

void MultiContinentalARS548HwInterface::PrintDebug(std::string debug)
{
  if (parent_node_logger) {
    RCLCPP_DEBUG_STREAM((*parent_node_logger), debug);
  } else {
    std::cout << debug << std::endl;
  }
}

void MultiContinentalARS548HwInterface::PrintDebug(const std::vector<uint8_t> & bytes)
{
  std::stringstream ss;
  for (const auto & b : bytes) {
    ss << static_cast<int>(b) << ", ";
  }
  ss << std::endl;
  PrintDebug(ss.str());
}

}  // namespace continental_ars548
}  // namespace drivers
}  // namespace nebula
