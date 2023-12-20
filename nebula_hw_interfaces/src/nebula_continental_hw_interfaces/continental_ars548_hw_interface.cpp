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

#include "nebula_hw_interfaces/nebula_hw_interfaces_continental/continental_ars548_hw_interface.hpp"

#include "nebula_common/continental/continental_ars548.hpp"

#include <limits>

namespace nebula
{
namespace drivers
{
namespace continental_ars548
{
ContinentalARS548HwInterface::ContinentalARS548HwInterface()
: cloud_io_context_{new ::drivers::common::IoContext(1)},
  sensor_udp_driver_{new ::drivers::udp_driver::UdpDriver(*cloud_io_context_)},
  nebula_packets_ptr_{std::make_unique<nebula_msgs::msg::NebulaPackets>()}
{
}

Status ContinentalARS548HwInterface::SetSensorConfiguration(
  std::shared_ptr<SensorConfigurationBase> sensor_configuration)
{
  Status status = Status::OK;

  try {
    sensor_configuration_ =
      std::static_pointer_cast<ContinentalARS548SensorConfiguration>(sensor_configuration);
  } catch (const std::exception & ex) {
    status = Status::SENSOR_CONFIG_ERROR;
    std::cerr << status << std::endl;
    return status;
  }

  return Status::OK;
}

Status ContinentalARS548HwInterface::CloudInterfaceStart()
{
  try {
    sensor_udp_driver_->init_receiver(
      sensor_configuration_->multicast_ip, sensor_configuration_->data_port,
      sensor_configuration_->host_ip, sensor_configuration_->data_port, 2 << 16);
    sensor_udp_driver_->receiver()->setMulticast(true);
    sensor_udp_driver_->receiver()->open();
    sensor_udp_driver_->receiver()->bind();
    sensor_udp_driver_->receiver()->asyncReceiveWithSender(std::bind(
      &ContinentalARS548HwInterface::ReceiveCloudPacketCallbackWithSender, this,
      std::placeholders::_1, std::placeholders::_2));

    sensor_udp_driver_->init_sender(
      sensor_configuration_->sensor_ip, sensor_configuration_->configuration_sensor_port,
      sensor_configuration_->host_ip, sensor_configuration_->configuration_host_port);

    sensor_udp_driver_->sender()->open();
    sensor_udp_driver_->sender()->bind();

    if (!sensor_udp_driver_->sender()->isOpen()) {
      return Status::ERROR_1;
    }
  } catch (const std::exception & ex) {
    Status status = Status::UDP_CONNECTION_ERROR;
    std::cerr << status << sensor_configuration_->sensor_ip << ","
              << sensor_configuration_->data_port << std::endl;
    return status;
  }
  return Status::OK;
}

Status ContinentalARS548HwInterface::RegisterScanCallback(
  std::function<void(std::unique_ptr<nebula_msgs::msg::NebulaPackets>)> callback)
{
  nebula_packets_reception_callback_ = std::move(callback);
  return Status::OK;
}

void ContinentalARS548HwInterface::ReceiveCloudPacketCallbackWithSender(
  const std::vector<uint8_t> & buffer, const std::string & sender_ip)
{
  if (sender_ip == sensor_configuration_->sensor_ip) {
    ReceiveCloudPacketCallback(buffer);
  }
}
void ContinentalARS548HwInterface::ReceiveCloudPacketCallback(const std::vector<uint8_t> & buffer)
{
  /*constexpr int DETECTION_LIST_METHOD_ID = 336;
  constexpr int OBJECT_LIST_METHOD_ID = 329;
  constexpr int SENSOR_STATUS_METHOD_ID = 380;
  constexpr int FILTER_STATUS_METHOD_ID = 396;

  constexpr int DETECTION_LIST_UDP_PAYLOAD = 35336;
  constexpr int OBJECT_LIST_UDP_PAYLOAD = 9401;
  constexpr int SENSOR_STATUS_UDP_PAYLOAD = 84;
  constexpr int FILTER_STATUS_UDP_PAYLOAD = 330;

  constexpr int DETECTION_LIST_PDU_LENGTH = 35328;
  constexpr int OBJECT_LIST_PDU_LENGTH = 9393;
  constexpr int SENSOR_STATUS_PDU_LENGTH = 76;
  constexpr int FILTER_STATUS_PDU_LENGTH = 322;

  */

  if (buffer.size() < sizeof(Header)) {
    PrintError("Unrecognized packet. Too short");
    return;
  }

  Header header{};
  std::memcpy(&header, buffer.data(), sizeof(Header));

  const uint16_t service_id =
    (static_cast<uint32_t>(buffer[SERVICE_ID_BYTE]) << 8) | buffer[SERVICE_ID_BYTE + 1];
  const uint16_t method_id =
    (static_cast<uint32_t>(buffer[METHOD_ID_BYTE]) << 8) | buffer[METHOD_ID_BYTE + 1];
  const uint32_t length = (static_cast<uint32_t>(buffer[LENGTH_BYTE]) << 24) |
                          (static_cast<uint32_t>(buffer[LENGTH_BYTE + 1]) << 16) |
                          (static_cast<uint32_t>(buffer[LENGTH_BYTE + 2]) << 8) |
                          buffer[LENGTH_BYTE + 3];

  assert(header.service_id.value() == service_id);
  assert(header.method_id.value() == method_id);
  assert(header.length.value() == length);

  if (service_id != 0) {
    PrintError("Invalid service id");
    return;
  } else if (method_id == SENSOR_STATUS_METHOD_ID) {
    if (buffer.size() != SENSOR_STATUS_UDP_PAYLOAD || length != SENSOR_STATUS_PDU_LENGTH) {
      PrintError("SensorStatus message with invalid size");
      return;
    }
    ProcessSensorStatusPacket(buffer);
  } else if (method_id == FILTER_STATUS_METHOD_ID) {
    if (buffer.size() != FILTER_STATUS_UDP_PAYLOAD || length != FILTER_STATUS_PDU_LENGTH) {
      PrintError("FilterStatus message with invalid size");
      return;
    }

    ProcessFilterStatusPacket(buffer);
  } else if (method_id == DETECTION_LIST_METHOD_ID) {
    if (buffer.size() != DETECTION_LIST_UDP_PAYLOAD || length != DETECTION_LIST_PDU_LENGTH) {
      PrintError("DetectionList message with invalid size");
      return;
    }

    ProcessDataPacket(buffer);
  } else if (method_id == OBJECT_LIST_METHOD_ID) {
    if (buffer.size() != OBJECT_LIST_UDP_PAYLOAD || length != OBJECT_LIST_PDU_LENGTH) {
      PrintError("ObjectList message with invalid size");
      return;
    }

    ProcessDataPacket(buffer);
  }
}

void ContinentalARS548HwInterface::ProcessSensorStatusPacket(const std::vector<uint8_t> & buffer)
{
  radar_status_.timestamp_nanoseconds =
    (static_cast<uint32_t>(buffer[STATUS_TIMESTAMP_NANOSECONDS_BYTE]) << 24) |
    (static_cast<uint32_t>(buffer[STATUS_TIMESTAMP_NANOSECONDS_BYTE + 1]) << 16) |
    (static_cast<uint32_t>(buffer[STATUS_TIMESTAMP_NANOSECONDS_BYTE + 2]) << 8) |
    buffer[STATUS_TIMESTAMP_NANOSECONDS_BYTE + 3];
  radar_status_.timestamp_seconds =
    (static_cast<uint32_t>(buffer[STATUS_TIMESTAMP_SECONDS_BYTE]) << 24) |
    (static_cast<uint32_t>(buffer[STATUS_TIMESTAMP_SECONDS_BYTE + 1]) << 16) |
    (static_cast<uint32_t>(buffer[STATUS_TIMESTAMP_SECONDS_BYTE + 2]) << 8) |
    buffer[STATUS_TIMESTAMP_SECONDS_BYTE + 3];

  const uint8_t & sync_status = buffer[STATUS_SYNC_STATUS_BYTE];

  SensorStatus sensor_status{};
  std::memcpy(&sensor_status, buffer.data(), sizeof(SensorStatus));
  assert(sensor_status.stamp.timestamp_nanoseconds.value() == radar_status_.timestamp_nanoseconds);
  assert(sensor_status.stamp.timestamp_seconds.value() == radar_status_.timestamp_seconds);
  assert(sync_status == sensor_status.stamp.timestamp_sync_status);

  if (sync_status == 1) {
    radar_status_.timestamp_sync_status = "SYNC_OK";
  } else if (sync_status == 2) {
    radar_status_.timestamp_sync_status = "NEVER_SYNC";
  } else if (sync_status == 3) {
    radar_status_.timestamp_sync_status = "SYNC_LOST";
  } else {
    radar_status_.timestamp_sync_status = "INVALID_VALUE";
  }

  radar_status_.sw_version_major = buffer[STATUS_SW_VERSION_MAJOR_BYTE];
  radar_status_.sw_version_minor = buffer[STATUS_SW_VERSION_MINOR_BYTE];
  radar_status_.sw_version_patch = buffer[STATUS_SW_VERSION_PATCH_BYTE];

  assert(sensor_status.sw_version_major == radar_status_.sw_version_major);
  assert(sensor_status.sw_version_minor == radar_status_.sw_version_minor);
  assert(sensor_status.sw_version_patch == radar_status_.sw_version_patch);

  const uint32_t status_longitudinal_u =
    (static_cast<uint32_t>(buffer[STATUS_LONGITUDINAL_BYTE]) << 24) |
    (static_cast<uint32_t>(buffer[STATUS_LONGITUDINAL_BYTE + 1]) << 16) |
    (static_cast<uint32_t>(buffer[STATUS_LONGITUDINAL_BYTE + 2]) << 8) |
    buffer[STATUS_LONGITUDINAL_BYTE + 3];
  const uint32_t status_lateral_u = (static_cast<uint32_t>(buffer[STATUS_LATERAL_BYTE]) << 24) |
                                    (static_cast<uint32_t>(buffer[STATUS_LATERAL_BYTE + 1]) << 16) |
                                    (static_cast<uint32_t>(buffer[STATUS_LATERAL_BYTE + 2]) << 8) |
                                    buffer[STATUS_LATERAL_BYTE + 3];
  const uint32_t status_vertical_u =
    (static_cast<uint32_t>(buffer[STATUS_VERTICAL_BYTE]) << 24) |
    (static_cast<uint32_t>(buffer[STATUS_VERTICAL_BYTE + 1]) << 16) |
    (static_cast<uint32_t>(buffer[STATUS_VERTICAL_BYTE + 2]) << 8) |
    buffer[STATUS_VERTICAL_BYTE + 3];
  const uint32_t status_yaw_u = (static_cast<uint32_t>(buffer[STATUS_YAW_BYTE]) << 24) |
                                (static_cast<uint32_t>(buffer[STATUS_YAW_BYTE + 1]) << 16) |
                                (static_cast<uint32_t>(buffer[STATUS_YAW_BYTE + 2]) << 8) |
                                buffer[STATUS_YAW_BYTE + 3];
  const uint32_t status_pitch_u = (static_cast<uint32_t>(buffer[STATUS_PITCH_BYTE]) << 24) |
                                  (static_cast<uint32_t>(buffer[STATUS_PITCH_BYTE + 1]) << 16) |
                                  (static_cast<uint32_t>(buffer[STATUS_PITCH_BYTE + 2]) << 8) |
                                  buffer[STATUS_PITCH_BYTE + 3];
  const uint8_t & plug_orientation = buffer[STATUS_PLUG_ORIENTATION_BYTE];
  radar_status_.plug_orientation = plug_orientation == 0   ? "PLUG_RIGHT"
                                   : plug_orientation == 1 ? "PLUG_LEFT"
                                                           : "INVALID_VALUE";
  const uint32_t status_length_u = (static_cast<uint32_t>(buffer[STATUS_LENGTH_BYTE]) << 24) |
                                   (static_cast<uint32_t>(buffer[STATUS_LENGTH_BYTE + 1]) << 16) |
                                   (static_cast<uint32_t>(buffer[STATUS_LENGTH_BYTE + 2]) << 8) |
                                   buffer[STATUS_LENGTH_BYTE + 3];
  const uint32_t status_width_u = (static_cast<uint32_t>(buffer[STATUS_WIDTH_BYTE]) << 24) |
                                  (static_cast<uint32_t>(buffer[STATUS_WIDTH_BYTE + 1]) << 16) |
                                  (static_cast<uint32_t>(buffer[STATUS_WIDTH_BYTE + 2]) << 8) |
                                  buffer[STATUS_WIDTH_BYTE + 3];
  const uint32_t status_height_u = (static_cast<uint32_t>(buffer[STATUS_HEIGHT_BYTE]) << 24) |
                                   (static_cast<uint32_t>(buffer[STATUS_HEIGHT_BYTE + 1]) << 16) |
                                   (static_cast<uint32_t>(buffer[STATUS_HEIGHT_BYTE + 2]) << 8) |
                                   buffer[STATUS_HEIGHT_BYTE + 3];
  const uint32_t status_wheel_base_u =
    (static_cast<uint32_t>(buffer[STATUS_WHEEL_BASE_BYTE]) << 24) |
    (static_cast<uint32_t>(buffer[STATUS_WHEEL_BASE_BYTE + 1]) << 16) |
    (static_cast<uint32_t>(buffer[STATUS_WHEEL_BASE_BYTE + 2]) << 8) |
    buffer[STATUS_WHEEL_BASE_BYTE + 3];
  radar_status_.max_distance = (static_cast<uint32_t>(buffer[STATUS_MAXIMUM_DISTANCE_BYTE]) << 8) |
                               buffer[STATUS_MAXIMUM_DISTANCE_BYTE + 1];

  std::memcpy(&radar_status_.longitudinal, &status_longitudinal_u, sizeof(status_longitudinal_u));
  std::memcpy(&radar_status_.lateral, &status_lateral_u, sizeof(status_lateral_u));
  std::memcpy(&radar_status_.vertical, &status_vertical_u, sizeof(status_vertical_u));
  std::memcpy(&radar_status_.yaw, &status_yaw_u, sizeof(status_yaw_u));

  std::memcpy(&radar_status_.pitch, &status_pitch_u, sizeof(status_pitch_u));
  std::memcpy(&radar_status_.length, &status_length_u, sizeof(status_length_u));
  std::memcpy(&radar_status_.width, &status_width_u, sizeof(status_width_u));
  std::memcpy(&radar_status_.height, &status_height_u, sizeof(status_height_u));
  std::memcpy(&radar_status_.wheel_base, &status_wheel_base_u, sizeof(status_wheel_base_u));

  const uint8_t & frequency_slot = buffer[STATUS_FREQUENCY_SLOT_BYTE];

  if (frequency_slot == 0) {
    radar_status_.frequency_slot = "0:Low (76.23 GHz)";
  } else if (frequency_slot == 1) {
    radar_status_.frequency_slot = "1:Mid (76.48 GHz)";
  } else if (frequency_slot == 2) {
    radar_status_.frequency_slot = "2:High (76.73 GHz)";
  } else {
    radar_status_.frequency_slot = "INVALID VALUE";
  }

  radar_status_.cycle_time = buffer[STATUS_CYCLE_TIME_BYTE];
  radar_status_.time_slot = buffer[STATUS_TIME_SLOT_BYTE];

  const uint8_t & hcc = buffer[STATUS_HCC_BYTE];

  radar_status_.hcc = hcc == 1   ? "Worldwide"
                      : hcc == 2 ? "Japan"
                                 : ("INVALID VALUE=" + std::to_string(hcc));

  const uint8_t & power_save_standstill = buffer[STATUS_POWER_SAVING_STANDSTILL_BYTE];
  radar_status_.power_save_standstill = power_save_standstill == 0   ? "Off"
                                        : power_save_standstill == 1 ? "On"
                                                                     : "INVALID VALUE";

  const uint8_t status_sensor_ip_address00 = buffer[STATUS_SENSOR_IP_ADDRESS0_BYTE];
  const uint8_t status_sensor_ip_address01 = buffer[STATUS_SENSOR_IP_ADDRESS0_BYTE + 1];
  const uint8_t status_sensor_ip_address02 = buffer[STATUS_SENSOR_IP_ADDRESS0_BYTE + 2];
  const uint8_t status_sensor_ip_address03 = buffer[STATUS_SENSOR_IP_ADDRESS0_BYTE + 3];
  const uint8_t status_sensor_ip_address10 = buffer[STATUS_SENSOR_IP_ADDRESS1_BYTE];
  const uint8_t status_sensor_ip_address11 = buffer[STATUS_SENSOR_IP_ADDRESS1_BYTE + 1];
  const uint8_t status_sensor_ip_address12 = buffer[STATUS_SENSOR_IP_ADDRESS1_BYTE + 2];
  const uint8_t status_sensor_ip_address13 = buffer[STATUS_SENSOR_IP_ADDRESS1_BYTE + 3];

  std::stringstream ss0, ss1;
  ss0 << std::to_string(status_sensor_ip_address00) << "."
      << std::to_string(status_sensor_ip_address01) << "."
      << std::to_string(status_sensor_ip_address02) << "."
      << std::to_string(status_sensor_ip_address03);
  radar_status_.sensor_ip_address0 = ss0.str();

  ss1 << std::to_string(status_sensor_ip_address10) << "."
      << std::to_string(status_sensor_ip_address11) << "."
      << std::to_string(status_sensor_ip_address12) << "."
      << std::to_string(status_sensor_ip_address13);
  radar_status_.sensor_ip_address1 = ss1.str();

  assert(sensor_status.status.plug_orientation == plug_orientation);
  assert(sensor_status.status.maximum_distance.value() == radar_status_.max_distance);
  assert(sensor_status.status.longitudinal.value() == radar_status_.longitudinal);
  assert(sensor_status.status.lateral.value() == radar_status_.lateral);
  assert(sensor_status.status.vertical.value() == radar_status_.vertical);
  assert(sensor_status.status.yaw.value() == radar_status_.yaw);
  assert(sensor_status.status.pitch.value() == radar_status_.pitch);
  assert(sensor_status.status.length.value() == radar_status_.length);
  assert(sensor_status.status.width.value() == radar_status_.width);
  assert(sensor_status.status.height.value() == radar_status_.height);
  assert(sensor_status.status.wheelbase.value() == radar_status_.wheel_base);
  assert(sensor_status.status.frequency_slot == frequency_slot);
  assert(sensor_status.status.cycle_time == radar_status_.cycle_time);
  assert(sensor_status.status.time_slot == radar_status_.time_slot);
  assert(sensor_status.status.hcc == hcc);
  assert(sensor_status.status.powersave_standstill == power_save_standstill);
  assert(sensor_status.status.sensor_ip_address00 == status_sensor_ip_address00);
  assert(sensor_status.status.sensor_ip_address01 == status_sensor_ip_address01);
  assert(sensor_status.status.sensor_ip_address02 == status_sensor_ip_address02);
  assert(sensor_status.status.sensor_ip_address03 == status_sensor_ip_address03);

  assert(sensor_status.status.sensor_ip_address10 == status_sensor_ip_address10);
  assert(sensor_status.status.sensor_ip_address11 == status_sensor_ip_address11);
  assert(sensor_status.status.sensor_ip_address12 == status_sensor_ip_address12);
  assert(sensor_status.status.sensor_ip_address13 == status_sensor_ip_address13);

  radar_status_.configuration_counter = buffer[STATUS_CONFIGURATION_COUNTER_BYTE];
  assert(sensor_status.configuration_counter == radar_status_.configuration_counter);

  const uint8_t & status_longitudinal_velocity = buffer[STATUS_LONGITUDINAL_VELOCITY_BYTE];
  assert(sensor_status.status_longitudinal_velocity == status_longitudinal_velocity);
  radar_status_.status_longitudinal_velocity = status_longitudinal_velocity == 0 ? "VDY_OK"
                                               : status_longitudinal_velocity == 1
                                                 ? "VDY_NOTOK"
                                                 : "INVALID VALUE";

  const uint8_t & status_longitudinal_acceleration = buffer[STATUS_LONGITUDINAL_ACCELERATION_BYTE];
  assert(sensor_status.status_longitudinal_acceleration == status_longitudinal_acceleration);
  radar_status_.status_longitudinal_acceleration = status_longitudinal_acceleration == 0 ? "VDY_OK"
                                                   : status_longitudinal_acceleration == 1
                                                     ? "VDY_NOTOK"
                                                     : "INVALID VALUE";

  const uint8_t & status_lateral_acceleration = buffer[STATUS_LATERAL_ACCELERATION_BYTE];
  assert(sensor_status.status_lateral_acceleration == status_lateral_acceleration);
  radar_status_.status_lateral_acceleration = status_lateral_acceleration == 0   ? "VDY_OK"
                                              : status_lateral_acceleration == 1 ? "VDY_NOTOK"
                                                                                 : "INVALID VALUE";

  const uint8_t & status_yaw_rate = buffer[STATUS_YAW_RATE_BYTE];
  assert(sensor_status.status_yaw_rate == status_yaw_rate);
  radar_status_.status_yaw_rate = status_yaw_rate == 0   ? "VDY_OK"
                                  : status_yaw_rate == 1 ? "VDY_NOTOK"
                                                         : "INVALID VALUE";

  const uint8_t & status_steering_angle = buffer[STATUS_STEERING_ANGLE_BYTE];
  assert(sensor_status.status_steering_angle == status_steering_angle);
  radar_status_.status_steering_angle = status_steering_angle == 0   ? "VDY_OK"
                                        : status_steering_angle == 1 ? "VDY_NOTOK"
                                                                     : "INVALID VALUE";

  const uint8_t & status_driving_direction = buffer[STATUS_DRIVING_DIRECTION_BYTE];
  assert(sensor_status.status_driving_direction == status_driving_direction);
  radar_status_.status_driving_direction = status_driving_direction == 0   ? "VDY_OK"
                                           : status_driving_direction == 1 ? "VDY_NOTOK"
                                                                           : "INVALID VALUE";

  const uint8_t & characteristic_speed = buffer[STATUS_CHARACTERISTIC_SPEED_BYTE];
  assert(sensor_status.status_characteristic_speed == characteristic_speed);
  radar_status_.characteristic_speed = characteristic_speed == 0   ? "VDY_OK"
                                       : characteristic_speed == 1 ? "VDY_NOTOK"
                                                                   : "INVALID VALUE";

  const uint8_t & radar_status = buffer[STATUS_RADAR_STATUS_BYTE];
  assert(sensor_status.status_radar_status == radar_status);
  if (radar_status == 0) {
    radar_status_.radar_status = "STATE_INIT";
  } else if (radar_status == 1) {
    radar_status_.radar_status = "STATE_OK";
  } else if (radar_status == 2) {
    radar_status_.radar_status = "STATE_INVALID";
  } else {
    radar_status_.radar_status = "INVALID VALUE";
  }

  const uint8_t & voltage_status = buffer[STATUS_VOLTAGE_STATUS_BYTE];
  assert(sensor_status.status_voltage_status == voltage_status);
  if (voltage_status == 0) {
    radar_status_.voltage_status = "Ok";
  }
  if (voltage_status & 0x01) {
    radar_status_.voltage_status += "Current under voltage";
  }
  if (voltage_status & 0x02) {
    radar_status_.voltage_status = "Past under voltage";
  }
  if (voltage_status & 0x03) {
    radar_status_.voltage_status = "Current over voltage";
  }
  if (voltage_status & 0x04) {
    radar_status_.voltage_status = "Past over voltage";
  }

  const uint8_t & temperature_status = buffer[STATUS_TEMPERATURE_STATUS_BYTE];
  assert(sensor_status.status_temperature_status == temperature_status);
  if (temperature_status == 0) {
    radar_status_.temperature_status = "Ok";
  }
  if (temperature_status & 0x01) {
    radar_status_.temperature_status += "Current under temperature";
  }
  if (temperature_status & 0x02) {
    radar_status_.temperature_status += "Past under temperature";
  }
  if (temperature_status & 0x03) {
    radar_status_.temperature_status += "Current over temperature";
  }
  if (temperature_status & 0x04) {
    radar_status_.temperature_status += "Past over temperature";
  }

  const uint8_t & blockage_status = buffer[STATUS_BLOCKAGE_STATUS_BYTE];
  assert(sensor_status.status_blockage_status == blockage_status);
  const uint8_t & blockage_status0 = blockage_status & 0x0f;
  const uint8_t & blockage_status1 = (blockage_status & 0xf0) >> 4;

  if (blockage_status0 == 0) {
    radar_status_.blockage_status = "Blind";
  } else if (blockage_status0 == 1) {
    radar_status_.blockage_status = "High";
  } else if (blockage_status0 == 2) {
    radar_status_.blockage_status = "Mid";
  } else if (blockage_status0 == 3) {
    radar_status_.blockage_status = "Low";
  } else if (blockage_status0 == 4) {
    radar_status_.blockage_status = "None";
  } else {
    radar_status_.blockage_status = "INVALID VALUE";
  }

  if (blockage_status1 == 0) {
    radar_status_.blockage_status += ". Self test failed";
  } else if (blockage_status1 == 1) {
    radar_status_.blockage_status += ". Self test passed";
  } else if (blockage_status1 == 2) {
    radar_status_.blockage_status += ". Self test ongoing";
  } else {
    radar_status_.blockage_status += ". Invalid self test";
  }
}

void ContinentalARS548HwInterface::ProcessFilterStatusPacket(const std::vector<uint8_t> & buffer)
{
  // assert(false); // it seems 548 does not have this anymore....
  //  Unused available data
  //  constexpr int FILTER_STATUS_TIMESTAMP_NANOSECONDS_BYTE = 8;
  //  constexpr int FILTER_STATUS_TIMESTAMP_SECONDS_BYTE = 12;
  //  constexpr int FILTER_STATUS_SYNC_STATUS_BYTE = 16;
  //  constexpr int FILTER_STATUS_FILTER_CONFIGURATION_COUNTER_BYTE = 17;
  //  constexpr int FILTER_STATUS_DETECTION_SORT_INDEX_BYTE = 18;
  //  constexpr int FILTER_STATUS_OBJECT_SORT_INDEX_BYTE = 19;
  constexpr int FILTER_STATUS_DETECTION_FILTER_BYTE = 20;
  constexpr int FILTER_STATUS_OBJECT_FILTER_BYTE = 90;

  // Unused available data
  // const uint32_t filter_status_timestamp_nanoseconds =
  // (static_cast<uint32_t>(buffer[FILTER_STATUS_TIMESTAMP_NANOSECONDS_BYTE]) << 24) |
  // (static_cast<uint32_t>(buffer[FILTER_STATUS_TIMESTAMP_NANOSECONDS_BYTE + 1]) << 16) |
  // (static_cast<uint32_t>(buffer[FILTER_STATUS_TIMESTAMP_NANOSECONDS_BYTE + 2]) << 8) |
  // buffer[FILTER_STATUS_TIMESTAMP_NANOSECONDS_BYTE + 3]; const uint32_t
  // filter_status_timestamp_seconds =
  // (static_cast<uint32_t>(buffer[FILTER_STATUS_TIMESTAMP_SECONDS_BYTE]) << 24) |
  // (static_cast<uint32_t>(buffer[FILTER_STATUS_TIMESTAMP_SECONDS_BYTE + 1]) << 16) |
  // (static_cast<uint32_t>(buffer[FILTER_STATUS_TIMESTAMP_SECONDS_BYTE + 2]) << 8) |
  // buffer[FILTER_STATUS_TIMESTAMP_SECONDS_BYTE + 3]; const uint8_t filter_status_sync_status =
  // buffer[FILTER_STATUS_SYNC_STATUS_BYTE]; const uint8_t
  // filter_status_filter_configuration_counter =
  // buffer[FILTER_STATUS_FILTER_CONFIGURATION_COUNTER_BYTE]; const uint8_t
  // filter_status_detection_sort_index = buffer[FILTER_STATUS_DETECTION_SORT_INDEX_BYTE]; const
  // uint8_t filter_status_object_sort_index = buffer[FILTER_STATUS_OBJECT_SORT_INDEX_BYTE];

  std::size_t byte_index = FILTER_STATUS_DETECTION_FILTER_BYTE;
  for (int property_index = 1; property_index <= 7; property_index++) {
    FilterStatus filter_status;
    filter_status.active = buffer[byte_index++];
    assert(buffer[byte_index] == property_index);
    filter_status.filter_id = buffer[byte_index++];
    filter_status.min_value = (static_cast<uint32_t>(buffer[byte_index]) << 24) |
                              (static_cast<uint32_t>(buffer[byte_index + 1]) << 16) |
                              (static_cast<uint32_t>(buffer[byte_index + 2]) << 8) |
                              buffer[byte_index + 3];
    byte_index += 4;
    filter_status.max_value = (static_cast<uint32_t>(buffer[byte_index]) << 24) |
                              (static_cast<uint32_t>(buffer[byte_index + 1]) << 16) |
                              (static_cast<uint32_t>(buffer[byte_index + 2]) << 8) |
                              buffer[byte_index + 3];
    byte_index += 4;

    detection_filters_status_[property_index - 1] = filter_status;
  }

  byte_index = FILTER_STATUS_OBJECT_FILTER_BYTE;
  for (int property_index = 1; property_index <= 24; property_index++) {
    FilterStatus filter_status;
    filter_status.active = buffer[byte_index++];
    assert(buffer[byte_index] == property_index);
    filter_status.filter_id = buffer[byte_index++];
    filter_status.min_value = (static_cast<uint32_t>(buffer[byte_index]) << 24) |
                              (static_cast<uint32_t>(buffer[byte_index + 1]) << 16) |
                              (static_cast<uint32_t>(buffer[byte_index + 2]) << 8) |
                              buffer[byte_index + 3];
    byte_index += 4;
    filter_status.max_value = (static_cast<uint32_t>(buffer[byte_index]) << 24) |
                              (static_cast<uint32_t>(buffer[byte_index + 1]) << 16) |
                              (static_cast<uint32_t>(buffer[byte_index + 2]) << 8) |
                              buffer[byte_index + 3];
    byte_index += 4;

    object_filters_status_[property_index - 1] = filter_status;
  }
}

void ContinentalARS548HwInterface::ProcessDataPacket(const std::vector<uint8_t> & buffer)
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

  nebula_packets_reception_callback_(std::move(nebula_packets_ptr_));
  nebula_packets_ptr_ = std::make_unique<nebula_msgs::msg::NebulaPackets>();
}

Status ContinentalARS548HwInterface::CloudInterfaceStop()
{
  return Status::ERROR_1;
}

Status ContinentalARS548HwInterface::GetSensorConfiguration(
  SensorConfigurationBase & sensor_configuration)
{
  std::stringstream ss;
  ss << sensor_configuration;
  PrintDebug(ss.str());
  return Status::ERROR_1;
}

Status ContinentalARS548HwInterface::SetSensorMounting(
  float longitudinal_autosar, float lateral_autosar, float vertical_autosar, float yaw_autosar,
  float pitch_autosar, uint8_t plug_orientation)
{
  constexpr int CONFIGURATION_LONGITUDINAL_BYTE = 8;
  constexpr int CONFIGURATION_LATERAL_BYTE = 12;
  constexpr int CONFIGURATION_VERTICAL_BYTE = 16;
  constexpr int CONFIGURATION_YAW_BYTE = 20;
  constexpr int CONFIGURATION_PITCH_BYTE = 24;
  constexpr int CONFIGURATION_PLUG_ORIENTATION_BYTE = 28;
  constexpr int CONFIGURATION_NEW_SENSOR_MOUNTING_BYTE = 60;

  if (
    longitudinal_autosar < -100.f || longitudinal_autosar > 100.f || lateral_autosar < -100.f ||
    lateral_autosar > 100.f || vertical_autosar < 0.01f || vertical_autosar > 10.f ||
    yaw_autosar < -M_PI || yaw_autosar > M_PI || pitch_autosar < -M_PI_2 ||
    pitch_autosar > M_PI_2) {
    PrintError("Invalid SetSensorMounting values");
    return Status::SENSOR_CONFIG_ERROR;
  }

  std::vector<uint8_t> send_vector(CONFIGURATION_PAYLOAD_LENGTH + 8);
  uint8_t longitudinal_autosar_bytes[4];
  uint8_t lateral_autosar_bytes[4];
  uint8_t vertical_autosar_bytes[4];
  uint8_t yaw_autosar_bytes[4];
  uint8_t pitch_autosar_bytes[4];
  std::memcpy(longitudinal_autosar_bytes, &longitudinal_autosar, sizeof(longitudinal_autosar));
  std::memcpy(lateral_autosar_bytes, &lateral_autosar, sizeof(lateral_autosar));
  std::memcpy(vertical_autosar_bytes, &vertical_autosar, sizeof(vertical_autosar));
  std::memcpy(yaw_autosar_bytes, &yaw_autosar, sizeof(yaw_autosar));
  std::memcpy(pitch_autosar_bytes, &pitch_autosar, sizeof(pitch_autosar));

  send_vector[METHOD_ID_BYTE + 0] = static_cast<uint8_t>((CONFIGURATION_METHOD_ID >> 8) & 0xff);
  send_vector[METHOD_ID_BYTE + 1] = static_cast<uint8_t>(CONFIGURATION_METHOD_ID & 0xff);
  send_vector[LENGTH_BYTE + 0] = static_cast<uint8_t>((CONFIGURATION_PAYLOAD_LENGTH >> 24) & 0xff);
  send_vector[LENGTH_BYTE + 1] = static_cast<uint8_t>((CONFIGURATION_PAYLOAD_LENGTH >> 16) & 0xff);
  send_vector[LENGTH_BYTE + 2] = static_cast<uint8_t>((CONFIGURATION_PAYLOAD_LENGTH >> 8) & 0xff);
  send_vector[LENGTH_BYTE + 3] = static_cast<uint8_t>(CONFIGURATION_PAYLOAD_LENGTH & 0xff);

  send_vector[CONFIGURATION_LONGITUDINAL_BYTE + 0] = longitudinal_autosar_bytes[3];
  send_vector[CONFIGURATION_LONGITUDINAL_BYTE + 1] = longitudinal_autosar_bytes[2];
  send_vector[CONFIGURATION_LONGITUDINAL_BYTE + 2] = longitudinal_autosar_bytes[1];
  send_vector[CONFIGURATION_LONGITUDINAL_BYTE + 3] = longitudinal_autosar_bytes[0];

  send_vector[CONFIGURATION_LATERAL_BYTE + 0] = lateral_autosar_bytes[3];
  send_vector[CONFIGURATION_LATERAL_BYTE + 1] = lateral_autosar_bytes[2];
  send_vector[CONFIGURATION_LATERAL_BYTE + 2] = lateral_autosar_bytes[1];
  send_vector[CONFIGURATION_LATERAL_BYTE + 3] = lateral_autosar_bytes[0];

  send_vector[CONFIGURATION_VERTICAL_BYTE + 0] = vertical_autosar_bytes[3];
  send_vector[CONFIGURATION_VERTICAL_BYTE + 1] = vertical_autosar_bytes[2];
  send_vector[CONFIGURATION_VERTICAL_BYTE + 2] = vertical_autosar_bytes[1];
  send_vector[CONFIGURATION_VERTICAL_BYTE + 3] = vertical_autosar_bytes[0];

  send_vector[CONFIGURATION_YAW_BYTE + 0] = yaw_autosar_bytes[3];
  send_vector[CONFIGURATION_YAW_BYTE + 1] = yaw_autosar_bytes[2];
  send_vector[CONFIGURATION_YAW_BYTE + 2] = yaw_autosar_bytes[1];
  send_vector[CONFIGURATION_YAW_BYTE + 3] = yaw_autosar_bytes[0];

  send_vector[CONFIGURATION_PITCH_BYTE + 0] = pitch_autosar_bytes[3];
  send_vector[CONFIGURATION_PITCH_BYTE + 1] = pitch_autosar_bytes[2];
  send_vector[CONFIGURATION_PITCH_BYTE + 2] = pitch_autosar_bytes[1];
  send_vector[CONFIGURATION_PITCH_BYTE + 3] = pitch_autosar_bytes[0];

  send_vector[CONFIGURATION_PLUG_ORIENTATION_BYTE] = plug_orientation;
  send_vector[CONFIGURATION_NEW_SENSOR_MOUNTING_BYTE] = 1;

  Configuration configuration{};
  assert(send_vector.size() == sizeof(Configuration));
  configuration.header.service_id = CONFIGURATION_SERVICE_ID;
  configuration.header.method_id = CONFIGURATION_METHOD_ID;
  configuration.header.length = CONFIGURATION_PAYLOAD_LENGTH;
  configuration.configuration.longitudinal = longitudinal_autosar;
  configuration.configuration.lateral = lateral_autosar;
  configuration.configuration.vertical = vertical_autosar;
  configuration.configuration.yaw = yaw_autosar;
  configuration.configuration.pitch = pitch_autosar;
  configuration.configuration.plug_orientation = plug_orientation;
  configuration.new_sensor_mounting = 1;

  std::vector<uint8_t> send_vector2(sizeof(Configuration));
  std::memcpy(send_vector2.data(), &configuration, sizeof(Configuration));
  assert(send_vector2 == send_vector2);

  sensor_udp_driver_->sender()->asyncSend(send_vector);

  return Status::OK;
}

Status ContinentalARS548HwInterface::SetVehicleParameters(
  float length_autosar, float width_autosar, float height_autosar, float wheel_base_autosar)
{
  constexpr int CONFIGURATION_LENGTH_BYTE = 29;
  constexpr int CONFIGURATION_WIDTH_BYTE = 33;
  constexpr int CONFIGURATION_HEIGHT_BYTE = 37;
  constexpr int CONFIGURATION_WHEEL_BASE_BYTE = 41;
  constexpr int CONFIGURATION_NEW_VEHICLE_PARAMETERS_BYTE = 61;

  if (
    length_autosar < 0.01f || length_autosar > 100.f || width_autosar < 0.01f ||
    width_autosar > 100.f || height_autosar < 0.01f || height_autosar > 100.f ||
    wheel_base_autosar < 0.01f || wheel_base_autosar > 100.f) {
    PrintError("Invalid SetVehicleParameters values");
    return Status::SENSOR_CONFIG_ERROR;
  }

  std::vector<uint8_t> send_vector(CONFIGURATION_PAYLOAD_LENGTH + 8);
  uint8_t length_autosar_bytes[4];
  uint8_t width_autosar_bytes[4];
  uint8_t height_autosar_bytes[4];
  uint8_t wheel_base_autosar_bytes[4];
  std::memcpy(length_autosar_bytes, &length_autosar, sizeof(length_autosar));
  std::memcpy(width_autosar_bytes, &width_autosar, sizeof(width_autosar));
  std::memcpy(height_autosar_bytes, &height_autosar, sizeof(height_autosar));
  std::memcpy(wheel_base_autosar_bytes, &wheel_base_autosar, sizeof(wheel_base_autosar));

  send_vector[METHOD_ID_BYTE + 0] = static_cast<uint8_t>((CONFIGURATION_METHOD_ID >> 8) & 0xff);
  send_vector[METHOD_ID_BYTE + 1] = static_cast<uint8_t>(CONFIGURATION_METHOD_ID & 0xff);
  send_vector[LENGTH_BYTE + 0] = static_cast<uint8_t>((CONFIGURATION_PAYLOAD_LENGTH >> 24) & 0xff);
  send_vector[LENGTH_BYTE + 1] = static_cast<uint8_t>((CONFIGURATION_PAYLOAD_LENGTH >> 16) & 0xff);
  send_vector[LENGTH_BYTE + 2] = static_cast<uint8_t>((CONFIGURATION_PAYLOAD_LENGTH >> 8) & 0xff);
  send_vector[LENGTH_BYTE + 3] = static_cast<uint8_t>(CONFIGURATION_PAYLOAD_LENGTH & 0xff);

  send_vector[CONFIGURATION_LENGTH_BYTE + 0] = length_autosar_bytes[3];
  send_vector[CONFIGURATION_LENGTH_BYTE + 1] = length_autosar_bytes[2];
  send_vector[CONFIGURATION_LENGTH_BYTE + 2] = length_autosar_bytes[1];
  send_vector[CONFIGURATION_LENGTH_BYTE + 3] = length_autosar_bytes[0];

  send_vector[CONFIGURATION_WIDTH_BYTE + 0] = width_autosar_bytes[3];
  send_vector[CONFIGURATION_WIDTH_BYTE + 1] = width_autosar_bytes[2];
  send_vector[CONFIGURATION_WIDTH_BYTE + 2] = width_autosar_bytes[1];
  send_vector[CONFIGURATION_WIDTH_BYTE + 3] = width_autosar_bytes[0];

  send_vector[CONFIGURATION_HEIGHT_BYTE + 0] = height_autosar_bytes[3];
  send_vector[CONFIGURATION_HEIGHT_BYTE + 1] = height_autosar_bytes[2];
  send_vector[CONFIGURATION_HEIGHT_BYTE + 2] = height_autosar_bytes[1];
  send_vector[CONFIGURATION_HEIGHT_BYTE + 3] = height_autosar_bytes[0];

  send_vector[CONFIGURATION_WHEEL_BASE_BYTE + 0] = wheel_base_autosar_bytes[3];
  send_vector[CONFIGURATION_WHEEL_BASE_BYTE + 1] = wheel_base_autosar_bytes[2];
  send_vector[CONFIGURATION_WHEEL_BASE_BYTE + 2] = wheel_base_autosar_bytes[1];
  send_vector[CONFIGURATION_WHEEL_BASE_BYTE + 3] = wheel_base_autosar_bytes[0];

  send_vector[CONFIGURATION_NEW_VEHICLE_PARAMETERS_BYTE] = 1;

  Configuration configuration{};
  assert(send_vector.size() == sizeof(Configuration));
  configuration.header.service_id = CONFIGURATION_SERVICE_ID;
  configuration.header.method_id = CONFIGURATION_METHOD_ID;
  configuration.header.length = CONFIGURATION_PAYLOAD_LENGTH;
  configuration.configuration.length = length_autosar;
  configuration.configuration.width = width_autosar;
  configuration.configuration.height = height_autosar;
  configuration.configuration.wheelbase = wheel_base_autosar;
  configuration.new_vehicle_parameters = 1;

  std::vector<uint8_t> send_vector2(sizeof(Configuration));
  std::memcpy(send_vector2.data(), &configuration, sizeof(Configuration));
  assert(send_vector2 == send_vector2);

  sensor_udp_driver_->sender()->asyncSend(send_vector);

  return Status::OK;
}

Status ContinentalARS548HwInterface::SetRadarParameters(
  uint16_t maximum_distance, uint8_t frequency_slot, uint8_t cycle_time, uint8_t time_slot,
  uint8_t hcc, uint8_t power_save_standstill)
{
  constexpr int CONFIGURATION_MAXIMUM_DISTANCE_BYTE = 45;
  constexpr int CONFIGURATION_FREQUENCY_SLOT_BYTE = 47;
  constexpr int CONFIGURATION_CYCLE_TIME_BYTE = 48;
  constexpr int CONFIGURATION_TIME_SLOT_BYTE = 49;
  constexpr int CONFIGURATION_HCC_BYTE = 50;
  constexpr int CONFIGURATION_power_save_STANDSTILL_BYTE = 51;
  constexpr int CONFIGURATION_NEW_RADAR_PARAMETERS_BYTE = 62;

  if (
    maximum_distance < 93 || maximum_distance > 1514 || frequency_slot > 2 || cycle_time < 50 ||
    cycle_time > 100 || time_slot < 10 || time_slot > 90 || hcc < 1 || hcc > 2 ||
    power_save_standstill > 1) {
    PrintError("Invalid SetRadarParameters values");
    return Status::SENSOR_CONFIG_ERROR;
  }

  std::vector<uint8_t> send_vector(CONFIGURATION_PAYLOAD_LENGTH + 8);
  send_vector[METHOD_ID_BYTE + 0] = static_cast<uint8_t>((CONFIGURATION_METHOD_ID >> 8) & 0xff);
  send_vector[METHOD_ID_BYTE + 1] = static_cast<uint8_t>(CONFIGURATION_METHOD_ID & 0xff);
  send_vector[LENGTH_BYTE + 0] = static_cast<uint8_t>((CONFIGURATION_PAYLOAD_LENGTH >> 24) & 0xff);
  send_vector[LENGTH_BYTE + 1] = static_cast<uint8_t>((CONFIGURATION_PAYLOAD_LENGTH >> 16) & 0xff);
  send_vector[LENGTH_BYTE + 2] = static_cast<uint8_t>((CONFIGURATION_PAYLOAD_LENGTH >> 8) & 0xff);
  send_vector[LENGTH_BYTE + 3] = static_cast<uint8_t>(CONFIGURATION_PAYLOAD_LENGTH & 0xff);

  send_vector[CONFIGURATION_MAXIMUM_DISTANCE_BYTE + 0] =
    static_cast<uint8_t>((maximum_distance >> 8) & 0xff);
  send_vector[CONFIGURATION_MAXIMUM_DISTANCE_BYTE + 1] =
    static_cast<uint8_t>(maximum_distance & 0xff);

  send_vector[CONFIGURATION_FREQUENCY_SLOT_BYTE] = frequency_slot;
  send_vector[CONFIGURATION_CYCLE_TIME_BYTE] = cycle_time;
  send_vector[CONFIGURATION_TIME_SLOT_BYTE] = time_slot;
  send_vector[CONFIGURATION_HCC_BYTE] = hcc;
  send_vector[CONFIGURATION_power_save_STANDSTILL_BYTE] = power_save_standstill;

  send_vector[CONFIGURATION_NEW_RADAR_PARAMETERS_BYTE] = 1;

  Configuration configuration{};
  assert(send_vector.size() == sizeof(Configuration));
  configuration.header.service_id = CONFIGURATION_SERVICE_ID;
  configuration.header.method_id = CONFIGURATION_METHOD_ID;
  configuration.header.length = CONFIGURATION_PAYLOAD_LENGTH;
  configuration.configuration.maximum_distance = maximum_distance;
  configuration.configuration.frequency_slot = frequency_slot;
  configuration.configuration.cycle_time = cycle_time;
  configuration.configuration.hcc = hcc;
  configuration.configuration.powersave_standstill = power_save_standstill;
  configuration.new_radar_parameters = 1;

  std::vector<uint8_t> send_vector2(sizeof(Configuration));
  std::memcpy(send_vector2.data(), &configuration, sizeof(Configuration));
  assert(send_vector2 == send_vector2);

  sensor_udp_driver_->sender()->asyncSend(send_vector);

  return Status::OK;
}

Status ContinentalARS548HwInterface::SetSensorIPAddress(const std::string & sensor_ip_address)
{
  constexpr int CONFIGURATION_SENSOR_IP_ADDRESS0 = 52;
  constexpr int CONFIGURATION_SENSOR_IP_ADDRESS1 = 56;
  constexpr int CONFIGURATION_NEW_NETWORK_CONFIGURATION_BYTE = 63;

  std::array<uint8_t, 4> ip_bytes;

  try {
    auto sensor_ip = boost::asio::ip::address::from_string(sensor_ip_address);
    ip_bytes = sensor_ip.to_v4().to_bytes();
  } catch (const std::exception & ex) {
    PrintError("Setting invalid IP");
    return Status::SENSOR_CONFIG_ERROR;
  }

  std::vector<uint8_t> send_vector(CONFIGURATION_PAYLOAD_LENGTH + 8);

  send_vector[METHOD_ID_BYTE + 0] = static_cast<uint8_t>((CONFIGURATION_METHOD_ID >> 8) & 0xff);
  send_vector[METHOD_ID_BYTE + 1] = static_cast<uint8_t>(CONFIGURATION_METHOD_ID & 0xff);
  send_vector[LENGTH_BYTE + 0] = static_cast<uint8_t>((CONFIGURATION_PAYLOAD_LENGTH >> 24) & 0xff);
  send_vector[LENGTH_BYTE + 1] = static_cast<uint8_t>((CONFIGURATION_PAYLOAD_LENGTH >> 16) & 0xff);
  send_vector[LENGTH_BYTE + 2] = static_cast<uint8_t>((CONFIGURATION_PAYLOAD_LENGTH >> 8) & 0xff);
  send_vector[LENGTH_BYTE + 3] = static_cast<uint8_t>(CONFIGURATION_PAYLOAD_LENGTH & 0xff);

  send_vector[CONFIGURATION_SENSOR_IP_ADDRESS0 + 0] = ip_bytes[0];
  send_vector[CONFIGURATION_SENSOR_IP_ADDRESS0 + 1] = ip_bytes[1];
  send_vector[CONFIGURATION_SENSOR_IP_ADDRESS0 + 2] = ip_bytes[2];
  send_vector[CONFIGURATION_SENSOR_IP_ADDRESS0 + 3] = ip_bytes[3];

  send_vector[CONFIGURATION_SENSOR_IP_ADDRESS1 + 0] = 169;
  send_vector[CONFIGURATION_SENSOR_IP_ADDRESS1 + 1] = 254;
  send_vector[CONFIGURATION_SENSOR_IP_ADDRESS1 + 2] = 116;
  send_vector[CONFIGURATION_SENSOR_IP_ADDRESS1 + 3] = 113;

  send_vector[CONFIGURATION_NEW_NETWORK_CONFIGURATION_BYTE] = 1;

  Configuration configuration{};
  assert(send_vector.size() == sizeof(Configuration));
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

  std::vector<uint8_t> send_vector2(sizeof(Configuration));
  std::memcpy(send_vector2.data(), &configuration, sizeof(Configuration));
  assert(send_vector2 == send_vector2);

  sensor_udp_driver_->sender()->asyncSend(send_vector);

  return Status::OK;
}

Status ContinentalARS548HwInterface::SetAccelerationLateralCog(float lateral_acceleration)
{
  constexpr uint16_t ACCELERATION_LATERAL_COG_SERVICE_ID = 0;
  constexpr uint16_t ACCELERATION_LATERAL_COG_METHOD_ID = 321;
  constexpr uint8_t ACCELERATION_LATERAL_COG_LENGTH = 32;
  const int ACCELERATION_LATERAL_COG_PAYLOAD_SIZE = ACCELERATION_LATERAL_COG_LENGTH + 8;

  if (lateral_acceleration < -65.f || lateral_acceleration > 65.f) {
    PrintError("Invalid lateral_acceleration value");
    return Status::ERROR_1;
  }

  uint8_t bytes[4];
  std::memcpy(bytes, &lateral_acceleration, sizeof(lateral_acceleration));

  std::vector<uint8_t> send_vector(ACCELERATION_LATERAL_COG_PAYLOAD_SIZE, 0);
  send_vector[1] = ACCELERATION_LATERAL_COG_SERVICE_ID;
  send_vector[2] = static_cast<uint8_t>(ACCELERATION_LATERAL_COG_METHOD_ID >> 8);
  send_vector[3] = static_cast<uint8_t>(ACCELERATION_LATERAL_COG_METHOD_ID & 0x00ff);
  send_vector[7] = ACCELERATION_LATERAL_COG_LENGTH;
  send_vector[14] = bytes[3];
  send_vector[15] = bytes[2];
  send_vector[16] = bytes[1];
  send_vector[17] = bytes[0];

  AccelerationLateralCoG acceleration_lateral_cog{};
  assert(send_vector.size() == sizeof(AccelerationLateralCoG));
  acceleration_lateral_cog.header.service_id = ACCELERATION_LATERAL_COG_SERVICE_ID;
  acceleration_lateral_cog.header.method_id = ACCELERATION_LATERAL_COG_METHOD_ID;
  acceleration_lateral_cog.header.length = ACCELERATION_LATERAL_COG_LENGTH;
  acceleration_lateral_cog.acceleration_lateral = lateral_acceleration;

  std::vector<uint8_t> send_vector2(sizeof(AccelerationLateralCoG));
  std::memcpy(send_vector2.data(), &acceleration_lateral_cog, sizeof(AccelerationLateralCoG));
  assert(send_vector2 == send_vector2);

  sensor_udp_driver_->sender()->asyncSend(send_vector);

  return Status::OK;
}

Status ContinentalARS548HwInterface::SetAccelerationLongitudinalCog(float longitudinal_acceleration)
{
  constexpr uint16_t ACCELERATION_LONGITUDINAL_COG_SERVICE_ID = 0;
  constexpr uint16_t ACCELERATION_LONGITUDINAL_COG_METHOD_ID = 322;
  constexpr uint8_t ACCELERATION_LONGITUDINAL_COG_LENGTH = 32;
  const int ACCELERATION_LONGITUDINAL_COG_PAYLOAD_SIZE = ACCELERATION_LONGITUDINAL_COG_LENGTH + 8;

  if (longitudinal_acceleration < -65.f || longitudinal_acceleration > 65.f) {
    PrintError("Invalid longitudinal_acceleration value");
    return Status::ERROR_1;
  }

  uint8_t bytes[4];
  std::memcpy(bytes, &longitudinal_acceleration, sizeof(longitudinal_acceleration));

  std::vector<uint8_t> send_vector(ACCELERATION_LONGITUDINAL_COG_PAYLOAD_SIZE, 0);
  send_vector[1] = ACCELERATION_LONGITUDINAL_COG_SERVICE_ID;
  send_vector[2] = static_cast<uint8_t>(ACCELERATION_LONGITUDINAL_COG_METHOD_ID >> 8);
  send_vector[3] = static_cast<uint8_t>(ACCELERATION_LONGITUDINAL_COG_METHOD_ID & 0x00ff);
  send_vector[7] = ACCELERATION_LONGITUDINAL_COG_LENGTH;
  send_vector[14] = bytes[3];
  send_vector[15] = bytes[2];
  send_vector[16] = bytes[1];
  send_vector[17] = bytes[0];

  AccelerationLongitudinalCoG acceleration_longitudinal_cog{};
  assert(send_vector.size() == sizeof(AccelerationLongitudinalCoG));
  acceleration_longitudinal_cog.header.service_id = ACCELERATION_LONGITUDINAL_COG_SERVICE_ID;
  acceleration_longitudinal_cog.header.method_id = ACCELERATION_LONGITUDINAL_COG_METHOD_ID;
  acceleration_longitudinal_cog.header.length = ACCELERATION_LONGITUDINAL_COG_LENGTH;
  acceleration_longitudinal_cog.acceleration_lateral = longitudinal_acceleration;

  std::vector<uint8_t> send_vector2(sizeof(AccelerationLongitudinalCoG));
  std::memcpy(
    send_vector2.data(), &acceleration_longitudinal_cog, sizeof(AccelerationLongitudinalCoG));
  assert(send_vector2 == send_vector2);

  sensor_udp_driver_->sender()->asyncSend(send_vector);

  return Status::OK;
}

Status ContinentalARS548HwInterface::SetCharacteristicSpeed(float characteristic_speed)
{
  constexpr uint16_t CHARACTERISTIC_SPEED_SERVICE_ID = 0;
  constexpr uint16_t CHARACTERISTIC_SPEED_METHOD_ID = 328;
  constexpr uint8_t CHARACTERISTIC_SPEED_LENGTH = 11;
  const int CHARACTERISTIC_SPEED_PAYLOAD_SIZE = CHARACTERISTIC_SPEED_LENGTH + 8;

  if (characteristic_speed < 0.f || characteristic_speed > 255.f) {
    PrintError("Invalid characteristic_speed value");
    return Status::ERROR_1;
  }

  std::vector<uint8_t> send_vector(CHARACTERISTIC_SPEED_PAYLOAD_SIZE, 0);
  send_vector[1] = CHARACTERISTIC_SPEED_SERVICE_ID;
  send_vector[2] = static_cast<uint8_t>(CHARACTERISTIC_SPEED_METHOD_ID >> 8);
  send_vector[3] = static_cast<uint8_t>(CHARACTERISTIC_SPEED_METHOD_ID & 0x00ff);
  send_vector[7] = CHARACTERISTIC_SPEED_LENGTH;
  send_vector[10] = static_cast<uint8_t>(characteristic_speed);

  CharasteristicSpeed characteristic_speed_packet{};
  assert(send_vector.size() == sizeof(CharasteristicSpeed));
  characteristic_speed_packet.header.service_id = CHARACTERISTIC_SPEED_SERVICE_ID;
  characteristic_speed_packet.header.method_id = CHARACTERISTIC_SPEED_METHOD_ID;
  characteristic_speed_packet.header.length = CHARACTERISTIC_SPEED_LENGTH;
  characteristic_speed_packet.characteristic_speed = characteristic_speed;

  std::vector<uint8_t> send_vector2(sizeof(CharasteristicSpeed));
  std::memcpy(send_vector2.data(), &characteristic_speed_packet, sizeof(CharasteristicSpeed));
  assert(send_vector2 == send_vector2);

  sensor_udp_driver_->sender()->asyncSend(send_vector);

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
  const int DRIVING_DIRECTION_PAYLOAD_SIZE = DRIVING_DIRECTION_LENGTH + 8;

  std::vector<uint8_t> send_vector(DRIVING_DIRECTION_PAYLOAD_SIZE, 0);
  send_vector[1] = DRIVING_DIRECTION_SERVICE_ID;
  send_vector[2] = static_cast<uint8_t>(DRIVING_DIRECTION_METHOD_ID >> 8);
  send_vector[3] = static_cast<uint8_t>(DRIVING_DIRECTION_METHOD_ID & 0xff);
  send_vector[7] = DRIVING_DIRECTION_LENGTH;

  if (direction == 0) {
    send_vector[9] = DRIVING_DIRECTION_STANDSTILL;
  } else if (direction > 0) {
    send_vector[9] = DRIVING_DIRECTION_FORWARD;
  } else {
    send_vector[9] = DRIVING_DIRECTION_BACKWARDS;
  }

  DrivingDirection driving_direction_packet{};
  assert(send_vector.size() == sizeof(DrivingDirection));
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

  std::vector<uint8_t> send_vector2(sizeof(DrivingDirection));
  std::memcpy(send_vector2.data(), &driving_direction_packet, sizeof(DrivingDirection));
  assert(send_vector2 == send_vector2);

  sensor_udp_driver_->sender()->asyncSend(send_vector);

  return Status::OK;
}

Status ContinentalARS548HwInterface::SetSteeringAngleFrontAxle(float angle_rad)
{
  constexpr uint16_t STEERING_ANGLE_SERVICE_ID = 0;
  constexpr uint16_t STEERING_ANGLE_METHOD_ID = 327;
  constexpr uint8_t STEERING_ANGLE_LENGTH = 32;
  const int STEERING_ANGLE_PAYLOAD_SIZE = STEERING_ANGLE_LENGTH + 8;

  if (angle_rad < -M_PI_2 || angle_rad > M_PI_2) {
    PrintError("Invalid angle_rad value");
    return Status::ERROR_1;
  }

  uint8_t bytes[4];
  std::memcpy(bytes, &angle_rad, sizeof(angle_rad));

  std::vector<uint8_t> send_vector(STEERING_ANGLE_PAYLOAD_SIZE, 0);
  send_vector[1] = STEERING_ANGLE_SERVICE_ID;
  send_vector[2] = static_cast<uint8_t>(STEERING_ANGLE_METHOD_ID >> 8);
  send_vector[3] = static_cast<uint8_t>(STEERING_ANGLE_METHOD_ID & 0x00ff);
  send_vector[7] = STEERING_ANGLE_LENGTH;
  send_vector[14] = bytes[3];
  send_vector[15] = bytes[2];
  send_vector[16] = bytes[1];
  send_vector[17] = bytes[0];

  SteeringAngleFrontAxle steering_angle_front_axle_packet{};
  assert(send_vector.size() == sizeof(SteeringAngleFrontAxle));
  steering_angle_front_axle_packet.header.service_id = STEERING_ANGLE_SERVICE_ID;
  steering_angle_front_axle_packet.header.method_id = STEERING_ANGLE_METHOD_ID;
  steering_angle_front_axle_packet.header.length = STEERING_ANGLE_LENGTH;
  steering_angle_front_axle_packet.steering_angle_front_axle = angle_rad;

  std::vector<uint8_t> send_vector2(sizeof(SteeringAngleFrontAxle));
  std::memcpy(
    send_vector2.data(), &steering_angle_front_axle_packet, sizeof(SteeringAngleFrontAxle));
  assert(send_vector2 == send_vector2);

  sensor_udp_driver_->sender()->asyncSend(send_vector);

  return Status::OK;
}

Status ContinentalARS548HwInterface::SetVelocityVehicle(float velocity)
{
  constexpr uint16_t VELOCITY_VEHICLE_SERVICE_ID = 0;
  constexpr uint16_t VELOCITY_VEHICLE_METHOD_ID = 323;
  constexpr uint8_t VELOCITY_VEHICLE_LENGTH = 28;
  const int VELOCITY_VEHICLE_PAYLOAD_SIZE = VELOCITY_VEHICLE_LENGTH + 8;

  uint8_t bytes[4];
  std::memcpy(bytes, &velocity, sizeof(velocity));

  std::vector<uint8_t> send_vector(VELOCITY_VEHICLE_PAYLOAD_SIZE, 0);
  send_vector[1] = VELOCITY_VEHICLE_SERVICE_ID;
  send_vector[2] = static_cast<uint8_t>(VELOCITY_VEHICLE_METHOD_ID >> 8);
  send_vector[3] = static_cast<uint8_t>(VELOCITY_VEHICLE_METHOD_ID & 0x00ff);
  send_vector[7] = VELOCITY_VEHICLE_LENGTH;
  send_vector[11] = bytes[3];
  send_vector[12] = bytes[2];
  send_vector[13] = bytes[1];
  send_vector[14] = bytes[0];

  VelocityVehicle steering_angle_front_axle_packet{};
  assert(send_vector.size() == sizeof(VelocityVehicle));
  steering_angle_front_axle_packet.header.service_id = VELOCITY_VEHICLE_SERVICE_ID;
  steering_angle_front_axle_packet.header.method_id = VELOCITY_VEHICLE_METHOD_ID;
  steering_angle_front_axle_packet.header.length = VELOCITY_VEHICLE_LENGTH;
  steering_angle_front_axle_packet.velocity_vehicle = velocity;

  std::vector<uint8_t> send_vector2(sizeof(VelocityVehicle));
  std::memcpy(send_vector2.data(), &steering_angle_front_axle_packet, sizeof(VelocityVehicle));
  assert(send_vector2 == send_vector2);

  sensor_udp_driver_->sender()->asyncSend(send_vector);

  return Status::OK;
}

Status ContinentalARS548HwInterface::SetYawRate(float yaw_rate)
{
  constexpr uint16_t YAW_RATE_SERVICE_ID = 0;
  constexpr uint16_t YAW_RATE_METHOD_ID = 326;
  constexpr uint8_t YAW_RATE_LENGTH = 32;
  const int YAW_RATE_PAYLOAD_SIZE = YAW_RATE_LENGTH + 8;

  uint8_t bytes[4];
  std::memcpy(bytes, &yaw_rate, sizeof(yaw_rate));

  std::vector<uint8_t> send_vector(YAW_RATE_PAYLOAD_SIZE, 0);
  send_vector[1] = YAW_RATE_SERVICE_ID;
  send_vector[2] = static_cast<uint8_t>(YAW_RATE_METHOD_ID >> 8);
  send_vector[3] = static_cast<uint8_t>(YAW_RATE_METHOD_ID & 0x00ff);
  send_vector[7] = YAW_RATE_LENGTH;
  send_vector[14] = bytes[3];
  send_vector[15] = bytes[2];
  send_vector[16] = bytes[1];
  send_vector[17] = bytes[0];

  YawRate yaw_rate_packet{};
  assert(send_vector.size() == sizeof(YawRate));
  yaw_rate_packet.header.service_id = YAW_RATE_SERVICE_ID;
  yaw_rate_packet.header.method_id = YAW_RATE_METHOD_ID;
  yaw_rate_packet.header.length = YAW_RATE_LENGTH;
  yaw_rate_packet.yaw_rate = yaw_rate;

  std::vector<uint8_t> send_vector2(sizeof(YawRate));
  std::memcpy(send_vector2.data(), &yaw_rate_packet, sizeof(YawRate));
  assert(send_vector2 == send_vector2);

  sensor_udp_driver_->sender()->asyncSend(send_vector);

  return Status::OK;
}

ContinentalARS548Status ContinentalARS548HwInterface::GetRadarStatus()
{
  std::lock_guard l(sensor_status_mutex_);
  return radar_status_;
}

void ContinentalARS548HwInterface::SetLogger(std::shared_ptr<rclcpp::Logger> logger)
{
  parent_node_logger = logger;
}

void ContinentalARS548HwInterface::PrintInfo(std::string info)
{
  if (parent_node_logger) {
    RCLCPP_INFO_STREAM((*parent_node_logger), info);
  } else {
    std::cout << info << std::endl;
  }
}

void ContinentalARS548HwInterface::PrintError(std::string error)
{
  if (parent_node_logger) {
    RCLCPP_ERROR_STREAM((*parent_node_logger), error);
  } else {
    std::cerr << error << std::endl;
  }
}

void ContinentalARS548HwInterface::PrintDebug(std::string debug)
{
  if (parent_node_logger) {
    RCLCPP_DEBUG_STREAM((*parent_node_logger), debug);
  } else {
    std::cout << debug << std::endl;
  }
}

void ContinentalARS548HwInterface::PrintDebug(const std::vector<uint8_t> & bytes)
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
