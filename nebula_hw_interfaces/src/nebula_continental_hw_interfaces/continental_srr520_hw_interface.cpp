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

#include "nebula_hw_interfaces/nebula_hw_interfaces_continental/continental_srr520_hw_interface.hpp"

#include <nebula_common/continental/continental_srr520.hpp>
#include <nebula_common/continental/crc.hpp>
#include <rclcpp/time.hpp>

#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <utility>

namespace nebula::drivers::continental_srr520
{
ContinentalSRR520HwInterface::ContinentalSRR520HwInterface(
  const std::shared_ptr<loggers::Logger> & logger)
: logger_(logger)
{
}

Status ContinentalSRR520HwInterface::set_sensor_configuration(
  const std::shared_ptr<
    const nebula::drivers::continental_srr520::ContinentalSRR520SensorConfiguration>
    new_config_ptr)
{
  config_ptr_ = new_config_ptr;

  return Status::OK;
}

Status ContinentalSRR520HwInterface::sensor_interface_start()
{
  std::lock_guard lock(receiver_mutex_);

  try {
    can_sender_ptr_ =
      std::make_unique<::drivers::socketcan::SocketCanSender>(config_ptr_->interface, true);
    can_receiver_ptr_ =
      std::make_unique<::drivers::socketcan::SocketCanReceiver>(config_ptr_->interface, true);

    can_receiver_ptr_->SetCanFilters(
      ::drivers::socketcan::SocketCanReceiver::CanFilterList(config_ptr_->filters));
    logger_->info(std::string("applied filters: ") + config_ptr_->filters);

    sensor_interface_active_ = true;
    receiver_thread_ptr_ =
      std::make_unique<std::thread>(&ContinentalSRR520HwInterface::receive_loop, this);
  } catch (const std::exception & ex) {
    Status status = Status::CAN_CONNECTION_ERROR;
    logger_->error(std::string("Error connecting to CAN interface: ") + ex.what());
    return status;
  }
  return Status::OK;
}

template <std::size_t N>
bool ContinentalSRR520HwInterface::send_frame(const std::array<uint8_t, N> & data, int can_frame_id)
{
  ::drivers::socketcan::CanId send_id(
    can_frame_id, 0, ::drivers::socketcan::FrameType::DATA, ::drivers::socketcan::StandardFrame);

  try {
    can_sender_ptr_->send_fd(
      data.data(), data.size(), send_id,
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(config_ptr_->sender_timeout_sec)));
    return true;
  } catch (const std::exception & ex) {
    logger_->error(std::string("Error sending CAN message: ") + ex.what());
    return false;
  }
}

void ContinentalSRR520HwInterface::receive_loop()
{
  ::drivers::socketcan::CanId receive_id{};
  std::chrono::nanoseconds receiver_timeout_nsec;
  bool use_bus_time = false;

  while (true) {
    auto packet_msg_ptr = std::make_unique<nebula_msgs::msg::NebulaPacket>();

    {
      std::lock_guard lock(receiver_mutex_);
      receiver_timeout_nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(config_ptr_->receiver_timeout_sec));
      use_bus_time = config_ptr_->use_bus_time;

      if (!sensor_interface_active_) {
        break;
      }
    }

    try {
      packet_msg_ptr->data.resize(68);  // 64 bytes of data + 4 bytes of ID
      receive_id = can_receiver_ptr_->receive_fd(
        packet_msg_ptr->data.data() + 4 * sizeof(uint8_t), receiver_timeout_nsec);
    } catch (const std::exception & ex) {
      logger_->error(std::string("Error receiving CAN FD message: ") + ex.what());
      continue;
    }

    packet_msg_ptr->data.resize(receive_id.length() + 4);

    uint32_t id = receive_id.identifier();
    packet_msg_ptr->data[0] = (id & 0xFF000000) >> 24;
    packet_msg_ptr->data[1] = (id & 0x00FF0000) >> 16;
    packet_msg_ptr->data[2] = (id & 0x0000FF00) >> 8;
    packet_msg_ptr->data[3] = (id & 0x000000FF) >> 0;

    int64_t stamp = use_bus_time
                      ? static_cast<int64_t>(receive_id.get_bus_time() * 1000U)
                      : static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                                std::chrono::system_clock::now().time_since_epoch())
                                                .count());

    packet_msg_ptr->stamp.sec = stamp / 1'000'000'000;
    packet_msg_ptr->stamp.nanosec = stamp % 1'000'000'000;

    if (receive_id.frame_type() == ::drivers::socketcan::FrameType::ERROR) {
      logger_->error("CAN FD message is an error frame");
      continue;
    }

    nebula_packet_callback_(std::move(packet_msg_ptr));
  }
}

Status ContinentalSRR520HwInterface::register_packet_callback(
  std::function<void(std::unique_ptr<nebula_msgs::msg::NebulaPacket>)> callback)
{
  nebula_packet_callback_ = std::move(callback);
  return Status::OK;
}

void ContinentalSRR520HwInterface::sensor_sync_follow_up(builtin_interfaces::msg::Time stamp)
{
  if (!can_sender_ptr_) {
    logger_->error("Can sender is invalid so can not do follow up");
  }

  if (!config_ptr_->sync_use_bus_time || sync_follow_up_sent_) {
    return;
  }

  auto t0s = last_sync_stamp_;
  t0s.nanosec = 0;
  const auto & t1r = stamp;

  builtin_interfaces::msg::Time t4r =
    rclcpp::Time(rclcpp::Time() + (rclcpp::Time(t1r) - rclcpp::Time(t0s)));
  uint8_t t4r_seconds = static_cast<uint8_t>(t4r.sec);
  uint32_t t4r_nanoseconds = t4r.nanosec;
  std::array<uint8_t, 8> data;
  data[0] = 0x28;  // mode 0x18 is without CRC
  data[2] = (((static_cast<uint16_t>(time_domain_id) << 4) & 0xF0)) |
            (sync_counter_ & 0x0F);  // Domain and counter
  data[3] = t4r_seconds & 0x3;       // SGW and OVS
  data[4] = (t4r_nanoseconds & 0xFF000000) >> 24;
  data[5] = (t4r_nanoseconds & 0x00FF0000) >> 16;
  data[6] = (t4r_nanoseconds & 0x0000FF00) >> 8;
  data[7] = (t4r_nanoseconds & 0x000000FF) >> 0;

  std::array<uint8_t, 7> follow_up_crc_array{data[2], data[3], data[4], data[5],
                                             data[6], data[7], 0x00};
  uint8_t follow_up_crc = crc8h2f(follow_up_crc_array.begin(), follow_up_crc_array.end());
  data[1] = follow_up_crc;

  send_frame(data, sync_follow_up_can_message_id);

  sync_follow_up_sent_ = true;
  sync_counter_ = sync_counter_ == 15 ? 0 : sync_counter_ + 1;
}

void ContinentalSRR520HwInterface::sensor_sync()
{
  if (!can_sender_ptr_) {
    logger_->error("Can sender is invalid so can not do sync up");
    return;
  }

  if (!sync_follow_up_sent_) {
    logger_->error("We will send a SYNC message without having sent a FollowUp message first!");
  }

  auto now = std::chrono::system_clock::now();
  auto now_secs = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
  auto now_nanosecs =
    std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();

  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int>(now_secs);
  stamp.nanosec = static_cast<std::uint32_t>(now_nanosecs % 1'000'000'000);
  last_sync_stamp_ = stamp;

  std::array<uint8_t, 8> data;
  data[0] = 0x20;  // mode 0x10 is without CRC
  data[2] = (((static_cast<uint16_t>(time_domain_id) << 4) & 0xF0)) |
            (sync_counter_ & 0x0F);  // Domain and counter
  data[3] = 0;                       // use data
  data[4] = (stamp.sec & 0xFF000000) >> 24;
  data[5] = (stamp.sec & 0x00FF0000) >> 16;
  data[6] = (stamp.sec & 0x0000FF00) >> 8;
  data[7] = (stamp.sec & 0x000000FF) >> 0;

  std::array<uint8_t, 7> sync_crc_array{data[2], data[3], data[4], data[5], data[6], data[7], 0x00};
  uint8_t sync_crc = crc8h2f(sync_crc_array.begin(), sync_crc_array.end());
  data[1] = sync_crc;

  send_frame(data, sync_follow_up_can_message_id);

  if (config_ptr_->sync_use_bus_time) {
    sync_follow_up_sent_ = false;
    return;
  }

  data[0] = 0x28;  // mode 0x18 is without CRC
  data[2] = (((static_cast<uint16_t>(time_domain_id) << 4) & 0xF0)) |
            (sync_counter_ & 0x0F);  // Domain and counter
  data[3] = 0;                       // SGW and OVS
  data[4] = (stamp.nanosec & 0xFF000000) >> 24;
  data[5] = (stamp.nanosec & 0x00FF0000) >> 16;
  data[6] = (stamp.nanosec & 0x0000FF00) >> 8;
  data[7] = (stamp.nanosec & 0x000000FF) >> 0;

  std::array<uint8_t, 7> follow_up_crc_array{data[2], data[3], data[4], data[5],
                                             data[6], data[7], 0x00};
  uint8_t follow_up_crc = crc8h2f(follow_up_crc_array.begin(), follow_up_crc_array.end());
  data[1] = follow_up_crc;

  send_frame(data, sync_follow_up_can_message_id);

  sync_counter_ = sync_counter_ == 15 ? 0 : sync_counter_ + 1;
  sync_follow_up_sent_ = true;
}

Status ContinentalSRR520HwInterface::sensor_interface_stop()
{
  {
    std::lock_guard l(receiver_mutex_);
    sensor_interface_active_ = false;
  }

  receiver_thread_ptr_->join();
  return Status::ERROR_1;
}

Status ContinentalSRR520HwInterface::configure_sensor(
  uint8_t sensor_id, float longitudinal_autosar, float lateral_autosar, float vertical_autosar,
  float yaw_autosar, float longitudinal_cog, float wheelbase, float cover_damping, bool plug_bottom,
  bool reset)
{
  logger_->info("longitudinal_autosar=" + std::to_string(longitudinal_autosar));
  logger_->info("lateral_autosar=" + std::to_string(lateral_autosar));
  logger_->info("vertical_autosar=" + std::to_string(vertical_autosar));
  logger_->info("longitudinal_cog=" + std::to_string(longitudinal_cog));
  logger_->info("wheelbase=" + std::to_string(wheelbase));
  logger_->info("yaw_autosar=" + std::to_string(yaw_autosar));
  logger_->info("sensor_id=" + std::to_string(static_cast<uint16_t>(sensor_id)));
  logger_->info("plug_bottom=" + std::to_string(plug_bottom));

  if (
    longitudinal_autosar < -32.767f || longitudinal_autosar > 32.767f ||
    lateral_autosar < -32.767f || lateral_autosar > 32.767f || vertical_autosar < -32.767f ||
    vertical_autosar > 32.767f || longitudinal_cog < -32.767f || longitudinal_cog > 32.767f ||
    wheelbase < 0.f || wheelbase > 65.534f || yaw_autosar < -3.14159f || yaw_autosar > 3.14159f ||
    cover_damping < -32.767f || cover_damping > 32.767f) {
    logger_->error("Sensor configuration values out of range!");
    return Status::SENSOR_CONFIG_ERROR;
  }

  const uint16_t u_long_pos = static_cast<uint16_t>((longitudinal_autosar + 32.767f) / 0.001f);
  const uint16_t u_lat_pos = static_cast<uint16_t>((lateral_autosar + 32.767f) / 0.001f);
  const uint16_t u_vert_pos = static_cast<uint16_t>((vertical_autosar + 32.767f) / 0.001f);
  const uint16_t u_long_pos_cog = static_cast<uint16_t>((longitudinal_cog + 32.767f) / 0.001f);
  const uint16_t u_wheelbase = static_cast<uint16_t>(wheelbase / 0.001f);
  const uint16_t u_yaw_angle = static_cast<uint16_t>((yaw_autosar + 3.14159f) / 9.5877e-05);
  const uint16_t u_cover_damping = static_cast<uint16_t>((cover_damping + 32.767f) / 0.001f);

  std::array<uint8_t, 16> data;
  data[0] = sensor_id;
  data[1] = static_cast<uint8_t>((u_long_pos & 0xff00) >> 8);
  data[2] = static_cast<uint8_t>((u_long_pos & 0x00ff));

  data[3] = static_cast<uint8_t>((u_lat_pos & 0xff00) >> 8);
  data[4] = static_cast<uint8_t>((u_lat_pos & 0x00ff));

  data[5] = static_cast<uint8_t>((u_vert_pos & 0xff00) >> 8);
  data[6] = static_cast<uint8_t>((u_vert_pos & 0x00ff));

  data[7] = static_cast<uint8_t>((u_long_pos_cog & 0xff00) >> 8);
  data[8] = static_cast<uint8_t>((u_long_pos_cog & 0x00ff));

  data[9] = static_cast<uint8_t>((u_wheelbase & 0xff00) >> 8);
  data[10] = static_cast<uint8_t>((u_wheelbase & 0x00ff));

  data[11] = static_cast<uint8_t>((u_yaw_angle & 0xff00) >> 8);
  data[12] = static_cast<uint8_t>((u_yaw_angle & 0x00ff));

  data[13] = static_cast<uint8_t>((u_cover_damping & 0xff00) >> 8);
  data[14] = static_cast<uint8_t>((u_cover_damping & 0x00ff));

  uint8_t plug_value = plug_bottom ? 0x00 : 0x01;
  uint8_t reset_value = reset ? 0x80 : 0x00;
  data[15] = plug_value | reset_value;

  if (send_frame(data, sensor_config_can_message_id)) {
    return Status::OK;
  } else {
    return Status::CAN_CONNECTION_ERROR;
  }
}

Status ContinentalSRR520HwInterface::set_vehicle_dynamics(
  float longitudinal_acceleration, float lateral_acceleration, float yaw_rate,
  float longitudinal_velocity, bool standstill)
{
  if (
    longitudinal_acceleration < -12.7 || longitudinal_acceleration > 12.7 ||
    lateral_acceleration < -12.7 || lateral_acceleration > 12.7 || yaw_rate < -3.14159 ||
    yaw_rate > 3.14159 || abs(longitudinal_velocity) > 100.0) {
    logger_->error("Vehicle dynamics out of range!");
    return Status::SENSOR_CONFIG_ERROR;
  }

  const uint8_t u_long_accel = static_cast<uint8_t>((longitudinal_acceleration + 12.7) / 0.1);
  const uint8_t u_lat_accel = static_cast<uint8_t>((lateral_acceleration + 12.7) / 0.1);
  const uint16_t u_yaw_rate = static_cast<uint16_t>((yaw_rate + 3.14159) / 0.001534729);
  const uint16_t u_long_vel = static_cast<uint16_t>(std::abs(longitudinal_velocity) / 0.024425989);
  uint8_t u_long_dir;

  if (standstill) {
    u_long_dir = 0;
  } else if (longitudinal_velocity > 0) {
    u_long_dir = 1;
  } else {
    u_long_dir = 2;
  }

  std::array<uint8_t, 8> data;
  data[0] = u_long_accel;
  data[1] = u_lat_accel;
  data[2] = static_cast<uint8_t>((u_yaw_rate & 0xff0) >> 4);
  data[3] = (static_cast<uint8_t>(u_yaw_rate & 0x0f) << 4) |
            static_cast<uint8_t>((u_long_vel & 0xf00) >> 8);
  data[4] = static_cast<uint8_t>(u_long_vel & 0xff);
  data[5] = u_long_dir;
  data[6] = 0x00;
  data[7] = 0x00;

  if (send_frame(data, veh_dyn_can_message_id)) {
    return Status::OK;
  } else {
    return Status::CAN_CONNECTION_ERROR;
  }
}

}  // namespace nebula::drivers::continental_srr520
