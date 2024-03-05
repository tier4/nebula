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

#include <nebula_common/continental/continental_srr520.hpp>
#include <nebula_common/continental/crc.hpp>
#include <nebula_hw_interfaces/nebula_hw_interfaces_continental/continental_srr520_hw_interface.hpp>

#include <limits>
#include <sstream>
namespace nebula
{
namespace drivers
{
namespace continental_srr520
{
ContinentalSRR520HwInterface::ContinentalSRR520HwInterface()
: nebula_packets_ptr_{std::make_unique<nebula_msgs::msg::NebulaPackets>()}
{
}

Status ContinentalSRR520HwInterface::SetSensorConfiguration(
  std::shared_ptr<SensorConfigurationBase> sensor_configuration)
{
  Status status = Status::OK;

  try {
    std::lock_guard lock(receiver_mutex_);
    sensor_configuration_ =
      std::static_pointer_cast<ContinentalSRR520SensorConfiguration>(sensor_configuration);
  } catch (const std::exception & ex) {
    status = Status::SENSOR_CONFIG_ERROR;
    std::cerr << status << std::endl;
    return status;
  }

  return Status::OK;
}

Status ContinentalSRR520HwInterface::SensorInterfaceStart()
{
  std::lock_guard lock(receiver_mutex_);

  try {
    can_sender_ = std::make_unique<::drivers::socketcan::SocketCanSender>(
      sensor_configuration_->interface, true);
    can_receiver_ = std::make_unique<::drivers::socketcan::SocketCanReceiver>(
      sensor_configuration_->interface, true);

    can_receiver_->SetCanFilters(
      ::drivers::socketcan::SocketCanReceiver::CanFilterList(sensor_configuration_->filters));
    PrintInfo(std::string("applied filters: ") + sensor_configuration_->filters);

    sensor_interface_active_ = true;
    receiver_thread_ =
      std::make_unique<std::thread>(&ContinentalSRR520HwInterface::ReceiveLoop, this);
  } catch (const std::exception & ex) {
    Status status = Status::CAN_CONNECTION_ERROR;
    std::cerr << status << sensor_configuration_->interface << std::endl;
    return status;
  }
  return Status::OK;
}

template <std::size_t N>
bool ContinentalSRR520HwInterface::SendFrame(const std::array<uint8_t, N> & data, int can_frame_id)
{
  ::drivers::socketcan::CanId send_id(
    can_frame_id, 0, ::drivers::socketcan::FrameType::DATA, ::drivers::socketcan::StandardFrame);

  try {
    can_sender_->send_fd(
      data.data(), data.size(), send_id,
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(sensor_configuration_->sender_timeout_sec)));
    return true;
  } catch (const std::exception & ex) {
    PrintError(std::string("Error sending CAN message: ") + ex.what());
    return false;
  }
}

void ContinentalSRR520HwInterface::ReceiveLoop()
{
  std::vector<uint8_t> buffer(64);
  ::drivers::socketcan::CanId receive_id{};
  std::chrono::nanoseconds receiver_timeout_nsec;
  bool use_bus_time;

  rdi_near_packets_ptr_ = std::make_unique<nebula_msgs::msg::NebulaPackets>();
  rdi_hrr_packets_ptr_ = std::make_unique<nebula_msgs::msg::NebulaPackets>();
  object_packets_ptr_ = std::make_unique<nebula_msgs::msg::NebulaPackets>();

  while (true) {
    {
      std::lock_guard lock(receiver_mutex_);
      receiver_timeout_nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(sensor_configuration_->receiver_timeout_sec));
      use_bus_time = sensor_configuration_->use_bus_time;

      if (!sensor_interface_active_) {
        break;
      }
    }

    try {
      buffer.resize(64);
      receive_id = can_receiver_->receive_fd(buffer.data(), receiver_timeout_nsec);
    } catch (const std::exception & ex) {
      PrintError(std::string("Error receiving CAN FD message: ") + ex.what());
      continue;
    }

    buffer.resize(receive_id.length());

    int64_t stamp = use_bus_time
                      ? static_cast<int64_t>(receive_id.get_bus_time() * 1000U)
                      : static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                                std::chrono::system_clock::now().time_since_epoch())
                                                .count());

    if (receive_id.frame_type() == ::drivers::socketcan::FrameType::ERROR) {
      PrintError("CAN FD message is an error frame");
      continue;
    }

    ReceiveSensorPacketCallback(buffer, receive_id.identifier(), stamp);
  }
}

Status ContinentalSRR520HwInterface::RegisterScanCallback(
  std::function<void(std::unique_ptr<nebula_msgs::msg::NebulaPackets>)> callback)
{
  nebula_packets_reception_callback_ = std::move(callback);
  return Status::OK;
}

void ContinentalSRR520HwInterface::ReceiveSensorPacketCallback(
  const std::vector<uint8_t> & buffer, int can_message_id, uint64_t stamp)
{
  if (buffer.size() < 4) {
    PrintError("Unrecognized packet. Too short");
    return;
  }

  if (can_message_id == RDI_NEAR_HEADER_CAN_MESSAGE_ID) {
    if (buffer.size() != RDI_NEAR_HEADER_PACKET_SIZE) {
      PrintError("RDI_NEAR_HEADER_CAN_MESSAGE_ID message with invalid size");
      return;
    }
    ProcessNearHeaderPacket(buffer, stamp);
  } else if (can_message_id == RDI_NEAR_ELEMENT_CAN_MESSAGE_ID) {
    if (buffer.size() != RDI_NEAR_ELEMENT_PACKET_SIZE) {
      PrintError("RDI_NEAR_ELEMENT_CAN_MESSAGE_ID message with invalid size");
      return;
    }

    ProcessNearElementPacket(buffer, stamp);
  } else if (can_message_id == RDI_HRR_HEADER_CAN_MESSAGE_ID) {
    if (buffer.size() != RDI_HRR_HEADER_PACKET_SIZE) {
      PrintError("RDI_HRR_HEADER_CAN_MESSAGE_ID message with invalid size");
      return;
    }
    ProcessHRRHeaderPacket(buffer, stamp);
  } else if (can_message_id == RDI_HRR_ELEMENT_CAN_MESSAGE_ID) {
    if (buffer.size() != RDI_HRR_ELEMENT_PACKET_SIZE) {
      PrintError("RDI_HRR_ELEMENT_CAN_MESSAGE_ID message with invalid size");
      return;
    }

    ProcessHRRElementPacket(buffer, stamp);
  } else if (can_message_id == OBJECT_HEADER_CAN_MESSAGE_ID) {
    if (buffer.size() != OBJECT_HEADER_PACKET_SIZE) {
      PrintError("OBJECT_HEADER_CAN_MESSAGE_ID message with invalid size");
      return;
    }
    ProcessObjectHeaderPacket(buffer, stamp);
  } else if (can_message_id == OBJECT_CAN_MESSAGE_ID) {
    if (buffer.size() != OBJECT_PACKET_SIZE) {
      PrintError("OBJECT_ELEMENT_CAN_MESSAGE_ID message with invalid size");
      return;
    }

    ProcessObjectElementPacket(buffer, stamp);
  } else if (can_message_id == CRC_LIST_CAN_MESSAGE_ID) {
    if (buffer.size() != CRC_LIST_PACKET_SIZE) {
      PrintError("CRC_LIST_CAN_MESSAGE_ID message with invalid size");
      return;
    }

    ProcessCRCListPacket(buffer, stamp);
  } else if (can_message_id == STATUS_CAN_MESSAGE_ID) {
    if (buffer.size() != STATUS_PACKET_SIZE) {
      PrintError("CRC_LIST_CAN_MESSAGE_ID message with invalid size");
      return;
    }

    ProcessSensorStatusPacket(buffer, stamp);
  } else if (can_message_id == SYNC_FUP_CAN_MESSAGE_ID) {
    if (buffer.size() != SYNC_FUP_CAN_PACKET_SIZE) {
      PrintError("SYNC_FUP_CAN_MESSAGE_ID message with invalid size");
      return;
    }

    ProcessSyncFupPacket(buffer, stamp);
  } else if (
    can_message_id != VEH_DYN_CAN_MESSAGE_ID && can_message_id != SENSOR_CONFIG_CAN_MESSAGE_ID) {
    PrintError("Unrecognized message ID=" + std::to_string(can_message_id));
  }
}

void ContinentalSRR520HwInterface::ProcessNearHeaderPacket(
  const std::vector<uint8_t> & buffer, const uint64_t stamp)
{
  nebula_msgs::msg::NebulaPacket packet;
  packet.data = {
    static_cast<uint8_t>(RDI_NEAR_HEADER_CAN_MESSAGE_ID & 0xff),
    static_cast<uint8_t>(RDI_NEAR_HEADER_CAN_MESSAGE_ID >> 8)};
  packet.stamp.sec = stamp / 1'000'000'000;
  packet.stamp.nanosec = stamp % 1'000'000'000;
  rdi_near_packets_ptr_->packets.emplace_back(packet);

  packet.data = buffer;
  rdi_near_packets_ptr_->packets.emplace_back(packet);

  rdi_near_packets_ptr_->header.stamp = packet.stamp;
}

void ContinentalSRR520HwInterface::ProcessNearElementPacket(
  const std::vector<uint8_t> & buffer, const uint64_t stamp)
{
  if (rdi_near_packets_ptr_->packets.size() == 0) {
    if (!first_rdi_near_packet_) {
      PrintError("Near element before header. This can happen during the first iteration");
    }
    return;
  }

  rdi_near_packets_ptr_->packets.front().data.push_back(
    static_cast<uint8_t>(RDI_NEAR_ELEMENT_CAN_MESSAGE_ID & 0xff));
  rdi_near_packets_ptr_->packets.front().data.push_back(
    static_cast<uint8_t>(RDI_NEAR_ELEMENT_CAN_MESSAGE_ID >> 8));

  nebula_msgs::msg::NebulaPacket packet;
  packet.data = buffer;
  packet.stamp.sec = stamp / 1'000'000'000;
  packet.stamp.nanosec = stamp % 1'000'000'000;

  rdi_near_packets_ptr_->packets.emplace_back(packet);
}

void ContinentalSRR520HwInterface::ProcessHRRHeaderPacket(
  const std::vector<uint8_t> & buffer, const uint64_t stamp)
{
  nebula_msgs::msg::NebulaPacket packet;
  packet.data = {
    static_cast<uint8_t>(RDI_HRR_HEADER_CAN_MESSAGE_ID & 0xff),
    static_cast<uint8_t>(RDI_HRR_HEADER_CAN_MESSAGE_ID >> 8)};
  packet.stamp.sec = stamp / 1'000'000'000;
  packet.stamp.nanosec = stamp % 1'000'000'000;
  rdi_hrr_packets_ptr_->packets.emplace_back(packet);

  packet.data = buffer;
  rdi_hrr_packets_ptr_->packets.emplace_back(packet);

  rdi_hrr_packets_ptr_->header.stamp = packet.stamp;
}

void ContinentalSRR520HwInterface::ProcessHRRElementPacket(
  const std::vector<uint8_t> & buffer, const uint64_t stamp)
{
  if (rdi_hrr_packets_ptr_->packets.size() == 0) {
    if (!first_rdi_hrr_packet_) {
      PrintError("HRR element before header. This can happen during the first iteration");
    }
    return;
  }

  rdi_hrr_packets_ptr_->packets.front().data.push_back(
    static_cast<uint8_t>(RDI_HRR_ELEMENT_CAN_MESSAGE_ID & 0xff));
  rdi_hrr_packets_ptr_->packets.front().data.push_back(
    static_cast<uint8_t>(RDI_HRR_ELEMENT_CAN_MESSAGE_ID >> 8));

  nebula_msgs::msg::NebulaPacket packet;
  packet.data = buffer;
  packet.stamp.sec = stamp / 1'000'000'000;
  packet.stamp.nanosec = stamp % 1'000'000'000;

  rdi_hrr_packets_ptr_->packets.emplace_back(packet);
}

void ContinentalSRR520HwInterface::ProcessObjectHeaderPacket(
  const std::vector<uint8_t> & buffer, const uint64_t stamp)
{
  nebula_msgs::msg::NebulaPacket packet;
  packet.data = {
    static_cast<uint8_t>(OBJECT_HEADER_CAN_MESSAGE_ID & 0xff),
    static_cast<uint8_t>(OBJECT_HEADER_CAN_MESSAGE_ID >> 8)};
  packet.stamp.sec = stamp / 1'000'000'000;
  packet.stamp.nanosec = stamp % 1'000'000'000;
  object_packets_ptr_->packets.emplace_back(packet);

  packet.data = buffer;
  object_packets_ptr_->packets.emplace_back(packet);

  object_packets_ptr_->header.stamp = packet.stamp;
}

void ContinentalSRR520HwInterface::ProcessObjectElementPacket(
  const std::vector<uint8_t> & buffer, const uint64_t stamp)
{
  if (object_packets_ptr_->packets.size() == 0) {
    if (!first_object_packet_) {
      PrintError("Object element before header. This can happen during the first iteration");
    }
    return;
  }

  object_packets_ptr_->header.stamp.sec = stamp / 1'000'000'000;
  object_packets_ptr_->header.stamp.nanosec = stamp % 1'000'000'000;

  object_packets_ptr_->packets.front().data.push_back(
    static_cast<uint8_t>(OBJECT_CAN_MESSAGE_ID & 0xff));
  object_packets_ptr_->packets.front().data.push_back(
    static_cast<uint8_t>(OBJECT_CAN_MESSAGE_ID >> 8));

  nebula_msgs::msg::NebulaPacket packet;
  packet.data = buffer;
  packet.stamp = object_packets_ptr_->header.stamp;

  object_packets_ptr_->packets.emplace_back(packet);
}

void ContinentalSRR520HwInterface::ProcessCRCListPacket(
  const std::vector<uint8_t> & buffer, const uint64_t stamp)
{
  const auto crc_id = buffer[0];

  if (crc_id == NEAR_CRC_ID) {
    ProcessNearCRCListPacket(buffer, stamp);
  } else if (crc_id == HRR_CRC_ID) {
    ProcessHRRCRCListPacket(buffer, stamp);  // cspell: ignore HRRCRC
  } else if (crc_id == OBJECT_CRC_ID) {
    ProcessObjectCRCListPacket(buffer, stamp);
  } else {
    PrintError(std::string("Unrecognized CRC id=") + std::to_string(crc_id));
  }
}

void ContinentalSRR520HwInterface::ProcessNearCRCListPacket(
  const std::vector<uint8_t> & buffer, [[maybe_unused]] const uint64_t stamp)
{
  if (rdi_near_packets_ptr_->packets.size() != RDI_NEAR_PACKET_NUM + 2) {
    if (!first_rdi_near_packet_) {
      PrintError("Incorrect number of RDI Near elements before CRC list");
    }
    return;
  }

  uint16_t transmitted_crc = (static_cast<uint16_t>(buffer[1]) << 8) | buffer[2];
  uint16_t computed_crc =
    crc16_packets(rdi_near_packets_ptr_->packets.begin() + 1, rdi_near_packets_ptr_->packets.end());

  if (transmitted_crc != computed_crc) {
    PrintError(
      "RDI Near: Transmitted CRC list does not coincide with the computed one. Ignoring packet");
    return;
  }

  nebula_packets_reception_callback_(std::move(rdi_near_packets_ptr_));
  rdi_near_packets_ptr_ = std::make_unique<nebula_msgs::msg::NebulaPackets>();
  first_rdi_near_packet_ = false;
}

void ContinentalSRR520HwInterface::ProcessHRRCRCListPacket(
  const std::vector<uint8_t> & buffer, [[maybe_unused]] const uint64_t stamp)
{
  if (rdi_hrr_packets_ptr_->packets.size() != RDI_HRR_PACKET_NUM + 2) {
    if (!first_rdi_hrr_packet_) {
      PrintError("Incorrect number of RDI HRR elements before CRC list");
    }
    return;
  }

  uint16_t transmitted_crc = (static_cast<uint16_t>(buffer[1]) << 8) | buffer[2];
  uint16_t computed_crc =
    crc16_packets(rdi_hrr_packets_ptr_->packets.begin() + 1, rdi_hrr_packets_ptr_->packets.end());

  if (transmitted_crc != computed_crc) {
    PrintError(
      "RDI HRR: Transmitted CRC list does not coincide with the computed one. Ignoring packet");
    return;
  }

  nebula_packets_reception_callback_(std::move(rdi_hrr_packets_ptr_));
  rdi_hrr_packets_ptr_ = std::make_unique<nebula_msgs::msg::NebulaPackets>();
  first_rdi_hrr_packet_ = false;
}

void ContinentalSRR520HwInterface::ProcessObjectCRCListPacket(
  const std::vector<uint8_t> & buffer, [[maybe_unused]] const uint64_t stamp)
{
  if (object_packets_ptr_->packets.size() != OBJECT_PACKET_NUM + 2) {
    if (!first_object_packet_) {
      PrintError("Incorrect number of object packages before CRC list");
    }
    return;
  }

  uint16_t transmitted_crc = (static_cast<uint16_t>(buffer[1]) << 8) | buffer[2];
  uint16_t computed_crc =
    crc16_packets(object_packets_ptr_->packets.begin() + 1, object_packets_ptr_->packets.end());

  // uint8_t current_seq = buffer[3];

  if (transmitted_crc != computed_crc) {
    PrintError(
      "Object: Transmitted CRC list does not coincide with the computed one. Ignoring packet");
    return;
  }

  nebula_packets_reception_callback_(std::move(object_packets_ptr_));
  object_packets_ptr_ = std::make_unique<nebula_msgs::msg::NebulaPackets>();
  first_object_packet_ = false;
}

void ContinentalSRR520HwInterface::ProcessSensorStatusPacket(
  const std::vector<uint8_t> & buffer, const uint64_t stamp)
{
  auto status_packets_ptr = std::make_unique<nebula_msgs::msg::NebulaPackets>();

  nebula_msgs::msg::NebulaPacket packet;
  packet.stamp.sec = stamp / 1'000'000'000;
  packet.stamp.nanosec = stamp % 1'000'000'000;
  packet.data = {
    static_cast<uint8_t>(STATUS_CAN_MESSAGE_ID & 0xff),
    static_cast<uint8_t>(STATUS_CAN_MESSAGE_ID >> 8)};

  status_packets_ptr->packets.push_back(packet);

  packet.data = buffer;
  status_packets_ptr->packets.push_back(packet);

  status_packets_ptr->header.stamp = packet.stamp;
  nebula_packets_reception_callback_(std::move(status_packets_ptr));
}

void ContinentalSRR520HwInterface::ProcessSyncFupPacket(
  [[maybe_unused]] const std::vector<uint8_t> & buffer, [[maybe_unused]] const uint64_t stamp)
{
  if (!can_sender_) {
    PrintError("Can sender is invalid so can not do follow up");
  }

  if (!sensor_configuration_->sync_use_bus_time || sync_fup_sent_) {
    return;
  }

  auto t0s = last_sync_stamp_;
  t0s.nanosec = 0;
  const auto t1r = stamp;

  builtin_interfaces::msg::Time t4r =
    rclcpp::Time(rclcpp::Time() + (rclcpp::Time(t1r) - rclcpp::Time(t0s)));
  uint8_t t4r_seconds = static_cast<uint8_t>(t4r.sec);
  uint32_t t4r_nanoseconds = t4r.nanosec;
  std::array<uint8_t, 8> data;
  data[0] = 0x28;  // mode 0x18 is without CRC
  data[2] = (((static_cast<uint16_t>(TIME_DOMAIN_ID) << 4) & 0xF0)) |
            (sync_counter_ & 0x0F);  // Domain and counter
  data[3] = t4r_seconds & 0x3;       // SGW and OVS
  data[4] = (t4r_nanoseconds & 0xFF000000) >> 24;
  data[5] = (t4r_nanoseconds & 0x00FF0000) >> 16;
  data[6] = (t4r_nanoseconds & 0x0000FF00) >> 8;
  data[7] = (t4r_nanoseconds & 0x000000FF) >> 0;

  std::array<uint8_t, 7> fup_crc_array{data[2], data[3], data[4], data[5], data[6], data[7], 0x00};
  uint8_t fup_crc = crc8h2f(fup_crc_array.begin(), fup_crc_array.end());
  data[1] = fup_crc;

  SendFrame(data, SYNC_FUP_CAN_MESSAGE_ID);

  sync_fup_sent_ = true;
  sync_counter_ = sync_counter_ == 15 ? 0 : sync_counter_ + 1;
}

void ContinentalSRR520HwInterface::SensorSync()
{
  if (!can_sender_) {
    PrintError("Can sender is invalid so can not do sync up");
  }

  if (!sync_fup_sent_) {
    PrintError("We will send a SYNC message without having sent a FUP message first!");
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
  data[2] = (((static_cast<uint16_t>(TIME_DOMAIN_ID) << 4) & 0xF0)) |
            (sync_counter_ & 0x0F);  // Domain and counter
  data[3] = 0;                       // use data
  data[4] = (stamp.sec & 0xFF000000) >> 24;
  data[5] = (stamp.sec & 0x00FF0000) >> 16;
  data[6] = (stamp.sec & 0x0000FF00) >> 8;
  data[7] = (stamp.sec & 0x000000FF) >> 0;

  std::array<uint8_t, 7> sync_crc_array{data[2], data[3], data[4], data[5], data[6], data[7], 0x00};
  uint8_t sync_crc = crc8h2f(sync_crc_array.begin(), sync_crc_array.end());
  data[1] = sync_crc;

  SendFrame(data, SYNC_FUP_CAN_MESSAGE_ID);

  if (sensor_configuration_->sync_use_bus_time) {
    sync_fup_sent_ = false;
    return;
  }

  data[0] = 0x28;  // mode 0x18 is without CRC
  data[2] = (((static_cast<uint16_t>(TIME_DOMAIN_ID) << 4) & 0xF0)) |
            (sync_counter_ & 0x0F);  // Domain and counter
  data[3] = 0;                       // SGW and OVS
  data[4] = (stamp.nanosec & 0xFF000000) >> 24;
  data[5] = (stamp.nanosec & 0x00FF0000) >> 16;
  data[6] = (stamp.nanosec & 0x0000FF00) >> 8;
  data[7] = (stamp.nanosec & 0x000000FF) >> 0;

  std::array<uint8_t, 7> fup_crc_array{data[2], data[3], data[4], data[5], data[6], data[7], 0x00};
  uint8_t fup_crc = crc8h2f(fup_crc_array.begin(), fup_crc_array.end());
  data[1] = fup_crc;

  SendFrame(data, SYNC_FUP_CAN_MESSAGE_ID);

  sync_counter_ = sync_counter_ == 15 ? 0 : sync_counter_ + 1;
  sync_fup_sent_ = true;
}

void ContinentalSRR520HwInterface::ProcessDataPacket(const std::vector<uint8_t> & buffer)
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

  {
    std::lock_guard lock(receiver_mutex_);
    nebula_packets_ptr_->header.frame_id = sensor_configuration_->frame_id;
  }

  nebula_packets_reception_callback_(std::move(nebula_packets_ptr_));
  nebula_packets_ptr_ = std::make_unique<nebula_msgs::msg::NebulaPackets>();
}

Status ContinentalSRR520HwInterface::SensorInterfaceStop()
{
  {
    std::lock_guard l(receiver_mutex_);
    sensor_interface_active_ = true;
  }

  receiver_thread_->join();
  return Status::ERROR_1;
}

Status ContinentalSRR520HwInterface::GetSensorConfiguration(
  SensorConfigurationBase & sensor_configuration)
{
  std::stringstream ss;
  ss << sensor_configuration;
  PrintDebug(ss.str());
  return Status::ERROR_1;
}

Status ContinentalSRR520HwInterface::ConfigureSensor(
  uint8_t sensor_id, float longitudinal_autosar, float lateral_autosar, float vertical_autosar,
  float yaw_autosar, float longitudinal_cog, float wheelbase, float cover_damping, bool plug_bottom,
  bool reset)
{
  std::cout << "longitudinal_autosar=" << longitudinal_autosar << std::endl;
  std::cout << "lateral_autosar=" << lateral_autosar << std::endl;
  std::cout << "vertical_autosar=" << vertical_autosar << std::endl;
  std::cout << "longitudinal_cog=" << longitudinal_cog << std::endl;
  std::cout << "wheelbase=" << wheelbase << std::endl;
  std::cout << "yaw_autosar=" << yaw_autosar << std::endl;
  std::cout << "sensor_id=" << static_cast<uint16_t>(sensor_id) << std::endl << std::flush;

  if (
    longitudinal_autosar < -32.767f || longitudinal_autosar > 32.767f ||
    lateral_autosar < -32.767f || lateral_autosar > 32.767f || vertical_autosar < -32.767f ||
    vertical_autosar > 32.767f || longitudinal_cog < -32.767f || longitudinal_cog > 32.767f ||
    wheelbase < 0.f || wheelbase > 65.534f || yaw_autosar < -3.14159f || yaw_autosar > 3.14159f ||
    cover_damping < -32.767f || cover_damping > 32.767f) {
    PrintError("Sensor configuration values out of range!");
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

  if (SendFrame(data, SENSOR_CONFIG_CAN_MESSAGE_ID)) {
    return Status::OK;
  } else {
    return Status::CAN_CONNECTION_ERROR;
  }
}

Status ContinentalSRR520HwInterface::SetVehicleDynamics(
  float longitudinal_acceleration, float lateral_acceleration, float yaw_rate,
  float longitudinal_velocity, bool standstill)
{
  if (
    longitudinal_acceleration < -12.7 || longitudinal_acceleration > 12.7 ||
    lateral_acceleration < -12.7 || lateral_acceleration > 12.7 || yaw_rate < -3.14159 ||
    yaw_rate > 3.14159 || abs(longitudinal_velocity) > 100.0) {
    PrintError("Vehicle dynamics out of range!");
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

  if (SendFrame(data, VEH_DYN_CAN_MESSAGE_ID)) {
    return Status::OK;
  } else {
    return Status::CAN_CONNECTION_ERROR;
  }
}

void ContinentalSRR520HwInterface::SetLogger(std::shared_ptr<rclcpp::Logger> logger)
{
  parent_node_logger = logger;
}

void ContinentalSRR520HwInterface::PrintInfo(std::string info)
{
  if (parent_node_logger) {
    RCLCPP_INFO_STREAM((*parent_node_logger), info);
  } else {
    std::cout << info << std::endl;
  }
}

void ContinentalSRR520HwInterface::PrintError(std::string error)
{
  if (parent_node_logger) {
    RCLCPP_ERROR_STREAM((*parent_node_logger), error);
  } else {
    std::cerr << error << std::endl;
  }
}

void ContinentalSRR520HwInterface::PrintDebug(std::string debug)
{
  if (parent_node_logger) {
    RCLCPP_DEBUG_STREAM((*parent_node_logger), debug);
  } else {
    std::cout << debug << std::endl;
  }
}

void ContinentalSRR520HwInterface::PrintDebug(const std::vector<uint8_t> & bytes)
{
  std::stringstream ss;
  for (const auto & b : bytes) {
    ss << static_cast<int>(b) << ", ";
  }
  ss << std::endl;
  PrintDebug(ss.str());
}

}  // namespace continental_srr520
}  // namespace drivers
}  // namespace nebula
