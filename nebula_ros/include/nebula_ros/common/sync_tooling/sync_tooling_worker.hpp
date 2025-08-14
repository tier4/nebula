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

#pragma once

#include <nebula_common/util/errno.hpp>
#include <nebula_common/util/expected.hpp>
#include <nebula_common/util/rate_limiter.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/u_int8_multi_array.hpp>

#include <boost/range/algorithm/copy.hpp>

#include <google/protobuf/message.h>
#include <sync_tooling_msgs/clock_alias_update.pb.h>
#include <sync_tooling_msgs/clock_diff_measurement.pb.h>
#include <sync_tooling_msgs/clock_id.pb.h>
#include <sync_tooling_msgs/clock_master_update.pb.h>
#include <sync_tooling_msgs/graph_update.pb.h>
#include <sync_tooling_msgs/port_state.pb.h>
#include <sync_tooling_msgs/port_state_update.pb.h>
#include <sync_tooling_msgs/ptp_parent_update.pb.h>
#include <sync_tooling_msgs/self_reported_clock_state_update.pb.h>
#include <sync_tooling_msgs/system_clock_id.pb.h>
#include <unistd.h>

#include <cerrno>
#include <climits>
#include <cstdint>
#include <optional>
#include <string>

namespace nebula::ros
{

inline ClockId make_ptp_clock_id(const std::string & ptp_clock_id)
{
  ClockId id;
  id.mutable_ptp_clock_id()->set_id(ptp_clock_id);
  return id;
}

inline ClockId make_sensor_clock_id(const std::string & frame_id)
{
  ClockId id;
  id.mutable_sensor_id()->set_frame_id(frame_id);
  return id;
}

class SyncToolingWorker
{
public:
  SyncToolingWorker(
    rclcpp::Node * const parent_node, const std::string & topic, const std::string & frame_id,
    uint8_t ptp_domain_id)
  : publisher_(parent_node->create_publisher<std_msgs::msg::UInt8MultiArray>(topic, 10)),
    hostname_(get_hostname()),
    sensor_id_(make_sensor_clock_id(frame_id)),
    ptp_domain_id_(ptp_domain_id)
  {
  }

  void submit_clock_alias(const std::string & ptp_clock_id)
  {
    GraphUpdate gu;
    ClockAliasUpdate * u = gu.mutable_clock_alias_update();
    u->add_aliases()->CopyFrom(sensor_id_);
    u->add_aliases()->mutable_ptp_clock_id()->set_id(ptp_clock_id);
    send_proto(gu);
  }

  void submit_sensor_clock_time_snapshot(uint64_t timestamp_ns)
  {
    GraphUpdate gu;
    ClockTimeSnapshot * u = gu.mutable_clock_time_snapshot();
    u->mutable_clock()->CopyFrom(sensor_id_);
    u->set_time_ns(timestamp_ns);
    send_proto(gu);
  }

  void submit_clock_diff_measurement(int64_t diff_ns)
  {
    GraphUpdate gu;
    ClockDiffMeasurement * m = gu.mutable_clock_diff_measurement();
    m->set_diff_ns(diff_ns);
    m->mutable_src()->mutable_system_clock_id()->set_hostname(hostname_);
    m->mutable_dst()->CopyFrom(sensor_id_);
    send_proto(gu);
  }

  void submit_master_update(std::optional<std::string> master_clock_id)
  {
    GraphUpdate gu;
    ClockMasterUpdate * u = gu.mutable_clock_master_update();
    u->mutable_clock_id()->CopyFrom(sensor_id_);
    if (master_clock_id) {
      u->mutable_master()->mutable_ptp_clock_id()->set_id(master_clock_id.value());
    } else {
      u->clear_master();
    }
    send_proto(gu);
  }

  void submit_parent_port(const std::string & parent_clock_id, uint16_t port_number)
  {
    GraphUpdate gu;
    PtpParentUpdate * u = gu.mutable_ptp_parent_update();
    u->mutable_clock_id()->CopyFrom(sensor_id_);
    u->mutable_parent()->mutable_clock_id()->mutable_ptp_clock_id()->set_id(parent_clock_id);
    u->mutable_parent()->set_port_number(port_number);
    u->mutable_parent()->set_ptp_domain(ptp_domain_id_);
    send_proto(gu);
  }

  void submit_port_state_update(const ClockId & clock_id, uint16_t port_number, uint8_t port_state)
  {
    GraphUpdate gu;
    PortStateUpdate * u = gu.mutable_port_state_update();
    u->mutable_port_id()->mutable_clock_id()->CopyFrom(clock_id);
    u->mutable_port_id()->set_port_number(port_number);
    u->mutable_port_id()->set_ptp_domain(ptp_domain_id_);

    u->set_port_state(static_cast<PortState>(port_state));
    send_proto(gu);
  }

  void submit_self_reported_clock_state(SelfReportedClockStateUpdate::State state)
  {
    GraphUpdate gu;
    SelfReportedClockStateUpdate * u = gu.mutable_self_reported_clock_state_update();
    u->mutable_clock_id()->CopyFrom(sensor_id_);
    u->set_state(state);
    send_proto(gu);
  }

private:
  static std::string get_hostname()
  {
    std::array<char, HOST_NAME_MAX + 1> hostname_raw{};
    auto result = gethostname(hostname_raw.data(), hostname_raw.size());

    if (result == -1) {
      throw std::runtime_error(util::errno_to_string(errno));
    }

    hostname_raw.at(hostname_raw.size() - 1) = '\0';
    return std::string{hostname_raw.data()};
  }

  void send_proto(const GraphUpdate & msg)
  {
    auto ros2_msg = std_msgs::msg::UInt8MultiArray();

    size_t serialized_size = msg.ByteSizeLong();
    ros2_msg.data.resize(serialized_size);
    bool success =
      msg.SerializeToArray(ros2_msg.data.data(), static_cast<int>(ros2_msg.data.size()));
    assert(success);

    if (!success) {
      RCLCPP_ERROR(rclcpp::get_logger("SyncToolingWorker"), "Failed to serialize protobuf message");
      return;
    }

    publisher_->publish(ros2_msg);
  }

  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;

  std::string hostname_;
  ClockId sensor_id_;
  uint8_t ptp_domain_id_;
};

}  // namespace nebula::ros
