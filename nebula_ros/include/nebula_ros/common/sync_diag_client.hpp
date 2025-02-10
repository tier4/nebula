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
#include <nebula_hw_interfaces/nebula_hw_interfaces_common/connections/http.hpp>

#include <google/protobuf/message.h>
#include <sync_tooling_msgs/clock_alias_update.pb.h>
#include <sync_tooling_msgs/clock_diff_measurement.pb.h>
#include <sync_tooling_msgs/clock_id.pb.h>
#include <sync_tooling_msgs/clock_master_update.pb.h>
#include <sync_tooling_msgs/ptp_parent_update.pb.h>
#include <sync_tooling_msgs/system_clock_id.pb.h>
#include <unistd.h>

#include <cerrno>
#include <climits>
#include <cstdint>
#include <future>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace nebula::ros
{

class SyncDiagClient
{
public:
  SyncDiagClient(std::string master_ip, uint16_t master_port, std::string sensor_id)
  : http_client_(std::move(master_ip), std::to_string(master_port)),
    hostname_(get_hostname()),
    sensor_id_(std::move(sensor_id))
  {
  }

  void submit_clock_alias(const std::string & ptp_clock_id)
  {
    ClockAliasUpdate u;
    u.add_aliases()->mutable_frame_id()->set_frame(sensor_id_);
    u.add_aliases()->mutable_ptp_clock_id()->set_id(ptp_clock_id);
    send_proto(u);
  }

  void submit_clock_diff_measurement(int64_t diff_ns)
  {
    ClockDiffMeasurement m{};
    m.set_diff_ns(diff_ns);
    m.mutable_src()->mutable_system_clock_id()->set_hostname(hostname_);
    m.mutable_dst()->mutable_frame_id()->set_frame(sensor_id_);
    send_proto(m);
  }

  void submit_master_update(std::optional<std::string> master_clock_id)
  {
    ClockMasterUpdate u;
    u.mutable_clock_id()->mutable_frame_id()->set_frame(sensor_id_);
    if (master_clock_id) {
      u.mutable_master()->mutable_ptp_clock_id()->set_id(master_clock_id.value());
    } else {
      u.clear_master();
    }
    send_proto(u);
  }

  void submit_parent_port(const std::string & parent_clock_id, uint16_t port_number)
  {
    PtpParentUpdate u;
    u.mutable_clock_id()->mutable_frame_id()->set_frame(sensor_id_);
    u.mutable_parent()->mutable_clock_id()->mutable_ptp_clock_id()->set_id(parent_clock_id);
    u.mutable_parent()->set_port_number(port_number);
    send_proto(u);
  }

private:
  static std::string get_hostname()
  {
    std::array<char, HOST_NAME_MAX + 1> hostname_raw;  // NOLINT
    auto result = gethostname(hostname_raw.data(), hostname_raw.size());
    if (result == -1) throw std::runtime_error(util::errno_to_string(errno));
    hostname_raw.at(hostname_raw.size() - 1) = '\0';
    return std::string{hostname_raw.data()};
  }

  nebula::util::expected<std::monostate, std::string> send_proto(
    const google::protobuf::Message & msg)
  {
    bool success = msg.SerializeToString(&serialization_buffer_);
    if (!success) {
      return std::string("Failed to serialize protobuf message");
    }

    // Check if there's a pending request and it's not ready
    if (
      last_request_ &&
      last_request_->wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      return std::string("Previous request still in flight");
    }

    auto future = http_client_.async_post(
      "/update_graph", serialization_buffer_,
      drivers::connections::HttpClient::content_type_cotest_stream);

    last_request_ = std::optional(std::move(future));
    return std::monostate{};  // Return monostate to indicate success
  }

  drivers::connections::HttpClient http_client_;

  std::string hostname_;
  std::string sensor_id_;

  std::string serialization_buffer_;
  std::optional<std::future<drivers::connections::HttpClient::HttpResponse>> last_request_;
};

}  // namespace nebula::ros
