// Copyright 2024 TIER IV, Inc.

#include "nebula_ros/aeva/hw_monitor_wrapper.hpp"

#include "nebula_ros/common/parameter_descriptors.hpp"

#include <nebula_common/aeva/packet_types.hpp>
#include <nebula_common/util/parsing.hpp>
#include <nlohmann/json.hpp>

#include <diagnostic_msgs/msg/detail/diagnostic_array__builder.hpp>
#include <diagnostic_msgs/msg/detail/diagnostic_array__struct.hpp>
#include <diagnostic_msgs/msg/detail/diagnostic_status__struct.hpp>
#include <diagnostic_msgs/msg/detail/key_value__struct.hpp>

#include <algorithm>
#include <chrono>
#include <iterator>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace nebula::ros
{

using diagnostic_msgs::msg::DiagnosticStatus;
using drivers::aeva::HealthCode;
using nebula::util::get_if_exists;

AevaHwMonitorWrapper::AevaHwMonitorWrapper(
  rclcpp::Node * const parent_node, std::shared_ptr<const drivers::aeva::Aeries2Config> config)
: logger_(parent_node->get_logger().get_child("HwMonitor")),
  parent_node_(parent_node),
  config_(std::move(config))
{
  diag_span_ = parent_node->declare_parameter<uint16_t>("diag_span", 500, param_read_only());

  diagnostics_pub_ =
    parent_node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 10);

  diagnostics_pub_timer_ = parent_node->create_wall_timer(
    std::chrono::milliseconds(diag_span_), [&]() { publishDiagnostics(); });
}

void AevaHwMonitorWrapper::onTelemetryFragment(const json & diff)
{
  std::scoped_lock lock(mtx_telemetry_, mtx_hardware_id_);

  // ////////////////////////////////////////
  // Update aggregated telemetry tree
  // ////////////////////////////////////////

  auto now = parent_node_->now();
  bool any_changed = false;
  for (const auto & node : diff.items()) {
    bool new_node = telemetry_.entries.count(node.key()) == 0;
    bool node_changed = !new_node && telemetry_.entries.at(node.key()).values != diff[node.key()];

    if (new_node) {
      telemetry_.entries[node.key()] = {json::object(), now};
    }

    if (new_node || node_changed) {
      any_changed = true;
      auto & entry = telemetry_.entries[node.key()];
      entry.values.update(diff[node.key()]);
      entry.last_update = now;
    }
  }

  if (!any_changed) {
    return;
  }

  // ////////////////////////////////////////
  // Initialize hardware ID if unset
  // ////////////////////////////////////////

  if (!hardware_id_) {
    auto serial = get_if_exists<std::string>(diff, {"sysinfo_main", "serial_number"});
    auto model = get_if_exists<std::string>(diff, {"sysinfo_main", "platform"});

    if (!serial || !model) return;

    hardware_id_.emplace(*model + ":" + *serial);
  }
}

void AevaHwMonitorWrapper::onHealthCodes(std::vector<HealthCode> health_codes)
{
  std::scoped_lock lock(mtx_health_, mtx_hardware_id_);

  if (!hardware_id_) {
    return;
  }

  auto now = parent_node_->now();
  health_ = {std::move(health_codes), now};
}

void AevaHwMonitorWrapper::publishDiagnostics()
{
  std::scoped_lock lock(mtx_health_, mtx_telemetry_, mtx_hardware_id_);

  if (!hardware_id_) {
    return;
  }

  auto now = parent_node_->now();

  diagnostic_msgs::msg::DiagnosticArray diagnostic_array_msg;
  diagnostic_array_msg.header.stamp.sec = static_cast<int>(now.seconds());
  diagnostic_array_msg.header.stamp.nanosec = now.nanoseconds() % 1'000'000'000;
  diagnostic_array_msg.header.frame_id = config_->frame_id;

  auto diag_entry = [](const std::string & key, const std::string & value) {
    diagnostic_msgs::msg::KeyValue entry{};
    entry.key = key;
    entry.value = value;
    return entry;
  };

  // ////////////////////////////////////////
  // Add telemetry to diagnostics
  // ////////////////////////////////////////

  for (const auto & entry : telemetry_.entries) {
    auto & status = diagnostic_array_msg.status.emplace_back();
    status.level = DiagnosticStatus::OK;
    status.hardware_id = *hardware_id_;
    status.name = "aeva.telemetry." + entry.first;

    for (const auto & item : entry.second.values.items()) {
      auto value = item.value();
      std::string stringified =
        value.is_string() ? value.template get<std::string>() : value.dump();
      status.values.emplace_back(diag_entry(item.key(), stringified));
    }
  }

  // ////////////////////////////////////////
  // Add health codes to diagnostics
  // ////////////////////////////////////////

  auto & status = diagnostic_array_msg.status.emplace_back();
  status.level = DiagnosticStatus::OK;
  status.hardware_id = *hardware_id_;
  status.name = "aeva.health";

  if (health_) {
    json warning_codes = json::array();
    json error_codes = json::array();

    for (const auto & code : health_->codes) {
      if (code.is_error()) {
        error_codes.push_back(code.get());
      } else {
        warning_codes.push_back(code.get());
      }
    }

    if (!error_codes.empty()) {
      status.level = DiagnosticStatus::ERROR;
    } else if (!warning_codes.empty()) {
      status.level = DiagnosticStatus::WARN;
    }

    status.values.emplace_back(diag_entry("warning_codes", warning_codes.dump()));
    status.values.emplace_back(diag_entry("error_codes", error_codes.dump()));
  }

  diagnostics_pub_->publish(diagnostic_array_msg);
}

}  // namespace nebula::ros
