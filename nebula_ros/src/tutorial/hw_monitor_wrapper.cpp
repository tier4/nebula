#include "nebula_ros/tutorial/hw_monitor_wrapper.hpp"

namespace nebula
{
namespace ros
{
TutorialHwMonitorWrapper::TutorialHwMonitorWrapper(rclcpp::Node* const parent_node,
                                             const std::shared_ptr<TutorialHwInterface>& hw_interface,
                                             std::shared_ptr<const TutorialSensorConfiguration>& /* config */)
  : parent_node_(parent_node)
  , logger_(parent_node->get_logger().get_child("HwMonitor"))
  , status_(Status::OK)
  , diagnostics_updater_(parent_node)
  , hw_interface_(hw_interface)
{
  diag_span_ = parent_node->declare_parameter<uint16_t>("diag_span", 1000, param_read_only());

  InitializeDiagnostics();
}

void TutorialHwMonitorWrapper::InitializeDiagnostics()
{
  std::string hardware_id = ""; // get hardware ID (model:serial_number) from sensor here
  diagnostics_updater_.setHardwareID(hardware_id);
  RCLCPP_INFO_STREAM(logger_, "hardware_id: " + hardware_id);

  diagnostics_updater_.add("diagnostic_info", this, &TutorialHwMonitorWrapper::ParseAndAddDiagnosticInfo);

  // Timer to fetch new info from sensor. Choose a sensible period, e.g. once per sec.
  fetch_diagnostics_timer_ =
      parent_node_->create_wall_timer(std::chrono::milliseconds(1000),
      std::bind(&TutorialHwMonitorWrapper::FetchDiagnosticInfo, this));

  // Timer to trigger diagnostic_updater updates and to check if fetched data is stale
  diagnostics_update_timer_ =
    parent_node_->create_wall_timer(std::chrono::milliseconds(diag_span_),
      std::bind(&TutorialHwMonitorWrapper::TriggerDiagnosticsUpdate, this));
}

void TutorialHwMonitorWrapper::ParseAndAddDiagnosticInfo(diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;

  diagnostics.add("voltage", current_diag_info_->get<std::string>("status.voltage"));
  diagnostics.add("error", current_diag_info_->get<std::string>("status.error"));
  // etc.

  diagnostics.summary(level, "OK");
}

void TutorialHwMonitorWrapper::TriggerDiagnosticsUpdate()
{
  RCLCPP_DEBUG_STREAM(logger_, "OnUpdateTimer");
  auto now = parent_node_->get_clock()->now();
  auto dif = (now - *current_diag_info_time_).seconds();

  RCLCPP_DEBUG_STREAM(logger_, "dif(status): " << dif);

  if (diag_span_ * 2.0 < dif * 1000)
  {
    RCLCPP_DEBUG_STREAM(logger_, "STALE");
  }
  else
  {
    RCLCPP_DEBUG_STREAM(logger_, "OK");
  }

  diagnostics_updater_.force_update();
}

void TutorialHwMonitorWrapper::FetchDiagnosticInfo()
{
  current_diag_info_ = {}; // fetch from sensor here
  current_diag_info_time_ = parent_node_->now();
}

Status TutorialHwMonitorWrapper::Status()
{
  return Status::NOT_IMPLEMENTED;  // TODO
}
}  // namespace ros
}  // namespace nebula