#include "nebula_ros/tutorial/hw_monitor_wrapper.hpp"

#include <boost/iostreams/stream.hpp>

namespace nebula
{
namespace ros
{
TutorialHwMonitorWrapper::TutorialHwMonitorWrapper(rclcpp::Node* const parent_node,
                                             const std::shared_ptr<nebula::drivers::TutorialHwInterface>& hw_interface,
                                             std::shared_ptr<const nebula::drivers::TutorialSensorConfiguration>& /* config */)
  : parent_node_(parent_node)
  , logger_(parent_node->get_logger().get_child("HwMonitor"))
  , status_(Status::OK)
  , diagnostics_updater_(parent_node)
  , hw_interface_(hw_interface)
{
  diag_span_ = parent_node->declare_parameter<uint16_t>("diag_span", param_read_only());

  InitializeDiagnostics();
}

void TutorialHwMonitorWrapper::InitializeDiagnostics()
{
  // get hardware ID (model:serial_number) from sensor here using HW interface
  std::string hardware_id = "Tutorial:ABC-12345-DEF";
  diagnostics_updater_.setHardwareID(hardware_id);
  RCLCPP_INFO_STREAM(logger_, "hardware_id: " + hardware_id);

  // This handler gets called on `diagnostics_updater_.force_update()`
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
  // ////////////////////////////////////////
  // Add diagnostics values to ROS object
  // ////////////////////////////////////////

  if (current_diag_info_.empty()) {
    return;
  }

  diagnostics.add("voltage", current_diag_info_.at("voltage"));

  auto error =  current_diag_info_.at("error");
  diagnostics.add("error", error);

  // ////////////////////////////////////////
  // Check error conditions and set status
  // ////////////////////////////////////////

  if (error != "No error") {
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, error);
    return;
  }

  diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "OK");
}

void TutorialHwMonitorWrapper::TriggerDiagnosticsUpdate()
{
  // ////////////////////////////////////////
  // Watch for diagnostics timeouts
  // ////////////////////////////////////////

  auto now = parent_node_->get_clock()->now();
  auto dif = (now - *current_diag_info_time_).seconds();

  if (diag_span_ * 2.0 < dif * 1000)
  {
    RCLCPP_WARN_STREAM(logger_, "Diagnostics STALE for " << dif << " s");
  }
  else
  {
    RCLCPP_DEBUG_STREAM(logger_, "Diagnostics OK (up to date)");
  }

  // ////////////////////////////////////////
  // Publish any new diagnostics
  // ////////////////////////////////////////

  diagnostics_updater_.force_update();
}

void TutorialHwMonitorWrapper::FetchDiagnosticInfo()
{
  // This is where you would fetch the newest diagnostics from the sensor using the hardware
  // interface. The below code is just a dummy.

  current_diag_info_.clear();
  current_diag_info_.insert({"voltage", "14.00"});
  current_diag_info_.insert({"error", "No error"});

  current_diag_info_time_ = parent_node_->now();
}

Status TutorialHwMonitorWrapper::Status()
{
  return Status::OK;
}
}  // namespace ros
}  // namespace nebula