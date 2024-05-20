#pragma once

#include "nebula_ros/common/parameter_descriptors.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <nebula_common/seyond/seyond_common.hpp>

#include <nebula_hw_interfaces/nebula_hw_interfaces_seyond/seyond_hw_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/property_tree/ptree.hpp>

#include <array>
#include <memory>
#include <optional>

namespace nebula
{
namespace ros
{
using SeyondHwInterface = nebula::drivers::SeyondHwInterface;
using SeyondSensorConfiguration = nebula::drivers::SeyondSensorConfiguration;

class SeyondHwMonitorWrapper
{
public:
  SeyondHwMonitorWrapper(rclcpp::Node* const parent_node,
                        const std::shared_ptr<SeyondHwInterface>& hw_interface,
                        std::shared_ptr<const SeyondSensorConfiguration>& config);

  void OnConfigChange(const std::shared_ptr<const SeyondSensorConfiguration> & /* new_config */) {}

  nebula::Status Status();

private:
  void InitializeDiagnostics();

  /// @brief Parse the stored diagnostic info and add it to the diagnostic status
  void ParseAndAddDiagnosticInfo(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);

  /// @brief Check if stored diagnostic info is up-to-date and force diagnostic updater update
  void TriggerDiagnosticsUpdate();

  /// @brief Get and store diagnostic info from sensor
  void FetchDiagnosticInfo();

  rclcpp::Node* const parent_node_;
  rclcpp::Logger logger_;
  nebula::Status status_;
  uint16_t diag_span_;
  diagnostic_updater::Updater diagnostics_updater_;

  const std::shared_ptr<SeyondHwInterface> hw_interface_;

  rclcpp::TimerBase::SharedPtr diagnostics_update_timer_{};
  rclcpp::TimerBase::SharedPtr fetch_diagnostics_timer_{};

  std::optional<boost::property_tree::ptree> current_diag_info_{};
  std::optional<rclcpp::Time> current_diag_info_time_{};
  std::mutex mtx_current_diag_info_;
};
}  // namespace ros
}  // namespace nebula