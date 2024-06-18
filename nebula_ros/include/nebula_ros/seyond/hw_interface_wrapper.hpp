#pragma once

#include "nebula_ros/common/parameter_descriptors.hpp"

#include <nebula_common/seyond/seyond_common.hpp>
#include <nebula_hw_interfaces/nebula_hw_interfaces_seyond/seyond_hw_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace nebula
{
namespace ros
{
using SeyondHwInterface = nebula::drivers::SeyondHwInterface;
using SeyondSensorConfiguration = nebula::drivers::SeyondSensorConfiguration;

class SeyondHwInterfaceWrapper
{
public:
  SeyondHwInterfaceWrapper(
    rclcpp::Node * const parent_node, std::shared_ptr<const SeyondSensorConfiguration> & config);
  SeyondHwInterfaceWrapper() = default;

  void OnConfigChange(const std::shared_ptr<const SeyondSensorConfiguration> & new_config);

  nebula::Status Status();

  std::shared_ptr<SeyondHwInterface> HwInterface() const;

private:
  std::shared_ptr<SeyondHwInterface> hw_interface_{};
  rclcpp::Logger logger_;
  nebula::Status status_{};
  bool setup_sensor_{};
};
}  // namespace ros
}  // namespace nebula
