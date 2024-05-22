#pragma once

#include "nebula_ros/common/parameter_descriptors.hpp"

#include <nebula_common/hesai/hesai_common.hpp>
#include <nebula_hw_interfaces/nebula_hw_interfaces_hesai/hesai_hw_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace nebula
{
namespace ros
{
class HesaiHwInterfaceWrapper
{
public:
  HesaiHwInterfaceWrapper(
    rclcpp::Node * const parent_node,
    std::shared_ptr<const nebula::drivers::HesaiSensorConfiguration> & config);

  void OnConfigChange(
    const std::shared_ptr<const nebula::drivers::HesaiSensorConfiguration> & new_config);

  nebula::Status Status();

  std::shared_ptr<drivers::HesaiHwInterface> HwInterface() const;

private:
  std::shared_ptr<drivers::HesaiHwInterface> hw_interface_;
  rclcpp::Logger logger_;
  nebula::Status status_;
  bool setup_sensor_;
};
}  // namespace ros
}  // namespace nebula
