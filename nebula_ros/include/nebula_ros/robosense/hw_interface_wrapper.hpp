#pragma once

#include <nebula_common/nebula_common.hpp>
#include <nebula_common/robosense/robosense_common.hpp>
#include <nebula_hw_interfaces/nebula_hw_interfaces_robosense/robosense_hw_interface.hpp>

#include <rclcpp/rclcpp.hpp>

namespace nebula
{
namespace ros
{
class RobosenseHwInterfaceWrapper
{
public:
  explicit RobosenseHwInterfaceWrapper(rclcpp::Node* const parent_node,
                                       std::shared_ptr<const nebula::drivers::RobosenseSensorConfiguration>& config);

  void OnConfigChange(const std::shared_ptr<const nebula::drivers::RobosenseSensorConfiguration> & new_config);

  nebula::Status Status();

  std::shared_ptr<drivers::RobosenseHwInterface> HwInterface() const;

private:
  std::shared_ptr<drivers::RobosenseHwInterface> hw_interface_;
  rclcpp::Logger logger_;
  nebula::Status status_;
};

}  // namespace ros
}  // namespace nebula
