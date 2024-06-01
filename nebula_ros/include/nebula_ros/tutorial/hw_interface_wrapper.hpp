#pragma once

#include "nebula_ros/common/parameter_descriptors.hpp"
#include "nebula_ros/tutorial/loggers/ros_logger.hpp"

#include <nebula_common/tutorial/tutorial_common.hpp>
#include <nebula_hw_interfaces/nebula_hw_interfaces_tutorial/tutorial_hw_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace nebula
{
namespace ros
{

class TutorialHwInterfaceWrapper
{
public:
  TutorialHwInterfaceWrapper(
    rclcpp::Node * const parent_node,
    std::shared_ptr<const nebula::drivers::TutorialSensorConfiguration> & config);

  void OnConfigChange(
    const std::shared_ptr<const nebula::drivers::TutorialSensorConfiguration> & new_config);

  nebula::Status Status();

  std::shared_ptr<nebula::drivers::TutorialHwInterface> HwInterface() const;

private:
  std::shared_ptr<nebula::drivers::TutorialHwInterface> hw_interface_;
  rclcpp::Logger logger_;
  nebula::Status status_;
  bool setup_sensor_;
};
}  // namespace ros
}  // namespace nebula