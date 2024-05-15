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
using TutorialHwInterface = nebula::drivers::HesaiHwInterface;
using TutorialSensorConfiguration = nebula::drivers::HesaiSensorConfiguration;

class TutorialHwInterfaceWrapper
{

public:
  TutorialHwInterfaceWrapper(rclcpp::Node* const parent_node,
                          std::shared_ptr<const TutorialSensorConfiguration>& config);

  void OnConfigChange(const std::shared_ptr<const TutorialSensorConfiguration> & new_config);

  nebula::Status Status();

  std::shared_ptr<TutorialHwInterface> HwInterface() const;

private:
  std::shared_ptr<TutorialHwInterface> hw_interface_;
  rclcpp::Logger logger_;
  nebula::Status status_;
  bool setup_sensor_;
};
}  // namespace ros
}  // namespace nebula