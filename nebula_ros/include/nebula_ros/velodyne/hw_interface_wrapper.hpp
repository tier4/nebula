#pragma once

#include "nebula_ros/common/parameter_descriptors.hpp"

#include <nebula_common/velodyne/velodyne_common.hpp>

#include <nebula_hw_interfaces/nebula_hw_interfaces_velodyne/velodyne_hw_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace nebula
{
namespace ros
{
class VelodyneHwInterfaceWrapper
{
public:
  VelodyneHwInterfaceWrapper(rclcpp::Node* const parent_node,
                          std::shared_ptr<const nebula::drivers::VelodyneSensorConfiguration>& config);

  void OnConfigChange(const std::shared_ptr<const nebula::drivers::VelodyneSensorConfiguration> & new_config);

  nebula::Status Status();

  std::shared_ptr<drivers::VelodyneHwInterface> HwInterface() const;

private:
  std::shared_ptr<drivers::VelodyneHwInterface> hw_interface_;
  rclcpp::Logger logger_;
  nebula::Status status_;
  bool setup_sensor_;
};
}  // namespace ros
}  // namespace nebula