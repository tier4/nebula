#pragma once

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
  HesaiHwInterfaceWrapper(rclcpp::Node* const parent_node,
                          std::shared_ptr<nebula::drivers::HesaiSensorConfiguration>& config);

  nebula::Status InitializeHwInterface();

  nebula::Status Status();

  std::shared_ptr<drivers::HesaiHwInterface> HwInterface() const;

private:
  std::shared_ptr<drivers::HesaiHwInterface> hw_interface_;
  rclcpp::Logger logger_;
  nebula::Status status_;
};
}  // namespace ros
}  // namespace nebula