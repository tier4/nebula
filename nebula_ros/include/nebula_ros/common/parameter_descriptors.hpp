#pragma once

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rclcpp/rclcpp.hpp>

namespace nebula
{
namespace ros
{

rcl_interfaces::msg::ParameterDescriptor param_read_write();

rcl_interfaces::msg::ParameterDescriptor param_read_only();

rcl_interfaces::msg::ParameterDescriptor::_floating_point_range_type float_range(double start, double stop,
                                                                                 double step);

rcl_interfaces::msg::ParameterDescriptor::_integer_range_type int_range(int start, int stop, int step);

double declare_fp_parameter(const std::string& name,
                            rcl_interfaces::msg::ParameterDescriptor parameter_descriptor,
                            rclcpp::Node* node);

/// @brief Get a parameter's value from a list of parameters, if that parameter is in the list.
/// @tparam T The parameter's expected value type
/// @param p A vector of parameters
/// @param name Target parameter name
/// @param value (out) Parameter value. Set if parameter is found, left untouched otherwise.
/// @return Whether the target name existed
template <typename T>
bool get_param(const std::vector<rclcpp::Parameter>& p, const std::string& name, T& value)
{
  auto it = std::find_if(p.cbegin(), p.cend(),
                         [&name](const rclcpp::Parameter & parameter) { return parameter.get_name() == name; });
  if (it != p.cend())
  {
    value = it->template get_value<T>();
    return true;
  }
  return false;
}

}  // namespace ros
}  // namespace nebula