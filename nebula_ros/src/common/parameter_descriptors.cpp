#include "nebula_ros/common/parameter_descriptors.hpp"

namespace nebula
{
namespace ros
{

rcl_interfaces::msg::ParameterDescriptor param_read_write()
{
  return rcl_interfaces::msg::ParameterDescriptor{};
};

rcl_interfaces::msg::ParameterDescriptor param_read_only()
{
  return rcl_interfaces::msg::ParameterDescriptor{}.set__read_only(true);
}

rcl_interfaces::msg::ParameterDescriptor::_floating_point_range_type float_range(
  double start, double stop, double step)
{
  return {
    rcl_interfaces::msg::FloatingPointRange().set__from_value(start).set__to_value(stop).set__step(
      step)};
}

rcl_interfaces::msg::ParameterDescriptor::_integer_range_type int_range(
  int start, int stop, int step)
{
  return {
    rcl_interfaces::msg::IntegerRange().set__from_value(start).set__to_value(stop).set__step(step)};
}

double declare_fp_parameter(
  rclcpp::Node * node, const std::string & name,
  rcl_interfaces::msg::ParameterDescriptor descriptor)
{
  rclcpp::ParameterValue raw_parameter;
  descriptor.dynamic_typing = true;

  raw_parameter = node->declare_parameter(name, raw_parameter, descriptor);

  switch (raw_parameter.get_type()) {
    case rclcpp::ParameterType::PARAMETER_DOUBLE: {
      return raw_parameter.get<double>();
    }
    case rclcpp::ParameterType::PARAMETER_INTEGER: {
      auto parameter = static_cast<double>(raw_parameter.get<int64_t>());
      node->undeclare_parameter(name);
      node->declare_parameter(name, parameter, descriptor, true);
      return parameter;
    }
    default:
      throw std::runtime_error("Non numeric value");
  }
}

}  // namespace ros
}  // namespace nebula
