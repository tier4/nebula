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

rcl_interfaces::msg::ParameterDescriptor::_floating_point_range_type float_range(double start, double stop, double step)
{
  return { rcl_interfaces::msg::FloatingPointRange().set__from_value(start).set__to_value(stop).set__step(step) };
}

rcl_interfaces::msg::ParameterDescriptor::_integer_range_type int_range(int start, int stop, int step)
{
  return { rcl_interfaces::msg::IntegerRange().set__from_value(start).set__to_value(stop).set__step(step) };
}

double declare_fp_parameter(const std::string& name, rcl_interfaces::msg::ParameterDescriptor parameter_descriptor, rclcpp::Node* node)
{
  parameter_descriptor.dynamic_typing = true;
  parameter_descriptor.read_only = false;  // Can't set param if read_only, force to false
  auto param = node->declare_parameter(name, rclcpp::ParameterValue{}, parameter_descriptor);
  double value;

  switch (param.get_type())
  {
    case rclcpp::PARAMETER_INTEGER:
      value = static_cast<double>(param.get<int>());
      RCLCPP_WARN_STREAM(
        node->get_logger(), "Parameter " << name << " was set as an integer, converting to double");
      break;
    case rclcpp::PARAMETER_DOUBLE:
      value = param.get<double>();
      break;
    default:
      throw std::runtime_error(
              (std::stringstream() << "Invalid type (" << param.get_type() <<
          ") for floating point parameter " << name).str());
  }
  node->set_parameter(rclcpp::Parameter(name, value));
  return value;
}

}  // namespace ros
}  // namespace nebula