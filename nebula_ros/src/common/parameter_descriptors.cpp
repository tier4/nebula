// Copyright 2024 TIER IV, Inc.

#include "nebula_ros/common/parameter_descriptors.hpp"

namespace nebula::ros
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

}  // namespace nebula::ros
