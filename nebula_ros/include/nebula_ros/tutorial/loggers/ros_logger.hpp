#pragma once

#include <nebula_hw_interfaces/nebula_hw_interfaces_tutorial/loggers/logger.hpp>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <string>

namespace nebula
{
namespace ros
{
namespace loggers
{

class RosLogger : public nebula::drivers::loggers::Logger
{
public:
  RosLogger(rclcpp::Logger parent, const std::string & name) : logger_(parent.get_child(name)) {}

  RosLogger(const std::string & name) : logger_(rclcpp::get_logger(name)) {}

  void debug(const std::string & message) override { RCLCPP_DEBUG(logger_, message.c_str()); }

  void info(const std::string & message) override { RCLCPP_INFO(logger_, message.c_str()); }

  void warn(const std::string & message) override { RCLCPP_WARN(logger_, message.c_str()); }

  void error(const std::string & message) override { RCLCPP_ERROR(logger_, message.c_str()); }

  std::shared_ptr<Logger> child(const std::string & name) override
  {
    return std::static_pointer_cast<Logger>(std::make_shared<RosLogger>(logger_, name));
  }

private:
  rclcpp::Logger logger_;
};

}  // namespace loggers
}  // namespace ros
}  // namespace nebula
