#ifndef NEBULA_STATUS_HPP
#define NEBULA_STATUS_HPP

#include <string>

namespace nebula
{
enum class Status {
  OK = 0,
  UDP_CONNECTION_ERROR,
  SENSOR_CONFIG_ERROR,
  INVALID_SENSOR_MODEL,
  ERROR_1
};

std::string NebulaStatusToString(const Status & nebula_status)
{
  switch (nebula_status) {
    case Status::OK:
      return "OK";
    case Status::UDP_CONNECTION_ERROR:
      return "Udp Connection Error";
    case Status::SENSOR_CONFIG_ERROR:
      return "Could not set SensorConfiguration";
      break;
    case Status::INVALID_SENSOR_MODEL:
      return "Invalid sensor model provided";
    case Status::ERROR_1:
    default:
      return "RUNTIME STOPPED";
  }
}

}  // namespace nebula
#endif  // NEBULA_STATUS_HPP
