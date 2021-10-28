#ifndef NEBULA_STATUS_HPP
#define NEBULA_STATUS_HPP

namespace nebula
{
enum class Status {
  OK = 0,
  UDP_CONNECTION_ERROR,
  SENSOR_CONFIG_ERROR,
  ERROR_1
};

std::string NebulaStatusToString(const Status& nebula_status)
{
  switch (nebula_status) {
    case Status::OK:
      return "OK";
    case Status::UDP_CONNECTION_ERROR:
      return "Udp Connection Error";
    case Status::SENSOR_CONFIG_ERROR:
      return "Could not set SensorConfiguration";
    case Status::ERROR_1:
    default:
      return "RUNTIME STOPPED";
  }
}

}  // namespace nebula
#endif  //NEBULA_STATUS_HPP
