#ifndef NEBULA_STATUS_HPP
#define NEBULA_STATUS_HPP

#include <string>
#include <ostream>

namespace nebula
{
enum class Status {
  OK = 0,
  UDP_CONNECTION_ERROR,
  SENSOR_CONFIG_ERROR,
  INVALID_SENSOR_MODEL,
  INVALID_ECHO_MODE,
  NOT_IMPLEMENTED,
  NOT_INITIALIZED,
  INVALID_CALIBRATION_FILE,
  CANNOT_SAVE_FILE,
  ERROR_1
};

inline std::ostream& operator<<(std::ostream& os, nebula::Status const& arg)
{
  switch (arg) {
    case Status::OK:
      os << "OK";
      break;
    case Status::UDP_CONNECTION_ERROR:
      os <<  "Udp Connection Error";
      break;
    case Status::SENSOR_CONFIG_ERROR:
      os <<  "Could not set SensorConfiguration";
      break;
    case Status::INVALID_SENSOR_MODEL:
      os <<  "Invalid sensor model provided";
      break;
    case Status::INVALID_ECHO_MODE:
      os <<  "Invalid echo model provided";
      break;
    case Status::NOT_IMPLEMENTED:
      os <<  "Not Implemented";
      break;
    case Status::NOT_INITIALIZED:
      os <<  "Not Initialized";
      break;
    case Status::INVALID_CALIBRATION_FILE:
      os <<  "Invalid Calibration File";
      break;
    case Status::CANNOT_SAVE_FILE:
      os <<  "Cannot Save File";
      break;
    case Status::ERROR_1:
    default:
      os <<  "Generic Error";
  }
  return os;
}

}  // namespace nebula
#endif  // NEBULA_STATUS_HPP
