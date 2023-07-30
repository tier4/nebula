#ifndef NEBULA_STATUS_HPP
#define NEBULA_STATUS_HPP

#include <ostream>
#include <string>

namespace nebula
{
// from https://marycore.jp/prog/cpp/extends-enum/

/// @brief Status definition
struct Status
{
  enum Type {
    OK = 0,
    UDP_CONNECTION_ERROR,
    SENSOR_CONFIG_ERROR,
    INVALID_SENSOR_MODEL,
    INVALID_ECHO_MODE,
    NOT_IMPLEMENTED,
    NOT_INITIALIZED,
    INVALID_CALIBRATION_FILE,
    CANNOT_SAVE_FILE,
    HTTP_CONNECTION_ERROR,
    WAITING_FOR_SENSOR_RESPONSE,
    ERROR_1,
    Type_end_of_Status = ERROR_1
  } _type;
  Status() : _type(Type::OK) {}
  Status(Type v) : _type(v) {}
  Status(int type) : _type(static_cast<Type>(type)) {}
  Type type() const { return _type; }
  friend bool operator==(const Status & L, const Status & R) { return L.type() == R.type(); }
  friend bool operator!=(const Status & L, const Status & R) { return L.type() != R.type(); }

  /// @brief Convert Status enum to string (Overloading the << operator)
  /// @param os
  /// @param arg
  /// @return stream
  friend std::ostream & operator<<(std::ostream & os, nebula::Status const & arg)
  {
    switch (arg.type()) {
      case Status::OK:
        os << "OK";
        break;
      case Status::UDP_CONNECTION_ERROR:
        os << "Udp Connection Error";
        break;
      case Status::SENSOR_CONFIG_ERROR:
        os << "Could not set SensorConfiguration";
        break;
      case Status::INVALID_SENSOR_MODEL:
        os << "Invalid sensor model provided";
        break;
      case Status::INVALID_ECHO_MODE:
        os << "Invalid echo model provided";
        break;
      case Status::NOT_IMPLEMENTED:
        os << "Not Implemented";
        break;
      case Status::NOT_INITIALIZED:
        os << "Not Initialized";
        break;
      case Status::INVALID_CALIBRATION_FILE:
        os << "Invalid Calibration File";
        break;
      case Status::CANNOT_SAVE_FILE:
        os << "Cannot Save File";
        break;
      case Status::HTTP_CONNECTION_ERROR:
        os << "Http Connection Error";
        break;
      case Status::WAITING_FOR_SENSOR_RESPONSE:
        os << "Waiting for Sensor Response";
        break;
      case Status::ERROR_1:
      default:
        os << "Generic Error";
    }
    return os;
  }
};

}  // namespace nebula
#endif  // NEBULA_STATUS_HPP
