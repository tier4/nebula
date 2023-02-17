#ifndef HESAI_STATUS_HPP
#define HESAI_STATUS_HPP

#include <ostream>
#include <string>

#include "common/nebula_status.hpp"

namespace nebula
{
// from https://marycore.jp/prog/cpp/extends-enum/

/// @brief Status definition for Hesai
struct HesaiStatus : Status
{
  using Status::Status;

private:
  int _type_num;

public:
  enum HesaiType {
    INVALID_RPM_ERROR = Type_end_of_Status + 1,
    INVALID_FOV_ERROR,
    INVALID_RETURN_MODE_ERROR,
    Type_end_of_Status = INVALID_RPM_ERROR
  } _hesai_type;
  HesaiStatus() : _type_num(static_cast<int>(Status::OK)) { _type = static_cast<Type>(type()); }
  HesaiStatus(Type v) : _type_num(static_cast<int>(v)) { _type = v; }
  HesaiStatus(HesaiType v) : _type_num(static_cast<int>(v)), _hesai_type(v)
  {
    _type = Type::Type_end_of_Status;
  }
  HesaiStatus(int type) : _type_num(type) {}
  int type() const { return _type_num; }
  friend bool operator==(const HesaiStatus & L, const HesaiStatus & R)
  {
    return L.type() == R.type();
  }
  friend bool operator!=(const HesaiStatus & L, const HesaiStatus & R)
  {
    return L.type() != R.type();
  }

  /// @brief Convert Status enum to string (Overloading the << operator)
  /// @param os
  /// @param arg
  /// @return stream
  friend std::ostream & operator<<(std::ostream & os, nebula::HesaiStatus const & arg)
  {
    switch (arg.type()) {
      /*
      // in nebula_status.hpp
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
      */
      // Velodyne
      case HesaiStatus::INVALID_RPM_ERROR:
        os << "Invalid rotation speed value(range from 300 to 1200, in increments of 60)";
        break;
      case HesaiStatus::INVALID_FOV_ERROR:
        os << "Invalid fov value(0 to 359)";
        break;
      case HesaiStatus::INVALID_RETURN_MODE_ERROR:
        os << "Invalid return mode(only SINGLE_STRONGEST, SINGLE_LAST, DUAL_ONLY)";
        break;
      default:
        os << Status(arg._type);
    }
    return os;
  }
};
}  // namespace nebula
#endif  // HESAI_STATUS_HPP
