#ifndef HESAI_STATUS_HPP
#define HESAI_STATUS_HPP

#include "nebula_common/nebula_status.hpp"

#include <ostream>
#include <string>

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
