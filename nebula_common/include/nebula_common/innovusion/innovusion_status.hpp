#ifndef Innovusion_STATUS_HPP
#define Innovusion_STATUS_HPP

#include "nebula_common/nebula_status.hpp"

#include <ostream>
#include <string>

namespace nebula
{
// from https://marycore.jp/prog/cpp/extends-enum/

/// @brief Status definition for Innovusion
struct InnovusionStatus : Status
{
  using Status::Status;

private:
  int _type_num;

public:
  enum InnovusionType {
    INVALID_RPM_ERROR = Type_end_of_Status + 1,
    INVALID_FOV_ERROR,
    INVALID_RETURN_MODE_ERROR,
    Type_end_of_Status = INVALID_RPM_ERROR
  } _Innovusion_type;
  InnovusionStatus() : _type_num(static_cast<int>(Status::OK)) { _type = static_cast<Type>(type()); }
  InnovusionStatus(Type v) : _type_num(static_cast<int>(v)) { _type = v; }
  InnovusionStatus(InnovusionType v) : _type_num(static_cast<int>(v)), _Innovusion_type(v)
  {
    _type = Type::Type_end_of_Status;
  }
  InnovusionStatus(int type) : _type_num(type) {}
  int type() const { return _type_num; }
  friend bool operator==(const InnovusionStatus & L, const InnovusionStatus & R)
  {
    return L.type() == R.type();
  }
  friend bool operator!=(const InnovusionStatus & L, const InnovusionStatus & R)
  {
    return L.type() != R.type();
  }

  /// @brief Convert Status enum to string (Overloading the << operator)
  /// @param os
  /// @param arg
  /// @return stream
  friend std::ostream & operator<<(std::ostream & os, nebula::InnovusionStatus const & arg)
  {
    switch (arg.type()) {
      // Velodyne
      case InnovusionStatus::INVALID_RPM_ERROR:
        os << "Invalid rotation speed value(range from 300 to 1200, in increments of 60)";
        break;
      case InnovusionStatus::INVALID_FOV_ERROR:
        os << "Invalid fov value(0 to 359)";
        break;
      case InnovusionStatus::INVALID_RETURN_MODE_ERROR:
        os << "Invalid return mode(only SINGLE_STRONGEST, SINGLE_LAST, DUAL_ONLY)";
        break;
      default:
        os << Status(arg._type);
    }
    return os;
  }
};
}  // namespace nebula
#endif  // Innovusion_STATUS_HPP
