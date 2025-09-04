// Copyright 2024 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef VELODYNE_STATUS_HPP
#define VELODYNE_STATUS_HPP

#include "nebula_common/nebula_status.hpp"

#include <ostream>

namespace nebula
{
// from https://marycore.jp/prog/cpp/extends-enum/

/// @brief Status definition for Velodyne
struct VelodyneStatus : Status
{
  using Status::Status;

private:
  int _type_num;

public:
  enum VelodyneType {
    INVALID_RPM_ERROR = Type_end_of_Status + 1,
    INVALID_FOV_ERROR,
    INVALID_RETURN_MODE_ERROR,
    Type_end_of_Status = INVALID_RPM_ERROR
  } _velo_type;
  VelodyneStatus() : _type_num(static_cast<int>(Status::OK)) { _type = static_cast<Type>(type()); }
  /* NOLINT(runtime/explicit) */ VelodyneStatus(Type v) : _type_num(static_cast<int>(v))
  {
    _type = v;
  }
  /* NOLINT(runtime/explicit) */ VelodyneStatus(VelodyneType v)
  : _type_num(static_cast<int>(v)), _velo_type(v)
  {
    _type = Type::Type_end_of_Status;
  }
  explicit VelodyneStatus(int type) : _type_num(type) {}
  int type() const { return _type_num; }
  friend bool operator==(const VelodyneStatus & L, const VelodyneStatus & R)
  {
    return L.type() == R.type();
  }
  friend bool operator!=(const VelodyneStatus & L, const VelodyneStatus & R)
  {
    return L.type() != R.type();
  }
  //  friend VelodyneStatus operator VelodyneStatus(const Status & s) {return VelodyneStatus(s);}
  // implicit
  //  operator Status() const {std::cout << "implicit" << std::endl; return Status(type());}
  //  operator const Status() {std::cout << "implicit" << std::endl; return Status(type());}
  //  explicit operator Status() const {std::cout << "explicit" << std::endl; return
  //  Status(type());} Status Status::operator=(const VelodyneStatus&) {std::cout << "implicit = "
  //  << std::endl; return Status(type());} Status operator=(const VelodyneStatus) {std::cout <<
  //  "implicit = " << std::endl; return Status(type());} Status& operator=(const VelodyneStatus&)
  //  {std::cout << "implicit = " << std::endl; Status rt = Status(type()); return rt;}
  //  Status::Status(const VelodyneStatus& vs) {_type = v;}

  /// @brief Convert Status enum to string (Overloading the << operator)
  /// @param os
  /// @param arg
  /// @return stream
  friend std::ostream & operator<<(std::ostream & os, nebula::VelodyneStatus const & arg)
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
      case VelodyneStatus::INVALID_RPM_ERROR:
        os << "Invalid rotation speed value(range from 300 to 1200, in increments of 60)";
        break;
      case VelodyneStatus::INVALID_FOV_ERROR:
        os << "Invalid fov value(0 to 359)";
        break;
      case VelodyneStatus::INVALID_RETURN_MODE_ERROR:
        os << "Invalid return mode(only SINGLE_STRONGEST, SINGLE_LAST, DUAL_ONLY)";
        break;
      default:
        os << Status(arg._type);
    }
    return os;
  }
};
}  // namespace nebula
#endif  // VELODYNE_STATUS_HPP
