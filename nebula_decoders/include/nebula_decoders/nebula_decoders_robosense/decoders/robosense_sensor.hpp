#pragma once

#include <type_traits>

namespace nebula
{
namespace drivers
{

/// @brief Base class for all sensor definitions
/// @tparam PacketT The packet type of the sensor
template <typename PacketT, AngleCorrectionType AngleCorrection = AngleCorrectionType::CALIBRATION>
class RobosenseSensor
{
private:
};

}  // namespace drivers
}  // namespace nebula