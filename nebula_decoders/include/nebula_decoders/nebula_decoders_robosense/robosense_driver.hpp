#ifndef NEBULA_ROBOSENSE_DRIVER_H
#define NEBULA_ROBOSENSE_DRIVER_H

#include "nebula_common/nebula_common.hpp"
#include "nebula_common/nebula_status.hpp"
#include "nebula_common/robosense/robosense_common.hpp"
#include "nebula_decoders/nebula_decoders_common/nebula_driver_base.hpp"
#include "nebula_decoders/nebula_decoders_robosense/decoders/robosense_decoder.hpp"

namespace nebula
{
namespace drivers
{
/// @brief Robosense driver
class RobosenseDriver : NebulaDriverBase
{
private:
  /// @brief Current driver status
  Status driver_status_;

  /// @brief Decoder according to the model
  std::shared_ptr<RobosenseScanDecoder> scan_decoder_;

public:
  RobosenseDriver() = delete;

  /// @brief Constructor
  /// @param sensor_configuration SensorConfiguration for this driver
  /// @param calibration_configuration CalibrationConfiguration for this driver
  /// @param correction_configuration CorrectionConfiguration for this driver (for AT)
  explicit RobosenseDriver(
    const std::shared_ptr<drivers::RobosenseSensorConfiguration> & sensor_configuration,
    const std::shared_ptr<drivers::RobosenseCalibrationConfiguration> & calibration_configuration);
};

}  // namespace drivers
}  // namespace nebula

#endif  // NEBULA_ROBOSENSE_DRIVER_H
