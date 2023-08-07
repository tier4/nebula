#ifndef NEBULA_HW_INTERFACE_BASE_H
#define NEBULA_HW_INTERFACE_BASE_H

#include "nebula_common/nebula_common.hpp"
#include "nebula_common/nebula_status.hpp"
#include "boost_udp_driver/udp_driver.hpp"

#include <stdexcept>
#include <string>
#include <vector>

namespace nebula
{
namespace drivers
{
/// @brief Base class for hardware interface of each LiDAR
class NebulaHwInterfaceBase
{
protected:
  /**
   * Callback function to receive the Cloud Packet data from the UDP Driver
   * @param buffer Buffer containing the data received from the UDP socket
   * @return Status::OK if no error occurred.
   */
  virtual void ReceiveCloudPacketCallback(const std::vector<uint8_t> & buffer) = 0;
  //  virtual Status RegisterScanCallback(
  //    std::function<void(std::unique_ptr<std::vector<std::vector<uint8_t>>>)> scan_callback) = 0;

public:
  NebulaHwInterfaceBase(NebulaHwInterfaceBase && c) = delete;
  NebulaHwInterfaceBase & operator=(NebulaHwInterfaceBase && c) = delete;
  NebulaHwInterfaceBase(const NebulaHwInterfaceBase & c) = delete;
  NebulaHwInterfaceBase & operator=(const NebulaHwInterfaceBase & c) = delete;

  NebulaHwInterfaceBase() = default;

  /// @brief Virtual function for starting the interface that handles UDP streams
  /// @return Resulting status
  virtual Status CloudInterfaceStart() = 0;

  /// @brief Virtual function for stopping the interface that handles UDP streams
  /// @return Resulting status
  virtual Status CloudInterfaceStop() = 0;
  // You may want to also implement GpsInterfaceStart() and ReceiveGpsCallback, but that is sensor
  // specific.

  /// @brief Virtual function for setting sensor configuration
  /// @param sensor_configuration SensorConfiguration for this interface
  /// @return Resulting status
  virtual Status SetSensorConfiguration(
    std::shared_ptr<SensorConfigurationBase> sensor_configuration) = 0;

  /// @brief Virtual function for printing sensor configuration
  /// @param sensor_configuration SensorConfiguration for the checking
  /// @return Resulting status
  virtual Status GetSensorConfiguration(SensorConfigurationBase & sensor_configuration) = 0;

  /// @brief Virtual function for printing calibration configuration
  /// @param calibration_configuration CalibrationConfiguration for the checking
  /// @return Resulting status
  virtual Status GetCalibrationConfiguration(
    CalibrationConfigurationBase & calibration_configuration) = 0;
};

}  // namespace drivers
}  // namespace nebula

#endif  // NEBULA_HW_INTERFACE_BASE_H
