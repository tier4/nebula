#ifndef LIVOX_DRIVER_LIVOX_DIAGNOSTICS_HPP_
#define LIVOX_DRIVER_LIVOX_DIAGNOSTICS_HPP_

#include <map>
#include <string>

#include "LidarDriver/livox_common.hpp"

namespace livox_driver
{
namespace diagnostics
{
/** @enum livox_driver::LivoxTemperatureStatus
     *  @brief is a strongly typed enum class representing the lidar temperature status
     */
enum class LivoxTemperatureStatus : uint8_t {
  kUnknown = 0,     /**< kUnknown, not set */
  kNormal,          /**< Temperature in normal state */
  kHighLow,         /**< High or Low */
  kExtremelyHighLow /**< Extremely High or Low */
};

const std::map<LivoxTemperatureStatus, const char *> livox_temperature_dict_ = {
  {LivoxTemperatureStatus::kUnknown, "Unknown"},
  {LivoxTemperatureStatus::kNormal, "Ok"},
  {LivoxTemperatureStatus::kHighLow, "High or Low"},
  {LivoxTemperatureStatus::kExtremelyHighLow, "Extremely High or Low"}};

/** @enum livox_driver::LivoxVoltageStatus
     *  @brief is a strongly typed enum class representing the lidar Voltage Status of Internal Module
     */
enum class LivoxVoltageStatus : uint8_t {
  kUnknown = 0,  /**< kUnknown, not set */
  kNormal,       /**< Voltage in Normal State  */
  kHigh,         /**< High */
  kExtremelyHigh /**< Extremely High */
};
const std::map<LivoxVoltageStatus, const char *> livox_voltage_dict_ = {
  {LivoxVoltageStatus::kUnknown, "Unknown"},
  {LivoxVoltageStatus::kNormal, "Ok"},
  {LivoxVoltageStatus::kHigh, "High Voltage"},
  {LivoxVoltageStatus::kExtremelyHigh, "Extremely High Voltage"}};

/** @enum livox_driver::LivoxMotorStatus
     *  @brief is a strongly typed enum class representing the lidar motor status
     */
enum class LivoxMotorStatus : uint8_t {
  kUnknown = 0, /**< kUnknown, not set */
  kNormal,      /**< Motor in Normal State */
  kWarning,     /**< Motor in Warning State */
  kError        /**< Motor in Error State, Unable to Work */
};
const std::map<LivoxMotorStatus, const char *> livox_motor_dict_ = {
  {LivoxMotorStatus::kUnknown, "Unknown"},
  {LivoxMotorStatus::kNormal, "Ok"},
  {LivoxMotorStatus::kWarning, "Warning State"},
  {LivoxMotorStatus::kError, "Motor in Error State, Unable to Work"}};

/** @enum livox_driver::LivoxFirmwareStatus
 *  @brief is a strongly typed enum class representing the lidar firmware status
 */
enum class LivoxFirmwareStatus : uint8_t {
  kUnknown = 0, /**< kUnknown, not set */
  kOk,          /**< Firmware is Ok */
  kAbnormal     /**< kUnknown, not set */
};
const std::map<LivoxFirmwareStatus, const char *> livox_firmware_dict_ = {
  {LivoxFirmwareStatus::kUnknown, "Unknown"},
  {LivoxFirmwareStatus::kOk, "Ok"},
  {LivoxFirmwareStatus::kAbnormal, "Motor in Warning State"}};

/** @enum livox_driver::LivoxDirtStatus
     *  @brief is a strongly typed enum class representing the lidar window dirt status
     */
enum class LivoxDirtStatus : uint8_t {
  kUnknown = 0, /**< kUnknown, not set */
  kNotDirty,    /**< Not Dirty or Blocked  */
  kDirty        /**< Dirty or Blocked */
};
const std::map<LivoxDirtStatus, const char *> livox_dirt_dict_ = {
  {LivoxDirtStatus::kUnknown, "Unknown"},
  {LivoxDirtStatus::kNotDirty, "Ok"},
  {LivoxDirtStatus::kDirty, "Window Dirty or Blocked"}};

/** @enum livox_driver::LivoxPpsStatus
     *  @brief is a strongly typed enum class representing the lidar system status
     */
enum class LivoxPpsStatus : uint8_t {
  kUnknown = 0, /**< kUnknown, not set */
  kNoSignal,    /**< No Signal */
  kSignalOk     /**< PPS Signal Ok  */
};
const std::map<LivoxPpsStatus, const char *> livox_pps_dict_ = {
  {LivoxPpsStatus::kUnknown, "Unknown"},
  {LivoxPpsStatus::kNoSignal, "No Signal"},
  {LivoxPpsStatus::kSignalOk, "OK"}};

/** @enum livox_driver::LivoxDeviceLifeStatus
     *  @brief is a strongly typed enum class representing the lidar life status
     */
enum class LivoxDeviceLifeStatus : uint8_t {
  kUnknown = 0, /**< kUnknown, not set */
  kNormal,      /**< Normal   */
  kWarning      /**< Warning for Approaching the End of Service Life */
};
const std::map<LivoxDeviceLifeStatus, const char *> livox_life_dict_ = {
  {LivoxDeviceLifeStatus::kUnknown, "Unknown"},
  {LivoxDeviceLifeStatus::kNormal, "Ok"},
  {LivoxDeviceLifeStatus::kWarning, "Warning for Approaching the End of Service Life"}};

/** @enum livox_driver::LivoxFanStatus
     *  @brief is a strongly typed enum class representing the lidar fan status
     */
enum class LivoxFanStatus : uint8_t {
  kUnknown = 0, /**< kUnknown, not set */
  kNormal,      /**< Fan in Normal State    */
  kWarning      /**< Fan in Warning State */
};
const std::map<LivoxFanStatus, const char *> livox_fan_dict_ = {
  {LivoxFanStatus::kUnknown, "Unknown"},
  {LivoxFanStatus::kNormal, "Ok"},
  {LivoxFanStatus::kWarning, "Warning"}};

/** @enum livox_driver::LivoxSelfHeatingStatus
     *  @brief is a strongly typed enum class representing the lidar self heating status
     */
enum class LivoxSelfHeatingStatus : uint8_t {
  kUnknown = 0, /**< kUnknown, not set */
  kOn,          /**< Low Temperature Self Heating On    */
  kOff          /**< Low Temperature Self Heating Off */
};
const std::map<LivoxSelfHeatingStatus, const char *> livox_heating_dict_ = {
  {LivoxSelfHeatingStatus::kUnknown, "Unknown"},
  {LivoxSelfHeatingStatus::kOn, "ON"},
  {LivoxSelfHeatingStatus::kOff, "OFF"}};

/** @enum livox_driver::LivoxPtpStatus
     *  @brief is a strongly typed enum class representing the lidar PTP 1588 status
     */
enum class LivoxPtpStatus : uint8_t {
  kUnknown = 0, /**< kUnknown, not set */
  kNoSignal,    /**< No 1588 Signal     */
  kSignalOk     /**< 1588 Signal is Ok */
};
const std::map<LivoxPtpStatus, const char *> livox_ptp_dict_ = {
  {LivoxPtpStatus::kUnknown, "Unknown"},
  {LivoxPtpStatus::kNoSignal, "No Signal"},
  {LivoxPtpStatus::kSignalOk, "OK"}};

/** @enum livox_driver::LivoxTimeSyncStatus
     *  @brief is a strongly typed enum class representing the lidar time synchronization status
     */
enum class LivoxTimeSyncStatus : uint8_t {
  kUnknown = 0, /**< kUnknown, not set */
  kNoSync,      /**< System does not start time synchronization   */
  kUsingPtp,    /**< Using PTP 1588 synchronization  */
  kUsingGps,    /**< Using GPS synchronization  */
  kUsingPps,    /**< Using PPS synchronization  */
  kAbnormal /**< System time synchronization is abnormal (The highest priority synchronization signal is abnormal) */
};
const std::map<LivoxTimeSyncStatus, const char *> livox_timesync_dict_ = {
  {LivoxTimeSyncStatus::kUnknown, "Unknown"},
  {LivoxTimeSyncStatus::kNoSync, "NO SYNC"},
  {LivoxTimeSyncStatus::kUsingPtp, "PTP Ok"},
  {LivoxTimeSyncStatus::kUsingGps, "GPS Ok"},
  {LivoxTimeSyncStatus::kUsingPps, "PPS Ok"},
  {LivoxTimeSyncStatus::kAbnormal,
   "System time synchronization is abnormal (The highest priority synchronization signal"
   "is abnormal)"}};

/** @enum livox_driver::LivoxSystemStatus
     *  @brief is a strongly typed enum class representing the lidar system status
     */
enum class LivoxSystemStatus : uint8_t {
  kUnknown = 0, /**< kUnknown, not set */
  kNormal,      /**< Normal */
  kWarning,     /**< Warning  */
  kError        /**< Causes the LiDAR to Shut Down and Enter the Error State. */
};
const std::map<LivoxSystemStatus, const char *> livox_system_dict_ = {
  {LivoxSystemStatus::kUnknown, "Unknown"},
  {LivoxSystemStatus::kNormal, "Ok"},
  {LivoxSystemStatus::kWarning, "Warning"},
  {LivoxSystemStatus::kError, "Error"}};

/**
    * Encapsulates all the Diagnostics
    */
class LivoxDiagnosticsSensorStatus
{
public:
  LivoxTemperatureStatus temperature_status;
  LivoxVoltageStatus voltage_status;
  LivoxMotorStatus motor_status;
  LivoxFirmwareStatus firmware_status;
  LivoxDirtStatus dirt_status;
  LivoxPpsStatus pps_status;
  LivoxDeviceLifeStatus device_life_status;
  LivoxFanStatus fan_status;
  LivoxSelfHeatingStatus self_heating_status;
  LivoxPtpStatus ptp_status;
  LivoxTimeSyncStatus time_sync_status;
  LivoxSystemStatus system_status;
};

/**
    * Parses the raw 32 bit word to LivoxTemperatureStatus\n
    * Bit0:1 temp_status\n
    * 0: Temperature in Normal State\n
    * 1: High or Low\n
    * 2: Extremely High or Extremely Low\n
    * @param raw_status_code 32 bit word containing the lidar_status_code
    * @return LivoxTemperatureStatus
    */
LivoxTemperatureStatus StatusCodeToLivoxTemperatureStatus(const uint32_t & raw_status_code);

/**
    * Parses the raw 32 bit word to LivoxVoltageStatus\n
    * Bit2:3 volt_status\n
    * 0: Voltage in Normal State\n
    * 1: High\n
    * 2: Extremely High\n
    * @param raw_status_code 32 bit word containing the lidar_status_code
    * @return LivoxVoltageStatus
    */
LivoxVoltageStatus StatusCodeToLivoxVoltageStatus(const uint32_t & raw_status_code);

/**
    * Parses the raw 32 bit word to LivoxMotorStatus\n
    * Bit4:5 motor_status \n
    * 0: Motor in Normal State\n
    * 1: Motor in Warning State\n
    * 2: Motor in Error State, Unable to Work
    * @param raw_status_code 32 bit word containing the lidar_status_code
    * @return LivoxMotorStatus
    */
LivoxMotorStatus StatusCodeToLivoxMotorStatus(const uint32_t & raw_status_code);

/**
    * Parses the raw 32 bit word to LivoxDirtStatus\n
    * Bit6:7 dirty_warn \n
    * 0: Not Dirty or Blocked\n
    * 1: Dirty or Blocked
    * @param raw_status_code 32 bit word containing the lidar_status_code
    * @return LivoxDirtStatus
    */
LivoxDirtStatus StatusCodeToLivoxDirtStatus(const uint32_t & raw_status_code);

/**
    * Parses the raw 32 bit word to LivoxFirmwareStatus\n
    * Bit8 firmware_status\n
    * 0: Firmware is OK\n
    * 1: Firmware is Abnormal, Need to be Upgraded
    * @param raw_status_code 32 bit word containing the lidar_status_code
    * @return LivoxFirmwareStatus
    */
LivoxFirmwareStatus StatusCodeToLivoxFirmwareStatus(const uint32_t & raw_status_code);

/**
    * Parses the raw 32 bit word to LivoxPpsStatus\n
    * Bit9 pps_status\n
    * 0: No PPS Signal\n
    * 1: PPS Signal is OK
    * @param raw_status_code 32 bit word containing the lidar_status_code
    * @return LivoxPpsStatus
    */
LivoxPpsStatus StatusCodeToLivoxPpsStatus(const uint32_t & raw_status_code);

/**
    * Parses the raw 32 bit word to LivoxDeviceLifeStatus\n
    * Bit10 fan_status\n
    * 0: Normal\n
    * 1: Warning for Approaching the End of Service Life
    * @param raw_status_code 32 bit word containing the lidar_status_code
    * @return LivoxDeviceLifeStatus
    */
LivoxDeviceLifeStatus StatusCodeToLivoxDeviceLifeStatus(const uint32_t & raw_status_code);

/**
    * Parses the raw 32 bit word to LivoxFanStatus\n
    * Bit11 fan_status\n
    * 0: Fan in Normal State\n
    * 1: Fan in Warning State
    * @param raw_status_code 32 bit word containing the lidar_status_code
    * @return LivoxFanStatus
    */
LivoxFanStatus StatusCodeToLivoxFanStatus(const uint32_t & raw_status_code);

/**
    * Parses the raw 32 bit word to LivoxPpsStatus\n
    * Bit13 ptp_status\n
    *  0: No 1588 Signal\n
    *  1: 1588 Signal is OK
    * @param raw_status_code 32 bit word containing the lidar_status_code
    * @return LivoxPpsStatus
    */
LivoxPtpStatus StatusCodeToLivoxPtpStatus(const uint32_t & raw_status_code);

/**
    * Parses the raw 32 bit word to LivoxTimeSyncStatus\n
    * Bit14:16 time_sync_status\n
    * 0: System dose not start time synchronization\n
    * 1: Using PTP 1588 synchronization\n
    * 2: Using GPS synchronization\n
    * 3: Using PPS synchronization\n
    * 4: System time synchronization is abnormal
    * @param raw_status_code 32 bit word containing the lidar_status_code
    * @return LivoxTimeSyncStatus
    */
LivoxTimeSyncStatus StatusCodeToLivoxTimeSyncStatus(const uint32_t & raw_status_code);

/**
    * Parses the raw 32 bit word to LivoxSystemStatus\n
    * Bit30:31 system_status\n
    * 0: Normal\n
    * 1: Warning\n
    * 2: Error
    * @param raw_status_code 32 bit word containing the lidar_status_code
    * @return LivoxTimeSyncStatus
    */
LivoxSystemStatus StatusCodeToLivoxSystemStatus(const uint32_t & raw_status_code);

/**
   * Converts the Raw 32 bit word contained in the Ethernet Packet received from the sensor to a LivoxDiagnosticsSensorStatus
   * object.
   * @param status_code The union containing the bit sets to transform
   * @return The parsed LivoxDiagnosticsSensorStatus object
   */
LivoxDiagnosticsSensorStatus StatusCodeToLivoxSensorStatus(const uint32_t & raw_status_code);
}  // namespace diagnostics
}  // namespace livox_driver

#endif  //LIVOX_DRIVER_LIVOX_DIAGNOSTICS_HPP_
