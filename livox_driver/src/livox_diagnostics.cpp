#include <iostream>

#include "LidarDriver/livox_common.hpp"
#include "LidarDriver/livox_diagnostics.hpp"

namespace livox_driver {
  namespace diagnostics {
    LivoxTemperatureStatus StatusCodeToLivoxTemperatureStatus(const uint32_t &raw_status_code) {
      LivoxTemperatureStatus temp_status;
      uint8_t temp_code = raw_status_code & 0x0003;
      switch (temp_code) {
        case 0:
          temp_status = LivoxTemperatureStatus::kNormal;
          break;
        case 1:
          temp_status = LivoxTemperatureStatus::kHighLow;
          break;
        case 2:
          temp_status = LivoxTemperatureStatus::kExtremelyHighLow;
          break;
        default:
          temp_status = LivoxTemperatureStatus::kUnknown;
          break;
      }
      return temp_status;
    }

    LivoxVoltageStatus StatusCodeToLivoxVoltageStatus(const uint32_t &raw_status_code) {
      LivoxVoltageStatus temp_status;
      uint8_t temp_code = (raw_status_code & 0x000C) >> 2;
      switch (temp_code) {
        case 0:
          temp_status = LivoxVoltageStatus::kNormal;
          break;
        case 1:
          temp_status = LivoxVoltageStatus::kHigh;
          break;
        case 2:
          temp_status = LivoxVoltageStatus::kExtremelyHigh;
          break;
        default:
          temp_status = LivoxVoltageStatus::kUnknown;
          break;
      }
      return temp_status;
    }

    LivoxDirtStatus StatusCodeToLivoxDirtStatus(const uint32_t &raw_status_code) {
      LivoxDirtStatus temp_status;
      uint8_t temp_code = (raw_status_code & 0x00C0) >> 6;
      switch (temp_code) {
        case 0:
          temp_status = LivoxDirtStatus::kNotDirty;
          break;
        case 1:
          temp_status = LivoxDirtStatus::kDirty;
          break;
        default:
          temp_status = LivoxDirtStatus::kUnknown;
          break;
      }
      return temp_status;
    }

    LivoxFirmwareStatus StatusCodeToLivoxFirmwareStatus(const uint32_t &raw_status_code) {
      bool temp_code = raw_status_code & (1 << 8);
      if (!temp_code) {
        return LivoxFirmwareStatus::kOk;
      }
      return LivoxFirmwareStatus::kAbnormal;
    }

    LivoxPpsStatus StatusCodeToLivoxPpsStatus(const uint32_t &raw_status_code) {
      bool temp_code = raw_status_code & (1 << 9);
      if (!temp_code) {
        return LivoxPpsStatus::kNoSignal;
      }
      return LivoxPpsStatus::kSignalOk;
    }

    LivoxDeviceLifeStatus StatusCodeToLivoxDeviceLifeStatus(const uint32_t &raw_status_code) {
      bool temp_code = raw_status_code & (1 << 10);
      if (!temp_code) {
        return LivoxDeviceLifeStatus::kNormal;
      }
      return LivoxDeviceLifeStatus::kWarning;
    }

    LivoxFanStatus StatusCodeToLivoxFanStatus(const uint32_t &raw_status_code) {
      bool temp_code = raw_status_code & (1 << 11);
      if (!temp_code) {
        return LivoxFanStatus::kNormal;
      }
      return LivoxFanStatus::kWarning;
    }

    LivoxPtpStatus StatusCodeToLivoxPtpStatus(const uint32_t &raw_status_code) {
      bool temp_code = raw_status_code & (1 << 13);
      if (!temp_code) {
        return LivoxPtpStatus::kNoSignal;
      }
      return LivoxPtpStatus::kSignalOk;
    }

    LivoxTimeSyncStatus StatusCodeToLivoxTimeSyncStatus(const uint32_t &raw_status_code) {
      LivoxTimeSyncStatus temp_status;
      uint8_t temp_code = (raw_status_code & 0x1C000) >> 14;
      switch (temp_code) {
        case 0:
          temp_status = LivoxTimeSyncStatus::kNoSync;
          break;
        case 1:
          temp_status = LivoxTimeSyncStatus::kUsingPtp;
          break;
        case 2:
          temp_status = LivoxTimeSyncStatus::kUsingGps;
          break;
        case 3:
          temp_status = LivoxTimeSyncStatus::kUsingPps;
          break;
        default:
          temp_status = LivoxTimeSyncStatus::kUnknown;
          break;
      }
      return temp_status;
    }

    LivoxSystemStatus StatusCodeToLivoxSystemStatus(const uint32_t &raw_status_code) {
      LivoxSystemStatus temp_status;
      uint8_t temp_code = (raw_status_code & 0xC0000000) >> 30;
      switch (temp_code) {
        case 0:
          temp_status = LivoxSystemStatus::kNormal;
          break;
        case 1:
          temp_status = LivoxSystemStatus::kWarning;
          break;
        case 2:
          temp_status = LivoxSystemStatus::kError;
          break;
        default:
          temp_status = LivoxSystemStatus::kUnknown;
          break;
      }
      return temp_status;
    }

    LivoxMotorStatus StatusCodeToLivoxMotorStatus(const uint32_t &raw_status_code) {
      LivoxMotorStatus temp_status;
      uint8_t temp_code = (raw_status_code & 0x0030) >> 4;
      switch (temp_code) {
        case 0:
          temp_status = LivoxMotorStatus::kNormal;
          break;
        case 1:
          temp_status = LivoxMotorStatus::kWarning;
          break;
        case 2:
          temp_status = LivoxMotorStatus::kError;
          break;
        default:
          temp_status = LivoxMotorStatus::kUnknown;
          break;
      }
      return temp_status;
    }

    LivoxDiagnosticsSensorStatus StatusCodeToLivoxSensorStatus(const uint32_t &raw_status_code) {
      LivoxDiagnosticsSensorStatus diagnostics_status{};

      diagnostics_status.temperature_status = StatusCodeToLivoxTemperatureStatus(raw_status_code);
      diagnostics_status.voltage_status = StatusCodeToLivoxVoltageStatus(raw_status_code);
      diagnostics_status.motor_status = StatusCodeToLivoxMotorStatus(raw_status_code);
      diagnostics_status.firmware_status = StatusCodeToLivoxFirmwareStatus(raw_status_code);
      diagnostics_status.dirt_status = StatusCodeToLivoxDirtStatus(raw_status_code);
      diagnostics_status.pps_status = StatusCodeToLivoxPpsStatus(raw_status_code);
      diagnostics_status.fan_status = StatusCodeToLivoxFanStatus(raw_status_code);
      diagnostics_status.ptp_status = StatusCodeToLivoxPtpStatus(raw_status_code);
      diagnostics_status.time_sync_status = StatusCodeToLivoxTimeSyncStatus(raw_status_code);
      diagnostics_status.system_status = StatusCodeToLivoxSystemStatus(raw_status_code);
      diagnostics_status.device_life_status = StatusCodeToLivoxDeviceLifeStatus(raw_status_code);

      return diagnostics_status;
    }
  }  // namespace diagnostics
}  // namespace livox_driver