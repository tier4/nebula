#pragma once

#include "nebula_common/robosense/robosense_common.hpp"
#include "nebula_decoders/nebula_decoders_robosense/decoders/robosense_info_decoder_base.hpp"
#include "nebula_decoders/nebula_decoders_robosense/decoders/robosense_packet.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cstdint>
#include <memory>
#include <vector>

namespace nebula
{
namespace drivers
{

template <typename SensorT>
class RobosenseInfoDecoder : public RobosenseInfoDecoderBase
{
protected:
  /// @brief Configuration for this decoder
  const std::shared_ptr<drivers::RobosenseSensorConfiguration> sensor_configuration_;

  /// @brief The sensor definition, used for return mode and time offset handling
  SensorT sensor_{};

  typename SensorT::info_t packet_{};

  rclcpp::Logger logger_;

public:
  /// @brief Validates and parse PandarPacket. Currently only checks size, not checksums etc.
  /// @param pandar_packet The incoming PandarPacket
  /// @return Whether the packet was parsed successfully
  bool parsePacket(const std::vector<uint8_t> & raw_packet) override
  {
    const auto packet_size = raw_packet.size();
    if (packet_size < sizeof(typename SensorT::info_t)) {
      RCLCPP_ERROR_STREAM(
        logger_, "Packet size mismatch:" << packet_size << " | Expected at least:"
                                         << sizeof(typename SensorT::info_t));
      return false;
    }
    try {
      if (std::memcpy(&packet_, raw_packet.data(), sizeof(typename SensorT::info_t)) == &packet_) {
        return true;
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(logger_, "Packet memcopy failed: " << e.what());
    }

    return false;
  }

  /// @brief Constructor
  /// @param sensor_configuration SensorConfiguration for this decoder
  /// @param calibration_configuration Calibration for this decoder
  /// calibration_configuration is set)
  explicit RobosenseInfoDecoder(
    const std::shared_ptr<RobosenseSensorConfiguration> & sensor_configuration,
    const std::shared_ptr<RobosenseCalibrationConfiguration> & calibration_configuration)
  : sensor_configuration_(sensor_configuration), logger_(rclcpp::get_logger("RobosenseInfoDecoder"))
  {
    logger_.set_level(rclcpp::Logger::Level::Debug);
    RCLCPP_INFO_STREAM(logger_, sensor_configuration_);
  }

  std::map<std::string, std::string> getSensorInfo() override
  {
    std::map<std::string, std::string> sensor_info;
    sensor_info["motor_speed"] = std::to_string(packet_.motor_speed.value());
    sensor_info["lidar_ip"] = packet_.ethernet.lidar_ip.to_string();
    sensor_info["dest_pc_ip"] = packet_.ethernet.dest_pc_ip.to_string();
    sensor_info["mac_addr"] = std::to_string(packet_.ethernet.mac_addr.value());
    sensor_info["lidar_out_msop_port"] =
      std::to_string(packet_.ethernet.lidar_out_msop_port.value());
    sensor_info["pc_dest_msop_port"] = std::to_string(packet_.ethernet.pc_dest_msop_port.value());
    sensor_info["lidar_out_difop_port"] =
      std::to_string(packet_.ethernet.lidar_out_difop_port.value());
    sensor_info["pc_dest_difop_port"] = std::to_string(packet_.ethernet.pc_dest_difop_port.value());
    sensor_info["fov_start"] = std::to_string(packet_.fov_setting.fov_start.value());
    sensor_info["fov_end"] = std::to_string(packet_.fov_setting.fov_end.value());
    sensor_info["phase_lock"] = std::to_string(packet_.phase_lock.value());
    sensor_info["top_firmware_version"] = std::to_string(packet_.top_firmware_version.value());
    sensor_info["bottom_firmware_version"] =
      std::to_string(packet_.bottom_firmware_version.value());
    sensor_info["bottom_software_version"] =
      std::to_string(packet_.bottom_software_version.value());
    sensor_info["motor_firmware_version"] = std::to_string(packet_.motor_firmware_version.value());
    sensor_info["sensor_hw_version"] = std::to_string(packet_.sensor_hw_version.value());
    sensor_info["web_page_version"] = std::to_string(packet_.web_page_version.value());
    sensor_info["top_backup_crc"] = std::to_string(packet_.top_backup_crc.value());
    sensor_info["bottom_backup_crc"] = std::to_string(packet_.bottom_backup_crc.value());
    sensor_info["software_backup_crc"] = std::to_string(packet_.software_backup_crc.value());
    sensor_info["webpage_backup_crc"] = std::to_string(packet_.webpage_backup_crc.value());
    sensor_info["ethernet_gateway"] = packet_.ethernet_gateway.to_string();
    sensor_info["subnet_mask"] = packet_.subnet_mask.to_string();
    sensor_info["serial_number"] = std::to_string(packet_.serial_number.value());
    sensor_info["zero_angle_offset"] = std::to_string(packet_.zero_angle_offset.value());

    if (packet_.return_mode.value() == 0x00) {
      sensor_info["return_mode"] = "dual";
    } else if (packet_.return_mode.value() == 0x04) {
      sensor_info["return_mode"] = "strongest";
    } else if (packet_.return_mode.value() == 0x05) {
      sensor_info["return_mode"] = "last";
    } else if (packet_.return_mode.value() == 0x06) {
      sensor_info["return_mode"] = "first";
    }

    sensor_info["time_sync_mode"] = std::to_string(packet_.time_sync_mode.value());
    sensor_info["sync_status"] = std::to_string(packet_.sync_status.value());
    sensor_info["time"] = std::to_string(packet_.time.get_time_in_ns());
    sensor_info["i_dat"] = std::to_string(packet_.operating_status.i_dat.value());
    sensor_info["v_dat"] = std::to_string(packet_.operating_status.v_dat.value());
    sensor_info["v_dat_12v"] = std::to_string(packet_.operating_status.v_dat_12v.value());
    sensor_info["v_dat_5v"] = std::to_string(packet_.operating_status.v_dat_5v.value());
    sensor_info["v_dat_2v5"] = std::to_string(packet_.operating_status.v_dat_2v5.value());
    sensor_info["v_dat_apd"] = std::to_string(packet_.operating_status.v_dat_apd.value());
    sensor_info["temperature1"] = std::to_string(packet_.fault_diagnosis.temperature1.value());
    sensor_info["temperature2"] = std::to_string(packet_.fault_diagnosis.temperature2.value());
    sensor_info["temperature3"] = std::to_string(packet_.fault_diagnosis.temperature3.value());
    sensor_info["temperature4"] = std::to_string(packet_.fault_diagnosis.temperature4.value());
    sensor_info["temperature5"] = std::to_string(packet_.fault_diagnosis.temperature5.value());
    sensor_info["r_rpm"] = std::to_string(packet_.fault_diagnosis.r_rpm.value());
    sensor_info["lane_up"] = std::to_string(packet_.fault_diagnosis.lane_up.value());
    sensor_info["lane_up_cnt"] = std::to_string(packet_.fault_diagnosis.lane_up_cnt.value());
    sensor_info["top_status"] = std::to_string(packet_.fault_diagnosis.top_status.value());
    sensor_info["gps_status"] = std::to_string(packet_.fault_diagnosis.gps_status.value());
    sensor_info["code_wheel_status"] = std::to_string(packet_.code_wheel_status.value());
    sensor_info["pps_trigger_mode"] = std::to_string(packet_.pps_trigger_mode.value());
    //        sensor_info["gprmc"] = std::to_string(packet_.gprmc.getGprmc());
    return sensor_info;
  }
};

}  // namespace drivers
}  // namespace nebula