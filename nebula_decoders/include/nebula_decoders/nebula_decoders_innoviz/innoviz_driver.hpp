#pragma once

#include "innoviz_msgs/msg/innoviz_scan.hpp"
#include "nebula_decoders/nebula_decoders_common/nebula_driver_base.hpp"
#include "nebula_decoders/nebula_decoders_innoviz/decoders/innoviz_scan_decoder.hpp"
#include "nebula_common/innoviz/innoviz_common.hpp"

namespace nebula
{
namespace drivers
{

/// @brief Innoviz Driver
class InnovizDriver : public NebulaDriverBase 
{
public:
    /// @brief Constructor
    /// @param sensor_configuration SensorConfiguration for this driver
    InnovizDriver(const std::shared_ptr<drivers::InnovizSensorConfiguration>& sensorConfiguration);

    /// @brief Convert InnovizScan message to point cloud
    /// @param innoviz_scan InnovizScan message to convert
    /// @return Point cloud
    drivers::NebulaPointCloudPtr ConvertScanToPointcloud(const std::shared_ptr<innoviz_msgs::msg::InnovizScan> & innoviz_scan);

    /// @brief Setting CalibrationConfiguration (not used)
    /// @param calibration_configuration
    /// @return Resulting status
    Status SetCalibrationConfiguration(const CalibrationConfigurationBase & calibration_configuration);

    /// @brief Get current status of this driver
    /// @return Current status
    Status GetStatus();

private:
    std::shared_ptr<drivers::InnovizScanDecoder> scan_decoder_;
    Status driver_status_;
};

}
}

