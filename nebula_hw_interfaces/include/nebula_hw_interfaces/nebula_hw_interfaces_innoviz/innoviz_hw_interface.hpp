#pragma once


#include "nebula_common/innoviz/innoviz_common.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_common/nebula_hw_interface_base.hpp"

#include <rclcpp/rclcpp.hpp>
#include "boost_udp_driver/udp_driver.hpp"

#include "innoviz_msgs/msg/innoviz_packet.hpp"
#include "innoviz_msgs/msg/innoviz_scan.hpp"

namespace nebula
{
namespace drivers
{
/// @brief Hardware interface of Innoviz driver
class InnovizHwInterface : NebulaHwInterfaceBase
{
public:
    InnovizHwInterface();
    
    /// @brief Virtual function for starting the interface that handles UDP streams
    /// @return Resulting status
    virtual Status CloudInterfaceStart() override;
    
    /// @brief Virtual function for stopping the interface that handles UDP streams
    /// @return Resulting status
    virtual Status CloudInterfaceStop() override;
    
    /// @brief Virtual function for setting sensor configuration
    /// @param sensor_configuration SensorConfiguration for this interface
    /// @return Resulting status
    Status SetSensorConfiguration(std::shared_ptr<SensorConfigurationBase> sensor_configuration) override;
    
    /// @brief Virtual function for printing sensor configuration
    /// @param sensor_configuration SensorConfiguration for the checking
    /// @return Resulting status
    Status GetSensorConfiguration(SensorConfigurationBase & sensor_configuration) override;
    
    /// @brief Virtual function for printing calibration configuration
    /// @param calibration_configuration CalibrationConfiguration for the checking
    /// @return Resulting status
    Status GetCalibrationConfiguration(CalibrationConfigurationBase & calibration_configuration) override;

    /// @brief Set callback function to call on completeion of scan read
    /// @param scan_callback Callback function to call
    void RegisterScanCallback(std::function<void(std::unique_ptr<innoviz_msgs::msg::InnovizScan>)> scan_callback);
    
    /// @brief Setting rclcpp::Logger
    /// @param logger Logger
    void SetLogger(std::shared_ptr<rclcpp::Logger> logger);
    
private:
    /// @brief Callback function to receive the Cloud Packet data from the UDP Driver
    /// @param buffer Buffer containing the data received from the UDP socket
    void ReceiveCloudPacketCallback(const std::vector<uint8_t> & buffer);

    /// @brief Printing the string to RCLCPP_ERROR_STREAM if logger is defined. If not prints to stderr
    /// @param error Target string
    void PrintError(std::string errorMessage);


private:
    std::shared_ptr<rclcpp::Logger> parent_node_logger;
    std::unique_ptr<::drivers::common::IoContext> cloud_io_context_;
    std::unique_ptr<::drivers::udp_driver::UdpDriver> cloud_udp_driver_;
    std::shared_ptr<SensorConfigurationBase> sensor_configuration_;
    std::unique_ptr<innoviz_msgs::msg::InnovizScan> scan_cloud_ptr_;
    std::function<void(std::unique_ptr<innoviz_msgs::msg::InnovizScan> buffer)> scan_reception_callback_;
    
    uint32_t processed_bytes_ = 0;
    uint32_t last_frame_id_ = 0;
    
};

}  // namespace drivers
}  // namespace nebula

