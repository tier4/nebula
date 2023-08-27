#pragma once

#include "nebula_common/innoviz/innoviz_common.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_innoviz/innoviz_hw_interface.hpp"
#include "nebula_ros/common/nebula_hw_interface_ros_wrapper_base.hpp"

#include <rclcpp/rclcpp.hpp>

#include "innoviz_msgs/msg/innoviz_packet.hpp"
#include "innoviz_msgs/msg/innoviz_scan.hpp"

namespace nebula
{
namespace ros
{

class InnovizHwInterfaceRosWrapper : public rclcpp::Node, NebulaHwInterfaceWrapperBase
{
public:
    InnovizHwInterfaceRosWrapper(const rclcpp::NodeOptions & options);

    /// @brief Start point cloud streaming (Call CloudInterfaceStart of HwInterface)
    /// @return Resulting status
    Status StreamStart() override;

    /// @brief Stop point cloud streaming (not used)
    /// @return Resulting status
    Status StreamStop() override;

    /// @brief Shutdown (not used)
    /// @return Resulting status
    Status Shutdown() override;

    /// @brief Get configurations from ROS parameters
    /// @param sensorConfiguration Ouptut SensorConfiguration
    /// @return Resulting status
    Status GetParameters(drivers::InnovizSensorConfiguration & sensorConfiguration);

private:
    Status InitializeHwInterface(const drivers::SensorConfigurationBase & sensor_configuration) override;
    /// @brief Callback for receiving InnovizScan
    /// @param scan_buffer Received InnovizScan
    void ReceiveScanDataCallback(std::unique_ptr<innoviz_msgs::msg::InnovizScan> scan_buffer);


private:
    drivers::InnovizHwInterface hw_interface_;
    drivers::InnovizSensorConfiguration sensor_configuration_;
    Status interface_status_;

    rclcpp::Publisher<innoviz_msgs::msg::InnovizScan>::SharedPtr innoviz_scan_pub_;
};

}
}