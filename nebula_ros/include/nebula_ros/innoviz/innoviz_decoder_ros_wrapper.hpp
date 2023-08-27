#pragma once

#include "nebula_common/innoviz/innoviz_common.hpp"
#include "nebula_decoders/nebula_decoders_innoviz/innoviz_driver.hpp"
#include "nebula_ros/common/nebula_driver_ros_wrapper_base.hpp"

#include <rclcpp/rclcpp.hpp>

#include "innoviz_msgs/msg/innoviz_packet.hpp"
#include "innoviz_msgs/msg/innoviz_scan.hpp"


namespace nebula
{
namespace ros
{
    
/// @brief ROS wrapper of Innoviz driver
class InnovizDriverRosWrapper : public rclcpp::Node, NebulaDriverRosWrapperBase
{
public:
    InnovizDriverRosWrapper(const rclcpp::NodeOptions & options);

    /// @brief Returns the Status of the driver
    Status GetStatus();

private:

    /// @brief  Convers ROS parameters to InnovizSensorConfiguration
    /// @param sensor_configuration Innoviz sensor configuration toe fill
    /// @return Resulting status
    Status GetParameters(drivers::InnovizSensorConfiguration & sensor_configuration);

    /// @brief Initializes the Innoviz Driver
    /// @param sensor_configuration Sensor configuration for the driver to initialize    
    Status InitializeDriver(std::shared_ptr<drivers::SensorConfigurationBase> sensor_configuration, 
                            std::shared_ptr<drivers::CalibrationConfigurationBase> calibration_configuration) override;

    /// @brief Callback for the InnovizScan subscriber
    /// @param scan_msg The receive InnovizScan to handle
    void ReceiveScanMsgCallback(const innoviz_msgs::msg::InnovizScan::SharedPtr scan_msg);

private:
    std::shared_ptr<drivers::InnovizDriver> driver_ptr_;
    Status wrapper_status_;
    rclcpp::Subscription<innoviz_msgs::msg::InnovizScan>::SharedPtr innoviz_scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr nebula_points_pub_;

    std::shared_ptr<drivers::CalibrationConfigurationBase> calibration_cfg_ptr_;
    std::shared_ptr<drivers::SensorConfigurationBase> sensor_cfg_ptr_;
};

} // namespace ros
} // namespace nebula
