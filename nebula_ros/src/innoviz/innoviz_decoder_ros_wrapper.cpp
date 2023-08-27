
#include "nebula_ros/innoviz/innoviz_decoder_ros_wrapper.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp_components/register_node_macro.hpp>

namespace nebula
{
namespace ros
{

InnovizDriverRosWrapper::InnovizDriverRosWrapper(const rclcpp::NodeOptions& options)
    : rclcpp::Node("innoviz_driver_ros_wrapper", options)
{
    drivers::InnovizSensorConfiguration sensorConfiguration;

    wrapper_status_ = GetParameters(sensorConfiguration);

    if (Status::OK != wrapper_status_) {
        RCLCPP_ERROR_STREAM(this->get_logger(), this->get_name() << " Error:" << wrapper_status_);
        return;
    }
    RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << ". Starting...");

    sensor_cfg_ptr_ = std::make_shared<drivers::InnovizSensorConfiguration>(sensorConfiguration);
    auto calib_ptr = std::make_shared<drivers::CalibrationConfigurationBase>();

    RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << ". Driver ");
    wrapper_status_ = InitializeDriver(std::const_pointer_cast<drivers::SensorConfigurationBase>(sensor_cfg_ptr_), calib_ptr);
    RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << "Wrapper=" << wrapper_status_);

    auto innovizScancallbackFunction = std::bind(&InnovizDriverRosWrapper::ReceiveScanMsgCallback, this, std::placeholders::_1);

    innoviz_scan_sub_ = create_subscription<innoviz_msgs::msg::InnovizScan>(
            "innoviz_packets", 
            rclcpp::SensorDataQoS(), 
            innovizScancallbackFunction);
    
    nebula_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "innoviz_points", 
            rclcpp::SensorDataQoS());
}

Status InnovizDriverRosWrapper::InitializeDriver(
    std::shared_ptr<drivers::SensorConfigurationBase> sensor_configuration,
    std::shared_ptr<drivers::CalibrationConfigurationBase> /*calibration_configuration*/)
{
    driver_ptr_ = std::make_shared<drivers::InnovizDriver>(
        std::static_pointer_cast<drivers::InnovizSensorConfiguration>(sensor_configuration));
    return driver_ptr_->GetStatus();
}

Status InnovizDriverRosWrapper::GetStatus()
{
    return wrapper_status_;
}


void InnovizDriverRosWrapper::ReceiveScanMsgCallback(const innoviz_msgs::msg::InnovizScan::SharedPtr scan_msg)
{
    nebula::drivers::NebulaPointCloudPtr pointcloud = driver_ptr_->ConvertScanToPointcloud(scan_msg);

    if (nebula_points_pub_->get_subscription_count() > 0 ||
        nebula_points_pub_->get_intra_process_subscription_count() > 0) 
    {
        auto ros_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*pointcloud, *ros_pc_msg_ptr);
        //TODO: Add timestamp from driver??
        //ros_pc_msg_ptr->header.stamp = rclcpp::Time(SecondsToChronoNanoSeconds(std::get<1>(pointcloud_ts)).count());
        ros_pc_msg_ptr->header.frame_id = sensor_cfg_ptr_->frame_id;
        nebula_points_pub_->publish(std::move(ros_pc_msg_ptr));
    }
}


Status InnovizDriverRosWrapper::GetParameters(drivers::InnovizSensorConfiguration & sensor_configuration)
{
    {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
        descriptor.read_only = true;
        descriptor.dynamic_typing = false;
        descriptor.additional_constraints = "";
        this->declare_parameter<std::string>("sensor_model", "");
        
        std::string sensorModelString = this->get_parameter("sensor_model").as_string();
        sensor_configuration.sensor_model = nebula::drivers::SensorModelFromString(sensorModelString);
    }
    {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
        descriptor.read_only = true;
        descriptor.dynamic_typing = false;
        descriptor.additional_constraints = "";
        this->declare_parameter<std::string>("frame_id", "innoviz", descriptor);
        sensor_configuration.frame_id = this->get_parameter("frame_id").as_string();
    }
    {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        descriptor.read_only = false;
        descriptor.dynamic_typing = false;
        descriptor.additional_constraints = "";
        this->declare_parameter<double>("min_range", 1.2, descriptor);
        sensor_configuration.min_range = this->get_parameter("min_range").as_double();
    }
    {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        descriptor.read_only = false;
        descriptor.dynamic_typing = false;
        descriptor.additional_constraints = "";
        this->declare_parameter<double>("max_range", 250., descriptor);
        sensor_configuration.max_range = this->get_parameter("max_range").as_double();
    }
    {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
        descriptor.read_only = false;
        descriptor.dynamic_typing = false;
        descriptor.additional_constraints = "";
        this->declare_parameter<int>("min_confidence", 21, descriptor);
        sensor_configuration.min_confidence = this->get_parameter("min_confidence").as_int();
    }
    {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
        descriptor.read_only = false;
        descriptor.dynamic_typing = false;
        descriptor.additional_constraints = "";
        this->declare_parameter<bool>("filter_artifacts", true, descriptor);
        sensor_configuration.filter_artifacts = this->get_parameter("filter_artifacts").as_bool();
    }

    if(sensor_configuration.sensor_model != nebula::drivers::SensorModel::INNOVIZ_TWO_CONDOR && 
        sensor_configuration.sensor_model != nebula::drivers::SensorModel::INNOVIZ_TWO_RAVEN)
    {
        return Status::INVALID_SENSOR_MODEL;
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "Sensor model: " << sensor_configuration.sensor_model);
    
    return Status::OK;
}

RCLCPP_COMPONENTS_REGISTER_NODE(InnovizDriverRosWrapper)
} // namespace ros
} // namespace nebula
