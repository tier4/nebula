
#include "nebula_ros/innoviz/innoviz_hw_interface_ros_wrapper.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace nebula
{
namespace ros
{

    InnovizHwInterfaceRosWrapper::InnovizHwInterfaceRosWrapper(const rclcpp::NodeOptions & options)
        : rclcpp::Node("innoviz_hw_interface_ros_wrapper", options), hw_interface_()
    {
        interface_status_ = GetParameters(sensor_configuration_);
        if (Status::OK != interface_status_) {
            RCLCPP_ERROR_STREAM(this->get_logger(), this->get_name() << " Error:" << interface_status_);
            return;
        }

        hw_interface_.SetLogger(std::make_shared<rclcpp::Logger>(this->get_logger()));

        RCLCPP_INFO_STREAM(this->get_logger(), "Initialize sensor_configuration");
        std::shared_ptr<drivers::SensorConfigurationBase> sensor_cfg_ptr =
            std::make_shared<drivers::InnovizSensorConfiguration>(sensor_configuration_);
        RCLCPP_INFO_STREAM(this->get_logger(), "hw_interface_.SetSensorConfiguration");
        hw_interface_.SetSensorConfiguration(std::static_pointer_cast<drivers::SensorConfigurationBase>(sensor_cfg_ptr));

        auto scanCallbackFunction = std::bind(&InnovizHwInterfaceRosWrapper::ReceiveScanDataCallback, this, std::placeholders::_1);
        // register scan callback and publisher
        hw_interface_.RegisterScanCallback(scanCallbackFunction);
        innoviz_scan_pub_ = this->create_publisher<innoviz_msgs::msg::InnovizScan>(
            "innoviz_packets",
            rclcpp::SensorDataQoS(rclcpp::KeepLast(10)).best_effort().durability_volatile());

        auto status = StreamStart();
        if (status == nebula::Status::OK) {
            RCLCPP_INFO_STREAM(get_logger(), "UDP Driver Started");
        } else {
            RCLCPP_ERROR_STREAM(get_logger(), status);
        }
    }

    Status InnovizHwInterfaceRosWrapper::StreamStart()
    {
        if (Status::OK == interface_status_) {
            interface_status_ = hw_interface_.CloudInterfaceStart();
        }
        return interface_status_;
    }

    Status InnovizHwInterfaceRosWrapper::StreamStop() { return Status::OK; }
    Status InnovizHwInterfaceRosWrapper::Shutdown() { return Status::OK; }


    void InnovizHwInterfaceRosWrapper::ReceiveScanDataCallback(std::unique_ptr<innoviz_msgs::msg::InnovizScan> scan_buffer)
    {
        scan_buffer->header.frame_id = sensor_configuration_.frame_id;
        //TODO: Add timestamp??
        innoviz_scan_pub_->publish(*scan_buffer);
    }

    Status InnovizHwInterfaceRosWrapper::InitializeHwInterface(const drivers::SensorConfigurationBase & /*sensor_configuration*/)
    {
        return Status::OK;
    }


    Status InnovizHwInterfaceRosWrapper::GetParameters(drivers::InnovizSensorConfiguration & sensor_configuration)
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
            this->declare_parameter<std::string>("host_ip", "192.168.0.2", descriptor);
            sensor_configuration.host_ip = this->get_parameter("host_ip").as_string();
        }
        {
            rcl_interfaces::msg::ParameterDescriptor descriptor;
            descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
            descriptor.read_only = true;
            descriptor.dynamic_typing = false;
            descriptor.additional_constraints = "";
            this->declare_parameter<std::string>("sensor_ip", "192.168.0.2", descriptor);
            sensor_configuration.sensor_ip = this->get_parameter("sensor_ip").as_string();
        }
        {
            rcl_interfaces::msg::ParameterDescriptor descriptor;
            descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
            descriptor.read_only = false;
            descriptor.dynamic_typing = false;
            descriptor.additional_constraints = "";
            this->declare_parameter<std::string>("frame_id", "innoviz", descriptor);
            sensor_configuration.frame_id = this->get_parameter("frame_id").as_string();
        }
        {
            rcl_interfaces::msg::ParameterDescriptor descriptor;
            descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
            descriptor.read_only = true;
            descriptor.dynamic_typing = false;
            descriptor.additional_constraints = "";
            this->declare_parameter<uint16_t>("data_port", 9101, descriptor);
            sensor_configuration.data_port = this->get_parameter("data_port").as_int();
        }

        if(sensor_configuration.sensor_model != nebula::drivers::SensorModel::INNOVIZ_TWO_CONDOR && 
            sensor_configuration.sensor_model != nebula::drivers::SensorModel::INNOVIZ_TWO_RAVEN)
        {
            return Status::INVALID_SENSOR_MODEL;
        }

        RCLCPP_INFO_STREAM(this->get_logger(), "SensorConfig:" << sensor_configuration);
        return Status::OK;
    }

RCLCPP_COMPONENTS_REGISTER_NODE(InnovizHwInterfaceRosWrapper)
}
}
