#include "nebula_ros/robosense/robosense_decoder_ros_wrapper.hpp"

namespace nebula
{
namespace ros
{
RobosenseDriverRosWrapper::RobosenseDriverRosWrapper(const rclcpp::NodeOptions & options)
: rclcpp::Node("robosense_driver_ros_wrapper", options), hw_interface_()
{
  drivers::RobosenseCalibrationConfiguration calibration_configuration;
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  hw_interface_.SetLogger(std::make_shared<rclcpp::Logger>(this->get_logger()));

  wrapper_status_ = GetParameters(sensor_configuration, calibration_configuration);

  if (Status::OK != wrapper_status_) {
    RCLCPP_ERROR_STREAM(this->get_logger(), this->get_name() << " Error:" << wrapper_status_);
    return;
  }

  RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << ". Starting...");
  calibration_cfg_ptr_ =
    std::make_shared<drivers::RobosenseCalibrationConfiguration>(calibration_configuration);

  sensor_cfg_ptr_ = std::make_shared<drivers::RobosenseSensorConfiguration>(sensor_configuration);

  RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << ". Driver ");

  wrapper_status_ = InitializeDriver(
    std::const_pointer_cast<drivers::SensorConfigurationBase>(sensor_cfg_ptr_),
    std::static_pointer_cast<drivers::CalibrationConfigurationBase>(calibration_cfg_ptr_));

  RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << "Wrapper=" << wrapper_status_);
  robosense_scan_sub_ = create_subscription<robosense_msgs::msg::RobosenseScan>(
    "robosense_packets", rclcpp::SensorDataQoS(),
    std::bind(&RobosenseDriverRosWrapper::ReceiveScanMsgCallback, this, std::placeholders::_1));

  nebula_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "robosense_points", rclcpp::SensorDataQoS());

  aw_points_base_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("aw_points", rclcpp::SensorDataQoS());

  aw_points_ex_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("aw_points_ex", rclcpp::SensorDataQoS());
  if (
    sensor_configuration.sensor_model == nebula::drivers::SensorModel::ROBOSENSE_HELIOS ||
    sensor_configuration.sensor_model == nebula::drivers::SensorModel::ROBOSENSE_HELIOS16P) {
    if (robosense_helios_monitor_info_pub_ == nullptr) {
      robosense_helios_monitor_info_pub_ =
        this->create_publisher<robosense_msgs::msg::RobosenseHeliosMonitorInfo>(
          "robosense_helios_monitor_info", rclcpp::SensorDataQoS());
    }
    lidar_type = 0;
  } else if (
    sensor_configuration.sensor_model == nebula::drivers::SensorModel::ROBOSENSE_BPEARL ||
    sensor_configuration.sensor_model == nebula::drivers::SensorModel::ROBOSENSE_RUBYPLUS) {
    if (robosense_bp_ruby_monitor_info_pub_ == nullptr) {
      robosense_bp_ruby_monitor_info_pub_ =
        this->create_publisher<robosense_msgs::msg::RobosenseBpRubyMonitorInfo>(
          "robosense_bp_ruby_monitor_info", rclcpp::SensorDataQoS());
    }
    lidar_type = 1;
  } else if (sensor_configuration.sensor_model == nebula::drivers::SensorModel::ROBOSENSE_M1PLUS) {
    robosense_mems_monitor_info_pub_ =
      this->create_publisher<robosense_msgs::msg::RobosenseMemsMonitorInfo>(
        "robosense_mems_monitor_info", rclcpp::SensorDataQoS());
    lidar_type = 2;
  } else if (sensor_configuration.sensor_model == nebula::drivers::SensorModel::ROBOSENSE_E1) {
    robosense_flash_monitor_info_pub_ =
      this->create_publisher<robosense_msgs::msg::RobosenseFlashMonitorInfo>(
        "robosense_flash_monitor_info", rclcpp::SensorDataQoS());
    lidar_type = 3;
  }
  difop_callback_ = [this](const drivers::AllDeviceInfo & device_info) {
    if (lidar_type == 0) {
      robosense_msgs::msg::RobosenseHeliosMonitorInfo helios_monitor_info_buffer;
      for (int i = 0; i < 6; ++i) {
        helios_monitor_info_buffer.device_sn[i] = device_info.sn[i];
      }
      helios_monitor_info_buffer.motor_rmp = device_info.rpm;
      helios_monitor_info_buffer.bot_fpga_temperature = device_info.bot_fpga_temperature;
      helios_monitor_info_buffer.recv_a_temperature = device_info.recv_A_temperature;
      helios_monitor_info_buffer.recv_b_temperature = device_info.recv_B_temperature;
      helios_monitor_info_buffer.main_fpga_temperature = device_info.main_fpga_temperature;
      helios_monitor_info_buffer.main_fpga_core_temperature =
        device_info.main_fpga_core_temperature;
      helios_monitor_info_buffer.lane_up = device_info.lane_up;
      helios_monitor_info_buffer.lane_up_cnt = device_info.lane_up_cnt;
      helios_monitor_info_buffer.main_status = device_info.main_status;
      helios_monitor_info_buffer.gps_status = device_info.gps_status;
      robosense_helios_monitor_info_pub_->publish(helios_monitor_info_buffer);
    } else if (lidar_type == 1) {
      robosense_msgs::msg::RobosenseBpRubyMonitorInfo bp_ruby_monitor_info_buffer;
      for (int i = 0; i < 6; ++i) {
        bp_ruby_monitor_info_buffer.device_sn[i] = device_info.sn[i];
      }
      bp_ruby_monitor_info_buffer.motor_rmp = device_info.rpm;
      bp_ruby_monitor_info_buffer.bot_fpga_temperature = device_info.bot_fpga_temperature;
      bp_ruby_monitor_info_buffer.apd_temperature = device_info.apd_temperature;
      bp_ruby_monitor_info_buffer.main_fpga_temperature = device_info.main_fpga_temperature;
      bp_ruby_monitor_info_buffer.main_bot_temperature = device_info.main_bot_temperature;
      bp_ruby_monitor_info_buffer.gps_status = device_info.gps_status;
      robosense_bp_ruby_monitor_info_pub_->publish(bp_ruby_monitor_info_buffer);
    } else if (lidar_type == 2) {
      robosense_msgs::msg::RobosenseMemsMonitorInfo mems_monitor_info_buffer;
      for (int i = 0; i < 6; ++i) {
        mems_monitor_info_buffer.device_sn[i] = device_info.sn[i];
      }
      mems_monitor_info_buffer.battery_volt = device_info.battery_volt;
      mems_monitor_info_buffer.lidar_fault_status = device_info.lidar_fault_status;
      mems_monitor_info_buffer.lidar_roi_status = device_info.lidar_roi_status;
      robosense_mems_monitor_info_pub_->publish(mems_monitor_info_buffer);
    } else if (lidar_type == 3) {
      robosense_msgs::msg::RobosenseFlashMonitorInfo flash_monitor_info_buffer;
      for (int i = 0; i < 6; ++i) {
        flash_monitor_info_buffer.device_sn[i] = device_info.sn[i];
      }
      flash_monitor_info_buffer.pl_vmon_vin_p = device_info.pl_vmon_vin_p;
      flash_monitor_info_buffer.fpga_pl_kernal_temperature = device_info.fpga_pl_kernal_temperature;
      flash_monitor_info_buffer.fpga_ps_kernal_temperature = device_info.fpga_ps_kernal_temperature;
      flash_monitor_info_buffer.window_temperature = device_info.window_temperature;
      robosense_flash_monitor_info_pub_->publish(flash_monitor_info_buffer);
    }
  };

  driver_ptr_->registDifopCallback(difop_callback_);
}

void RobosenseDriverRosWrapper::ReceiveScanMsgCallback(
  const robosense_msgs::msg::RobosenseScan::SharedPtr scan_msg)
{
  // take packets out of scan msg
  std::vector<robosense_msgs::msg::RobosensePacket> pkt_msgs = scan_msg->packets;
  std::tuple<nebula::drivers::NebulaPointCloudPtr, double> pointcloud_ts =
    driver_ptr_->ConvertScanToPointcloud(scan_msg);
  nebula::drivers::NebulaPointCloudPtr pointcloud = std::get<0>(pointcloud_ts);

  if (pointcloud == nullptr) {
    RCLCPP_WARN_STREAM(get_logger(), "Empty cloud parsed.");
    return;
  };

  if (
    nebula_points_pub_->get_subscription_count() > 0 ||
    nebula_points_pub_->get_intra_process_subscription_count() > 0) {
    auto ros_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*pointcloud, *ros_pc_msg_ptr);
    ros_pc_msg_ptr->header.stamp =
      rclcpp::Time(SecondsToChronoNanoSeconds(std::get<1>(pointcloud_ts)).count());
    PublishCloud(std::move(ros_pc_msg_ptr), nebula_points_pub_);
  }
  if (
    aw_points_base_pub_->get_subscription_count() > 0 ||
    aw_points_base_pub_->get_intra_process_subscription_count() > 0) {
    const auto autoware_cloud_xyzi =
      nebula::drivers::convertPointXYZIRCAEDTToPointXYZIR(pointcloud);
    auto ros_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*autoware_cloud_xyzi, *ros_pc_msg_ptr);
    ros_pc_msg_ptr->header.stamp =
      rclcpp::Time(SecondsToChronoNanoSeconds(std::get<1>(pointcloud_ts)).count());
    PublishCloud(std::move(ros_pc_msg_ptr), aw_points_base_pub_);
  }
  if (
    aw_points_ex_pub_->get_subscription_count() > 0 ||
    aw_points_ex_pub_->get_intra_process_subscription_count() > 0) {
    const auto autoware_ex_cloud = nebula::drivers::convertPointXYZIRCAEDTToPointXYZIRADT(
      pointcloud, std::get<1>(pointcloud_ts));
    auto ros_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*autoware_ex_cloud, *ros_pc_msg_ptr);
    ros_pc_msg_ptr->header.stamp =
      rclcpp::Time(SecondsToChronoNanoSeconds(std::get<1>(pointcloud_ts)).count());
    PublishCloud(std::move(ros_pc_msg_ptr), aw_points_ex_pub_);
  }
}

void RobosenseDriverRosWrapper::PublishCloud(
  std::unique_ptr<sensor_msgs::msg::PointCloud2> pointcloud,
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher)
{
  if (pointcloud->header.stamp.sec < 0) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Timestamp error, verify clock source.");
  }
  pointcloud->header.frame_id = sensor_cfg_ptr_->frame_id;
  publisher->publish(std::move(pointcloud));
}

Status RobosenseDriverRosWrapper::InitializeDriver(
  std::shared_ptr<drivers::SensorConfigurationBase> sensor_configuration,
  std::shared_ptr<drivers::CalibrationConfigurationBase> calibration_configuration)
{
  driver_ptr_ = std::make_shared<drivers::RobosenseDriver>(
    std::static_pointer_cast<drivers::RobosenseSensorConfiguration>(sensor_configuration),
    std::static_pointer_cast<drivers::RobosenseCalibrationConfiguration>(
      calibration_configuration));
  return driver_ptr_->GetStatus();
}

Status RobosenseDriverRosWrapper::GetStatus()
{
  return wrapper_status_;
}

Status RobosenseDriverRosWrapper::GetParameters(
  drivers::RobosenseSensorConfiguration & sensor_configuration,
  drivers::RobosenseCalibrationConfiguration & calibration_configuration)
{
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("host_ip", "255.255.255.255", descriptor);
    sensor_configuration.host_ip = this->get_parameter("host_ip").as_string();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("lidar_ip", "192.168.1.201", descriptor);
    sensor_configuration.sensor_ip = this->get_parameter("lidar_ip").as_string();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<uint16_t>("data_port", 2368, descriptor);
    sensor_configuration.data_port = this->get_parameter("data_port").as_int();
  }

  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<uint16_t>("difop_port", 7788, descriptor);
    sensor_configuration.difop_port = this->get_parameter("difop_port").as_int();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("sensor_model", "");
    sensor_configuration.sensor_model =
      nebula::drivers::SensorModelFromString(this->get_parameter("sensor_model").as_string());
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("return_mode", "", descriptor);
    sensor_configuration.return_mode = nebula::drivers::ReturnModeFromStringRobosense(
      this->get_parameter("return_mode").as_string(), sensor_configuration.sensor_model);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("frame_id", "robosense", descriptor);
    sensor_configuration.frame_id = this->get_parameter("frame_id").as_string();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "Angle where scans begin (degrees, [0.,360.]";
    rcl_interfaces::msg::FloatingPointRange range;
    range.set__from_value(0).set__to_value(360).set__step(0.01);
    descriptor.floating_point_range = {range};
    this->declare_parameter<double>("scan_phase", 0., descriptor);
    sensor_configuration.scan_phase = this->get_parameter("scan_phase").as_double();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<uint16_t>("packet_mtu_size", 1500, descriptor);
    sensor_configuration.packet_mtu_size = this->get_parameter("packet_mtu_size").as_int();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    rcl_interfaces::msg::IntegerRange range;
    if (
      sensor_configuration.sensor_model == nebula::drivers::SensorModel::ROBOSENSE_HELIOS ||
      sensor_configuration.sensor_model == nebula::drivers::SensorModel::ROBOSENSE_HELIOS16P ||
      sensor_configuration.sensor_model == nebula::drivers::SensorModel::ROBOSENSE_BPEARL ||
      sensor_configuration.sensor_model == nebula::drivers::SensorModel::ROBOSENSE_RUBYPLUS ||
      sensor_configuration.sensor_model == nebula::drivers::SensorModel::ROBOSENSE_M1PLUS ||
      sensor_configuration.sensor_model == nebula::drivers::SensorModel::ROBOSENSE_E1) {
      descriptor.additional_constraints = "300, 600, 1200";
      range.set__from_value(300).set__to_value(1200).set__step(300);
      descriptor.integer_range = {range};
      this->declare_parameter<uint16_t>("rotation_speed", 600, descriptor);
    }
    sensor_configuration.rotation_speed = this->get_parameter("rotation_speed").as_int();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<uint16_t>("cloud_min_angle", 0, descriptor);
    sensor_configuration.cloud_min_angle = this->get_parameter("cloud_min_angle").as_int();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<uint16_t>("cloud_max_angle", 360, descriptor);
    sensor_configuration.cloud_max_angle = this->get_parameter("cloud_max_angle").as_int();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("calibration_file", "", descriptor);
    calibration_configuration.calibration_file =
      this->get_parameter("calibration_file").as_string();
  }
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  descriptor.read_only = false;
  descriptor.dynamic_typing = false;
  descriptor.additional_constraints = "Dual return distance threshold [0.01, 0.5]";
  rcl_interfaces::msg::FloatingPointRange range;
  range.set__from_value(0.01).set__to_value(0.5).set__step(0.01);
  descriptor.floating_point_range = {range};
  this->declare_parameter<double>("dual_return_distance_threshold", 0.1, descriptor);
  sensor_configuration.dual_return_distance_threshold =
    this->get_parameter("dual_return_distance_threshold").as_double();

  if (sensor_configuration.sensor_model == nebula::drivers::SensorModel::UNKNOWN) {
    return Status::INVALID_SENSOR_MODEL;
  }
  if (sensor_configuration.return_mode == nebula::drivers::ReturnMode::UNKNOWN) {
    return Status::INVALID_ECHO_MODE;
  }
  if (sensor_configuration.frame_id.empty() || sensor_configuration.scan_phase > 360) {
    return Status::SENSOR_CONFIG_ERROR;
  }

  std::shared_ptr<drivers::SensorConfigurationBase> sensor_cfg_ptr =
    std::make_shared<drivers::RobosenseSensorConfiguration>(sensor_configuration);

  hw_interface_.SetSensorConfiguration(
    std::static_pointer_cast<drivers::SensorConfigurationBase>(sensor_cfg_ptr));
  RCLCPP_INFO_STREAM(this->get_logger(), "SensorConfig:" << sensor_configuration);
  return Status::OK;
}

RCLCPP_COMPONENTS_REGISTER_NODE(RobosenseDriverRosWrapper)
}  // namespace ros
}  // namespace nebula
