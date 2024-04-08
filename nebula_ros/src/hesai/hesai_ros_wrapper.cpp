#include "nebula_ros/hesai/hesai_ros_wrapper.hpp"

namespace nebula
{
namespace ros
{
HesaiRosWrapper::HesaiRosWrapper(const rclcpp::NodeOptions & options)
: rclcpp::Node("hesai_ros_wrapper", rclcpp::NodeOptions(options).use_intra_process_comms(true)),
  hw_interface_(),
  diagnostics_updater_(this),
  packet_queue_(3000)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  drivers::HesaiSensorConfiguration sensor_configuration;
  wrapper_status_ = GetParameters(sensor_configuration);
  sensor_cfg_ptr_ = std::make_shared<drivers::HesaiSensorConfiguration>(sensor_configuration);

  // hwiface
  {
    if (Status::OK != interface_status_) {
      RCLCPP_ERROR_STREAM(this->get_logger(), this->get_name() << " Error:" << interface_status_);
      return;
    }
    hw_interface_.SetLogger(std::make_shared<rclcpp::Logger>(this->get_logger()));
    hw_interface_.SetSensorConfiguration(
      std::static_pointer_cast<drivers::SensorConfigurationBase>(sensor_cfg_ptr_));
    Status rt = hw_interface_.InitializeTcpDriver();
    if (this->retry_hw_) {
      int cnt = 0;
      RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << " Retry: " << cnt);
      while (rt == Status::ERROR_1) {
        cnt++;
        std::this_thread::sleep_for(std::chrono::milliseconds(8000));  // >5000
        RCLCPP_ERROR_STREAM(this->get_logger(), this->get_name() << " Retry: " << cnt);
        rt = hw_interface_.InitializeTcpDriver();
      }
    }

    if (rt != Status::ERROR_1) {
      try {
        std::vector<std::thread> thread_pool{};
        thread_pool.emplace_back([this] {
          auto result = hw_interface_.GetInventory();
          RCLCPP_INFO_STREAM(get_logger(), result);
          hw_interface_.SetTargetModel(result.model);
        });
        for (std::thread & th : thread_pool) {
          th.join();
        }

      } catch (...) {
        std::cout << "catch (...) in parent" << std::endl;
        RCLCPP_ERROR_STREAM(get_logger(), "Failed to get model from sensor...");
      }
      if (this->setup_sensor) {
        hw_interface_.CheckAndSetConfig();
        updateParameters();
      }
    } else {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "Failed to get model from sensor... Set from config: " << sensor_cfg_ptr_->sensor_model);
      hw_interface_.SetTargetModel(sensor_cfg_ptr_->sensor_model);
    }
  }
  // decoder
  {
    drivers::HesaiCalibrationConfiguration calibration_configuration;
    drivers::HesaiCorrection correction_configuration;

    wrapper_status_ = GetCalibrationData(calibration_configuration, correction_configuration);
    if (Status::OK != wrapper_status_) {
      RCLCPP_ERROR_STREAM(this->get_logger(), this->get_name() << " Error:" << wrapper_status_);
      return;
    }
    RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << ". Starting...");

    calibration_cfg_ptr_ =
      std::make_shared<drivers::HesaiCalibrationConfiguration>(calibration_configuration);
    // Will be left empty for AT128, and ignored by all decoders except AT128
    correction_cfg_ptr_ = std::make_shared<drivers::HesaiCorrection>(correction_configuration);

    RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << ". Driver ");
    wrapper_status_ = InitializeCloudDriver(
      std::static_pointer_cast<drivers::SensorConfigurationBase>(sensor_cfg_ptr_),
      std::static_pointer_cast<drivers::CalibrationConfigurationBase>(calibration_cfg_ptr_),
      std::static_pointer_cast<drivers::HesaiCorrection>(correction_cfg_ptr_));

    RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << ". Wrapper=" << wrapper_status_);
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto pointcloud_qos =
      rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);

    decoder_thread_ = std::thread([this](){
      while(true) {
        auto pkt = packet_queue_.pop();
        this->ProcessCloudPacket(std::move(pkt));
      }
    });

    nebula_points_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("pandar_points", pointcloud_qos);
    aw_points_base_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("aw_points", pointcloud_qos);
    aw_points_ex_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("aw_points_ex", pointcloud_qos);
  }

  RCLCPP_DEBUG(this->get_logger(), "Starting stream");
  StreamStart();
  hw_interface_.RegisterScanCallback(
    std::bind(&HesaiRosWrapper::ReceiveCloudPacketCallback, this, std::placeholders::_1));

  InitializeHwMonitor(*sensor_cfg_ptr_);

  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&HesaiRosWrapper::paramCallback, this, std::placeholders::_1));
}

void HesaiRosWrapper::ReceiveCloudPacketCallback(std::vector<uint8_t> & packet)
{
  // static auto convert = nebula::util::Instrumentation("ReceiveCloudPacketCallback.convert");
  // static auto publish = nebula::util::Instrumentation("ReceiveCloudPacketCallback.publish");
  // static auto delay = nebula::util::TopicDelay("ReceiveCloudPacketCallback");
  // Driver is not initialized yet
  if (!driver_ptr_) {
    return;
  }

  // convert.tick();
  const auto now = std::chrono::system_clock::now();
  const auto timestamp_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();

  auto msg_ptr = std::make_unique<nebula_msgs::msg::NebulaPacket>();
  msg_ptr->stamp.sec = static_cast<int>(timestamp_ns / 1'000'000'000);
  msg_ptr->stamp.nanosec = static_cast<int>(timestamp_ns % 1'000'000'000);
  msg_ptr->data.swap(packet);
  // convert.tock();

  // delay.tick(msg_ptr->stamp);

  // publish.tick();
  if (!packet_queue_.push(std::move(msg_ptr))) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 500, "Packets dropped");
  }
  // packet_pub_->publish(std::move(msg_ptr));
  // publish.tock();
}

void HesaiRosWrapper::ProcessCloudPacket(std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg)
{
  // static auto parse = nebula::util::Instrumentation("ProcessCloudPacket.parse");
  // static auto convert = nebula::util::Instrumentation("ProcessCloudPacket.convert");
  // static auto publish = nebula::util::Instrumentation("ProcessCloudPacket.publish");
  // static auto delay = nebula::util::TopicDelay("ProcessCloudPacket");

  // delay.tick(packet_msg->stamp);

  // parse.tick();
  std::tuple<nebula::drivers::NebulaPointCloudPtr, double> pointcloud_ts =
    driver_ptr_->ParseCloudPacket(packet_msg->data);
  nebula::drivers::NebulaPointCloudPtr pointcloud = std::get<0>(pointcloud_ts);
  // parse.tock();

  if (pointcloud == nullptr) {
    // todo
    // RCLCPP_WARN_STREAM(get_logger(), "Empty cloud parsed.");
    return;
  };
  if (
    nebula_points_pub_->get_subscription_count() > 0 ||
    nebula_points_pub_->get_intra_process_subscription_count() > 0) {
    // convert.tick();
    auto ros_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*pointcloud, *ros_pc_msg_ptr);
    ros_pc_msg_ptr->header.stamp =
      rclcpp::Time(SecondsToChronoNanoSeconds(std::get<1>(pointcloud_ts)).count());
    // convert.tock();
    // publish.tick();
    PublishCloud(std::move(ros_pc_msg_ptr), nebula_points_pub_);
    // publish.tock();
  }
  if (
    aw_points_base_pub_->get_subscription_count() > 0 ||
    aw_points_base_pub_->get_intra_process_subscription_count() > 0) {
    // convert.tick();

    const auto autoware_cloud_xyzi =
      nebula::drivers::convertPointXYZIRCAEDTToPointXYZIR(pointcloud);
    auto ros_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*autoware_cloud_xyzi, *ros_pc_msg_ptr);
    ros_pc_msg_ptr->header.stamp =
      rclcpp::Time(SecondsToChronoNanoSeconds(std::get<1>(pointcloud_ts)).count());
    // convert.tock();
    // publish.tick();
    PublishCloud(std::move(ros_pc_msg_ptr), aw_points_base_pub_);
    // publish.tock();
  }
  if (
    aw_points_ex_pub_->get_subscription_count() > 0 ||
    aw_points_ex_pub_->get_intra_process_subscription_count() > 0) {
    // convert.tick();

    const auto autoware_ex_cloud = nebula::drivers::convertPointXYZIRCAEDTToPointXYZIRADT(
      pointcloud, std::get<1>(pointcloud_ts));
    auto ros_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*autoware_ex_cloud, *ros_pc_msg_ptr);
    ros_pc_msg_ptr->header.stamp =
      rclcpp::Time(SecondsToChronoNanoSeconds(std::get<1>(pointcloud_ts)).count());
    // convert.tock();
    // publish.tick();
    PublishCloud(std::move(ros_pc_msg_ptr), aw_points_ex_pub_);
    // publish.tock();
  }
}

void HesaiRosWrapper::PublishCloud(
  std::unique_ptr<sensor_msgs::msg::PointCloud2> pointcloud,
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher)
{
  if (pointcloud->header.stamp.sec < 0) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Timestamp error, verify clock source.");
  }
  pointcloud->header.frame_id = sensor_cfg_ptr_->frame_id;
  publisher->publish(std::move(pointcloud));
}

Status HesaiRosWrapper::InitializeCloudDriver(
  const std::shared_ptr<drivers::SensorConfigurationBase> & sensor_configuration,
  const std::shared_ptr<drivers::CalibrationConfigurationBase> & calibration_configuration,
  const std::shared_ptr<drivers::HesaiCorrection> & correction_configuration)
{
  driver_ptr_ = std::make_shared<drivers::HesaiDriver>(
    std::static_pointer_cast<drivers::HesaiSensorConfiguration>(sensor_configuration),
    std::static_pointer_cast<drivers::HesaiCalibrationConfiguration>(calibration_configuration),
    std::static_pointer_cast<drivers::HesaiCorrection>(correction_configuration));
  return driver_ptr_->GetStatus();
}

Status HesaiRosWrapper::GetStatus()
{
  return wrapper_status_;
}

Status HesaiRosWrapper::GetParameters(drivers::HesaiSensorConfiguration & sensor_configuration)
{
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
    sensor_configuration.return_mode = nebula::drivers::ReturnModeFromStringHesai(
      this->get_parameter("return_mode").as_string(), sensor_configuration.sensor_model);
  }
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
    this->declare_parameter<std::string>("sensor_ip", "192.168.1.201", descriptor);
    sensor_configuration.sensor_ip = this->get_parameter("sensor_ip").as_string();
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
    this->declare_parameter<uint16_t>("gnss_port", 2369, descriptor);
    sensor_configuration.gnss_port = this->get_parameter("gnss_port").as_int();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("frame_id", "pandar", descriptor);
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
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("calibration_file", "", descriptor);
    calibration_file_path = this->get_parameter("calibration_file").as_string();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<double>("min_range", 0.3, descriptor);
    sensor_configuration.min_range = this->get_parameter("min_range").as_double();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<double>("max_range", 300., descriptor);
    sensor_configuration.max_range = this->get_parameter("max_range").as_double();
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
    RCLCPP_DEBUG_STREAM(get_logger(), sensor_configuration.sensor_model);
    if (sensor_configuration.sensor_model == nebula::drivers::SensorModel::HESAI_PANDARAT128) {
      descriptor.additional_constraints = "200, 300, 400, 500";
      // range.set__from_value(200).set__to_value(500).set__step(100);
      // descriptor.integer_range = {range}; //todo
      this->declare_parameter<uint16_t>("rotation_speed", 200, descriptor);
    } else {
      descriptor.additional_constraints = "300, 600, 1200";
      // range.set__from_value(300).set__to_value(1200).set__step(300);
      // descriptor.integer_range = {range}; //todo
      this->declare_parameter<uint16_t>("rotation_speed", 600, descriptor);
    }
    sensor_configuration.rotation_speed = this->get_parameter("rotation_speed").as_int();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    rcl_interfaces::msg::IntegerRange range;
    range.set__from_value(0).set__to_value(360).set__step(1);
    descriptor.integer_range = {range};
    this->declare_parameter<uint16_t>("cloud_min_angle", 0, descriptor);
    sensor_configuration.cloud_min_angle = this->get_parameter("cloud_min_angle").as_int();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    rcl_interfaces::msg::IntegerRange range;
    range.set__from_value(0).set__to_value(360).set__step(1);
    descriptor.integer_range = {range};
    this->declare_parameter<uint16_t>("cloud_max_angle", 360, descriptor);
    sensor_configuration.cloud_max_angle = this->get_parameter("cloud_max_angle").as_int();
  }
  if (sensor_configuration.sensor_model == drivers::SensorModel::HESAI_PANDARAT128) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("correction_file", "", descriptor);
    correction_file_path = this->get_parameter("correction_file").as_string();
  }
  {
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
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<bool>("launch_hw", "", descriptor);
    launch_hw_ = this->get_parameter("launch_hw").as_bool();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<bool>("setup_sensor", true, descriptor);
    this->setup_sensor = this->get_parameter("setup_sensor").as_bool();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<bool>("retry_hw", true, descriptor);
    this->retry_hw_ = this->get_parameter("retry_hw").as_bool();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("ptp_profile", "");
    sensor_configuration.ptp_profile =
      nebula::drivers::PtpProfileFromString(this->get_parameter("ptp_profile").as_string());
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("ptp_transport_type", "");
    sensor_configuration.ptp_transport_type = nebula::drivers::PtpTransportTypeFromString(
      this->get_parameter("ptp_transport_type").as_string());
    if (static_cast<int>(sensor_configuration.ptp_profile) > 0) {
      sensor_configuration.ptp_transport_type = nebula::drivers::PtpTransportType::L2;
    }
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("ptp_switch_type", "");
    sensor_configuration.ptp_switch_type =
      nebula::drivers::PtpSwitchTypeFromString(this->get_parameter("ptp_switch_type").as_string());
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    rcl_interfaces::msg::IntegerRange range;
    range.set__from_value(0).set__to_value(127).set__step(1);
    descriptor.integer_range = {range};
    this->declare_parameter<uint8_t>("ptp_domain", 0, descriptor);
    sensor_configuration.ptp_domain = this->get_parameter("ptp_domain").as_int();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "milliseconds";
    this->declare_parameter<uint16_t>("diag_span", 1000, descriptor);
    this->diag_span_ = this->get_parameter("diag_span").as_int();
  }

  ///////////////////////////////////////////////
  // Validate ROS parameters
  ///////////////////////////////////////////////

  if (sensor_configuration.ptp_profile == nebula::drivers::PtpProfile::PROFILE_UNKNOWN) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "Invalid PTP Profile Provided. Please use '1588v2', '802.1as' or 'automotive'");
    return Status::SENSOR_CONFIG_ERROR;
  }
  if (
    sensor_configuration.ptp_transport_type ==
    nebula::drivers::PtpTransportType::UNKNOWN_TRANSPORT) {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "Invalid PTP Transport Provided. Please use 'udp' or 'l2', 'udp' is only available when "
      "using the '1588v2' PTP Profile");
    return Status::SENSOR_CONFIG_ERROR;
  }
  if (sensor_configuration.ptp_switch_type == nebula::drivers::PtpSwitchType::UNKNOWN_SWITCH) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "Invalid PTP Switch Type Provided. Please use 'tsn' or 'non_tsn'");
    return Status::SENSOR_CONFIG_ERROR;
  }
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
    std::make_shared<drivers::HesaiSensorConfiguration>(sensor_configuration);

  hw_interface_.SetSensorConfiguration(
    std::static_pointer_cast<drivers::SensorConfigurationBase>(sensor_cfg_ptr));

  RCLCPP_INFO_STREAM(this->get_logger(), "SensorConfig:" << sensor_configuration);
  return Status::OK;
}

Status HesaiRosWrapper::GetCalibrationData(
  drivers::HesaiCalibrationConfiguration & calibration_configuration,
  drivers::HesaiCorrection & correction_configuration)
{
  calibration_configuration.calibration_file = calibration_file_path;  // todo

  bool run_local = !launch_hw_;
  if (sensor_cfg_ptr_->sensor_model != drivers::SensorModel::HESAI_PANDARAT128) {
    std::string calibration_file_path_from_sensor;
    if (launch_hw_ && !calibration_configuration.calibration_file.empty()) {
      int ext_pos = calibration_configuration.calibration_file.find_last_of('.');
      calibration_file_path_from_sensor +=
        calibration_configuration.calibration_file.substr(0, ext_pos);
      calibration_file_path_from_sensor += "_from_sensor";
      calibration_file_path_from_sensor += calibration_configuration.calibration_file.substr(
        ext_pos, calibration_configuration.calibration_file.size() - ext_pos);
    }
    if (launch_hw_) {
      run_local = false;
      RCLCPP_INFO_STREAM(
        this->get_logger(),
        "Trying to acquire calibration data from sensor: '" << sensor_cfg_ptr_->sensor_ip << "'");
      std::future<void> future = std::async(
        std::launch::async,
        [this, &calibration_configuration, &calibration_file_path_from_sensor, &run_local]() {
          auto str = hw_interface_.GetLidarCalibrationString();
          auto rt =
            calibration_configuration.SaveFileFromString(calibration_file_path_from_sensor, str);
          RCLCPP_ERROR_STREAM(get_logger(), str);
          if (rt == Status::OK) {
            RCLCPP_INFO_STREAM(
              get_logger(),
              "SaveFileFromString success:" << calibration_file_path_from_sensor << "\n");
          } else {
            RCLCPP_ERROR_STREAM(
              get_logger(),
              "SaveFileFromString failed:" << calibration_file_path_from_sensor << "\n");
          }
          rt = calibration_configuration.LoadFromString(str);
          if (rt == Status::OK) {
            RCLCPP_INFO_STREAM(get_logger(), "LoadFromString success:" << str << "\n");
          } else {
            RCLCPP_ERROR_STREAM(get_logger(), "LoadFromString failed:" << str << "\n");
          }
        });
      std::future_status status;
      status = future.wait_for(std::chrono::milliseconds(5000));
      if (status == std::future_status::timeout) {
        std::cerr << "# std::future_status::timeout\n";
        RCLCPP_ERROR_STREAM(get_logger(), "GetCalibration Timeout");
        run_local = true;
      } else if (status == std::future_status::ready && !run_local) {
        RCLCPP_INFO_STREAM(
          this->get_logger(),
          "Acquired calibration data from sensor: '" << sensor_cfg_ptr_->sensor_ip << "'");
        RCLCPP_INFO_STREAM(
          this->get_logger(),
          "The calibration has been saved in '" << calibration_file_path_from_sensor << "'");
      }
    }
    if (run_local) {
      RCLCPP_WARN_STREAM(get_logger(), "Running locally");
      bool run_org = false;
      if (calibration_file_path_from_sensor.empty()) {
        run_org = true;
      } else {
        RCLCPP_INFO_STREAM(
          get_logger(), "Trying to load file: " << calibration_file_path_from_sensor);
        auto cal_status = calibration_configuration.LoadFromFile(calibration_file_path_from_sensor);

        if (cal_status != Status::OK) {
          run_org = true;
        } else {
          RCLCPP_INFO_STREAM(
            this->get_logger(),
            "Load calibration data from: '" << calibration_file_path_from_sensor << "'");
        }
      }
      if (run_org) {
        RCLCPP_INFO_STREAM(
          get_logger(), "Trying to load file: " << calibration_configuration.calibration_file);
        if (calibration_configuration.calibration_file.empty()) {
          RCLCPP_ERROR_STREAM(
            this->get_logger(),
            "Empty Calibration_file File: '" << calibration_configuration.calibration_file << "'");
          return Status::INVALID_CALIBRATION_FILE;
        } else {
          auto cal_status =
            calibration_configuration.LoadFromFile(calibration_configuration.calibration_file);

          if (cal_status != Status::OK) {
            RCLCPP_ERROR_STREAM(
              this->get_logger(),
              "Given Calibration File: '" << calibration_configuration.calibration_file << "'");
            return cal_status;
          } else {
            RCLCPP_INFO_STREAM(
              this->get_logger(),
              "Load calibration data from: '" << calibration_configuration.calibration_file << "'");
          }
        }
      }
    }
  } else {  // sensor_configuration.sensor_model == drivers::SensorModel::HESAI_PANDARAT128
    std::string correction_file_path_from_sensor;
    if (launch_hw_ && !correction_file_path.empty()) {
      int ext_pos = correction_file_path.find_last_of('.');
      correction_file_path_from_sensor += correction_file_path.substr(0, ext_pos);
      correction_file_path_from_sensor += "_from_sensor";
      correction_file_path_from_sensor +=
        correction_file_path.substr(ext_pos, correction_file_path.size() - ext_pos);
    }
    std::future<void> future = std::async(
      std::launch::async,
      [this, &correction_configuration, &correction_file_path_from_sensor, &run_local]() {
        if (launch_hw_) {
          RCLCPP_INFO_STREAM(this->get_logger(), "Trying to acquire calibration data from sensor");
          auto received_bytes = hw_interface_.GetLidarCalibrationBytes();
          RCLCPP_INFO_STREAM(
            get_logger(), "AT128 calibration size:" << received_bytes.size() << "\n");
          auto rt = correction_configuration.SaveFileFromBinary(
            correction_file_path_from_sensor, received_bytes);
          if (rt == Status::OK) {
            RCLCPP_INFO_STREAM(
              get_logger(),
              "SaveFileFromBinary success:" << correction_file_path_from_sensor << "\n");
          } else {
            RCLCPP_ERROR_STREAM(
              get_logger(),
              "SaveFileFromBinary failed:" << correction_file_path_from_sensor
                                           << ". Falling back to offline calibration file.");
            run_local = true;
          }
          rt = correction_configuration.LoadFromBinary(received_bytes);
          if (rt == Status::OK) {
            RCLCPP_INFO_STREAM(get_logger(), "LoadFromBinary success" << "\n");
            run_local = false;
          } else {
            RCLCPP_ERROR_STREAM(
              get_logger(),
              "LoadFromBinary failed" << ". Falling back to offline calibration file.");
            run_local = true;
          }
        } else {
          RCLCPP_ERROR_STREAM(get_logger(), "Falling back to offline calibration file.");
          run_local = true;
        }
      });
    if (!run_local) {
      std::future_status status;
      status = future.wait_for(std::chrono::milliseconds(8000));
      if (status == std::future_status::timeout) {
        std::cerr << "# std::future_status::timeout\n";
        run_local = true;
      } else if (status == std::future_status::ready && !run_local) {
        RCLCPP_INFO_STREAM(
          this->get_logger(),
          "Acquired correction data from sensor: '" << sensor_cfg_ptr_->sensor_ip << "'");
        RCLCPP_INFO_STREAM(
          this->get_logger(),
          "The correction has been saved in '" << correction_file_path_from_sensor << "'");
      }
    }
    if (run_local) {
      bool run_org = false;
      if (correction_file_path_from_sensor.empty()) {
        run_org = true;
      } else {
        auto cal_status = correction_configuration.LoadFromFile(correction_file_path_from_sensor);

        if (cal_status != Status::OK) {
          run_org = true;
        } else {
          RCLCPP_INFO_STREAM(
            this->get_logger(),
            "Load correction data from: '" << correction_file_path_from_sensor << "'");
        }
      }
      if (run_org) {
        if (correction_file_path.empty()) {
          RCLCPP_ERROR_STREAM(
            this->get_logger(), "Empty Correction File: '" << correction_file_path << "'");
          return Status::INVALID_CALIBRATION_FILE;
        } else {
          auto cal_status = correction_configuration.LoadFromFile(correction_file_path);

          if (cal_status != Status::OK) {
            RCLCPP_ERROR_STREAM(
              this->get_logger(), "Given Correction File: '" << correction_file_path << "'");
            return cal_status;
          } else {
            RCLCPP_INFO_STREAM(
              this->get_logger(), "Load correction data from: '" << correction_file_path << "'");
          }
        }
      }
    }
  }  // end AT128

  return Status::OK;
}

HesaiRosWrapper::~HesaiRosWrapper()
{
  RCLCPP_INFO_STREAM(get_logger(), "Closing TcpDriver");
  hw_interface_.FinalizeTcpDriver();
}

Status HesaiRosWrapper::StreamStart()
{
  if (Status::OK == interface_status_) {
    interface_status_ = hw_interface_.SensorInterfaceStart();
  }
  return interface_status_;
}

Status HesaiRosWrapper::StreamStop()
{
  return Status::OK;
}
Status HesaiRosWrapper::Shutdown()
{
  return Status::OK;
}

Status HesaiRosWrapper::InitializeHwInterface(  // todo: don't think this is needed
  const drivers::SensorConfigurationBase & sensor_configuration)
{
  std::stringstream ss;
  ss << sensor_configuration;
  RCLCPP_DEBUG_STREAM(this->get_logger(), ss.str());
  return Status::OK;
}

Status HesaiRosWrapper::InitializeHwMonitor(  // todo: don't think this is needed
  const drivers::SensorConfigurationBase & /* sensor_configuration */)
{
  cbg_r_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  cbg_m_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbg_m2_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  if (Status::OK != interface_status_) {
    RCLCPP_ERROR_STREAM(this->get_logger(), this->get_name() << " Error:" << interface_status_);
    return Status::ERROR_1;
  }

  message_sep = ": ";
  not_supported_message = "Not supported";
  error_message = "Error";

  switch (sensor_cfg_ptr_->sensor_model) {
    case nebula::drivers::SensorModel::HESAI_PANDARXT32:
    case nebula::drivers::SensorModel::HESAI_PANDARXT32M:
    case nebula::drivers::SensorModel::HESAI_PANDARAT128:
      temperature_names.emplace_back("Bottom circuit board T1");
      temperature_names.emplace_back("Bottom circuit board T2");
      temperature_names.emplace_back("Laser emitting board RT_L1(Internal)");
      temperature_names.emplace_back("Laser emitting board RT_L2");
      temperature_names.emplace_back("Receiving board RT_R");
      temperature_names.emplace_back("Receiving board RT2");
      temperature_names.emplace_back("Top circuit RT3");
      temperature_names.emplace_back("Not used");
      break;
    case nebula::drivers::SensorModel::HESAI_PANDAR64:
    case nebula::drivers::SensorModel::HESAI_PANDAR40P:
    case nebula::drivers::SensorModel::HESAI_PANDAR40M:
    case nebula::drivers::SensorModel::HESAI_PANDARQT64:
    case nebula::drivers::SensorModel::HESAI_PANDARQT128:
    case nebula::drivers::SensorModel::HESAI_PANDAR128_E3X:
    case nebula::drivers::SensorModel::HESAI_PANDAR128_E4X:
    default:
      temperature_names.emplace_back("Bottom circuit RT1");
      temperature_names.emplace_back("Bottom circuit RT2");
      temperature_names.emplace_back("Internal Temperature");
      temperature_names.emplace_back("Laser emitting board RT1");
      temperature_names.emplace_back("Laser emitting board RT2");
      temperature_names.emplace_back("Receiving board RT1");
      temperature_names.emplace_back("Top circuit RT1");
      temperature_names.emplace_back("Top circuit RT2");
      break;
  }

  auto result = hw_interface_.GetInventory();
  current_inventory.reset(new HesaiInventory(result));
  current_inventory_time.reset(new rclcpp::Time(this->get_clock()->now()));
  std::cout << "HesaiInventory" << std::endl;
  std::cout << result << std::endl;
  info_model = result.get_str_model();
  info_serial = std::string(result.sn.begin(), result.sn.end());
  hw_interface_.SetTargetModel(result.model);
  RCLCPP_INFO_STREAM(this->get_logger(), "Model:" << info_model);
  RCLCPP_INFO_STREAM(this->get_logger(), "Serial:" << info_serial);
  InitializeHesaiDiagnostics();
  return Status::OK;
}

rcl_interfaces::msg::SetParametersResult HesaiRosWrapper::paramCallback(
  const std::vector<rclcpp::Parameter> & p)
{
  std::scoped_lock lock(mtx_config_);
  RCLCPP_DEBUG_STREAM(this->get_logger(), "add_on_set_parameters_callback");
  RCLCPP_DEBUG_STREAM(this->get_logger(), p);
  RCLCPP_DEBUG_STREAM(this->get_logger(), *sensor_cfg_ptr_);
  RCLCPP_INFO_STREAM(this->get_logger(), p);

  std::shared_ptr<drivers::HesaiSensorConfiguration> new_param =
    std::make_shared<drivers::HesaiSensorConfiguration>(*sensor_cfg_ptr_);
  RCLCPP_INFO_STREAM(this->get_logger(), new_param);
  std::string sensor_model_str;
  std::string return_mode_str;
  if (
    get_param(p, "sensor_model", sensor_model_str) | get_param(p, "return_mode", return_mode_str) |
    get_param(p, "host_ip", new_param->host_ip) | get_param(p, "sensor_ip", new_param->sensor_ip) |
    get_param(p, "frame_id", new_param->frame_id) |
    get_param(p, "data_port", new_param->data_port) |
    get_param(p, "gnss_port", new_param->gnss_port) |
    get_param(p, "scan_phase", new_param->scan_phase) |
    get_param(p, "packet_mtu_size", new_param->packet_mtu_size) |
    get_param(p, "rotation_speed", new_param->rotation_speed) |
    get_param(p, "cloud_min_angle", new_param->cloud_min_angle) |
    get_param(p, "cloud_max_angle", new_param->cloud_max_angle) |
    get_param(p, "dual_return_distance_threshold", new_param->dual_return_distance_threshold)) {
    if (0 < sensor_model_str.length())
      new_param->sensor_model = nebula::drivers::SensorModelFromString(sensor_model_str);
    if (0 < return_mode_str.length())
      new_param->return_mode = nebula::drivers::ReturnModeFromString(return_mode_str);

    sensor_cfg_ptr_.swap(new_param);
    RCLCPP_INFO_STREAM(this->get_logger(), "Update sensor_configuration");
    RCLCPP_DEBUG_STREAM(this->get_logger(), "hw_interface_.SetSensorConfiguration");
    hw_interface_.SetSensorConfiguration(
      std::static_pointer_cast<drivers::SensorConfigurationBase>(sensor_cfg_ptr_));
    hw_interface_.CheckAndSetConfig();
  }

  auto result = std::make_shared<rcl_interfaces::msg::SetParametersResult>();
  result->successful = true;
  result->reason = "success";

  RCLCPP_DEBUG_STREAM(this->get_logger(), "add_on_set_parameters_callback success");

  return *result;
}

std::vector<rcl_interfaces::msg::SetParametersResult> HesaiRosWrapper::updateParameters()
{
  std::scoped_lock lock(mtx_config_);
  RCLCPP_DEBUG_STREAM(this->get_logger(), "updateParameters start");
  std::ostringstream os_sensor_model;
  os_sensor_model << sensor_cfg_ptr_->sensor_model;
  std::ostringstream os_return_mode;
  os_return_mode << sensor_cfg_ptr_->return_mode;
  RCLCPP_INFO_STREAM(this->get_logger(), "set_parameters");
  auto results = set_parameters(
    {rclcpp::Parameter("sensor_model", os_sensor_model.str()),
     rclcpp::Parameter("return_mode", os_return_mode.str()),
     rclcpp::Parameter("host_ip", sensor_cfg_ptr_->host_ip),
     rclcpp::Parameter("sensor_ip", sensor_cfg_ptr_->sensor_ip),
     rclcpp::Parameter("frame_id", sensor_cfg_ptr_->frame_id),
     rclcpp::Parameter("data_port", sensor_cfg_ptr_->data_port),
     rclcpp::Parameter("gnss_port", sensor_cfg_ptr_->gnss_port),
     rclcpp::Parameter("scan_phase", sensor_cfg_ptr_->scan_phase),
     rclcpp::Parameter("packet_mtu_size", sensor_cfg_ptr_->packet_mtu_size),
     rclcpp::Parameter("rotation_speed", sensor_cfg_ptr_->rotation_speed),
     rclcpp::Parameter("cloud_min_angle", sensor_cfg_ptr_->cloud_min_angle),
     rclcpp::Parameter("cloud_max_angle", sensor_cfg_ptr_->cloud_max_angle),
     rclcpp::Parameter(
       "dual_return_distance_threshold", sensor_cfg_ptr_->dual_return_distance_threshold)});
  RCLCPP_DEBUG_STREAM(this->get_logger(), "updateParameters end");
  return results;
}

void HesaiRosWrapper::InitializeHesaiDiagnostics()
{
  RCLCPP_INFO_STREAM(this->get_logger(), "InitializeHesaiDiagnostics");
  using std::chrono_literals::operator""s;
  std::ostringstream os;
  auto hardware_id = info_model + ": " + info_serial;
  diagnostics_updater_.setHardwareID(hardware_id);
  RCLCPP_INFO_STREAM(this->get_logger(), "hardware_id: " + hardware_id);

  diagnostics_updater_.add("hesai_status", this, &HesaiRosWrapper::HesaiCheckStatus);
  diagnostics_updater_.add("hesai_ptp", this, &HesaiRosWrapper::HesaiCheckPtp);
  diagnostics_updater_.add("hesai_temperature", this, &HesaiRosWrapper::HesaiCheckTemperature);
  diagnostics_updater_.add("hesai_rpm", this, &HesaiRosWrapper::HesaiCheckRpm);

  current_status.reset();
  current_monitor.reset();
  current_status_time.reset(new rclcpp::Time(this->get_clock()->now()));
  current_lidar_monitor_time.reset(new rclcpp::Time(this->get_clock()->now()));
  current_diag_status = diagnostic_msgs::msg::DiagnosticStatus::STALE;
  current_monitor_status = diagnostic_msgs::msg::DiagnosticStatus::STALE;

  auto fetch_diag_from_sensor = [this]() {
    OnHesaiStatusTimer();
    if (hw_interface_.UseHttpGetLidarMonitor()) {
      OnHesaiLidarMonitorTimerHttp();
    } else {
      OnHesaiLidarMonitorTimer();
    }
  };

  fetch_diagnostics_timer_ =
    std::make_shared<rclcpp::GenericTimer<decltype(fetch_diag_from_sensor)>>(
      this->get_clock(), std::chrono::milliseconds(diag_span_), std::move(fetch_diag_from_sensor),
      this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(fetch_diagnostics_timer_, cbg_m_);

  if (hw_interface_.UseHttpGetLidarMonitor()) {
    diagnostics_updater_.add("hesai_voltage", this, &HesaiRosWrapper::HesaiCheckVoltageHttp);
  } else {
    diagnostics_updater_.add("hesai_voltage", this, &HesaiRosWrapper::HesaiCheckVoltage);
  }

  auto on_timer_update = [this] {
    RCLCPP_DEBUG_STREAM(get_logger(), "OnUpdateTimer");
    auto now = this->get_clock()->now();
    auto dif = (now - *current_status_time).seconds();

    RCLCPP_DEBUG_STREAM(get_logger(), "dif(status): " << dif);

    if (diag_span_ * 2.0 < dif * 1000) {
      current_diag_status = diagnostic_msgs::msg::DiagnosticStatus::STALE;
      RCLCPP_DEBUG_STREAM(get_logger(), "STALE");
    } else {
      current_diag_status = diagnostic_msgs::msg::DiagnosticStatus::OK;
      RCLCPP_DEBUG_STREAM(get_logger(), "OK");
    }
    dif = (now - *current_lidar_monitor_time).seconds();
    RCLCPP_DEBUG_STREAM(get_logger(), "dif(monitor): " << dif);
    if (diag_span_ * 2.0 < dif * 1000) {
      current_monitor_status = diagnostic_msgs::msg::DiagnosticStatus::STALE;
      RCLCPP_DEBUG_STREAM(get_logger(), "STALE");
    } else {
      current_monitor_status = diagnostic_msgs::msg::DiagnosticStatus::OK;
      RCLCPP_DEBUG_STREAM(get_logger(), "OK");
    }
    diagnostics_updater_.force_update();
  };
  diagnostics_update_timer_ = std::make_shared<rclcpp::GenericTimer<decltype(on_timer_update)>>(
    this->get_clock(), std::chrono::milliseconds(1000), std::move(on_timer_update),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(diagnostics_update_timer_, cbg_r_);

  RCLCPP_DEBUG_STREAM(get_logger(), "add_timer");
}

std::string HesaiRosWrapper::GetPtreeValue(
  boost::property_tree::ptree * pt, const std::string & key)
{
  boost::optional<std::string> value = pt->get_optional<std::string>(key);
  if (value) {
    return value.get();
  } else {
    return not_supported_message;
  }
}
std::string HesaiRosWrapper::GetFixedPrecisionString(double val, int pre)
{
  std::stringstream ss;
  ss << std::fixed << std::setprecision(pre) << val;
  return ss.str();
}

void HesaiRosWrapper::OnHesaiStatusTimer()
{
  RCLCPP_DEBUG_STREAM(this->get_logger(), "OnHesaiStatusTimer" << std::endl);
  try {
    auto result = hw_interface_.GetLidarStatus();
    std::scoped_lock lock(mtx_status);
    current_status_time.reset(new rclcpp::Time(this->get_clock()->now()));
    current_status.reset(new HesaiLidarStatus(result));
  } catch (const std::system_error & error) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("HesaiRosWrapper::OnHesaiStatusTimer(std::system_error)"), error.what());
  } catch (const boost::system::system_error & error) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("HesaiRosWrapper::OnHesaiStatusTimer(boost::system::system_error)"),
      error.what());
  }
  RCLCPP_DEBUG_STREAM(this->get_logger(), "OnHesaiStatusTimer END" << std::endl);
}

void HesaiRosWrapper::OnHesaiLidarMonitorTimerHttp()
{
  RCLCPP_DEBUG_STREAM(this->get_logger(), "OnHesaiLidarMonitorTimerHttp");
  try {
    hw_interface_.GetLidarMonitorAsyncHttp([this](const std::string & str) {
      std::scoped_lock lock(mtx_lidar_monitor);
      current_lidar_monitor_time.reset(new rclcpp::Time(this->get_clock()->now()));
      current_lidar_monitor_tree =
        std::make_unique<boost::property_tree::ptree>(hw_interface_.ParseJson(str));
    });
  } catch (const std::system_error & error) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("HesaiRosWrapper::OnHesaiLidarMonitorTimerHttp(std::system_error)"),
      error.what());
  } catch (const boost::system::system_error & error) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger(
        "HesaiRosWrapper::OnHesaiLidarMonitorTimerHttp(boost::system::system_error)"),
      error.what());
  }
  RCLCPP_DEBUG_STREAM(this->get_logger(), "OnHesaiLidarMonitorTimerHttp END");
}

void HesaiRosWrapper::OnHesaiLidarMonitorTimer()
{
  RCLCPP_DEBUG_STREAM(this->get_logger(), "OnHesaiLidarMonitorTimer");
  try {
    auto ios = std::make_shared<boost::asio::io_service>();
    auto result = hw_interface_.GetLidarMonitor();
    std::scoped_lock lock(mtx_lidar_monitor);
    current_lidar_monitor_time.reset(new rclcpp::Time(this->get_clock()->now()));
    current_monitor.reset(new HesaiLidarMonitor(result));
  } catch (const std::system_error & error) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("HesaiRosWrapper::OnHesaiLidarMonitorTimer(std::system_error)"),
      error.what());
  } catch (const boost::system::system_error & error) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("HesaiRosWrapper::OnHesaiLidarMonitorTimer(boost::system::system_error)"),
      error.what());
  }
  RCLCPP_DEBUG_STREAM(this->get_logger(), "OnHesaiLidarMonitorTimer END");
}

void HesaiRosWrapper::HesaiCheckStatus(diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  std::scoped_lock lock(mtx_status);
  if (current_status) {
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::vector<std::string> msg;

    diagnostics.add("system_uptime", std::to_string(current_status->system_uptime));
    diagnostics.add("startup_times", std::to_string(current_status->startup_times));
    diagnostics.add("total_operation_time", std::to_string(current_status->total_operation_time));

    diagnostics.summary(level, boost::algorithm::join(msg, ", "));
  } else {
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No data available");
  }
}

void HesaiRosWrapper::HesaiCheckPtp(diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  std::scoped_lock lock(mtx_status);
  if (current_status) {
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::vector<std::string> msg;
    auto gps_status = current_status->get_str_gps_pps_lock();
    auto gprmc_status = current_status->get_str_gps_gprmc_status();
    auto ptp_status = current_status->get_str_ptp_clock_status();
    std::transform(gps_status.cbegin(), gps_status.cend(), gps_status.begin(), toupper);
    std::transform(gprmc_status.cbegin(), gprmc_status.cend(), gprmc_status.begin(), toupper);
    std::transform(ptp_status.cbegin(), ptp_status.cend(), ptp_status.begin(), toupper);
    diagnostics.add("gps_pps_lock", gps_status);
    diagnostics.add("gps_gprmc_status", gprmc_status);
    diagnostics.add("ptp_clock_status", ptp_status);
    if (gps_status != "UNKNOWN") {
      msg.emplace_back("gps_pps_lock: " + gps_status);
    }
    if (gprmc_status != "UNKNOWN") {
      msg.emplace_back("gprmc_status: " + gprmc_status);
    }
    if (ptp_status != "UNKNOWN") {
      msg.emplace_back("ptp_status: " + ptp_status);
    }
    if (ptp_status == "FREE RUN" && gps_status == "UNKNOWN") {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    }
    diagnostics.summary(level, boost::algorithm::join(msg, ", "));
  } else {
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No data available");
  }
}

void HesaiRosWrapper::HesaiCheckTemperature(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  std::scoped_lock lock(mtx_status);
  if (current_status) {
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::vector<std::string> msg;
    for (size_t i = 0; i < current_status->temperature.size(); i++) {
      diagnostics.add(
        temperature_names[i], GetFixedPrecisionString(current_status->temperature[i] * 0.01));
    }
    diagnostics.summary(level, boost::algorithm::join(msg, ", "));
  } else {
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No data available");
  }
}

void HesaiRosWrapper::HesaiCheckRpm(diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  std::scoped_lock lock(mtx_status);
  if (current_status) {
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::vector<std::string> msg;
    diagnostics.add("motor_speed", std::to_string(current_status->motor_speed));

    diagnostics.summary(level, boost::algorithm::join(msg, ", "));
  } else {
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No data available");
  }
}

void HesaiRosWrapper::HesaiCheckVoltageHttp(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  std::scoped_lock lock(mtx_lidar_monitor);
  if (current_lidar_monitor_tree) {
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::vector<std::string> msg;
    std::string key = "";

    std::string mes;
    key = "lidarInCur";
    try {
      mes = GetPtreeValue(current_lidar_monitor_tree.get(), "Body." + key);
    } catch (boost::bad_lexical_cast & ex) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      mes = error_message + std::string(ex.what());
    }
    diagnostics.add(key, mes);
    key = "lidarInVol";
    try {
      mes = GetPtreeValue(current_lidar_monitor_tree.get(), "Body." + key);
    } catch (boost::bad_lexical_cast & ex) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      mes = error_message + std::string(ex.what());
    }
    diagnostics.add(key, mes);

    diagnostics.summary(level, boost::algorithm::join(msg, ", "));
  } else {
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No data available");
  }
}

void HesaiRosWrapper::HesaiCheckVoltage(diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  std::scoped_lock lock(mtx_lidar_monitor);
  if (current_monitor) {
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::vector<std::string> msg;
    diagnostics.add(
      "input_voltage", GetFixedPrecisionString(current_monitor->input_voltage * 0.01) + " V");
    diagnostics.add(
      "input_current", GetFixedPrecisionString(current_monitor->input_current * 0.01) + " mA");
    diagnostics.add(
      "input_power", GetFixedPrecisionString(current_monitor->input_power * 0.01) + " W");

    diagnostics.summary(level, boost::algorithm::join(msg, ", "));
  } else {
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No data available");
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(HesaiRosWrapper)
}  // namespace ros
}  // namespace nebula
