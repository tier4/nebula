#include "nebula_ros/hesai/hesai_decoder_ros_wrapper.hpp"

namespace nebula
{
namespace ros
{
HesaiDriverRosWrapper::HesaiDriverRosWrapper(const rclcpp::NodeOptions & options)
: rclcpp::Node("hesai_driver_ros_wrapper", options), hw_interface_()
{
  drivers::HesaiCalibrationConfiguration calibration_configuration;
  drivers::HesaiSensorConfiguration sensor_configuration;
  drivers::HesaiCorrection correction_configuration;

  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  hw_interface_.SetLogger(std::make_shared<rclcpp::Logger>(this->get_logger()));

  wrapper_status_ =
    GetParameters(sensor_configuration, calibration_configuration, correction_configuration);
  if (Status::OK != wrapper_status_) {
    RCLCPP_ERROR_STREAM(this->get_logger(), this->get_name() << " Error:" << wrapper_status_);
    return;
  }
  RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << ". Starting...");

  calibration_cfg_ptr_ =
    std::make_shared<drivers::HesaiCalibrationConfiguration>(calibration_configuration);

  sensor_cfg_ptr_ = std::make_shared<drivers::HesaiSensorConfiguration>(sensor_configuration);

  RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << ". Driver ");
  if (sensor_configuration.sensor_model == drivers::SensorModel::HESAI_PANDARAT128) {
    correction_cfg_ptr_ = std::make_shared<drivers::HesaiCorrection>(correction_configuration);
    wrapper_status_ = InitializeDriver(
      std::const_pointer_cast<drivers::SensorConfigurationBase>(sensor_cfg_ptr_),
      std::static_pointer_cast<drivers::CalibrationConfigurationBase>(calibration_cfg_ptr_),
      std::static_pointer_cast<drivers::HesaiCorrection>(correction_cfg_ptr_));
  } else {
    wrapper_status_ = InitializeDriver(
      std::const_pointer_cast<drivers::SensorConfigurationBase>(sensor_cfg_ptr_),
      std::static_pointer_cast<drivers::CalibrationConfigurationBase>(calibration_cfg_ptr_));
  }

  RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << ". Wrapper=" << wrapper_status_);
  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10),
                         qos_profile);
  pandar_scan_sub_ = create_subscription<pandar_msgs::msg::PandarScan>(
    "pandar_packets", qos,
    std::bind(&HesaiDriverRosWrapper::ReceiveScanMsgCallback, this, std::placeholders::_1));
  nebula_points_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("pandar_points", rclcpp::SensorDataQoS());
  aw_points_base_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("aw_points", rclcpp::SensorDataQoS());
  aw_points_ex_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("aw_points_ex", rclcpp::SensorDataQoS());
}

void HesaiDriverRosWrapper::ReceiveScanMsgCallback(
  const pandar_msgs::msg::PandarScan::SharedPtr scan_msg)
{
  auto t_start = std::chrono::high_resolution_clock::now();
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
    const auto autoware_ex_cloud =
      nebula::drivers::convertPointXYZIRCAEDTToPointXYZIRADT(pointcloud, std::get<1>(pointcloud_ts));
    auto ros_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*autoware_ex_cloud, *ros_pc_msg_ptr);
    ros_pc_msg_ptr->header.stamp =
      rclcpp::Time(SecondsToChronoNanoSeconds(std::get<1>(pointcloud_ts)).count());
    PublishCloud(std::move(ros_pc_msg_ptr), aw_points_ex_pub_);
  }

  auto runtime = std::chrono::high_resolution_clock::now() - t_start;
  RCLCPP_DEBUG(get_logger(), "PROFILING {'d_total': %lu, 'n_out': %lu}", runtime.count(), pointcloud->size());
}

void HesaiDriverRosWrapper::PublishCloud(
  std::unique_ptr<sensor_msgs::msg::PointCloud2> pointcloud,
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher)
{
  if (pointcloud->header.stamp.sec < 0) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Timestamp error, verify clock source.");
  }
  pointcloud->header.frame_id = sensor_cfg_ptr_->frame_id;
  publisher->publish(std::move(pointcloud));
}

Status HesaiDriverRosWrapper::InitializeDriver(
  std::shared_ptr<drivers::SensorConfigurationBase> sensor_configuration,
  std::shared_ptr<drivers::CalibrationConfigurationBase> calibration_configuration)
{
  driver_ptr_ = std::make_shared<drivers::HesaiDriver>(
    std::static_pointer_cast<drivers::HesaiSensorConfiguration>(sensor_configuration),
    std::static_pointer_cast<drivers::HesaiCalibrationConfiguration>(calibration_configuration));
  return driver_ptr_->GetStatus();
}

Status HesaiDriverRosWrapper::InitializeDriver(
  const std::shared_ptr<drivers::SensorConfigurationBase> & sensor_configuration,
  const std::shared_ptr<drivers::CalibrationConfigurationBase> & calibration_configuration,
  const std::shared_ptr<drivers::HesaiCorrection> & correction_configuration)
{
  // driver should be initialized here with proper decoder
  driver_ptr_ = std::make_shared<drivers::HesaiDriver>(
    std::static_pointer_cast<drivers::HesaiSensorConfiguration>(sensor_configuration),
    std::static_pointer_cast<drivers::HesaiCalibrationConfiguration>(calibration_configuration),
    std::static_pointer_cast<drivers::HesaiCorrection>(correction_configuration));
  return driver_ptr_->GetStatus();
}

Status HesaiDriverRosWrapper::GetStatus() { return wrapper_status_; }

Status HesaiDriverRosWrapper::GetParameters(
  drivers::HesaiSensorConfiguration & sensor_configuration,
  drivers::HesaiCalibrationConfiguration & calibration_configuration,
  drivers::HesaiCorrection & correction_configuration)
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
    calibration_configuration.calibration_file =
      this->get_parameter("calibration_file").as_string();
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
  bool launch_hw;
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<bool>("launch_hw", "", descriptor);
    launch_hw = this->get_parameter("launch_hw").as_bool();
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
  
  bool run_local = !launch_hw;
  if (sensor_configuration.sensor_model != drivers::SensorModel::HESAI_PANDARAT128) {
    std::string calibration_file_path_from_sensor;
    if (launch_hw && !calibration_configuration.calibration_file.empty()) {
      int ext_pos = calibration_configuration.calibration_file.find_last_of('.');
      calibration_file_path_from_sensor += calibration_configuration.calibration_file.substr(0, ext_pos);
      calibration_file_path_from_sensor += "_from_sensor";
      calibration_file_path_from_sensor += calibration_configuration.calibration_file.substr(ext_pos, calibration_configuration.calibration_file.size() - ext_pos);
    }
    if(launch_hw) {
      run_local = false;
      RCLCPP_INFO_STREAM(
        this->get_logger(), "Trying to acquire calibration data from sensor: '"
        << sensor_configuration.sensor_ip << "'");
      std::future<void> future = std::async(std::launch::async,
                                            [this, &calibration_configuration, &calibration_file_path_from_sensor, &run_local]() {
                                              if (hw_interface_.InitializeTcpDriver() == Status::OK) {
                                                auto str = hw_interface_.GetLidarCalibrationString();
                                                  auto rt = calibration_configuration.SaveFileFromString(
                                                    calibration_file_path_from_sensor, str);
                                                  RCLCPP_ERROR_STREAM(get_logger(), str);
                                                  if (rt == Status::OK) {
                                                    RCLCPP_INFO_STREAM(get_logger(), "SaveFileFromString success:"
                                                      << calibration_file_path_from_sensor << "\n");
                                                  } else {
                                                    RCLCPP_ERROR_STREAM(get_logger(), "SaveFileFromString failed:"
                                                      << calibration_file_path_from_sensor << "\n");
                                                  }
                                                  rt = calibration_configuration.LoadFromString(str);
                                                  if (rt == Status::OK) {
                                                    RCLCPP_INFO_STREAM(get_logger(),
                                                                        "LoadFromString success:" << str << "\n");
                                                  } else {
                                                    RCLCPP_ERROR_STREAM(get_logger(),
                                                                        "LoadFromString failed:" << str << "\n");
                                                  }
                                              } else {
                                                run_local = true;
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
          this->get_logger(), "Acquired calibration data from sensor: '"
          << sensor_configuration.sensor_ip << "'");
        RCLCPP_INFO_STREAM(
          this->get_logger(), "The calibration has been saved in '"
          << calibration_file_path_from_sensor << "'");
      }
    }
    if(run_local) {
      RCLCPP_WARN_STREAM(get_logger(), "Running locally");
      bool run_org = false;
      if (calibration_file_path_from_sensor.empty()) {
        run_org = true;
      } else {
        RCLCPP_INFO_STREAM(get_logger(),"Trying to load file: " << calibration_file_path_from_sensor);
        auto cal_status =
          calibration_configuration.LoadFromFile(calibration_file_path_from_sensor);

        if (cal_status != Status::OK) {
          run_org = true;
        }else{
          RCLCPP_INFO_STREAM(
            this->get_logger(), "Load calibration data from: '"
                                  << calibration_file_path_from_sensor << "'");
        }
      }
      if(run_org) {
        RCLCPP_INFO_STREAM(get_logger(),"Trying to load file: " << calibration_configuration.calibration_file);
        if (calibration_configuration.calibration_file.empty()) {
          RCLCPP_ERROR_STREAM(
            this->get_logger(), "Empty Calibration_file File: '" << calibration_configuration.calibration_file << "'");
          return Status::INVALID_CALIBRATION_FILE;
        } else {
          auto cal_status =
            calibration_configuration.LoadFromFile(calibration_configuration.calibration_file);

          if (cal_status != Status::OK) {
            RCLCPP_ERROR_STREAM(
              this->get_logger(),
              "Given Calibration File: '" << calibration_configuration.calibration_file << "'");
            return cal_status;
          }else{
            RCLCPP_INFO_STREAM(
              this->get_logger(), "Load calibration data from: '"
                                    << calibration_configuration.calibration_file << "'");
          }
        }
      }
    }
  } else { // sensor_configuration.sensor_model == drivers::SensorModel::HESAI_PANDARAT128
    std::string correction_file_path_from_sensor;
    if (launch_hw && !correction_file_path.empty()) {
      int ext_pos = correction_file_path.find_last_of('.');
      correction_file_path_from_sensor += correction_file_path.substr(0, ext_pos);
      correction_file_path_from_sensor += "_from_sensor";
      correction_file_path_from_sensor += correction_file_path.substr(ext_pos, correction_file_path.size() - ext_pos);
    }
    std::future<void> future = std::async(std::launch::async, [this, &correction_configuration, &correction_file_path_from_sensor, &run_local, &launch_hw]() {
      if (launch_hw && hw_interface_.InitializeTcpDriver() == Status::OK) {
        RCLCPP_INFO_STREAM(
          this->get_logger(), "Trying to acquire calibration data from sensor");
        auto received_bytes = hw_interface_.GetLidarCalibrationBytes();
        RCLCPP_INFO_STREAM(get_logger(), "AT128 calibration size:" << received_bytes.size() << "\n");
        auto rt = correction_configuration.SaveFileFromBinary(correction_file_path_from_sensor, received_bytes);
        if(rt == Status::OK)
        {
          RCLCPP_INFO_STREAM(get_logger(), "SaveFileFromBinary success:" << correction_file_path_from_sensor << "\n");
        }
        else
        {
          RCLCPP_ERROR_STREAM(get_logger(), "SaveFileFromBinary failed:" << correction_file_path_from_sensor << ". Falling back to offline calibration file.");
          run_local = true;
        }
        rt = correction_configuration.LoadFromBinary(received_bytes);
        if(rt == Status::OK)
        {
          RCLCPP_INFO_STREAM(get_logger(), "LoadFromBinary success" << "\n");
          run_local = false;
        }
        else
        {
          RCLCPP_ERROR_STREAM(get_logger(), "LoadFromBinary failed" << ". Falling back to offline calibration file.");
          run_local = true;
        }
      }else{
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
          this->get_logger(), "Acquired correction data from sensor: '"
          << sensor_configuration.sensor_ip << "'");
        RCLCPP_INFO_STREAM(
          this->get_logger(), "The correction has been saved in '"
          << correction_file_path_from_sensor << "'");
      }
    }
    if(run_local) {
      bool run_org = false;
      if (correction_file_path_from_sensor.empty()) {
        run_org = true;
      } else {
        auto cal_status =
          correction_configuration.LoadFromFile(correction_file_path_from_sensor);

        if (cal_status != Status::OK) {
          run_org = true;
        }else{
          RCLCPP_INFO_STREAM(
            this->get_logger(), "Load correction data from: '"
                                  << correction_file_path_from_sensor << "'");
        }
      }
      if(run_org) {
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
          }else{
            RCLCPP_INFO_STREAM(
              this->get_logger(), "Load correction data from: '"
                                    << correction_file_path << "'");
          }
        }
      }
    }
  } // end AT128
  // Do not use outside of this location
  hw_interface_.~HesaiHwInterface();
  RCLCPP_INFO_STREAM(this->get_logger(), "SensorConfig:" << sensor_configuration);
  return Status::OK;
}

RCLCPP_COMPONENTS_REGISTER_NODE(HesaiDriverRosWrapper)
}  // namespace ros
}  // namespace nebula
