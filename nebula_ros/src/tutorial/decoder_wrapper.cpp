#include "nebula_ros/tutorial/decoder_wrapper.hpp"

namespace nebula
{
namespace ros
{

using namespace std::chrono_literals;

TutorialDecoderWrapper::TutorialDecoderWrapper(rclcpp::Node* const parent_node,
                                         const std::shared_ptr<TutorialHwInterface>& hw_interface,
                                         std::shared_ptr<const TutorialSensorConfiguration>& config)
  : status_(nebula::Status::NOT_INITIALIZED)
  , logger_(parent_node->get_logger().get_child("TutorialDecoder"))
  , hw_interface_(hw_interface)
  , sensor_cfg_(config)
{
  if (!config)
  {
    throw std::runtime_error("TutorialDecoderWrapper cannot be instantiated without a valid config!");
  }

  auto calibration_file_path = parent_node->declare_parameter<std::string>("calibration_file", param_read_write());
  auto calibration_result = GetCalibrationData(calibration_file_path, false);

  if (!calibration_result.has_value())
  {
    throw std::runtime_error(
        (std::stringstream() << "No valid calibration found: " << calibration_result.error()).str());
  }

  calibration_cfg_ptr_ = calibration_result.value();
  RCLCPP_INFO_STREAM(logger_, "Using calibration data from " << calibration_cfg_ptr_->calibration_file);

  RCLCPP_INFO(logger_, "Starting Decoder");

  driver_ptr_ = std::make_shared<TutorialDriver>(config, calibration_cfg_ptr_);
  status_ = driver_ptr_->GetStatus();

  if (Status::OK != status_)
  {
    throw std::runtime_error((std::stringstream() << "Error instantiating decoder: " << status_).str());
  }

  // Publish packets only if HW interface is connected
  if (hw_interface_)
  {
    current_scan_msg_ = std::make_unique<nebula_msgs::msg::NebulaPackets>();
    packets_pub_ =
        parent_node->create_publisher<nebula_msgs::msg::NebulaPackets>("nebula_packets", rclcpp::SensorDataQoS());
  }

  auto qos_profile = rmw_qos_profile_sensor_data;
  auto pointcloud_qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);

  nebula_points_pub_ = parent_node->create_publisher<sensor_msgs::msg::PointCloud2>("nebula_points", pointcloud_qos);

  RCLCPP_INFO_STREAM(logger_, ". Wrapper=" << status_);

  cloud_watchdog_ = std::make_shared<WatchdogTimer>(*parent_node, 100'000us, [this, parent_node](bool ok) {
    if (ok)
      return;
    RCLCPP_WARN_THROTTLE(logger_, *parent_node->get_clock(), 5000, "Missed pointcloud output deadline");
  });
}

void TutorialDecoderWrapper::OnConfigChange(
    const std::shared_ptr<const TutorialSensorConfiguration>& new_config)
{
  std::lock_guard lock(mtx_driver_ptr_);
  auto new_driver = std::make_shared<TutorialDriver>(new_config, calibration_cfg_ptr_);
  driver_ptr_ = new_driver;
  sensor_cfg_ = new_config;
}

void TutorialDecoderWrapper::OnCalibrationChange(
    const std::shared_ptr<const TutorialCalibrationConfiguration>& new_calibration)
{
  std::lock_guard lock(mtx_driver_ptr_);
  auto new_driver = std::make_shared<TutorialDriver>(sensor_cfg_, new_calibration);
  driver_ptr_ = new_driver;
  calibration_cfg_ptr_ = new_calibration;
  calibration_file_path_ = calibration_cfg_ptr_->calibration_file;
}

rcl_interfaces::msg::SetParametersResult TutorialDecoderWrapper::OnParameterChange(const std::vector<rclcpp::Parameter>& p)
{
  using namespace rcl_interfaces::msg;

  std::string calibration_path = "";

  // Only one of the two parameters is defined, so not checking for sensor model explicitly here is fine
  bool got_any = get_param(p, "calibration_file", calibration_path) | get_param(p, "correction_file", calibration_path);
  if (!got_any)
  {
    return rcl_interfaces::build<SetParametersResult>().successful(true).reason("");
  }

  if (!std::filesystem::exists(calibration_path))
  {
    auto result = SetParametersResult();
    result.successful = false;
    result.reason = "The given calibration path does not exist, ignoring: '" + calibration_path + "'";
    return result;
  }

  auto get_calibration_result = GetCalibrationData(calibration_path, true);
  if (!get_calibration_result.has_value())
  {
    auto result = SetParametersResult();
    result.successful = false;
    result.reason = (std::stringstream() << "Could not change calibration file to '" << calibration_path
                                         << "': " << get_calibration_result.error())
                        .str();
    return result;
  }

  OnCalibrationChange(get_calibration_result.value());
  RCLCPP_INFO_STREAM(logger_, "Changed calibration to '" << calibration_cfg_ptr_->calibration_file << "'");
  return rcl_interfaces::build<SetParametersResult>().successful(true).reason("");
}

get_calibration_result_t
TutorialDecoderWrapper::GetCalibrationData(const std::string& calibration_file_path, bool ignore_others)
{
  auto calib = std::make_shared<TutorialCalibrationConfiguration>();

  bool hw_connected = hw_interface_ != nullptr;
  std::string calibration_file_path_from_sensor;

  {
    int ext_pos = calibration_file_path.find_last_of('.');
    calibration_file_path_from_sensor = calibration_file_path.substr(0, ext_pos);
    // TODO: if multiple different sensors of the same type are used, this will mix up their calibration data
    calibration_file_path_from_sensor += "_from_sensor";
    calibration_file_path_from_sensor += calibration_file_path.substr(ext_pos, calibration_file_path.size() - ext_pos);
  }

  // If a sensor is connected, try to download and save its calibration data
  if (!ignore_others && hw_connected)
  {
    try
    {
      auto raw_data = hw_interface_->GetLidarCalibrationBytes();
      RCLCPP_INFO(logger_, "Downloaded calibration data from sensor.");
      auto status = calib->SaveToFileFromBytes(calibration_file_path_from_sensor, raw_data);
      if (status != Status::OK)
      {
        RCLCPP_ERROR_STREAM(logger_, "Could not save calibration data: " << status);
      }
      else
      {
        RCLCPP_INFO_STREAM(logger_, "Saved downloaded data to " << calibration_file_path_from_sensor);
      }
    }
    catch (std::runtime_error& e)
    {
      RCLCPP_ERROR_STREAM(logger_, "Could not download calibration data: " << e.what());
    }
  }

  // If saved calibration data from a sensor exists (either just downloaded above, or previously), try to load it
  if (!ignore_others && std::filesystem::exists(calibration_file_path_from_sensor))
  {
    auto status = calib->LoadFromFile(calibration_file_path_from_sensor);
    if (status == Status::OK)
    {
      calib->calibration_file = calibration_file_path_from_sensor;
      return calib;
    }

    RCLCPP_ERROR_STREAM(logger_, "Could not load downloaded calibration data: " << status);
  }
  else if (!ignore_others)
  {
    RCLCPP_ERROR(logger_, "No downloaded calibration data found.");
  }

  if (!ignore_others)
  {
    RCLCPP_WARN(logger_, "Falling back to generic calibration file.");
  }

  // If downloaded data did not exist or could not be loaded, fall back to a generic file.
  // If that file does not exist either, return an error code
  if (!std::filesystem::exists(calibration_file_path))
  {
    RCLCPP_ERROR(logger_, "No calibration data found.");
    return nebula::Status(Status::INVALID_CALIBRATION_FILE);
  }

  // Try to load the existing fallback calibration file. Return an error if this fails
  auto status = calib->LoadFromFile(calibration_file_path);
  if (status != Status::OK)
  {
    RCLCPP_ERROR_STREAM(logger_, "Could not load calibration file at '" << calibration_file_path << "'");
    return status;
  }

  // Return the fallback calibration file
  calib->calibration_file = calibration_file_path;
  return calib;
}

void TutorialDecoderWrapper::ProcessCloudPacket(std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg)
{
  // ////////////////////////////////////////
  // Accumulate packets for recording
  // ////////////////////////////////////////

  // Accumulate packets for recording only if someone is subscribed to the topic (for performance)
  if (hw_interface_ &&
      (packets_pub_->get_subscription_count() > 0 || packets_pub_->get_intra_process_subscription_count() > 0))
  {
    if (current_scan_msg_->packets.size() == 0)
    {
      current_scan_msg_->header.stamp = packet_msg->stamp;
    }

    current_scan_msg_->packets.emplace_back(*packet_msg);
  }

  // ////////////////////////////////////////
  // Decode packet
  // ////////////////////////////////////////

  std::tuple<nebula::drivers::NebulaPointCloudPtr, double> pointcloud_ts{};
  nebula::drivers::NebulaPointCloudPtr pointcloud = nullptr;
  {
    std::lock_guard lock(mtx_driver_ptr_);
    pointcloud_ts = driver_ptr_->ParseCloudPacket(packet_msg->data);
    pointcloud = std::get<0>(pointcloud_ts);
  }

  if (pointcloud == nullptr)
  {
    return;
  };

  // ////////////////////////////////////////
  // If scan completed, publish pointcloud
  // ////////////////////////////////////////

  // A pointcloud has been produced, reset the watchdog timer
  cloud_watchdog_->update();

  // Publish scan message only if it has been written to
  if (current_scan_msg_ && !current_scan_msg_->packets.empty())
  {
    packets_pub_->publish(std::move(current_scan_msg_));
    current_scan_msg_ = std::make_unique<nebula_msgs::msg::NebulaPackets>();
  }

  if (nebula_points_pub_->get_subscription_count() > 0 ||
      nebula_points_pub_->get_intra_process_subscription_count() > 0)
  {
    auto ros_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*pointcloud, *ros_pc_msg_ptr);
    auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(
                         std::chrono::duration<double>(std::get<1>(pointcloud_ts)))
                         .count();
    ros_pc_msg_ptr->header.stamp = rclcpp::Time(nanoseconds);
    PublishCloud(std::move(ros_pc_msg_ptr), nebula_points_pub_);
  }
}

void TutorialDecoderWrapper::PublishCloud(std::unique_ptr<sensor_msgs::msg::PointCloud2> pointcloud,
                                       const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher)
{
  if (pointcloud->header.stamp.sec < 0)
  {
    RCLCPP_WARN_STREAM(logger_, "Timestamp error, verify clock source.");
  }
  pointcloud->header.frame_id = sensor_cfg_->frame_id;
  publisher->publish(std::move(pointcloud));
}

nebula::Status TutorialDecoderWrapper::Status()
{
  std::lock_guard lock(mtx_driver_ptr_);

  if (!driver_ptr_)
  {
    return nebula::Status::NOT_INITIALIZED;
  }

  return driver_ptr_->GetStatus();
}
}  // namespace ros
}  // namespace nebula