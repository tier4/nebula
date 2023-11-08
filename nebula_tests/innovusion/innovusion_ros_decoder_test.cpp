#include "innovusion_ros_decoder_test.hpp"

#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rcpputils/filesystem_helper.hpp"
#include "rcutils/time.h"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_cpp/writers/sequential_writer.hpp"
#include "rosbag2_storage/storage_options.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <regex>

namespace nebula
{
namespace ros
{
InnovusionRosDecoderTest::InnovusionRosDecoderTest(
  const rclcpp::NodeOptions & options, const std::string & node_name)
: rclcpp::Node(node_name, options)
{
  drivers::InnovusionCalibrationConfiguration calibration_configuration;
  drivers::InnovusionSensorConfiguration sensor_configuration;

  wrapper_status_ = GetParameters(sensor_configuration, calibration_configuration);
  if (Status::OK != wrapper_status_) {
    RCLCPP_ERROR_STREAM(this->get_logger(), this->get_name() << " Error:" << wrapper_status_);
    return;
  }
  RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << ". Starting...");

  calibration_cfg_ptr_ =
    std::make_shared<drivers::InnovusionCalibrationConfiguration>(calibration_configuration);

  sensor_cfg_ptr_ = std::make_shared<drivers::InnovusionSensorConfiguration>(sensor_configuration);

  RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << ". Driver ");
  wrapper_status_ = InitializeDriver(
    std::const_pointer_cast<drivers::SensorConfigurationBase>(sensor_cfg_ptr_),
    std::static_pointer_cast<drivers::CalibrationConfigurationBase>(calibration_cfg_ptr_));

  RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << "Wrapper=" << wrapper_status_);
}

Status InnovusionRosDecoderTest::InitializeDriver(
  std::shared_ptr<drivers::SensorConfigurationBase> sensor_configuration,
  std::shared_ptr<drivers::CalibrationConfigurationBase> calibration_configuration)
{
  // driver should be initialized here with proper decoder
  driver_ptr_ = std::make_shared<drivers::InnovusionDriver>(
    std::static_pointer_cast<drivers::InnovusionSensorConfiguration>(sensor_configuration),
    std::static_pointer_cast<drivers::InnovusionCalibrationConfiguration>(calibration_configuration));
  return driver_ptr_->GetStatus();
}

Status InnovusionRosDecoderTest::GetStatus() { return wrapper_status_; }

Status InnovusionRosDecoderTest::GetParameters(
  drivers::InnovusionSensorConfiguration & sensor_configuration,
  drivers::InnovusionCalibrationConfiguration & calibration_configuration)
{
  std::filesystem::path calib_dir =
    _SRC_CALIBRATION_DIR_PATH;  // variable defined in CMakeLists.txt;
  calib_dir /= "innovusion";
  std::filesystem::path bag_root_dir =
    _SRC_RESOURCES_DIR_PATH;  // variable defined in CMakeLists.txt;
  bag_root_dir /= "innovusion";
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 4;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("sensor_model", "Falcon");
    sensor_configuration.sensor_model =
      nebula::drivers::SensorModelFromString(this->get_parameter("sensor_model").as_string());
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 4;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("return_mode", "Dual", descriptor);
    sensor_configuration.return_mode =
      nebula::drivers::ReturnModeFromString(this->get_parameter("return_mode").as_string());
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 4;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("frame_id", "innovusion", descriptor);
    sensor_configuration.frame_id = this->get_parameter("frame_id").as_string();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 4;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>(
      "calibration_file", (calib_dir / "Falcon.yaml").string(), descriptor);
    calibration_configuration.calibration_file =
      this->get_parameter("calibration_file").as_string();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 3;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<double>("min_range", 0.3, descriptor);
    sensor_configuration.min_range = this->get_parameter("min_range").as_double();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 3;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<double>("max_range", 300., descriptor);
    sensor_configuration.max_range = this->get_parameter("max_range").as_double();
  }
  double view_width = 360 * M_PI / 180;
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 3;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<double>("view_width", 300., descriptor);
    view_width = this->get_parameter("view_width").as_double() * M_PI / 180;
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 4;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>(
      "bag_path", (bag_root_dir / "1699435339354672202").string(), descriptor);
    bag_path = this->get_parameter("bag_path").as_string();
    std::cout << bag_path << std::endl;
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 4;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("storage_id", "sqlite3", descriptor);
    storage_id = this->get_parameter("storage_id").as_string();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 4;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("format", "cdr", descriptor);
    format = this->get_parameter("format").as_string();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = 4;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("target_topic", "/innovusion_packets", descriptor);
    target_topic = this->get_parameter("target_topic").as_string();
  }

  if (sensor_configuration.sensor_model == nebula::drivers::SensorModel::UNKNOWN) {
    return Status::INVALID_SENSOR_MODEL;
  }
  if (sensor_configuration.return_mode == nebula::drivers::ReturnMode::UNKNOWN) {
    return Status::INVALID_ECHO_MODE;
  }
  if (calibration_configuration.calibration_file.empty()) {
    return Status::INVALID_CALIBRATION_FILE;
  } else {
    auto cal_status =
      calibration_configuration.LoadFromFile(calibration_configuration.calibration_file);
    if (cal_status != Status::OK) {
      RCLCPP_ERROR_STREAM(
        this->get_logger(),
        "Given Calibration File: '" << calibration_configuration.calibration_file << "'");
      return cal_status;
    }
  }

  RCLCPP_INFO_STREAM(this->get_logger(), "SensorConfig:" << sensor_configuration);
  return Status::OK;
}

void printPCD(nebula::drivers::NebulaPointCloudPtr pp)
{
  for (auto p : pp->points) {
    std::cout << "(" << p.x << ", " << p.y << "," << p.z << "): " << p.intensity << ", "
              << p.channel << ", " << p.azimuth << ", " << p.return_type << ", " << p.time_stamp
              << std::endl;
  }
}

void checkPCDs(nebula::drivers::NebulaPointCloudPtr pp1, nebula::drivers::NebulaPointCloudPtr pp2)
{
  EXPECT_EQ(pp1->points.size(), pp2->points.size());
  for (uint32_t i = 0; i < pp1->points.size(); i++) {
    auto p1 = pp1->points[i];
    auto p2 = pp2->points[i];
    EXPECT_FLOAT_EQ(p1.x, p2.x);
    EXPECT_FLOAT_EQ(p1.y, p2.y);
    EXPECT_FLOAT_EQ(p1.z, p2.z);
    EXPECT_FLOAT_EQ(p1.intensity, p2.intensity);
    EXPECT_EQ(p1.channel, p2.channel);
    EXPECT_FLOAT_EQ(p1.azimuth, p2.azimuth);
    EXPECT_EQ(p1.return_type, p2.return_type);
    EXPECT_DOUBLE_EQ(p1.time_stamp, p2.time_stamp);
  }
}

void checkPCDs(nebula::drivers::NebulaPointCloudPtr pp1, pcl::PointCloud<pcl::PointXYZ>::Ptr pp2)
{
  std::cout<<"checkPCDs, pp1 size:"<<pp1->points.size()<<", pp2 size:"<<pp2->points.size()<<std::endl;
  EXPECT_EQ(pp1->points.size(), pp2->points.size());
  for (uint32_t i = 0; i < pp1->points.size(); i++) {
    auto p1 = pp1->points[i];
    auto p2 = pp2->points[i];
    EXPECT_FLOAT_EQ(p1.x, p2.x);
    EXPECT_FLOAT_EQ(p1.y, p2.y);
    EXPECT_FLOAT_EQ(p1.z, p2.z);
  }
}

void InnovusionRosDecoderTest::ReadBag()
{
  rosbag2_storage::StorageOptions storage_options;
  rosbag2_cpp::ConverterOptions converter_options;

  std::cout << bag_path << std::endl;
  std::cout << storage_id << std::endl;
  std::cout << format << std::endl;
  std::cout << target_topic << std::endl;

  auto target_topic_name = target_topic;
  if (target_topic_name.substr(0, 1) == "/") {
    target_topic_name = target_topic_name.substr(1);
  }
  target_topic_name = std::regex_replace(target_topic_name, std::regex("/"), "_");

  pcl::PCDReader pcd_reader;

  rcpputils::fs::path bag_dir(bag_path);
  rcpputils::fs::path pcd_dir = bag_dir.parent_path();
  int check_cnt = 0;

  storage_options.uri = bag_path;
  storage_options.storage_id = storage_id;
  converter_options.output_serialization_format = format;  //"cdr";
  rclcpp::Serialization<innovusion_msgs::msg::InnovusionScan> serialization;
  nebula::drivers::NebulaPointCloudPtr pointcloud(new nebula::drivers::NebulaPointCloud);
  // nebula::drivers::NebulaPointCloudPtr ref_pointcloud(new nebula::drivers::NebulaPointCloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr ref_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
  {
    rosbag2_cpp::Reader bag_reader(std::make_unique<rosbag2_cpp::readers::SequentialReader>());
    bag_reader.open(storage_options, converter_options);
    while (bag_reader.has_next()) {
      auto bag_message = bag_reader.read_next();

      std::cout << "Found topic name " << bag_message->topic_name << std::endl;

      if (bag_message->topic_name == target_topic) {
        innovusion_msgs::msg::InnovusionScan extracted_msg;
        rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
        serialization.deserialize_message(&extracted_serialized_msg, &extracted_msg);

        std::cout << "Found data in topic " << bag_message->topic_name << ": "
                  << bag_message->time_stamp << std::endl;

        auto extracted_msg_ptr = std::make_shared<innovusion_msgs::msg::InnovusionScan>(extracted_msg);
        auto pointcloud_ts = driver_ptr_->ConvertScanToPointcloud(extracted_msg_ptr);
        pointcloud = std::get<0>(pointcloud_ts);

        // There are very rare cases where has_scanned_ does not become true, but it is not known
        // whether it is because of decoder or deserialize_message.
        if (!pointcloud) continue;

        auto fn = std::to_string(bag_message->time_stamp) + ".pcd";

        auto target_pcd_path = (pcd_dir / fn);
        std::cout << target_pcd_path << std::endl;
        if (target_pcd_path.exists()) {
          std::cout << "exists: " << target_pcd_path << std::endl;
          auto rt = pcd_reader.read(target_pcd_path.string(), *ref_pointcloud);
          std::cout << rt << " loaded: " << target_pcd_path << std::endl;
          checkPCDs(pointcloud, ref_pointcloud);
          check_cnt++;
          // ref_pointcloud.reset(new nebula::drivers::NebulaPointCloud);
          ref_pointcloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        }
        pointcloud.reset(new nebula::drivers::NebulaPointCloud);
      }
    }
    EXPECT_GT(check_cnt, 0);
    // close on scope exit
  }
}

}  // namespace ros
}  // namespace nebula
