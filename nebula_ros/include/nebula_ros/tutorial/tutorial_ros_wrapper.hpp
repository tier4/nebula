#pragma once

#include "nebula_ros/common/mt_queue.hpp"
#include "nebula_ros/common/parameter_descriptors.hpp"
#include "nebula_ros/tutorial/decoder_wrapper.hpp"
#include "nebula_ros/tutorial/hw_interface_wrapper.hpp"
#include "nebula_ros/tutorial/hw_monitor_wrapper.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>
#include <nebula_common/nebula_common.hpp>
#include <nebula_common/nebula_status.hpp>
#include <nebula_common/hesai/hesai_common.hpp>
#include <nebula_hw_interfaces/nebula_hw_interfaces_hesai/hesai_hw_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <nebula_msgs/msg/nebula_packet.hpp>
#include <nebula_msgs/msg/nebula_packets.hpp>

#include <boost/algorithm/string/join.hpp>
#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>

#include <array>
#include <chrono>
#include <mutex>
#include <optional>
#include <thread>

namespace nebula
{
namespace ros
{
using TutorialHwInterface = nebula::drivers::HesaiHwInterface;
using TutorialSensorConfiguration = nebula::drivers::HesaiSensorConfiguration;
using TutorialCalibrationConfiguration = nebula::drivers::HesaiCalibrationConfiguration;

class TutorialRosWrapper final : public rclcpp::Node
{

public:
  explicit TutorialRosWrapper(const rclcpp::NodeOptions & options);
  ~TutorialRosWrapper() noexcept {};

  Status GetStatus();

  Status StreamStart();

private:
  void ReceiveCloudPacketCallback(std::vector<uint8_t> & packet);

  void ReceiveScanMessageCallback(std::unique_ptr<nebula_msgs::msg::NebulaPackets> scan_msg);

  Status DeclareAndGetSensorConfigParams();

  rcl_interfaces::msg::SetParametersResult OnParameterChange(
    const std::vector<rclcpp::Parameter> & p);

  Status ValidateAndSetConfig(
    std::shared_ptr<const TutorialSensorConfiguration> & new_config);

  Status wrapper_status_;

  std::shared_ptr<const TutorialSensorConfiguration> sensor_cfg_ptr_{};
  std::mutex mtx_config_;

  mt_queue<std::unique_ptr<nebula_msgs::msg::NebulaPacket>> packet_queue_;

  /// @brief Thread to isolate decoding from receiving
  std::thread decoder_thread_;

  rclcpp::Subscription<nebula_msgs::msg::NebulaPackets>::SharedPtr packets_sub_{};

  bool launch_hw_;

  std::optional<TutorialHwInterfaceWrapper> hw_interface_wrapper_;
  std::optional<TutorialHwMonitorWrapper> hw_monitor_wrapper_;
  std::optional<TutorialDecoderWrapper> decoder_wrapper_;

  OnSetParametersCallbackHandle::SharedPtr parameter_event_cb_;
};

}  // namespace ros
}  // namespace nebula
