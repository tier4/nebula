#pragma once

#include "nebula_ros/common/mt_queue.hpp"
#include "nebula_ros/common/parameter_descriptors.hpp"
#include "nebula_ros/seyond/decoder_wrapper.hpp"
#include "nebula_ros/seyond/hw_interface_wrapper.hpp"
#include "nebula_ros/seyond/hw_monitor_wrapper.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>
#include <nebula_common/nebula_common.hpp>
#include <nebula_common/nebula_status.hpp>
#include <nebula_common/seyond/seyond_common.hpp>
#include <nebula_hw_interfaces/nebula_hw_interfaces_seyond/seyond_hw_interface.hpp>
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
using SeyondHwInterface = nebula::drivers::SeyondHwInterface;
using SeyondSensorConfiguration = nebula::drivers::SeyondSensorConfiguration;
using SeyondCalibrationConfiguration = nebula::drivers::SeyondCalibrationConfiguration;

class SeyondRosWrapper final : public rclcpp::Node
{

public:
  explicit SeyondRosWrapper(const rclcpp::NodeOptions & options);
  ~SeyondRosWrapper() noexcept {};

  Status GetStatus();

  Status StreamStart();

private:
  void ReceiveCloudPacketCallback(std::vector<uint8_t> & packet);

  void ReceiveScanMessageCallback(std::unique_ptr<nebula_msgs::msg::NebulaPackets> scan_msg);

  Status DeclareAndGetSensorConfigParams();

  rcl_interfaces::msg::SetParametersResult OnParameterChange(
    const std::vector<rclcpp::Parameter> & p);

  Status ValidateAndSetConfig(
    std::shared_ptr<const SeyondSensorConfiguration> & new_config);

  Status wrapper_status_;

  std::shared_ptr<const SeyondSensorConfiguration> sensor_cfg_ptr_{};
  std::mutex mtx_config_;

  mt_queue<std::unique_ptr<nebula_msgs::msg::NebulaPacket>> packet_queue_;

  /// @brief Thread to isolate decoding from receiving
  std::thread decoder_thread_;

  rclcpp::Subscription<nebula_msgs::msg::NebulaPackets>::SharedPtr packets_sub_{};

  bool launch_hw_;

  std::optional<SeyondHwInterfaceWrapper> hw_interface_wrapper_;
  std::optional<SeyondHwMonitorWrapper> hw_monitor_wrapper_;
  std::optional<SeyondDecoderWrapper> decoder_wrapper_;

  OnSetParametersCallbackHandle::SharedPtr parameter_event_cb_;
};

}  // namespace ros
}  // namespace nebula
