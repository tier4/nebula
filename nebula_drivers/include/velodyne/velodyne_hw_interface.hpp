#ifndef NEBULA_VELODYNE_HW_INTERFACE_H
#define NEBULA_VELODYNE_HW_INTERFACE_H

#include <boost/property_tree/ptree.hpp>
#include <rclcpp/rclcpp.hpp>

#include "common/nebula_hw_interface_base.hpp"
#include "tcp_driver/http_client_driver.hpp"
#include "udp_driver/udp_driver.hpp"
#include "velodyne/velodyne_common.hpp"
#include "velodyne/velodyne_status.hpp"
#include "velodyne_msgs/msg/velodyne_packet.hpp"
#include "velodyne_msgs/msg/velodyne_scan.hpp"

namespace nebula
{
namespace drivers
{
class VelodyneHwInterface : NebulaHwInterfaceBase
{
private:
  std::unique_ptr<::drivers::common::IoContext> cloud_io_context_;
  std::unique_ptr<::drivers::udp_driver::UdpDriver> cloud_udp_driver_;
  std::shared_ptr<VelodyneSensorConfiguration> sensor_configuration_;
  std::shared_ptr<VelodyneCalibrationConfiguration> calibration_configuration_;
  size_t azimuth_index_{};
  size_t mtu_size_{};
  std::unique_ptr<velodyne_msgs::msg::VelodyneScan> scan_cloud_ptr_;
  std::function<bool(size_t)>
    is_valid_packet_; /*Lambda Function Array to verify proper packet size*/
  std::function<void(std::unique_ptr<velodyne_msgs::msg::VelodyneScan> buffer)>
    scan_reception_callback_; /**This function pointer is called when the scan is complete*/

  uint16_t packet_first_azm_ = 0;
  uint16_t packet_first_azm_phased_ = 0;
  uint16_t packet_last_azm_ = 0;
  uint16_t packet_last_azm_phased_ = 0;
  uint16_t prev_packet_first_azm_phased_ = 0;
  uint16_t phase_ = 0;
  uint processed_packets_ = 0;

  std::shared_ptr<boost::asio::io_context> boost_ctx_;
  std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> http_client_driver_;

  /*
  inline static constexpr std::string TARGET_STATUS { "cgi/status.json" };
  inline static constexpr std::string TARGET_DIAG { "cgi/diag.json" };
  inline static constexpr std::string TARGET_SNAPSHOT { "cgi/snapshot.hdl" };
  inline static constexpr std::string TARGET_SETTING { "cgi/setting" };
  inline static constexpr std::string TARGET_FOV { "cgi/setting/fov" };
  inline static constexpr std::string TARGET_SAVE { "cgi/save" };
  inline static constexpr std::string TARGET_RESET { "cgi/reset" };
  */
  std::string TARGET_STATUS{"/cgi/status.json"};
  std::string TARGET_DIAG{"/cgi/diag.json"};
  std::string TARGET_SNAPSHOT{"/cgi/snapshot.hdl"};
  std::string TARGET_SETTING{"/cgi/setting"};
  std::string TARGET_FOV{"/cgi/setting/fov"};
  std::string TARGET_HOST{"/cgi/setting/host"};
  std::string TARGET_NET{"/cgi/setting/net"};
  std::string TARGET_SAVE{"/cgi/save"};
  std::string TARGET_RESET{"/cgi/reset"};
  void str_cb(const std::string & str);
  //  std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> GetHttpClientDriverOnce(std::shared_ptr<boost::asio::io_context> ctx);
  //  std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> GetHttpClientDriverOnce();

  VelodyneStatus GetHttpClientDriverOnce(
    std::shared_ptr<boost::asio::io_context> ctx,
    std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> & hcd);
  VelodyneStatus GetHttpClientDriverOnce(
    std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> & hcd);

  VelodyneStatus CheckAndSetConfig(
    std::shared_ptr<VelodyneSensorConfiguration> sensor_configuration,
    boost::property_tree::ptree tree);

  std::shared_ptr<rclcpp::Logger> parent_node_logger;
  void PrintInfo(std::string info);
  void PrintError(std::string error);
  void PrintDebug(std::string error);

public:
  VelodyneHwInterface();

  void ReceiveCloudPacketCallback(const std::vector<uint8_t> & buffer) final;
  Status CloudInterfaceStart() final;
  Status CloudInterfaceStop() final;
  Status GetSensorConfiguration(SensorConfigurationBase & sensor_configuration) final;
  Status GetCalibrationConfiguration(
    CalibrationConfigurationBase & calibration_configuration) final;
  Status InitializeSensorConfiguration(
    std::shared_ptr<SensorConfigurationBase> sensor_configuration);
  Status SetSensorConfiguration(
    std::shared_ptr<SensorConfigurationBase> sensor_configuration) final;
  Status RegisterScanCallback(
    std::function<void(std::unique_ptr<velodyne_msgs::msg::VelodyneScan>)> scan_callback);

  boost::property_tree::ptree ParseJson(const std::string & str);

  VelodyneStatus InitHttpClient();
  std::string GetStatus();
  std::string GetDiag();
  std::string GetSnapshot();
  VelodyneStatus SetRpm(uint16_t rpm);
  VelodyneStatus SetFovStart(uint16_t fov_start);
  VelodyneStatus SetFovEnd(uint16_t fov_end);
  VelodyneStatus SetReturnType(ReturnMode return_mode);
  VelodyneStatus SaveConfig();
  VelodyneStatus ResetSystem();
  VelodyneStatus LaserOn();
  VelodyneStatus LaserOff();
  VelodyneStatus LaserOnOff(bool on);
  VelodyneStatus SetHostAddr(std::string addr);
  VelodyneStatus SetHostDport(uint16_t dport);
  VelodyneStatus SetHostTport(uint16_t tport);
  VelodyneStatus SetNetAddr(std::string addr);
  VelodyneStatus SetNetMask(std::string mask);
  VelodyneStatus SetNetGateway(std::string gateway);
  VelodyneStatus SetNetDhcp(bool use_dhcp);

  VelodyneStatus InitHttpClientAsync();
  VelodyneStatus GetStatusAsync(std::function<void(const std::string & str)> str_callback);
  VelodyneStatus GetStatusAsync();
  VelodyneStatus GetDiagAsync(std::function<void(const std::string & str)> str_callback);
  VelodyneStatus GetDiagAsync();
  VelodyneStatus GetSnapshotAsync(std::function<void(const std::string & str)> str_callback);
  VelodyneStatus GetSnapshotAsync();
  VelodyneStatus CheckAndSetConfigBySnapshotAsync();
  VelodyneStatus SetRpmAsync(uint16_t rpm);
  VelodyneStatus SetFovStartAsync(uint16_t fov_start);
  VelodyneStatus SetFovEndAsync(uint16_t fov_end);
  VelodyneStatus SetReturnTypeAsync(ReturnMode return_mode);
  VelodyneStatus SaveConfigAsync();
  VelodyneStatus ResetSystemAsync();
  VelodyneStatus LaserOnAsync();
  VelodyneStatus LaserOffAsync();
  VelodyneStatus LaserOnOffAsync(bool on);
  VelodyneStatus SetHostAddrAsync(std::string addr);
  VelodyneStatus SetHostDportAsync(uint16_t dport);
  VelodyneStatus SetHostTportAsync(uint16_t tport);
  VelodyneStatus SetNetAddrAsync(std::string addr);
  VelodyneStatus SetNetMaskAsync(std::string mask);
  VelodyneStatus SetNetGatewayAsync(std::string gateway);
  VelodyneStatus SetNetDhcpAsync(bool use_dhcp);

  void SetLogger(std::shared_ptr<rclcpp::Logger> node);
};

}  // namespace drivers
}  // namespace nebula

#endif  // NEBULA_VELODYNE_HW_INTERFACE_H
