#ifndef NEBULA_HESAI_HW_INTERFACE_H
#define NEBULA_HESAI_HW_INTERFACE_H

#include "common/nebula_hw_interface_base.hpp"
#include "hesai/hesai_common.hpp"
#include "hesai/hesai_status.hpp"
#include "udp_driver/udp_driver.hpp"
#include "tcp_driver/tcp_driver.hpp"
#include "tcp_driver/http_client_driver.hpp"

#include "pandar_msgs/msg/pandar_jumbo_packet.hpp"
#include "pandar_msgs/msg/pandar_packet.hpp"
#include "pandar_msgs/msg/pandar_scan.hpp"

#include "hesai/hesai_cmd_response.hpp"
#include <mutex>
#include <boost/property_tree/ptree.hpp>

#include <rclcpp/rclcpp.hpp>

namespace nebula
{
namespace drivers
{
class HesaiHwInterface : NebulaHwInterfaceBase
{
private:
  std::unique_ptr<IoContext> cloud_io_context_;
  std::shared_ptr<boost::asio::io_context> m_owned_ctx;
  std::shared_ptr<boost::asio::io_context> m_owned_ctx_s;
  std::unique_ptr<::drivers::udp_driver::UdpDriver> cloud_udp_driver_;
//  std::unique_ptr<::drivers::tcp_driver::TcpDriver> tcp_driver_;
  std::shared_ptr<::drivers::tcp_driver::TcpDriver> tcp_driver_;
  std::shared_ptr<::drivers::tcp_driver::TcpDriver> tcp_driver_s_;
  std::shared_ptr<HesaiSensorConfiguration> sensor_configuration_;
  std::shared_ptr<HesaiCalibrationConfiguration> calibration_configuration_;
  size_t azimuth_index_{};
  size_t mtu_size_{};
  std::unique_ptr<pandar_msgs::msg::PandarScan> scan_cloud_ptr_;
  std::function<bool(size_t)>
    is_valid_packet_; /*Lambda Function Array to verify proper packet size*/
  std::function<void(std::unique_ptr<pandar_msgs::msg::PandarScan> buffer)>
    scan_reception_callback_; /**This function pointer is called when the scan is complete*/

  int prev_phase_{};

  int timeout_ = 2000;
  std::timed_mutex tm_;
  int tm_fail_cnt = 0;
  int tm_fail_cnt_max = 0;//1;
  std::timed_mutex tms_;
  int tms_fail_cnt = 0;
  int tms_fail_cnt_max = 0;//1;
//  bool wl = false;
  bool wl = true;
  bool is_solid_state = false;
  int target_model_no;

  HesaiStatus GetHttpClientDriverOnce(std::shared_ptr<boost::asio::io_context> ctx, std::unique_ptr<::drivers::tcp_driver::HttpClientDriver>& hcd);
  HesaiStatus GetHttpClientDriverOnce(std::unique_ptr<::drivers::tcp_driver::HttpClientDriver>& hcd);
  void str_cb(const std::string &str);

//  std::vector<std::shared_ptr<::drivers::tcp_driver::TcpDriver>> tcp_drivers_status;

  bool CheckLock(std::timed_mutex &tm, int &fail_cnt, const int &fail_cnt_max, std::string name);
  void CheckUnlock(std::timed_mutex &tm, std::string name);

  std::shared_ptr<rclcpp::Logger> parent_node_logger;
  void PrintInfo(std::string info);
  void PrintError(std::string error);
  void PrintDebug(std::string error);
  void PrintDebug(const std::vector<uint8_t> & bytes);

public:
  HesaiHwInterface();
  Status InitializeTcpDriver();
  boost::property_tree::ptree ParseJson(const std::string &str);

  void ReceiveCloudPacketCallback(const std::vector<uint8_t> & buffer) final;
  Status CloudInterfaceStart() final;
  Status CloudInterfaceStop() final;
  Status GetSensorConfiguration(SensorConfigurationBase & sensor_configuration) final;
  Status GetCalibrationConfiguration(
    CalibrationConfigurationBase & calibration_configuration);
  Status SetSensorConfiguration(
    std::shared_ptr<SensorConfigurationBase> sensor_configuration) final;
  Status RegisterScanCallback(
    std::function<void(std::unique_ptr<pandar_msgs::msg::PandarScan>)> scan_callback);


  Status GetLidarCalib(std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, bool with_run=true);
  Status GetLidarCalib(std::shared_ptr<boost::asio::io_context> ctx, bool with_run=true);
  Status GetLidarCalib(bool with_run=true);
  Status GetPtpDiagStatus(std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, bool with_run=true);
  Status GetPtpDiagStatus(std::shared_ptr<boost::asio::io_context> ctx, bool with_run=true);
  Status GetPtpDiagStatus(bool with_run=true);
  Status GetPtpDiagPort(std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, bool with_run=true);
  Status GetPtpDiagPort(std::shared_ptr<boost::asio::io_context> ctx, bool with_run=true);
  Status GetPtpDiagPort(bool with_run=true);
  Status GetPtpDiagTime(std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, bool with_run=true);
  Status GetPtpDiagTime(std::shared_ptr<boost::asio::io_context> ctx, bool with_run=true);
  Status GetPtpDiagTime(bool with_run=true);
  Status GetPtpDiagGrandmaster(std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, bool with_run=true);
  Status GetPtpDiagGrandmaster(std::shared_ptr<boost::asio::io_context> ctx, bool with_run=true);
  Status GetPtpDiagGrandmaster(bool with_run=true);
  Status GetInventory(std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, std::function<void(HesaiInventory &result)> callback, bool with_run=true);
  Status GetInventory(std::shared_ptr<boost::asio::io_context> ctx, std::function<void(HesaiInventory &result)> callback, bool with_run=true);
  Status GetInventory(std::shared_ptr<boost::asio::io_context> ctx, bool with_run=true);
  Status GetInventory(std::function<void(HesaiInventory &result)> callback, bool with_run=true);
  Status GetInventory(bool with_run=true);
  Status GetConfig(std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, std::function<void(HesaiConfig &result)> callback, bool with_run=true);
  Status GetConfig(std::shared_ptr<boost::asio::io_context> ctx, std::function<void(HesaiConfig &result)> callback, bool with_run=true);
  Status GetConfig(std::shared_ptr<boost::asio::io_context> ctx, bool with_run=true);
  Status GetConfig(std::function<void(HesaiConfig &result)> callback, bool with_run=true);
  Status GetConfig(bool with_run=true);
  Status GetLidarStatus(std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, std::function<void(HesaiLidarStatus &result)> callback, bool with_run=true);
  Status GetLidarStatus(std::shared_ptr<boost::asio::io_context> ctx, std::function<void(HesaiLidarStatus &result)> callback, bool with_run=true);
  Status GetLidarStatus(std::shared_ptr<boost::asio::io_context> ctx, bool with_run=true);
  Status GetLidarStatus(std::function<void(HesaiLidarStatus &result)> callback, bool with_run=true);
  Status GetLidarStatus(bool with_run=true);
  Status SetSpinRate(std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, uint16_t rpm, bool with_run=true);
  Status SetSpinRate(std::shared_ptr<boost::asio::io_context> ctx, uint16_t rpm, bool with_run=true);
  Status SetSpinRate(uint16_t rpm, bool with_run=true);
  Status SetSyncAngle(std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, int sync_angle, int angle, bool with_run=true);
  Status SetSyncAngle(std::shared_ptr<boost::asio::io_context> ctx, int sync_angle, int angle, bool with_run=true);
  Status SetSyncAngle(int sync_angle, int angle, bool with_run=true);
  Status SetTriggerMethod(std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, int trigger_method, bool with_run=true);
  Status SetTriggerMethod(std::shared_ptr<boost::asio::io_context> ctx, int trigger_method, bool with_run=true);
  Status SetTriggerMethod(int trigger_method, bool with_run=true);
  Status SetStandbyMode(std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, int standby_mode, bool with_run=true);
  Status SetStandbyMode(std::shared_ptr<boost::asio::io_context> ctx, int standby_mode, bool with_run=true);
  Status SetStandbyMode(int flg, bool with_run=true);
  Status SetReturnMode(std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, int return_mode, bool with_run=true);
  Status SetReturnMode(std::shared_ptr<boost::asio::io_context> ctx, int return_mode, bool with_run=true);
  Status SetReturnMode(int return_mode, bool with_run=true);
  Status SetDestinationIp(std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, int dest_ip_1, int dest_ip_2, int dest_ip_3, int dest_ip_4, int port, int gps_port, bool with_run=true);
  Status SetDestinationIp(std::shared_ptr<boost::asio::io_context> ctx, int dest_ip_1, int dest_ip_2, int dest_ip_3, int dest_ip_4, int port, int gps_port, bool with_run=true);
  Status SetDestinationIp(int dest_ip_1, int dest_ip_2, int dest_ip_3, int dest_ip_4, int port, int gps_port, bool with_run=true);
  Status SetControlPort(std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver,
    int ip_1, int ip_2, int ip_3, int ip_4,
    int mask_1, int mask_2, int mask_3, int mask_4,
    int gateway_1, int gateway_2, int gateway_3, int gateway_4,
    int vlan_flg, int vlan_id, bool with_run=true);
  Status SetControlPort(std::shared_ptr<boost::asio::io_context> ctx,
    int ip_1, int ip_2, int ip_3, int ip_4,
    int mask_1, int mask_2, int mask_3, int mask_4,
    int gateway_1, int gateway_2, int gateway_3, int gateway_4,
    int vlan_flg, int vlan_id, bool with_run=true);
  Status SetControlPort(
    int ip_1, int ip_2, int ip_3, int ip_4,
    int mask_1, int mask_2, int mask_3, int mask_4,
    int gateway_1, int gateway_2, int gateway_3, int gateway_4,
    int vlan_flg, int vlan_id, bool with_run=true);
  Status SetLidarRange(std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, int method, std::vector<unsigned char> data, bool with_run=true);
  Status SetLidarRange(std::shared_ptr<boost::asio::io_context> ctx, int method, std::vector<unsigned char> data, bool with_run=true);
  Status SetLidarRange(int method, std::vector<unsigned char> data, bool with_run=true);
  Status SetLidarRange(std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, int start, int end, bool with_run=true);
  Status SetLidarRange(std::shared_ptr<boost::asio::io_context> ctx, int start, int end, bool with_run=true);
  Status SetLidarRange(int start, int end, bool with_run=true);
  Status GetLidarRange(std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, std::function<void(HesaiLidarRangeAll &result)> callback, bool with_run=true);
  Status GetLidarRange(std::shared_ptr<boost::asio::io_context> ctx, std::function<void(HesaiLidarRangeAll &result)> callback, bool with_run=true);
  Status GetLidarRange(std::shared_ptr<boost::asio::io_context> ctx, bool with_run=true);
  Status GetLidarRange(std::function<void(HesaiLidarRangeAll &result)> callback, bool with_run=true);
  Status GetLidarRange(bool with_run=true);
  Status SetPtpConfig(std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver,
    int profile,
    int domain,
    int network,
    int logAnnounceInterval,
    int logSyncInterval,
    int logMinDelayReqInterval, bool with_run=true);
  Status SetPtpConfig(std::shared_ptr<boost::asio::io_context> ctx,
    int profile,
    int domain,
    int network,
    int logAnnounceInterval,
    int logSyncInterval,
    int logMinDelayReqInterval, bool with_run=true);
  Status SetPtpConfig(
    int profile,
    int domain,
    int network,
    int logAnnounceInterval,
    int logSyncInterval,
    int logMinDelayReqInterval, bool with_run=true);
  Status GetPtpConfig(std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, bool with_run=true);
  Status GetPtpConfig(std::shared_ptr<boost::asio::io_context> ctx, bool with_run=true);
  Status GetPtpConfig(bool with_run=true);
  Status SendReset(std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, bool with_run=true);
  Status SendReset(std::shared_ptr<boost::asio::io_context> ctx, bool with_run=true);
  Status SendReset(bool with_run=true);
  Status SetRotDir(std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, int mode, bool with_run=true);
  Status SetRotDir(std::shared_ptr<boost::asio::io_context> ctx, int mode, bool with_run=true);
  Status SetRotDir(int mode, bool with_run=true);
  Status GetLidarMonitor(std::shared_ptr<::drivers::tcp_driver::TcpDriver> target_tcp_driver, std::function<void(HesaiLidarMonitor &result)> callback, bool with_run=true);
  Status GetLidarMonitor(std::shared_ptr<boost::asio::io_context> ctx, std::function<void(HesaiLidarMonitor &result)> callback, bool with_run=true);
  Status GetLidarMonitor(std::shared_ptr<boost::asio::io_context> ctx, bool with_run=true);
  Status GetLidarMonitor(std::function<void(HesaiLidarMonitor &result)> callback, bool with_run=true);
  Status GetLidarMonitor(bool with_run=true);

  void IOContextRun();
  std::shared_ptr<boost::asio::io_context> GetIOContext();

  HesaiStatus SetSpinSpeedAsyncHttp(std::shared_ptr<boost::asio::io_context> ctx, uint16_t rpm);
  HesaiStatus SetSpinSpeedAsyncHttp(uint16_t rpm);
  HesaiStatus GetLidarMonitorAsyncHttp(std::shared_ptr<boost::asio::io_context> ctx, std::function<void(const std::string &str)> str_callback);
  HesaiStatus GetLidarMonitorAsyncHttp(std::function<void(const std::string &str)> str_callback);

  HesaiStatus CheckAndSetConfig(std::shared_ptr<HesaiSensorConfiguration> sensor_configuration, HesaiConfig hesai_config);
  HesaiStatus CheckAndSetConfig(std::shared_ptr<HesaiSensorConfiguration> sensor_configuration, HesaiLidarRangeAll hesai_lidar_range_all);
  HesaiStatus CheckAndSetConfig();

  void SetTargetModel(int model);

  bool UseHttpSetSpinRate(int model);
  bool UseHttpSetSpinRate();
  bool UseHttpGetLidarMonitor(int model);
  bool UseHttpGetLidarMonitor();

  void SetLogger(std::shared_ptr<rclcpp::Logger> node);

};
}  // namespace drivers
}  // namespace nebula

#endif  // NEBULA_HESAI_HW_INTERFACE_H
