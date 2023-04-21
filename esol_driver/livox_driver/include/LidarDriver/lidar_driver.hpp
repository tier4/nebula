#ifndef LIDARDRIVER_LIVOX_LIDARDRIVER_HPP_
#define LIDARDRIVER_LIVOX_LIDARDRIVER_HPP_

#include "HwInterface/udp_socket.hpp"
#include "LidarDriver/livox_command.hpp"
#include "LidarDriver/livox_common.hpp"
#include "third_party/FastCRC/FastCRC.h"

#include <functional>
#include <map>
#include <mutex>
#include <thread>

namespace lidar_driver
{
class LidarDriver
{
public:
  enum class LidarDriverStatus {
    kDisconnect = 0,
    kConnect,
    kStreaming,
  };
  const std::map<LidarDriverStatus, const char *> lidar_driver_status_dict_ = {
    {LidarDriverStatus::kDisconnect, "Disconnected"},
    {LidarDriverStatus::kConnect, "Connected"},
    {LidarDriverStatus::kStreaming, "Streaming"}};

  using PublishLidarDataCallback = std::function<void(
    const std::vector<uint8_t> & buff, int pkt_len, uint64_t timestamp, uint32_t point_num)>;
  using PublishImuPacketCallback =
    std::function<void(const livox_driver::LivoxImuPoint & imu_pkt, uint64_t timestamp)>;
  LidarDriver();
  virtual ~LidarDriver();

  bool SetConfiguration(const livox_driver::LivoxSensorConfiguration & param);
  bool SetConfiguration(const livox_driver::LivoxCloudConfiguration & param)
  {
    cloud_config_ = param;
    return true;
  }

  bool GetConfiguration(livox_driver::LivoxSensorConfiguration & param);
  bool GetConfiguration(livox_driver::LivoxCloudConfiguration & param)
  {
    param = cloud_config_;
    return true;
  }

  bool SetCalibration() { return true; };
  bool GetCalibration() { return true; };

  bool StartHwRxInterface();
  void StopHwRxInterface();

  CommandResult StartHwTxInterface();
  void StopHwTxInterface();

  int ParsePacket(livox_driver::LivoxLidarPacket & packet);
  std::unique_ptr<livox_driver::LivoxPublishData> GenerateCloud();

  bool Initialize();

  LidarDriverStatus GetLidarStatus() { return lidarstatus_; }

  void SetPublishLidarDataFunc(const PublishLidarDataCallback & func)
  {
    publish_lidar_data_cb_ = func;
  }

  void SetPublishImuPacketFunc(const PublishImuPacketCallback & func)
  {
    publish_imu_packet_cb_ = func;
  }

private:  // Resource, Configuration
  HwInterface::UdpSocket commandsock_;
  HwInterface::UdpSocket rx_datasock_;
  HwInterface::UdpSocket rx_imusock_;

  std::shared_ptr<std::thread> thread_txheartbeat_;
  std::shared_ptr<std::thread> thread_rxcommand_;
  std::shared_ptr<std::thread> thread_rxdata_;
  std::shared_ptr<std::thread> thread_rximu_;

  void LivoxHwTxInterfaceHeartbeat();
  void LivoxHwRxInterfaceCommand();
  void LivoxHwRxInterfaceData();
  void LivoxHwRxInterfaceImu();

  livox_driver::LivoxSensorConfiguration lidarconfig_;
  livox_driver::LivoxCloudConfiguration cloud_config_;

  bool UdpSocketOpen(
    HwInterface::UdpSocket & sock, const std::string & sensor_ip, uint16_t sensor_port,
    uint16_t my_port);
  void CloseAllUdpSocket();

private:  // Status, Command
  enum class DriverStatus {
    kRunning = 0,
    kTerminate,
  };
  const std::map<DriverStatus, const char *> driver_status_dict_ = {
    {DriverStatus::kRunning, "Running"}, {DriverStatus::kTerminate, "Not Running"}};
  volatile DriverStatus driverstatus_{DriverStatus::kTerminate};

  volatile LidarDriverStatus lidarstatus_{LidarDriverStatus::kDisconnect};

  std::vector<uint8_t> MakeSendCommand(LidarCommandType cmd_type);
  CommandHandshake MakeCommandHandshake();
  static CommandSampling MakeCommandStartStream();
  static CommandSampling MakeCommandStopStream();
  static CommandHeartbeat MakeCommandHeartbeat();
  CommandResult SendAckWait(std::vector<uint8_t> & snd_buff, GeneralCommandID cmd_id);
  static GeneralCommandID GetCommandId(LidarCommandType cmd_type);

  FastCRC16 crc16_{kCrcSeed16};
  FastCRC32 crc32_{kCrcSeed32};
  void SetCrc16Crc32(std::vector<uint8_t> & buff, GeneralCommandID cmd_id);
  bool CheckRecvData(const std::vector<uint8_t> & buff);

  CountingSemaphore semaphore_;
  std::mutex mtx_;

  uint16_t snd_seq_num_;

private:  // Receive
  enum DeviceType {
    kHub = 0,          /**< Livox Hub. */
    kLidarMid40 = 1,   /**< Mid-40. */
    kLidarTele = 2,    /**< Tele. */
    kLidarHorizon = 3, /**< Horizon. */
    kLidarMid70 = 6,   /**< Livox Mid-70. */
    kLidarAvia = 7     /**< Avia. */
  };

  struct LidarPacketStatistic
  {
    int64_t last_timestamp;
    int64_t timebase; /**< unit:ns */
    uint32_t timebase_state;
  };

  /** Lidar data source info abstract */
  struct LidarDevice
  {
    uint8_t raw_data_type; /**< The data type in eth packaet */
    volatile uint32_t
      packet_interval; /**< The time interval between packets of current lidar, unit:ns */
    volatile uint32_t packet_interval_max; /**< If more than it, have packet loss */
    volatile uint32_t onetime_publish_packets;
    DeviceType device_info_type;
    uint32_t line_num;
    LidarPacketStatistic statistic_info_data;
    LidarPacketStatistic statistic_info_imu;
    uint32_t lidar_status_code; /**< Stores the lidar device status */
    bool status_code_ready_{};
    bool data_is_published;
  } lidar_device_;

  PublishLidarDataCallback publish_lidar_data_cb_;
  PublishImuPacketCallback publish_imu_packet_cb_;

  void DataRecvInit();
  int64_t RecvTimeBase(
    LidarPacketStatistic & packet_statistic, uint8_t timestamp_type, int64_t cur_timestamp_stamp);
  void StorageRawPacket(const std::vector<uint8_t> & buff, int rcv_len);
  void UpdateLidarInfoByEthPacket(uint8_t data_type);
  /**
   * Updates the LivoxStatusCode union (error_status) from Livox Ethernet Packet, and sets the
   * received flag to true.
   * @param lidar_status_code The 32 bit word received in the packet containing the error status.
   */
  void UpdateLidarStatusCode(uint32_t lidar_status_code);

private:  // Parse
  uint32_t buffer_time_ms_;
  uint32_t publish_period_ns_;
  uint64_t last_timestamp_;
  uint32_t accumulate_count_;
  bool skip_start_packet_;
  uint32_t last_remaning_time_;

  std::unique_ptr<livox_driver::LivoxPublishData> temp_publish_data_;
  std::unique_ptr<livox_driver::LivoxPublishData> publish_data_;

  bool SetLastTimeStamp(uint64_t timestamp);
  void LivoxExtendRawPointToPxyzrtl(
    uint8_t * point_buf, const std::vector<uint8_t> & raw_packet, uint32_t line_num);

public:
  bool GetLidarStatusCode(uint32_t & out_lidar_status_code) const
  {
    if (lidar_device_.status_code_ready_) {
      out_lidar_status_code = lidar_device_.lidar_status_code;
      return true;
    }
    return false;
  }
#ifdef UTEST
public:
  friend class TestFriend_LivoxDriver;
#endif  // UTEST
};

}  // namespace lidar_driver
#endif  // LIDARDRIVER_LIVOX_LIDARDRIVER_HPP_
