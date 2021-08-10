#ifndef LIDARDRIVER_LIVOX_LIDARDRIVER_HPP_
#define LIDARDRIVER_LIVOX_LIDARDRIVER_HPP_

#include <functional>
#include <mutex>
#include <thread>

#include "HwInterface/udp_socket.hpp"
#include "LidarDriver/livox_command.hpp"
#include "LidarDriver/livox_common.hpp"
#include "third_party/FastCRC/FastCRC.h"

namespace lidar_driver
{
class LidarDriver
{
public:
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

  using PublishLidarDataCallback = std::function<void(
    const std::vector<uint8_t> & buff, int pkt_len, uint64_t timestamp, uint32_t point_num)>;
  void SetPublishLidarDataFunc(const PublishLidarDataCallback & func)
  {
    publish_lidar_data_cb_ = func;
  }
  using PublishImuPacketCallback =
    std::function<void(const livox_driver::LivoxImuPoint & imu_pkt, uint64_t timestamp)>;
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
  volatile DriverStatus driverstatus_{DriverStatus::kTerminate};

  enum class LidarDriverStatus {
    kDisconnect = 0,
    kConnect,
    kStreaming,
  };
  volatile LidarDriverStatus lidarstatus_{LidarDriverStatus::kDisconnect};

  std::vector<uint8_t> MakeSendCommand(LidarCommandType cmd_type);
  CommandHandshake MakeCommandHandshake();
  CommandSampling MakeCommandStartStream();
  CommandSampling MakeCommandStopStream();
  CommandHeartbeat MakeCommandHeartbeat();
  CommandResult SendAckWait(std::vector<uint8_t> & snd_buff, GeneralCommandID cmd_id);
  GeneralCommandID GetCommandId(LidarCommandType cmd_type);

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
    LidarPacketStatistic statistic_info_data;
    LidarPacketStatistic statistic_info_imu;
  } lidars_;

  PublishLidarDataCallback publish_lidar_data_cb_;
  PublishImuPacketCallback publish_imu_packet_cb_;

  void DataRecvInit();
  int64_t RecvTimeBase(
    LidarPacketStatistic & packet_statistic, uint8_t timestamp_type, int64_t cur_timestamp_stamp);
  void StorageRawPacket(const std::vector<uint8_t> & buff, int rcv_len);
  void UpdateLidarInfoByEthPacket(uint8_t data_type);

private:  // Parse
  uint32_t buffer_time_ms_;
  uint32_t publish_period_ns_;
  uint64_t last_timestamp_;
  uint32_t accumulate_count_;

  std::unique_ptr<livox_driver::LivoxPublishData> temp_publish_data_;
  std::unique_ptr<livox_driver::LivoxPublishData> publish_data_;

  void LivoxExtendRawPointToPxyzrtl(
    uint8_t * point_buf, const std::vector<uint8_t> & raw_packet, uint32_t line_num);

#ifdef UTEST
public:
  friend class TestFriend_LivoxDriver;
#endif  // UTEST
};

}  // namespace lidar_driver
#endif  // LIDARDRIVER_LIVOX_LIDARDRIVER_HPP_
