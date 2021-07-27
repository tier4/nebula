#ifndef LIDARDRIVER_LIVOX_LIDARDRIVER_HPP_
#define LIDARDRIVER_LIVOX_LIDARDRIVER_HPP_

#include <functional>
#include <thread>

#include "HwInterface/udp_socket.hpp"
#include "LidarDriver/livox_common.hpp"
#include "third_party/FastCRC/FastCRC.h"

namespace lidar_driver
{
const uint16_t kCommandPort = 65000;
const uint8_t kCommandRxCommunicationSettingAck = 0x01;
const uint8_t kCommancRxStreamingAck = 0x04;
const uint8_t kCommandRxHeartbeatAck = 0x03;
const uint16_t kCrcSeed16 = 0x4c49;
const uint32_t kCrcSeed32 = 0x564f580a;
const uint32_t kSdkPacketCrcSize = 4;
const uint32_t kSdkPacketPreambleCrcSize = 2;
const uint32_t kMaxPointPerEthPacket = 100;

enum class DriverStatus {
  kRunning = 0,
  kTerminate,
};

enum class LidarDriverStatus {
  kDisconnect = 0,
  kConnect,
  kStreaming,
};

enum class LidarCommandType {
  kLidarCommandHandshake = 0,
  kLidarCommandStartStreaming,
  kLidarCommandStopStreaming,
  kLidarCommandHeartbeat,
};

#pragma pack(1)
struct CommandHeader
{
  uint8_t sof;
  uint8_t version;
  uint16_t length;
  uint8_t cmd_type;
  uint16_t seq_num;
  uint16_t crc16;
};
#pragma pack()

/** Point cloud packet. */
#pragma pack(1)
struct LivoxEthPacket
{
  uint8_t version;        /**< Packet protocol version. */
  uint8_t slot;           /**< Slot number used for connecting LiDAR. */
  uint8_t id;             /**< LiDAR id. */
  uint8_t rsvd;           /**< Reserved. */
  uint32_t err_code;      /**< Device error status indicator information. */
  uint8_t timestamp_type; /**< Timestamp type. */
  /** Point cloud coordinate format, refer to \ref PointDataType . */
  uint8_t data_type;
  uint8_t timestamp[8]; /**< Nanosecond or UTC format timestamp. */
  uint8_t data[1];      /**< Point cloud data. */
};
#pragma pack()

/** Cartesian coordinate format. */
#pragma pack(1)
struct LivoxRawPoint
{
  int32_t x;            /**< X axis, Unit:mm */
  int32_t y;            /**< Y axis, Unit:mm */
  int32_t z;            /**< Z axis, Unit:mm */
  uint8_t reflectivity; /**< Reflectivity */
};
#pragma pack()

/** Standard point cloud format */
#pragma pack(1)
struct LivoxPoint
{
  float x;              /**< X axis, Unit:m */
  float y;              /**< Y axis, Unit:m */
  float z;              /**< Z axis, Unit:m */
  uint8_t reflectivity; /**< Reflectivity */
  uint8_t tag;
};
#pragma pack()

#pragma pack(1)
struct CommandHandshake
{
  uint8_t cmd_set;
  uint8_t cmd_id;
  uint32_t host_ip;
  uint16_t data_port;
  uint16_t command_port;
  uint16_t imu_port;
};
#pragma pack()

#pragma pack(1)
struct CommandHeartbeat
{
  uint8_t cmd_set;
  uint8_t cmd_id;
};
#pragma pack()

#pragma pack(1)
struct CommandSampling
{
  uint8_t cmd_set;
  uint8_t cmd_id;
  uint8_t sample_ctrl;
};
#pragma pack()

#pragma pack(1)
struct CommandAck
{
  uint8_t cmd_set;
  uint8_t cmd_id;
  uint8_t result;
};
#pragma pack()

using EulerAngle = float [3];        /**< Roll, Pitch, Yaw, unit:radian. */
using TranslationVector = float [3]; /**< x, y, z translation, unit: m. */
using RotationMatrix = float [3][3];

#pragma pack(1)
struct ExtrinsicParameter
{
  EulerAngle euler;
  TranslationVector trans;
  RotationMatrix rotation;
  bool enable;
};
#pragma pack()

class LidarDriver
{
public:
  std::unique_ptr<livox_driver::LivoxPublishData> temp_publish_data_;
  std::unique_ptr<livox_driver::LivoxPublishData> publish_data_;
  livox_driver::LivoxSensorConfiguration lidarconfig_;

private:
  volatile DriverStatus driverstatus_;
  volatile LidarDriverStatus lidarstatus_;

  HwInterface::UdpSocket commandsock_;
  HwInterface::UdpSocket rx_datasock_;
  HwInterface::UdpSocket rx_imusock_;

  std::shared_ptr<std::thread> thread_rxdata_;
  std::shared_ptr<std::thread> thread_rximu_;
  std::shared_ptr<std::thread> thread_rxcommand_;
  std::shared_ptr<std::thread> thread_txheartbeat_;

  FastCRC16 crc16_;
  FastCRC32 crc32_;

public:
  LidarDriver();
  ~LidarDriver();

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

  bool SetCalibration();
  bool GetCalibration();

  int StartHwRxInterface();
  void StopHwRxInterface();

  int StartHwTxInterface();
  void StopHwTxInterface();

  int ParsePacket(livox_driver::LivoxLidarPacket & packet);
  std::unique_ptr<livox_driver::LivoxPublishData> GenerateCloud();

  bool Initialize();

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

private:
  bool LivoxHwRxInterfaceData();
  bool LivoxHwRxInterfaceCommand();
  bool LivoxHwRxInterfaceImu();
  bool LivoxHwTxInterfaceHeartbeat();

  bool SendCommand(LidarCommandType command);
  void MakeCommandHandshake(std::vector<uint8_t> & buff);
  void MakeCommandStartStream(std::vector<uint8_t> & buff);
  void MakeCommandStopStream(std::vector<uint8_t> & buff);
  void MakeCommandHeartbeat(std::vector<uint8_t> & buff);

  bool UdpSocketOpen(
    HwInterface::UdpSocket & sock, const std::string & sensor_ip, uint16_t sensor_port,
    uint16_t my_port);
  void CloseAllUdpSocket();
  bool CheckRecvData(const std::vector<uint8_t> & buff);

private:
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

  /** 8bytes stamp to uint64_t stamp */
  union LdsStamp {
    struct
    {
      uint32_t low;
      uint32_t high;
    } stamp_word;
    std::array<uint8_t, 8> stamp_bytes;
    int64_t stamp;
  };

  const size_t kMaxBufferSize = 2048;
  uint32_t buffer_time_ms_;
  uint32_t publish_period_ns_;
  uint32_t accumulate_count_;
  uint64_t last_timestamp_;
  volatile int stream_start_ack_;
  PublishLidarDataCallback publish_lidar_data_cb_;
  PublishImuPacketCallback publish_imu_packet_cb_;
  livox_driver::LivoxCloudConfiguration cloud_config_;

  ExtrinsicParameter extrinsic_;

  void DataRecvInit();
  uint64_t RawLdsStampToNs(LdsStamp & timestamp, uint8_t timestamp_type);
  int64_t RecvTimeBase(
    LidarPacketStatistic & packet_statistic, uint8_t timestamp_type, int64_t cur_timestamp_stamp);
  void StorageRawPacket(const std::vector<uint8_t> & buff, int rcv_len);
  void UpdateLidarInfoByEthPacket(uint8_t data_type);

  void EulerAnglesToRotationMatrix(EulerAngle euler, RotationMatrix matrix);
  uint8_t * LivoxExtendRawPointToPxyzrtl(
    uint8_t * point_buf, LivoxEthPacket * eth_packet, ExtrinsicParameter & extrinsic,
    uint32_t line_num);
  void PointExtrisincCompensation(
    livox_driver::PointXyz * dst_point, const livox_driver::PointXyz & src_point,
    ExtrinsicParameter & extrinsic);

  inline void RawPointConvert(
    livox_driver::LivoxPointXyzrtl & dst_point, const LivoxRawPoint & raw_point)
  {
    dst_point.x = raw_point.x / 1000.0f;
    dst_point.y = raw_point.y / 1000.0f;
    dst_point.z = raw_point.z / 1000.0f;
    dst_point.reflectivity = (float)raw_point.reflectivity;
  }
  inline bool IsTripleFloatNoneZero(float x, float y, float z)
  {
    return ((x != 0.0f) || (y != 0.0f) || (z != 0.0f));
  }

  inline bool IsTripleIntNoneZero(int32_t x, int32_t y, int32_t z) { return (x | y | z); }

#ifdef UTEST
public:
  friend class TestFriend_LivoxDriver;
#endif  // UTEST
};

}  // namespace lidar_driver
#endif  // LIDARDRIVER_LIVOX_LIDARDRIVER_HPP_
