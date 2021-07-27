#include <cstring>

#include "LidarDriver/lidar_driver.hpp"
#include "LidarDriver/livox_common.hpp"
#include "livox_data_recv.hpp"

namespace lidar_driver
{
/// @brief receive before initialize.
void LidarDriver::DataRecvInit()
{
  lidars_.raw_data_type = 0xFF;
  lidars_.device_info_type = DeviceType::kLidarHorizon;
}

/// @brief Raw data timestamp to nano sec.
/// @param timestamp : Raw data timestamp
/// @param timestamp_type : timestamp type
/// @return nano sec timestamp
uint64_t LidarDriver::RawLdsStampToNs(LdsStamp & timestamp, uint8_t timestamp_type)
{
  uint64_t time_epoch = timestamp.stamp;

  switch (timestamp_type) {
    case TimestampType::kPps:
      break;

    case TimestampType::kNoSync:
      break;

    case TimestampType::kPtp:
      break;

    case TimestampType::kPpsGps:
      struct tm time_utc;
      time_utc.tm_isdst = 0;
      time_utc.tm_year = timestamp.stamp_bytes[0] + 100;  // map 2000 to 1990
      time_utc.tm_mon = timestamp.stamp_bytes[1] - 1;     // map 1~12 to 0~11
      time_utc.tm_mday = timestamp.stamp_bytes[2];
      time_utc.tm_hour = timestamp.stamp_bytes[3];
      time_utc.tm_min = 0;
      time_utc.tm_sec = 0;

      // uint64_t time_epoch = mktime(&time_utc);
      time_epoch = timegm(&time_utc);                                 // no timezone
      time_epoch = time_epoch * 1000000 + timestamp.stamp_word.high;  // to us
      time_epoch = time_epoch * 1000;                                 // to ns
      break;

    default:
      printf("Timestamp type[%d] invalid.\n", timestamp_type);
      time_epoch = 0;
      break;
  }

  return time_epoch;
}

/// @brief timestamp base value
/// @param packet_statistic : keep data
/// @param timestamp_type : timestamp type
/// @param cur_timestamp_stamp : receive packet timestamp
/// @return time base value
int64_t LidarDriver::RecvTimeBase(
  LidarPacketStatistic & packet_statistic, uint8_t timestamp_type, int64_t cur_timestamp_stamp)
{
  static const int64_t kPacketTimeGap = 1000000; /**< 1ms = 1000000ns */

  if (timestamp_type == TimestampType::kPps) {
    /** Whether a new sync frame */
    if (
      (cur_timestamp_stamp < packet_statistic.last_timestamp) &&
      (cur_timestamp_stamp < kPacketTimeGap)) {
      auto cur_time = std::chrono::high_resolution_clock::now();
      int64_t sync_time = cur_time.time_since_epoch().count();
      /** used receive time as timebase */
      packet_statistic.timebase = sync_time;
    }
  }
  packet_statistic.last_timestamp = cur_timestamp_stamp;
  return packet_statistic.timebase;
}

/// @brief timestamp base value
/// @param data_type : raw data type
void LidarDriver::UpdateLidarInfoByEthPacket(uint8_t data_type)
{
  if (lidars_.raw_data_type != data_type) {
    lidars_.raw_data_type = data_type;
    lidars_.packet_interval = GetPacketInterval(lidars_.device_info_type, data_type);
    lidars_.packet_interval_max = lidars_.packet_interval * 1.8f;
    lidars_.onetime_publish_packets =
      GetPacketNumPerSec(lidars_.device_info_type, data_type) * buffer_time_ms_ / 1000;
  }
}

/// @brief receive data check & publis RosWrapper
/// @param buff : receive data.
/// @param rcv_len : receive data length.
void LidarDriver::StorageRawPacket(const std::vector<uint8_t> & buff, int rcv_len)
{
  static const uint64_t kRosTimeMax = 4294967296000000000; /**< 2^32 * 1000000000ns */
  const LivoxEthPacket * eth_packet = reinterpret_cast<const LivoxEthPacket *>(buff.data());
  int32_t point_num = GetPointsPerPacket(eth_packet->data_type);
  int32_t pkt_len = GetEthPacketLen(eth_packet->data_type);

  int data_cnt = RecvDataCnt(eth_packet->data_type, rcv_len);
  if (data_cnt <= 0) {
    return;
  } else if (data_cnt == point_num) {
  } else {
    printf("WARNING GetEthPacketLen(%d) != rcv_len(%d)\n", pkt_len, rcv_len);
    printf("WARNING data_cnt(%d) != point_num(%d)\n", data_cnt, point_num);
    pkt_len = GetRealPacketLen(eth_packet->data_type, data_cnt);
  }

  uint8_t timestamp_type = eth_packet->timestamp_type;
  uint64_t time_base;
  LdsStamp cur_timestamp;

  std::memcpy(cur_timestamp.stamp_bytes.data(), eth_packet->timestamp, sizeof(cur_timestamp));
  uint64_t time_stamp = RawLdsStampToNs(cur_timestamp, timestamp_type);
  if (time_stamp >= kRosTimeMax) {
    printf("Raw EthPacket time out of range Lidar[%ld]\n", time_stamp);
    return;
  }

  if (eth_packet->data_type == PointDataType::kImu) {
    if (publish_imu_packet_cb_) {
      if (timestamp_type == TimestampType::kPps) {
        time_base = RecvTimeBase(lidars_.statistic_info_imu, timestamp_type, cur_timestamp.stamp);
        time_stamp += time_base;
      }
      const livox_driver::LivoxImuPoint * ptr =
        reinterpret_cast<const livox_driver::LivoxImuPoint *>(&eth_packet->data[0]);
      publish_imu_packet_cb_(*ptr, time_stamp);
    }
  } else { /* if (PointDataType::kImu != eth_packet->data_type) */
    if (publish_lidar_data_cb_) {
      if (timestamp_type == TimestampType::kPps) {
        time_base = RecvTimeBase(lidars_.statistic_info_data, timestamp_type, cur_timestamp.stamp);
        time_stamp += time_base;
      }
      UpdateLidarInfoByEthPacket(eth_packet->data_type);
      publish_lidar_data_cb_(buff, pkt_len, time_stamp, data_cnt);
    }
  }

  return;
}

/// @brief data port receive
bool LidarDriver::LivoxHwRxInterfaceData(void)
{
  DataRecvInit();

  while (driverstatus_ == DriverStatus::kRunning) {
    std::vector<uint8_t> buff(kMaxBufferSize);
    int rcv_len = rx_datasock_.Recv(buff);
    if (lidarstatus_ != LidarDriverStatus::kStreaming) {
      rcv_len = rx_datasock_.Recv(buff, 500);
    } else if (rcv_len < 0) {
      printf("socket receive error val:%d :%s\n", rcv_len, __func__);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));  // ToDo
    } else if (rcv_len > 0) {
      StorageRawPacket(buff, rcv_len);
    } else { /* nothing */
    }
  }

  return true;
}

/// @brief imu port receive
bool LidarDriver::LivoxHwRxInterfaceImu(void)
{
  while (driverstatus_ == DriverStatus::kRunning) {
    std::vector<uint8_t> buff(kMaxBufferSize);
    int rcv_len = rx_imusock_.Recv(buff);
    if (lidarstatus_ != LidarDriverStatus::kStreaming) {
    } else if (rcv_len < 0) {
      printf("socket receive error val:%d :%s\n", rcv_len, __func__);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));  // ToDo
    } else if (rcv_len > 0) {
      StorageRawPacket(buff, rcv_len);
    } else { /* nothing */
    }
  }

  return true;
}

}  // namespace lidar_driver
