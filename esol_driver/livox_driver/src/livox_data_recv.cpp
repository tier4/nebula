#include "livox_data_recv.hpp"

#include "LidarDriver/lidar_driver.hpp"
#include "LidarDriver/livox_common.hpp"

#include <cstring>

namespace lidar_driver
{
/// @brief receive before initialize.
void LidarDriver::DataRecvInit()
{
  lidar_device_.raw_data_type = 0xFF;
  lidar_device_.data_is_published = false;
  if (temp_publish_data_.get()) {
    temp_publish_data_->num = 0;
  }
  accumulate_count_ = 0;
  skip_start_packet_ = false;
}

/// @brief Raw data timestamp to nano sec.
/// @param timestamp : Raw data timestamp
/// @param timestamp_type : timestamp type
/// @return nano sec timestamp
uint64_t RawLdsStampToNs(LdsStamp & timestamp, uint8_t timestamp_type)
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
  if (lidar_device_.raw_data_type != data_type) {
    lidar_device_.raw_data_type = data_type;
    lidar_device_.packet_interval = GetPacketInterval(lidar_device_.device_info_type, data_type);
    lidar_device_.packet_interval_max = lidar_device_.packet_interval * 1.8f;
    lidar_device_.onetime_publish_packets =
      GetPacketNumPerSec(lidar_device_.device_info_type, data_type) * buffer_time_ms_ / 1000;
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
        time_base =
          RecvTimeBase(lidar_device_.statistic_info_imu, timestamp_type, cur_timestamp.stamp);
        time_stamp += time_base;
      }
      const livox_driver::LivoxImuPoint * ptr =
        reinterpret_cast<const livox_driver::LivoxImuPoint *>(&eth_packet->data[0]);
      publish_imu_packet_cb_(*ptr, time_stamp);
    }
  } else { /* if (PointDataType::kImu != eth_packet->data_type) */
    if (publish_lidar_data_cb_) {
      if (timestamp_type == TimestampType::kPps) {
        time_base =
          RecvTimeBase(lidar_device_.statistic_info_data, timestamp_type, cur_timestamp.stamp);
        time_stamp += time_base;
      }
      UpdateLidarInfoByEthPacket(eth_packet->data_type);
      UpdateLidarStatusCode(eth_packet->err_code);
      publish_lidar_data_cb_(buff, pkt_len, time_stamp, data_cnt);
    }
  }

  return;
}

void LidarDriver::UpdateLidarStatusCode(uint32_t lidar_status_code)
{
  lidar_device_.lidar_status_code = lidar_status_code;  // update lidar status
  lidar_device_.status_code_ready_ = true;
}

/// @brief data port receive
void LidarDriver::LivoxHwRxInterfaceData()
{
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
  // printf("Thread end %s\n", __func__);
  return;
}

/// @brief imu port receive
void LidarDriver::LivoxHwRxInterfaceImu()
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
  // printf("Thread end %s\n", __func__);
  return;
}

/// @brief set start timestamp to last_timestamp_
/// @param timestamp : start timestamp
/// @retval true: success
/// @retval false: skip start packet
bool LidarDriver::SetLastTimeStamp(uint64_t timestamp)
{
  uint32_t remaining_time = timestamp % publish_period_ns_;
  uint32_t diff_time = publish_period_ns_ - remaining_time;

  if (skip_start_packet_) {
    if (last_remaning_time_ > remaining_time) {
      skip_start_packet_ = false;
    } else if (diff_time <= lidar_device_.packet_interval) {
      skip_start_packet_ = false;
    } else {
      last_remaning_time_ = remaining_time;
      return false;
    }
  }

  /** Get start time, down to the period boundary */
  if (diff_time > (publish_period_ns_ / 4)) {
    // printf("GetPublishStartTime : 0 : diff_time=%u timestamp=%lu\n", diff_time, timestamp);
    last_timestamp_ = timestamp - remaining_time;
  } else if (diff_time <= lidar_device_.packet_interval_max) {
    // printf("GetPublishStartTime : 1 : diff_time=%u timestamp=%lu\n", diff_time, timestamp);
    last_timestamp_ = timestamp;
  } else {
    // printf("GetPublishStartTime : 2 : diff_time=%u timestamp=%lu\n", diff_time, timestamp);
    /* the remaning packets in queue maybe not enough after skip */
    last_remaning_time_ = remaining_time;
    skip_start_packet_ = true;
    return false;
  }

  return true;
}

/// @brief packet data parse
/// @retval 0: accumulating
/// @retval >0: Accumulation completed data count
int LidarDriver::ParsePacket(livox_driver::LivoxLidarPacket & packet)
{
  // printf("parse packet packet.time %ld  temp.time %ld \n", packet.time,
  // temp_publish_data_->time);
  int iret = 0;
  uint64_t timestamp = packet.time_stamp;
  bool timegap_over = false;

  if (!temp_publish_data_) {
    temp_publish_data_ = std::make_unique<livox_driver::LivoxPublishData>();
  }

  if (temp_publish_data_->num == 0) {
    if (!SetLastTimeStamp(timestamp)) {
      return 0;
    }
  }

  do {
    int64_t time_gap = timestamp - last_timestamp_;
    // if timeout -> packet publish
    if (time_gap > lidar_device_.packet_interval_max && lidar_device_.data_is_published) {
      // printf( "NG:time_gap=%ld timestamp=%ld last_timestamp=%ld\n", time_gap, timestamp,
      // last_timestamp_ );
      packet.data.clear();
      last_timestamp_ += lidar_device_.packet_interval;
      timegap_over = true;
    } else {
      // printf( "kOk:time_gap=%ld timestamp=%ld last_timestamp=%ld\n", time_gap, timestamp,
      // last_timestamp_ );
      last_timestamp_ = timestamp;
      timegap_over = false;
    }

    if (temp_publish_data_->num == 0) {
      /* new timestamp */
      temp_publish_data_->time = timestamp;
      temp_publish_data_->data.resize(
        lidar_device_.onetime_publish_packets * kMaxPointPerEthPacket *
        sizeof(livox_driver::LivoxPointXyzrtl));
    }

    uint8_t * point_base =
      &temp_publish_data_->data[temp_publish_data_->num * sizeof(livox_driver::LivoxPointXyzrtl)];

    LivoxExtendRawPointToPxyzrtl(point_base, packet.data, lidar_device_.line_num);
    temp_publish_data_->num += packet.data_cnt;

    accumulate_count_++;

    if (accumulate_count_ >= lidar_device_.onetime_publish_packets) {
      uint32_t echo_num = GetEchoNumPerPoint(lidar_device_.raw_data_type);
      temp_publish_data_->num = temp_publish_data_->num * echo_num;
      publish_data_ = std::move(temp_publish_data_);
      iret = publish_data_->num;
      lidar_device_.data_is_published = true;

      accumulate_count_ = 0;
      skip_start_packet_ = false;
      break;
    }
  } while (timegap_over);

  return iret;
}

/// @brief return publish data
std::unique_ptr<livox_driver::LivoxPublishData> LidarDriver::GenerateCloud()
{
  return std::move(publish_data_);
}

/// @brief Extend raw point to point xyzrtl.
void LidarDriver::LivoxExtendRawPointToPxyzrtl(
  uint8_t * point_buf, const std::vector<uint8_t> & raw_packet, uint32_t line_num)
{
  const LivoxEthPacket * eth_packet = reinterpret_cast<const LivoxEthPacket *>(raw_packet.data());
  livox_driver::LivoxPointXyzrtl * dst_point =
    reinterpret_cast<livox_driver::LivoxPointXyzrtl *>(point_buf);
  uint32_t points_per_packet = GetPointsPerPacket(eth_packet->data_type);
  const LivoxExtendRawPoint * raw_point =
    reinterpret_cast<const LivoxExtendRawPoint *>(eth_packet->data);

  uint8_t line_id = 0;
  while (points_per_packet) {
    dst_point->x = raw_point->x / 1000.0f;
    dst_point->y = raw_point->y / 1000.0f;
    dst_point->z = raw_point->z / 1000.0f;
    dst_point->reflectivity = static_cast<float>(raw_point->reflectivity);
    dst_point->tag = raw_point->tag;
    if (line_num > 1) {
      dst_point->line = line_id % line_num;
    } else {
      dst_point->line = 0;
    }
    ++raw_point;
    ++dst_point;
    ++line_id;
    --points_per_packet;
  }

  return;
}

}  // namespace lidar_driver
