#include "LidarDriver/lidar_driver.hpp"
#include "livox_data_recv.hpp"

#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

namespace lidar_driver
{
/// @brief constractor
LidarDriver::LidarDriver()
: driverstatus_(DriverStatus::kRunning),
  lidarstatus_(LidarDriverStatus::kDisconnect),
  commandsock_(),
  rx_datasock_(),
  rx_imusock_(),
  crc16_(kCrcSeed16),
  crc32_(kCrcSeed32)
{
}

/// @brief destractor
LidarDriver::~LidarDriver() { CloseAllUdpSocket(); }

/// @brief set livox configration
bool LidarDriver::SetConfiguration(const livox_driver::LivoxSensorConfiguration & param)
{
  bool is_success = false;

  CloseAllUdpSocket();

  is_success = UdpSocketOpen(commandsock_, param.sensor_ip, kCommandPort, param.cmd_port);
  if (is_success != false) {
    is_success = UdpSocketOpen(rx_datasock_, param.sensor_ip, 0, param.data_port);
  }
  if (is_success != false) {
    is_success = UdpSocketOpen(rx_imusock_, param.sensor_ip, 0, param.imu_port);
  }

  if (is_success == false) {
    CloseAllUdpSocket();
    return false;
  }

  const int64_t kNsPerSecond = 1000000000; /**< 1s  = 1000000000ns */
  buffer_time_ms_ = param.frequency_ms;
  publish_period_ns_ = kNsPerSecond / (1000 / param.frequency_ms);
  lidarconfig_ = param;

  // set extrinsic parameter / (beta) -start
  float roll = -0.001;
  float pitch = 0.015;
  float yaw = -0.0364;
  float trans_x = 0.9;
  float trans_y = 0;
  float trans_z = 2.0;

  extrinsic_.euler[0] = static_cast<float>(roll * M_PI / 180.0);
  extrinsic_.euler[1] = static_cast<float>(pitch * M_PI / 180.0);
  extrinsic_.euler[2] = static_cast<float>(yaw * M_PI / 180.0);
  extrinsic_.trans[0] = static_cast<float>(trans_x / 1000.0);
  extrinsic_.trans[1] = static_cast<float>(trans_y / 1000.0);
  extrinsic_.trans[2] = static_cast<float>(trans_z / 1000.0);
  EulerAnglesToRotationMatrix(extrinsic_.euler, extrinsic_.rotation);
  extrinsic_.enable = true;
  // set extrinsic parameter / (beta) -end

  return is_success;
}

bool LidarDriver::GetConfiguration(livox_driver::LivoxSensorConfiguration & param)
{
  param = lidarconfig_;
  return true;
}

/// @brief set command data
/// @param command  : command type
std::vector<uint8_t> LidarDriver::MakeSendCommand(LidarCommandType command)
{
  std::vector<uint8_t> buff(kMaxBufferSize);
  CommandAll * command_all = reinterpret_cast<CommandAll *>(buff.data());

  command_all->header.sof = 0xAA;
  command_all->header.version = 0x01;
  command_all->header.cmd_type = 0x00;
  command_all->header.seq_num = 0x00;

  uint32_t * crc32_ptr;
  uint16_t cmd_size;
  switch (command) {
    case LidarCommandType::kLidarCommandHandshake:
      command_all->data.hand_shake = MakeCommandHandshake();
      crc32_ptr = &command_all->data.hand_shake.crc32;
      cmd_size = sizeof(command_all->data.hand_shake);
      break;
    case LidarCommandType::kLidarCommandStartStreaming:
      command_all->data.sampling = MakeCommandStartStream();
      crc32_ptr = &command_all->data.sampling.crc32;
      cmd_size = sizeof(command_all->data.sampling);
      break;
    case LidarCommandType::kLidarCommandStopStreaming:
      command_all->data.sampling = MakeCommandStopStream();
      crc32_ptr = &command_all->data.sampling.crc32;
      cmd_size = sizeof(command_all->data.sampling);
      break;
    case LidarCommandType::kLidarCommandHeartbeat:
      command_all->data.heart_beat = MakeCommandHeartbeat();
      crc32_ptr = &command_all->data.heart_beat.crc32;
      cmd_size = sizeof(command_all->data.heart_beat);
      break;
  }

  uint16_t header_size = sizeof(command_all->header);
  uint16_t data_size = header_size + cmd_size;
  command_all->header.length = data_size;

  uint8_t * crcbuf = reinterpret_cast<uint8_t *>(&command_all->header);
  uint32_t crc16 = crc16_.mcrf4xx_calc(crcbuf, header_size - kSdkPacketPreambleCrcSize);
  command_all->header.crc16 = crc16;
  *crc32_ptr = crc32_.crc32_calc(&buff[0], data_size - sizeof(*crc32_ptr));
  buff.resize(data_size);

  return buff;
}

/// @brief Command send and ack wait.
/// @return CommandResult
/// @details Received unknown packet 10 retries.
CommandResult LidarDriver::SendAckWait(
  const std::vector<uint8_t> & snd_buff, GeneralCommandID cmd_id)
{
  //printf("%s Start cmd_id=%d\n", __func__, (int)cmd_id);
  std::unique_lock<std::mutex> lock(mtx_);
  CommandResult result = CommandResult::kUnknownPacket;
  int retval = commandsock_.Send(snd_buff);
  if (retval < 0) {
    result = CommandResult::kError;  // send error.
  } else {
    std::vector<uint8_t> rcv_buff(kMaxBufferSize);
    CommandAck * ack = reinterpret_cast<CommandAck *>(rcv_buff.data());

    int rcv_len;
    for (int retry_cnt = 10; retry_cnt > 0; --retry_cnt) {
      rcv_len = commandsock_.Recv(rcv_buff, 100);  // 100ms
      if (rcv_len < 0) {
        result = CommandResult::kError;  // receive error
        break;
      } else if (rcv_len == 0) {
        result = CommandResult::kTimeout;  // timeout
        break;
      } else if (rcv_len < static_cast<int>(sizeof(*ack))) {
      } else if (CheckRecvData(rcv_buff) == false) {
      } else if (ack->cmd_id != cmd_id) {
      } else if (ack->result > 1) {
      } else {
        // ack->result  0:ack, 1:nack
        result = (ack->result == 0) ? CommandResult::kAck : CommandResult::kNack;
        break;
      }
    }
    //printf("rcv_len=%d ack->result=%d\n", rcv_len, ack->result);
  }
  //printf("%s End cmd_id=%d\n", __func__, (int)cmd_id);

  return result;
}

GeneralCommandID LidarDriver::GetCommandId(LidarCommandType cmd_type)
{
  GeneralCommandID cmd_id;
  switch (cmd_type) {
    case LidarCommandType::kLidarCommandHandshake:
      cmd_id = GeneralCommandID::kHandshake;
      break;
    case LidarCommandType::kLidarCommandStartStreaming:
      cmd_id = GeneralCommandID::kControlSample;
      break;
    case LidarCommandType::kLidarCommandStopStreaming:
      cmd_id = GeneralCommandID::kControlSample;
      break;
    case LidarCommandType::kLidarCommandHeartbeat:
      cmd_id = GeneralCommandID::kHeartbeat;
      break;
  }
  return cmd_id;
}

/// @brief Rx socket open and start receive thread
/// @return true:success, false:faile.
bool LidarDriver::StartHwRxInterface()
{
  if (rx_datasock_.IsOpen() == false || rx_imusock_.IsOpen() == false) {
    return false;
  }

  if (thread_rxdata_.get() == nullptr) {
    thread_rxdata_ = std::make_shared<std::thread>(&LidarDriver::LivoxHwRxInterfaceData, this);
  }
  if (thread_rximu_.get() == nullptr) {
    thread_rximu_ = std::make_shared<std::thread>(&LidarDriver::LivoxHwRxInterfaceImu, this);
  }
  if (thread_rxcommand_.get() == nullptr) {
    thread_rxcommand_ =
      std::make_shared<std::thread>(&LidarDriver::LivoxHwRxInterfaceCommand, this);
  }

  // Send StartStreaming: wake up thread LivoxHwRxInterfaceCommand
  if (semaphore_.GetCount() <= 0) {
    semaphore_.Signal();
  }

  return true;
}

/// @brief Rx socket close and stop receive thread
void LidarDriver::StopHwRxInterface()
{
  if (lidarstatus_ == LidarDriverStatus::kStreaming) {
    std::vector<uint8_t> buff = MakeSendCommand(LidarCommandType::kLidarCommandStopStreaming);
    GeneralCommandID cmd_id = GetCommandId(LidarCommandType::kLidarCommandStopStreaming);
    CommandResult result = CommandResult::kTimeout;

    lidarstatus_ = LidarDriverStatus::kConnect;
    for (int snd_cnt = 3; snd_cnt > 0; --snd_cnt) {
      if (result != CommandResult::kTimeout) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      result = SendAckWait(buff, cmd_id);
      if (result == CommandResult::kAck) {
        break;
      }
    }
  }

  // Stop thread LivoxHwRxInterfaceCommand
  driverstatus_ = DriverStatus::kTerminate;
  semaphore_.Signal();

  rx_datasock_.Close();
  rx_imusock_.Close();

  thread_rxcommand_->join();
  thread_rxdata_->join();
  thread_rximu_->join();
}

/// @brief Tx socket open and start send thread
/// @return CommandResult
CommandResult LidarDriver::StartHwTxInterface()
{
  CommandResult ret;

  if (commandsock_.IsOpen() == false) {
    ret = CommandResult::kError;
  } else {
    std::vector<uint8_t> buff = MakeSendCommand(LidarCommandType::kLidarCommandHandshake);
    GeneralCommandID cmd_id = GetCommandId(LidarCommandType::kLidarCommandHandshake);

    ret = SendAckWait(buff, cmd_id);
    if (ret == CommandResult::kAck) {
      lidarstatus_ = LidarDriverStatus::kConnect;
      // thread start
      thread_txheartbeat_ =
        std::make_shared<std::thread>(&LidarDriver::LivoxHwTxInterfaceHeartbeat, this);
    }
  }

  return ret;
}

/// @brief Tx socket close and stop send thread
void LidarDriver::StopHwTxInterface()
{
  thread_txheartbeat_->join();
  commandsock_.Close();
}

/// @brief send thread
bool LidarDriver::LivoxHwTxInterfaceHeartbeat()
{
  std::vector<uint8_t> hand_shake_buff = MakeSendCommand(LidarCommandType::kLidarCommandHandshake);
  GeneralCommandID hand_shake_id = GetCommandId(LidarCommandType::kLidarCommandHandshake);
  std::vector<uint8_t> heart_beat_buff = MakeSendCommand(LidarCommandType::kLidarCommandHeartbeat);
  GeneralCommandID heart_beat_id = GetCommandId(LidarCommandType::kLidarCommandHeartbeat);
  CommandResult retval;
  int retry_cnt = 0;
  int wait_ms = 1000;   // wait 1.0 sec

  while (driverstatus_ == DriverStatus::kRunning) {
    std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
    if (driverstatus_ != DriverStatus::kRunning) {
      //printf("%s Exit\n", __func__);
      break;
    }
    wait_ms = 1000;

    if (lidarstatus_ == LidarDriverStatus::kDisconnect) {
      retval = SendAckWait(hand_shake_buff, hand_shake_id);
      if (retval == CommandResult::kAck) {
        lidarstatus_ = LidarDriverStatus::kConnect;
        retry_cnt = 0;
        // Send StartStreaming: wake up thread LivoxHwRxInterfaceCommand
        if (semaphore_.GetCount() <= 0) {
          semaphore_.Signal();
        }
      } else if (retval == CommandResult::kTimeout) {
        wait_ms = 900;    // 100 + 900 == 1000
      } else {
        /* do nothing. */
      }
    } else {
      retval = SendAckWait(heart_beat_buff, heart_beat_id);
      if (retval == CommandResult::kAck) {
        retry_cnt = 0;
      } else if (++retry_cnt >= 3) {
        lidarstatus_ = LidarDriverStatus::kDisconnect;
        retry_cnt = 0;
        wait_ms = 110;    // requires 100ms over.
      } else if (retval == CommandResult::kTimeout) {
        wait_ms = 900;    // 100 + 900 == 1000
      } else {
        /* do nothing. */
      }
    }
  }  // while

  return true;
}

/// @brief command receive thread
bool LidarDriver::LivoxHwRxInterfaceCommand()
{
  std::vector<uint8_t> snd_buff = MakeSendCommand(LidarCommandType::kLidarCommandStartStreaming);
  GeneralCommandID cmd_id = GetCommandId(LidarCommandType::kLidarCommandStartStreaming);

  while (driverstatus_ == DriverStatus::kRunning) {
    // Wait HandShake Ack(StartHwRxInterface or LivoxHwTxInterfaceHeartbeat) or StopHwRxInterface
    semaphore_.Wait();

    if (driverstatus_ != DriverStatus::kRunning) {
      break;
    } else if (lidarstatus_ == LidarDriverStatus::kDisconnect) {
      // continue;
    } else {
      if (lidarstatus_ == LidarDriverStatus::kConnect) {
        lidarstatus_ = LidarDriverStatus::kStreaming;
      }

      //printf("StartStreaming Loop start\n" );
      CommandResult result;
      while (driverstatus_ == DriverStatus::kRunning &&
             lidarstatus_ == LidarDriverStatus::kStreaming) {
        result = SendAckWait(snd_buff, cmd_id);
        if (result == CommandResult::kAck) {
          break;
        } else if (result == CommandResult::kTimeout) {
          std::this_thread::sleep_for(std::chrono::milliseconds(150));
        } else {
          std::this_thread::sleep_for(std::chrono::milliseconds(250));
        }
      }  // while
      //if (result != CommandResult::kAck) { printf("StartStreaming result=%d\n", (int)result ); }
    }
  }

  return true;
}

/// @brief socket open
/// @param sock         : target UdpSoket class
/// @param sensor_ip    : sensor ip
/// @param sensor_port  : data port
/// @param my_port      : data receive port
bool LidarDriver::UdpSocketOpen(
  HwInterface::UdpSocket & sock, const std::string & sensor_ip, uint16_t sensor_port,
  uint16_t my_port)
{
  bool bret = true;
  sock.SetSensorIpPort(sensor_ip, sensor_port);
  int retval = sock.Open(my_port, true, false);

  if (retval < 0) {
    printf("socket open error val:%d :%s(%d)\n", retval, __FILE__, __LINE__);
    bret = false;
  }
  return bret;
}

void LidarDriver::CloseAllUdpSocket()
{
  commandsock_.Close();
  rx_datasock_.Close();
  rx_imusock_.Close();
}

/// @brief make handshake command data
CommandHandshake LidarDriver::MakeCommandHandshake()
{
  CommandHandshake command{};
  command.cmd_id = GeneralCommandID::kHandshake;
  command.host_ip = inet_addr(lidarconfig_.host_ip.c_str());
  command.data_port = lidarconfig_.data_port;
  command.command_port = lidarconfig_.cmd_port;
  command.imu_port = lidarconfig_.imu_port;
  return command;
}

/// @brief make stream command data(streaming start)
CommandSampling LidarDriver::MakeCommandStartStream()
{
  CommandSampling command{};
  command.cmd_id = GeneralCommandID::kControlSample;
  command.sample_ctrl = 0x01;
  return command;
}

/// @brief make stream command data(streaming stop)
CommandSampling LidarDriver::MakeCommandStopStream()
{
  CommandSampling command{};
  command.cmd_id = GeneralCommandID::kControlSample;
  command.sample_ctrl = 0x00;
  return command;
}

/// @brief make heartbeat command data
CommandHeartbeat LidarDriver::MakeCommandHeartbeat()
{
  CommandHeartbeat command{};
  command.cmd_id = GeneralCommandID::kHeartbeat;
  return command;
}

/// @brief check receive data
bool LidarDriver::CheckRecvData(const std::vector<uint8_t> & buff)
{
  uint32_t crc32 = 0;
  uint32_t crc32_calc = 0;
  uint16_t crc16_calc = 0;

  const CommandHeader * commandheader = reinterpret_cast<const CommandHeader *>(&buff[0]);

  uint16_t datasize = commandheader->length;
  uint16_t crc16 = commandheader->crc16;

  // check crc16 on headder
  crc16_calc = crc16_.mcrf4xx_calc(&buff[0], sizeof(*commandheader) - kSdkPacketPreambleCrcSize);
  if (crc16 != crc16_calc) {
    return false;
  }

  // check crc32
  memcpy(&crc32, &buff[datasize - kSdkPacketCrcSize], sizeof(crc32));
  crc32_calc = crc32_.crc32_calc(&buff[0], datasize - kSdkPacketCrcSize);
  if (crc32 != crc32_calc) {
    return false;
  }
  return true;
}

/// @brief packet data parse
int LidarDriver::ParsePacket(livox_driver::LivoxLidarPacket & packet)
{
  //printf("parse packet packet.time %ld  temp.time %ld \n", packet.time, temp_publish_data_->time);
  int iret = 0;
  uint64_t timestamp = packet.time_stamp;
  bool timegap_over = false;
  LivoxEthPacket * raw_packet = reinterpret_cast<LivoxEthPacket *>(packet.data.data());

  if (temp_publish_data_.get() == nullptr) {
    temp_publish_data_ = std::make_unique<livox_driver::LivoxPublishData>();
  }

  if (temp_publish_data_->num == 0) {
    uint32_t remaining_time = timestamp % publish_period_ns_;
    uint32_t diff_time = publish_period_ns_ - remaining_time;
    /** Get start time, down to the period boundary */
    if (diff_time > (publish_period_ns_ / 4)) {
      // RCLCPP_INFO(cur_node_->get_logger(), "0 : %u", diff_time);
      //printf("GetPublishStartTime : 0 : diff_time=%u timestamp=%lu\n", diff_time, timestamp);
      last_timestamp_ = timestamp - remaining_time;
    } else if (diff_time <= lidars_.packet_interval_max) {
      //printf("GetPublishStartTime : 1 : diff_time=%u timestamp=%lu\n", diff_time, timestamp);
      last_timestamp_ = timestamp;
    } else {
      //printf("GetPublishStartTime : 2 : diff_time=%u timestamp=%lu\n", diff_time, timestamp);
      /* the remaning packets in queue maybe not enough after skip */
      return 0;
    }
  }

  do {
    int64_t time_gap = timestamp - last_timestamp_;
    // if timeout -> packet publish
    if (time_gap > lidars_.packet_interval_max) {
      //printf( "NG:time_gap=%ld timestamp=%ld last_timestamp=%ld\n", time_gap, timestamp, last_timestamp_ );
      packet.data.clear();
      last_timestamp_ += lidars_.packet_interval;
      timegap_over = true;
    } else {
      //printf( "OK:time_gap=%ld timestamp=%ld last_timestamp=%ld\n", time_gap, timestamp, last_timestamp_ );
      last_timestamp_ = timestamp;
      timegap_over = false;
    }

    if (temp_publish_data_->num == 0) {
      /* new timestamp */
      temp_publish_data_->time = timestamp;
      temp_publish_data_->data.resize(
        lidars_.onetime_publish_packets * kMaxPointPerEthPacket *
        sizeof(livox_driver::LivoxPointXyzrtl));
    }

    uint8_t * point_base =
      &temp_publish_data_->data[temp_publish_data_->num * sizeof(livox_driver::LivoxPointXyzrtl)];

    point_base = LivoxExtendRawPointToPxyzrtl(point_base, raw_packet, extrinsic_, 1);
    temp_publish_data_->num += packet.data_cnt;

    accumulate_count_++;

    if (accumulate_count_ >= lidars_.onetime_publish_packets) {
      uint32_t echo_num = GetEchoNumPerPoint(lidars_.raw_data_type);
      temp_publish_data_->num = temp_publish_data_->num * echo_num;
      publish_data_ = std::move(temp_publish_data_);
      iret = publish_data_->num;

      accumulate_count_ = 0;
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

void LidarDriver::PointExtrisincCompensation(
  livox_driver::PointXyz * dst_point, const livox_driver::PointXyz & src_point,
  ExtrinsicParameter & extrinsic)
{
  dst_point->x = src_point.x * extrinsic.rotation[0][0] + src_point.y * extrinsic.rotation[0][1] +
                 src_point.z * extrinsic.rotation[0][2] + extrinsic.trans[0];
  dst_point->y = src_point.x * extrinsic.rotation[1][0] + src_point.y * extrinsic.rotation[1][1] +
                 src_point.z * extrinsic.rotation[1][2] + extrinsic.trans[1];
  dst_point->z = src_point.x * extrinsic.rotation[2][0] + src_point.y * extrinsic.rotation[2][1] +
                 src_point.z * extrinsic.rotation[2][2] + extrinsic.trans[2];
}

uint8_t * LidarDriver::LivoxExtendRawPointToPxyzrtl(
  uint8_t * point_buf, LivoxEthPacket * eth_packet, ExtrinsicParameter & extrinsic,
  uint32_t line_num)
{
  livox_driver::LivoxPointXyzrtl * dst_point =
    reinterpret_cast<livox_driver::LivoxPointXyzrtl *>(point_buf);
  uint32_t points_per_packet = GetPointsPerPacket(eth_packet->data_type);
  LivoxExtendRawPoint * raw_point = reinterpret_cast<LivoxExtendRawPoint *>(eth_packet->data);

  uint8_t line_id = 0;
  while (points_per_packet) {
    RawPointConvert(*dst_point, reinterpret_cast<const LivoxRawPoint &>(*raw_point));
    if (extrinsic.enable && IsTripleIntNoneZero(raw_point->x, raw_point->y, raw_point->z)) {
      livox_driver::PointXyz * xyz_ptr = reinterpret_cast<livox_driver::PointXyz *>(dst_point);
      livox_driver::PointXyz src_point = *xyz_ptr;
      PointExtrisincCompensation(xyz_ptr, src_point, extrinsic);
    }
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

  return (uint8_t *)dst_point;
}

void LidarDriver::EulerAnglesToRotationMatrix(EulerAngle euler, RotationMatrix matrix)
{
  double cos_roll = cos(static_cast<double>(euler[0]));
  double cos_pitch = cos(static_cast<double>(euler[1]));
  double cos_yaw = cos(static_cast<double>(euler[2]));
  double sin_roll = sin(static_cast<double>(euler[0]));
  double sin_pitch = sin(static_cast<double>(euler[1]));
  double sin_yaw = sin(static_cast<double>(euler[2]));

  matrix[0][0] = cos_pitch * cos_yaw;
  matrix[0][1] = sin_roll * sin_pitch * cos_yaw - cos_roll * sin_yaw;
  matrix[0][2] = cos_roll * sin_pitch * cos_yaw + sin_roll * sin_yaw;

  matrix[1][0] = cos_pitch * sin_yaw;
  matrix[1][1] = sin_roll * sin_pitch * sin_yaw + cos_roll * cos_yaw;
  matrix[1][2] = cos_roll * sin_pitch * sin_yaw - sin_roll * cos_yaw;

  matrix[2][0] = -sin_pitch;
  matrix[2][1] = sin_roll * cos_pitch;
  matrix[2][2] = cos_roll * cos_pitch;
}

}  // namespace lidar_driver
