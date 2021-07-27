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

  if (is_success != false) {
    const int64_t kNsPerSecond = 1000000000; /**< 1s  = 1000000000ns */
    buffer_time_ms_ = param.frequency_ms;
    publish_period_ns_ = kNsPerSecond / (1000 / param.frequency_ms);
    lidarconfig_ = param;
  } else {
    CloseAllUdpSocket();
  }

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

/// @brief send command to livox
/// @param command  : command type
bool LidarDriver::SendCommand(LidarCommandType command)
{
  std::vector<uint8_t> buff;
  std::vector<uint8_t> commanddata;

  CommandHeader headerdata;

  uint8_t * crcbuf;
  uint32_t crc16 = 0;
  uint32_t crc32 = 0;
  uint16_t headersize = sizeof(headerdata);

  headerdata.sof = 0xAA;
  headerdata.version = 0x01;
  headerdata.cmd_type = 0x00;
  headerdata.seq_num = 0x00;

  switch (command) {
    case LidarCommandType::kLidarCommandHandshake:
      MakeCommandHandshake(commanddata);
      break;
    case LidarCommandType::kLidarCommandStartStreaming:
      MakeCommandStartStream(commanddata);
      break;
    case LidarCommandType::kLidarCommandStopStreaming:
      MakeCommandStopStream(commanddata);
      break;
    case LidarCommandType::kLidarCommandHeartbeat:
      MakeCommandHeartbeat(commanddata);
      break;
  }

  uint16_t datasize = headersize + commanddata.size() + sizeof(crc32);
  headerdata.length = datasize;

  crcbuf = reinterpret_cast<uint8_t *>(&headerdata);

  crc16 = crc16_.mcrf4xx_calc(crcbuf, headersize - kSdkPacketPreambleCrcSize);
  headerdata.crc16 = crc16;

  buff.resize(datasize, 0);
  memcpy(&buff[0], &headerdata, headersize);
  memcpy(&buff[headersize], &commanddata[0], commanddata.size());

  crc32 = crc32_.crc32_calc(&buff[0], datasize - kSdkPacketCrcSize);
  memcpy(&buff[datasize - kSdkPacketCrcSize], &crc32, sizeof(crc32));

  int retval = commandsock_.Send(buff);
  if (retval < 0) {
    printf("command send error val:%d :%s(%d)\n", retval, __FILE__, __LINE__);
  }

  return true;
}

/// @brief Rx socket open and start receive thread
/// @retval 0  : success
/// @retval -1 : socket not open
/// @retval -2 : stream start timeout
/// @retval -3 : stream start ack result NG
int LidarDriver::StartHwRxInterface()
{
  int ret;

  if (rx_datasock_.IsOpen() == false || rx_imusock_.IsOpen() == false) {
    return -1;
  }

  if (thread_rxdata_.get() == nullptr) {
    thread_rxdata_ = std::make_shared<std::thread>(&LidarDriver::LivoxHwRxInterfaceData, this);
  }
  if (thread_rximu_.get() == nullptr) {
    thread_rximu_ = std::make_shared<std::thread>(&LidarDriver::LivoxHwRxInterfaceImu, this);
  }

  // send start streaming
  lidarstatus_ = LidarDriverStatus::kStreaming;
  for (int cnt = 0; cnt < 30; cnt++) {
    stream_start_ack_ = -2;
    SendCommand(LidarCommandType::kLidarCommandStartStreaming);
    for (int wait_50ms = 0; wait_50ms < 20; wait_50ms++) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      if (stream_start_ack_ == 0) {
        return 0;
      }
    }  // for(wait_50ms)

    if (lidarstatus_ != LidarDriverStatus::kStreaming) {
      break;
    }
  }  // for(cnt)

  ret = stream_start_ack_;
  if (ret > 0) {
    ret = -3;
  }

  return ret;
}

/// @brief Rx socket close and stop receive thread
void LidarDriver::StopHwRxInterface()
{
  rx_datasock_.Close();
  rx_imusock_.Close();
  thread_rxdata_->join();
  thread_rximu_->join();
}

/// @brief Tx socket open and start send thread
/// @retval 0  : success
/// @retval -1 : socket not open
/// @retval -2 : handshake timeout
/// @retval -3 : handshake ack result NG
int LidarDriver::StartHwTxInterface()
{
  int iret = -3;

  if (commandsock_.IsOpen()) {
    // send Handshake command
    SendCommand(LidarCommandType::kLidarCommandHandshake);
    std::vector<uint8_t> buff;
    buff.resize(kMaxBufferSize);

    // receive response
    int rcv_len = commandsock_.Recv(buff, 1000);
    if (rcv_len > 0) {
      if (CheckRecvData(buff)) {
        // check HandShake ack
        CommandAck * ack = reinterpret_cast<CommandAck *>(&buff[sizeof(CommandHeader)]);
        if (ack->cmd_id == kCommandRxCommunicationSettingAck && ack->result == 0x00) {
          lidarstatus_ = LidarDriverStatus::kConnect;
          // thread start
          thread_txheartbeat_ =
            std::make_shared<std::thread>(&LidarDriver::LivoxHwTxInterfaceHeartbeat, this);
          thread_rxcommand_ =
            std::make_shared<std::thread>(&LidarDriver::LivoxHwRxInterfaceCommand, this);

          iret = 0;
        }
      }
    } else if (rcv_len == 0) {
      // timeout
      iret = -2;
    }
  } else {
    // socket not open
    iret = -1;
  }
  return iret;
}

/// @brief Tx socket close and stop send thread
void LidarDriver::StopHwTxInterface()
{
  SendCommand(LidarCommandType::kLidarCommandStopStreaming);
  driverstatus_ = DriverStatus::kTerminate;

  thread_txheartbeat_->join();
  thread_rxcommand_->join();

  commandsock_.Close();
}

/// @brief send thread
bool LidarDriver::LivoxHwTxInterfaceHeartbeat()
{
  // wait 1.0 sec
  std::this_thread::sleep_for(std::chrono::seconds(1));
  while (DriverStatus::kRunning == driverstatus_) {
    if (LidarDriverStatus::kDisconnect == lidarstatus_) {
      // send Handshake command
      SendCommand(LidarCommandType::kLidarCommandHandshake);
    } else {
      // send heartbeat command
      SendCommand(LidarCommandType::kLidarCommandHeartbeat);
    }
    // wait 1.0 sec
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  return true;
}

/// @brief command receive thread
bool LidarDriver::LivoxHwRxInterfaceCommand()
{
  std::vector<uint8_t> buff;
  buff.resize(kMaxBufferSize);

  while (DriverStatus::kRunning == driverstatus_) {
    int rcv_len = commandsock_.Recv(buff);

    if (rcv_len > 0 && CheckRecvData(buff)) {
      CommandAck * ack = reinterpret_cast<CommandAck *>(&buff[sizeof(CommandHeader)]);
      switch (ack->cmd_id) {
        case kCommandRxCommunicationSettingAck:
          // first connection Communication port setting
          DataRecvInit();
          break;
        case kCommancRxStreamingAck:
          stream_start_ack_ = ack->result;  // 0:success, 1:fail.
          //if (LidarDriverStatus::kConnect == lidarstatus_) {
          //  if (ack->result == 1) {
          //    // send retry(tentative)
          //    std::this_thread::sleep_for(std::chrono::milliseconds(100));
          //    SendCommand(kLidarCommandStartStreaming);
          //  } else {
          //    // Start streaming
          //    lidarstatus_ = LidarDriverStatus::kStreaming;
          //  }
          //} else {
          //  // Stop streaming
          //  //lidarstatus_ = LidarDriverStatus::kConnect;
          //}
          break;
        case kCommandRxHeartbeatAck:
          break;
        default:
          break;
      }
    } else if (rcv_len < 0) {
      printf("socket receive error val:%d :%s(%d)\n", rcv_len, __FILE__, __LINE__);
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
void LidarDriver::MakeCommandHandshake(std::vector<uint8_t> & buff)
{
  buff.resize(sizeof(CommandHandshake), 0);

  CommandHandshake * command = reinterpret_cast<CommandHandshake *>(&buff[0]);
  command->cmd_set = 0x00;
  command->cmd_id = 0x01;
  command->host_ip = inet_addr(lidarconfig_.host_ip.c_str());
  command->data_port = lidarconfig_.data_port;
  command->command_port = lidarconfig_.cmd_port;
  command->imu_port = lidarconfig_.imu_port;
}

/// @brief make stream command data(streaming start)
void LidarDriver::MakeCommandStartStream(std::vector<uint8_t> & buff)
{
  buff.resize(sizeof(CommandSampling), 0);

  CommandSampling * command = reinterpret_cast<CommandSampling *>(&buff[0]);
  command->cmd_set = 0x00;
  command->cmd_id = 0x04;
  command->sample_ctrl = 0x01;
}

/// @brief make stream command data(streaming stop)
void LidarDriver::MakeCommandStopStream(std::vector<uint8_t> & buff)
{
  buff.resize(sizeof(CommandSampling), 0);

  CommandSampling * command = reinterpret_cast<CommandSampling *>(&buff[0]);
  command->cmd_set = 0x00;
  command->cmd_id = 0x04;
  command->sample_ctrl = 0x00;
}

/// @brief make heartbeat command data
void LidarDriver::MakeCommandHeartbeat(std::vector<uint8_t> & buff)
{
  buff.resize(sizeof(CommandHeartbeat), 0);

  CommandHeartbeat * command = reinterpret_cast<CommandHeartbeat *>(&buff[0]);
  command->cmd_set = 0x00;
  command->cmd_id = 0x03;
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
  crc16_calc = crc16_.mcrf4xx_calc(&buff[0], sizeof(CommandHeader) - kSdkPacketPreambleCrcSize);
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

  if( temp_publish_data_.get() == nullptr ) {
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
  livox_driver::LivoxPointXyzrtl * dst_point = (livox_driver::LivoxPointXyzrtl *)point_buf;
  uint32_t points_per_packet = GetPointsPerPacket(eth_packet->data_type);
  LivoxExtendRawPoint * raw_point = reinterpret_cast<LivoxExtendRawPoint *>(eth_packet->data);

  uint8_t line_id = 0;
  while (points_per_packet) {
    RawPointConvert(*dst_point, reinterpret_cast<const LivoxRawPoint &>(*raw_point));
    if (extrinsic.enable && IsTripleIntNoneZero(raw_point->x, raw_point->y, raw_point->z)) {
      livox_driver::PointXyz src_point = *((livox_driver::PointXyz *)dst_point);
      PointExtrisincCompensation((livox_driver::PointXyz *)dst_point, src_point, extrinsic);
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
