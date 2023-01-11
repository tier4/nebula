#include "LidarDriver/lidar_driver.hpp"

#include <arpa/inet.h>

#include <cmath>
#include <cstring>

#include "livox_data_recv.hpp"

namespace lidar_driver
{
/// @brief constructor
LidarDriver::LidarDriver() : driverstatus_(DriverStatus::kRunning) {}

/// @brief destructor
LidarDriver::~LidarDriver() { CloseAllUdpSocket(); }

/// @brief set livox configuration
bool LidarDriver::SetConfiguration(const livox_driver::LivoxSensorConfiguration & param)
{
  bool is_success = false;

  CloseAllUdpSocket();

  is_success = UdpSocketOpen(commandsock_, param.sensor_ip, kCommandPort, param.cmd_port);
  if (is_success) {
    is_success = UdpSocketOpen(rx_datasock_, param.sensor_ip, 0, param.data_port);
  }
  if (is_success) {
    is_success = UdpSocketOpen(rx_imusock_, param.sensor_ip, 0, param.imu_port);
  }

  if (is_success) {
    switch (param.sensor_model) {
      case livox_driver::LivoxSensorModel::HORIZON:
        lidar_device_.device_info_type = DeviceType::kLidarHorizon;
        lidar_device_.line_num = GetLaserLineNumber(lidar_device_.device_info_type);
        break;
      default:
        is_success = false;
    }
  }

  if (!is_success) {
    CloseAllUdpSocket();
    return false;
  }

  const int64_t kNsPerSecond = 1000000000; /**< 1s  = 1000000000ns */
  buffer_time_ms_ = param.frequency_ms;
  publish_period_ns_ = kNsPerSecond / (1000 / param.frequency_ms);
  lidarconfig_ = param;

  return is_success;
}

/// @brief get sensor configuration.
bool LidarDriver::GetConfiguration(livox_driver::LivoxSensorConfiguration & param)
{
  param = lidarconfig_;
  return true;
}

/// @brief set command data
/// @param command  : command type
std::vector<uint8_t> LidarDriver::MakeSendCommand(LidarCommandType cmd_type)
{
  std::vector<uint8_t> buff(kMaxBufferSize);
  CommandAll * command_all = reinterpret_cast<CommandAll *>(buff.data());

  command_all->header.sof = 0xAA;
  command_all->header.version = 0x01;
  command_all->header.cmd_type = 0x00;
  command_all->header.seq_num = 0x0000;

  uint16_t cmd_size;
  switch (cmd_type) {
    case LidarCommandType::kHandshake:
      command_all->data.hand_shake = MakeCommandHandshake();
      cmd_size = sizeof(command_all->data.hand_shake);
      break;
    case LidarCommandType::kStartStreaming:
      command_all->data.sampling = MakeCommandStartStream();
      cmd_size = sizeof(command_all->data.sampling);
      break;
    case LidarCommandType::kStopStreaming:
      command_all->data.sampling = MakeCommandStopStream();
      cmd_size = sizeof(command_all->data.sampling);
      break;
    case LidarCommandType::kHeartbeat:
      command_all->data.heart_beat = MakeCommandHeartbeat();
      cmd_size = sizeof(command_all->data.heart_beat);
      break;
  }

  uint16_t data_size = sizeof(command_all->header) + cmd_size;
  command_all->header.length = data_size;
  buff.resize(data_size);

  return buff;
}

/// @brief set CRC16 CRC32 Checksum
/// @param buff: send buffer.
/// @param cmd_id: command id.
void LidarDriver::SetCrc16Crc32(std::vector<uint8_t> & buff, GeneralCommandID cmd_id)
{
  CommandAll * command_all = reinterpret_cast<CommandAll *>(buff.data());
  uint16_t header_len = sizeof(command_all->header) - sizeof(command_all->header.crc16);
  command_all->header.crc16 = crc16_.mcrf4xx_calc(&buff[0], header_len);
  uint32_t crc32 = crc32_.crc32_calc(&buff[0], command_all->header.length - sizeof(crc32));
  switch (cmd_id) {
    case GeneralCommandID::kControlSample:
      command_all->data.sampling.crc32 = crc32;
      break;
    case GeneralCommandID::kHeartbeat:
      command_all->data.heart_beat.crc32 = crc32;
      break;
    default:  // kHandshake:
      command_all->data.hand_shake.crc32 = crc32;
      break;
  }
  return;
}

/// @brief Command send and ack wait.
/// @param snd_buff
/// @param cmd_id: response command id.
/// @return CommandResult
/// @details Received unknown packet 10 retries.
CommandResult LidarDriver::SendAckWait(std::vector<uint8_t> & snd_buff, GeneralCommandID cmd_id)
{
  //printf("%s Start cmd_id=%d\n", __func__, (int)cmd_id);
  std::unique_lock<std::mutex> lock(mtx_);
  CommandResult result = CommandResult::kUnknownPacket;
  CommandHeader * header = reinterpret_cast<CommandHeader *>(&snd_buff[0]);
  header->seq_num = ++snd_seq_num_;
  SetCrc16Crc32(snd_buff, cmd_id);

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
      } else if (!CheckRecvData(rcv_buff)) {
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
  //printf("%s End cmd_id=%d resukt=%d\n", __func__, (int)cmd_id, (int)result);

  return result;
}

/// @brief CommandType to GeneralCommandID.
/// @param cmd_type
/// @return GeneralCommandID
GeneralCommandID LidarDriver::GetCommandId(LidarCommandType cmd_type)
{
  GeneralCommandID cmd_id;
  switch (cmd_type) {
    case LidarCommandType::kHandshake:
      cmd_id = GeneralCommandID::kHandshake;
      break;
    case LidarCommandType::kStartStreaming:
    case LidarCommandType::kStopStreaming:
      cmd_id = GeneralCommandID::kControlSample;
      break;
    case LidarCommandType::kHeartbeat:
      cmd_id = GeneralCommandID::kHeartbeat;
      break;
  }
  return cmd_id;
}

/// @brief Rx socket open and start receive thread
/// @return true:success, false: failure.
bool LidarDriver::StartHwRxInterface()
{
  if (!rx_datasock_.IsOpen() || !rx_imusock_.IsOpen()) {
    return false;
  }

  if (!thread_rxdata_) {
    thread_rxdata_ = std::make_shared<std::thread>(&LidarDriver::LivoxHwRxInterfaceData, this);
  }
  if (!thread_rximu_) {
    thread_rximu_ = std::make_shared<std::thread>(&LidarDriver::LivoxHwRxInterfaceImu, this);
  }
  if (!thread_rxcommand_) {
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
    std::vector<uint8_t> buff = MakeSendCommand(LidarCommandType::kStopStreaming);
    GeneralCommandID cmd_id = GetCommandId(LidarCommandType::kStopStreaming);
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

  if (!commandsock_.IsOpen()) {
    ret = CommandResult::kError;
  } else {
    std::vector<uint8_t> buff = MakeSendCommand(LidarCommandType::kHandshake);
    GeneralCommandID cmd_id = GetCommandId(LidarCommandType::kHandshake);

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
  driverstatus_ = DriverStatus::kTerminate;
  thread_txheartbeat_->join();
  commandsock_.Close();
}

/// @brief send thread
void LidarDriver::LivoxHwTxInterfaceHeartbeat()
{
  std::vector<uint8_t> hand_shake_buff = MakeSendCommand(LidarCommandType::kHandshake);
  GeneralCommandID hand_shake_id = GetCommandId(LidarCommandType::kHandshake);
  std::vector<uint8_t> heart_beat_buff = MakeSendCommand(LidarCommandType::kHeartbeat);
  GeneralCommandID heart_beat_id = GetCommandId(LidarCommandType::kHeartbeat);
  CommandResult retval;
  int retry_cnt = 0;
  int wait_ms = 1000;  // wait 1.0 sec

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
        wait_ms = 900;  // 100 + 900 == 1000
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
        wait_ms = 110;  // requires 100ms over.
      } else if (retval == CommandResult::kTimeout) {
        wait_ms = 900;  // 100 + 900 == 1000
      } else {
        /* do nothing. */
      }
    }
  }  // while

  //printf("Thread end %s\n", __func__);
  return;
}

/// @brief command receive thread
void LidarDriver::LivoxHwRxInterfaceCommand()
{
  std::vector<uint8_t> snd_buff = MakeSendCommand(LidarCommandType::kStartStreaming);
  GeneralCommandID cmd_id = GetCommandId(LidarCommandType::kStartStreaming);

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
      DataRecvInit();

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

  //printf("Thread end %s\n", __func__);
  return;
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

/// @brief Close all UDP sockets.
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
  const CommandHeader * commandheader = reinterpret_cast<const CommandHeader *>(&buff[0]);

  // check crc16 on headder
  uint16_t header_len = sizeof(*commandheader) - sizeof(commandheader->crc16);
  uint16_t crc16_calc = crc16_.mcrf4xx_calc(&buff[0], header_len);
  if (commandheader->crc16 != crc16_calc) {
    return false;
  }

  // check crc32
  uint32_t crc32 = 0;
  uint16_t data_size = commandheader->length - sizeof(crc32);
  memcpy(&crc32, &buff[data_size], sizeof(crc32));
  uint32_t crc32_calc = crc32_.crc32_calc(&buff[0], data_size);
  if (crc32 != crc32_calc) {
    return false;
  }

  return true;
}

}  // namespace lidar_driver
