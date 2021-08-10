#ifndef LIDARDRIVER_LIVOX_COMMAND_HPP_
#define LIDARDRIVER_LIVOX_COMMAND_HPP_

#include <condition_variable>
#include <mutex>

namespace lidar_driver
{
const uint16_t kCrcSeed16 = 0x4c49;
const uint32_t kCrcSeed32 = 0x564f580a;

enum class LidarCommandType {
  kHandshake = 0,
  kStartStreaming,
  kStopStreaming,
  kHeartbeat,
};

enum class CommandResult { kAck = 0, kNack, kTimeout, kError, kUnknownPacket };

/**  Enum that represents the command id. */
enum class GeneralCommandID : uint8_t {
  kBroadcast = 0,                 // General command set, broadcast command.
  kHandshake = 1,                 // General command set, query the information of device.
  kDeviceInfo = 2,                // General command set, query the information of device.
  kHeartbeat = 3,                 // General command set, heartbeat command.
  kControlSample = 4,             // General command set, enable or disable the sampling.
  kCoordinateSystem = 5,          // General command set, change the coordinate of point cloud data.
  kDisconnect = 6,                // General command set, disconnect the device.
  kPushAbnormalState = 7,         // General command set, a message command from a connected device
  kConfigureStaticDynamicIp = 8,  // General command set, set the IP of the a device.
  kGetDeviceIpInformation = 9,    // General command set, get the IP of the a device.
  kRebootDevice = 0x0a,           // General command set, reboot a device.
  kSetDeviceParam = 0x0b,         // Set device's parameters.
  kGetDeviceParam = 0x0c,         // Get device's parameters.
  kResetDeviceParam = 0x0d,       // Reset device's all parameters.
  kCommandCount                   // Don't add command id after kCommandCount.
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

#pragma pack(1)
struct CommandHandshake
{
  uint8_t cmd_set;
  GeneralCommandID cmd_id;
  uint32_t host_ip;
  uint16_t data_port;
  uint16_t command_port;
  uint16_t imu_port;
  uint32_t crc32;
};
#pragma pack()

#pragma pack(1)
struct CommandHeartbeat
{
  uint8_t cmd_set;
  GeneralCommandID cmd_id;
  uint32_t crc32;
};
#pragma pack()

#pragma pack(1)
struct CommandSampling
{
  uint8_t cmd_set;
  GeneralCommandID cmd_id;
  uint8_t sample_ctrl;
  uint32_t crc32;
};
#pragma pack()

#pragma pack(1)
struct CommandAll
{
  CommandHeader header;
  union {
    CommandHandshake hand_shake;
    CommandHeartbeat heart_beat;
    CommandSampling sampling;
  } data;
};
#pragma pack()

#pragma pack(1)
struct CommandAck
{
  CommandHeader header;
  uint8_t cmd_set;
  GeneralCommandID cmd_id;
  uint8_t result;
  uint32_t crc32;
};
#pragma pack()

class CountingSemaphore
{
public:
  explicit CountingSemaphore(int count = 0) : count_(count) {}

  void Signal()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    ++count_;
    cv_.notify_one();
  }

  void Wait()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [=] { return count_ > 0; });
    --count_;
  }

  int GetCount() { return count_; }

private:
  std::mutex mutex_;
  std::condition_variable cv_;
  volatile int count_;
};

}  // namespace lidar_driver
#endif  //LIDARDRIVER_LIVOX_COMMAND_HPP_
