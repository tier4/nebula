#ifndef ROBOSENSE_CMD_RESPONSE_HPP
#define ROBOSENSE_CMD_RESPONSE_HPP

#include <boost/algorithm/string/join.hpp>
#include <boost/format.hpp>

#include <ostream>
const unsigned int g_frame_flag = 0x55AA2552;
#define NET_CMD_READ_CONFIG 0x009
#define NET_CMD_WRITE_CONFIG 0x00A
namespace nebula
{
/// @brief  struct of lidar param
struct RobosenseLidarParam
{
  uint16_t motorSpeed;
  uint16_t motorPhaseLock;
  uint16_t timeSyncMode;
  uint16_t startFov;
  uint16_t endFov;
  uint16_t waveMode;
  uint16_t topVersion[2];
  uint16_t botVersion[2];

  uint16_t appVersion[2];
  uint16_t motorVersion[2];
  uint16_t imageVersion[2];
  uint16_t webVersiom[2];
  uint16_t hardwareVersion[2];

  uint16_t pulseChannelEnble;
  uint16_t pulseStartAngle0;
  uint16_t pulseStartAngle1;
  uint16_t pulseStartAngle2;
  uint16_t pulseWidth0[2];
  uint16_t pulseWidth1[2];
  uint16_t pulseWidth2[2];

  uint16_t pulseStep0;
  uint16_t pulseStep1;
  uint16_t pulseStep2;
  uint16_t pulseStartWidthEnable;
  uint16_t sensorTemper;
  uint16_t motorRealTimeSpeed;
  uint16_t motorRealTimePhase;
  uint16_t statusOfPhaseLock;
  uint16_t statusOfCodeWheelCali;
  uint16_t res0;
};

struct RobosenseNetFrameHead
{
  uint32_t frameFlag;
  uint32_t length;
  uint32_t cmd;
  uint32_t checkSum;
};

union UnionParam {
  RobosenseLidarParam ldPara;
  uint16_t res[128];
};

struct RobosenseNetWorkParam
{
  uint8_t sn[6];
  uint8_t mac[6];

  uint8_t ipLocal[4];
  uint8_t ipRemote[4];

  uint16_t msopPort{0};
  uint16_t difopPort{0};

  UnionParam param;

  uint8_t netmaskLocal[4];
  uint8_t gatewayLocal[4];
};

uint16_t CheckSum(RobosenseNetFrameHead frameHead)
{
  uint32_t sum = 0;

  sum += frameHead.frameFlag & 0xFFFF;
  sum += (frameHead.frameFlag >> 16) & 0xFFFF;

  sum += frameHead.length & 0xFFFF;
  sum += (frameHead.length >> 16) & 0xFFFF;

  sum += frameHead.cmd & 0xFFFF;
  sum += (frameHead.cmd >> 16) & 0xFFFF;

  sum = (sum >> 16) + (sum & 0xFFFF);

  return static_cast<uint16_t>(~sum);
}

std::vector<unsigned char> FrameHeadPack(uint32_t cmd, uint32_t length)
{
  unsigned char frameHeadBytes[16];
  RobosenseNetFrameHead frameHead;
  frameHead.frameFlag = g_frame_flag;
  frameHead.cmd = cmd;
  frameHead.length = length;
  frameHead.checkSum = CheckSum(frameHead);

  memcpy(frameHeadBytes, reinterpret_cast<char *>(&frameHead), sizeof(frameHead));
  std::vector<unsigned char> vec_buffer;
  for (unsigned int i = 0; i < sizeof(frameHead); ++i) {
    vec_buffer.push_back(frameHeadBytes[i]);
  }
  return vec_buffer;
}
}  // namespace nebula
#endif  // ROBOSENSE_CMD_RESPONSE_HPP