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
  uint16_t webVersion[2];
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

struct RobosenseHeliosNetFrameHead
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

struct RobosenseHeliosNetWorkParamkParam
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

struct RobosenseBpRubyNetWorkParamkParam
{
  uint8_t mac[6];
  uint8_t ipLocal[4];
  uint8_t netmaskLocal[4];
  uint8_t gatewayLocal[4];
  uint16_t msopPort{0};
  uint16_t difopPort{0};
  uint8_t ipRemote[4];
};

uint16_t CheckSum(RobosenseHeliosNetFrameHead frameHead)
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
  RobosenseHeliosNetFrameHead frameHead;
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

int u16ToHex(unsigned char * pheDest, int const usSrc, int siHexlen)
{
  int i = 0;
  int num = usSrc;

  if ((NULL == pheDest) || (siHexlen < 0)) {
    return -1;
  }

  for (i = siHexlen; i > 0; i--) {
    *(pheDest + i - 1) = num % 256;
    num /= 256;
  }

  return siHexlen;
}

std::vector<unsigned char> BuildCommandFrame(char * inbuf, int length)
{
  unsigned char buffer[1024 * 2] = {0};
  int len = 0, i = 0, checkSum = 0;
  std::vector<unsigned char> vec_buffer;
  /* STX */
  buffer[len++] = 0xff;
  buffer[len++] = 0xff;
  /* Data type : 0x00 -- ack */
  buffer[len++] = 0x00;

  u16ToHex(&buffer[len], length, 2);
  len += 2;

  if (inbuf != NULL) {
    memcpy(&buffer[len], inbuf, length);
    len += length;
  }

  /* EXT */
  buffer[len++] = 0xFE;

  checkSum = 0;
  for (i = 2; i < len; i++) {
    checkSum += buffer[i];
  }

  /* checksum */
  u16ToHex(&buffer[len], checkSum, 2);
  len += 2;

#if 1
  // printf("func:%s, line:%d, send data(len = %d):\n", __func__, __LINE__, len);
  for (int i = 0; i < len; i++) {
    printf("%02x ", buffer[i]);
    vec_buffer.push_back(buffer[i]);
  }
  printf("\n");
  return vec_buffer;
  // printf("\n");
#endif
}

}  // namespace nebula
#endif  // ROBOSENSE_CMD_RESPONSE_HPP