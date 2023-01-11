#pragma once
/**
 * Pandar 40P
 */
#include <cstddef>
#include <cstdint>
#include <ctime>

namespace nebula
{
namespace drivers
{
namespace pandar_40
{
constexpr size_t SOB_ANGLE_SIZE = 4;
constexpr size_t RAW_MEASURE_SIZE = 3;
constexpr size_t LASER_COUNT = 40;
constexpr size_t BLOCKS_PER_PACKET = 10;
constexpr size_t BLOCK_SIZE = RAW_MEASURE_SIZE * LASER_COUNT + SOB_ANGLE_SIZE;
constexpr size_t TIMESTAMP_SIZE = 4;
constexpr size_t FACTORY_INFO_SIZE = 1;
constexpr size_t RETURN_SIZE = 1;
constexpr size_t RESERVE_SIZE = 8;
constexpr size_t REVOLUTION_SIZE = 2;
constexpr size_t INFO_SIZE =
  TIMESTAMP_SIZE + FACTORY_INFO_SIZE + RETURN_SIZE + RESERVE_SIZE + REVOLUTION_SIZE;
constexpr size_t UTC_TIME = 6;
constexpr size_t PACKET_SIZE = BLOCK_SIZE * BLOCKS_PER_PACKET + INFO_SIZE + UTC_TIME;
constexpr size_t SEQ_NUM_SIZE = 4;
constexpr double LASER_RETURN_TO_DISTANCE_RATE = 0.004;
constexpr uint32_t STRONGEST_RETURN = 0x37;
constexpr uint32_t LAST_RETURN = 0x38;
constexpr uint32_t DUAL_RETURN = 0x39;

struct Unit
{
  uint8_t intensity;
  float distance;
};

struct Block
{
  uint16_t azimuth;
  uint16_t sob;
  Unit units[LASER_COUNT];
};

struct Packet
{
  Block blocks[BLOCKS_PER_PACKET];
  struct tm t;
  uint32_t usec;
  uint32_t return_mode;
};

}  // namespace pandar_40
}  // namespace drivers
}  // namespace nebula