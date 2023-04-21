#pragma once
/**
 * Pandar XT-32
 */
#include <cstddef>
#include <cstdint>
#include <ctime>

namespace nebula
{
namespace drivers
{
namespace pandar_xt
{
// Head
constexpr size_t HEAD_SIZE = 12;
constexpr size_t PRE_HEADER_SIZE = 6;
constexpr size_t HEADER_SIZE = 6;
// Body
constexpr size_t BLOCKS_PER_PACKET = 8;
constexpr size_t BLOCK_HEADER_AZIMUTH = 2;
constexpr size_t LASER_COUNT = 32;
constexpr size_t UNIT_SIZE = 4;
constexpr size_t BLOCK_SIZE = UNIT_SIZE * LASER_COUNT + BLOCK_HEADER_AZIMUTH;
constexpr size_t BODY_SIZE = BLOCK_SIZE * BLOCKS_PER_PACKET;
// Tail
constexpr size_t RESERVED_SIZE = 10;
constexpr size_t ENGINE_VELOCITY = 2;
constexpr size_t TIMESTAMP_SIZE = 4;
constexpr size_t RETURN_SIZE = 1;
constexpr size_t FACTORY_SIZE = 1;
constexpr size_t UTC_SIZE = 6;
constexpr size_t SEQUENCE_SIZE = 4;
constexpr size_t PACKET_TAIL_SIZE = 28;

// All
constexpr size_t PACKET_SIZE = HEAD_SIZE + BODY_SIZE + PACKET_TAIL_SIZE;

// 0x33 - First Return      0x39 - Dual Return (Last, Strongest)
// 0x37 - Strongest Return  0x3B - Dual Return (Last, First)
// 0x38 - Last Return       0x3C - Dual Return (First, Strongest)

constexpr uint32_t FIRST_RETURN = 0x33;
constexpr uint32_t STRONGEST_RETURN = 0x37;
constexpr uint32_t LAST_RETURN = 0x38;
constexpr uint32_t DUAL_RETURN = 0x39;

struct Header
{
  uint16_t sob;            // 0xFFEE 2bytes
  int8_t chProtocolMajor;  // Protocol Version Major 1byte
  int8_t chProtocolMinor;  // Protocol Version Minor 1byte
  int8_t chLaserNumber;    // laser number 1byte
  int8_t chBlockNumber;    // block number 1byte
  int8_t chReturnType;     // return mode 1 byte  when dual return 0-Single Return
                           // 1-The first block is the 1 st return.
                           // 2-The first block is the 2 nd return
  int8_t chDisUnit;        // Distance unit, 4mm
};

struct Unit
{
  float distance;
  uint16_t intensity;
  uint16_t confidence;
};

struct Block
{
  uint16_t azimuth;  // packet angle,Azimuth = RealAzimuth * 100
  Unit units[LASER_COUNT];
};

struct Packet
{
  Header header;
  Block blocks[BLOCKS_PER_PACKET];
  uint32_t usec;  // ms
  uint32_t return_mode;
  tm t;
};
}  // namespace pandar_xt
}  // namespace drivers
}  // namespace nebula
