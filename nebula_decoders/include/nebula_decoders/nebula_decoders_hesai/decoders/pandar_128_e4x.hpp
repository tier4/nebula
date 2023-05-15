#pragma once
/**
 * Pandar 128 E4X
 */
#include <cstddef>
#include <cstdint>
#include <ctime>

namespace nebula
{
namespace drivers
{
namespace pandar_128_e4x
{
constexpr size_t LASER_COUNT = 128;

constexpr uint8_t SINGLE_FIRST_RETURN = 0x33;
constexpr uint8_t SINGLE_STRONGEST_RETURN = 0x37;
constexpr uint8_t SINGLE_LAST_RETURN = 0x38;

constexpr uint8_t DUAL_LAST_STRONGEST_RETURN = 0x39;
constexpr uint8_t DUAL_LAST_FIRST_RETURN = 0x3B;
constexpr uint8_t DUAL_FIRST_STRONGEST_RETURN = 0x3C;

constexpr uint8_t HIGH_RES_STATE = 0x00;
constexpr uint8_t STANDARD_RES_STATE = 0x01;

constexpr uint16_t MAX_AZIMUTH_STEPS = 3600;  // High Res mode
constexpr float DISTANCE_UNIT = 0.004f;       // 4mm

constexpr uint8_t HEADER_SIZE = 12;
constexpr uint16_t BODY_SIZE = 776;
constexpr uint8_t TAIL_SIZE = 56;
constexpr uint8_t FUNCTIONAL_SAFETY_SIZE = 17;
constexpr uint16_t PACKET_SIZE = HEADER_SIZE + BODY_SIZE + FUNCTIONAL_SAFETY_SIZE + TAIL_SIZE;
constexpr float MIN_RANGE = 0.1;
constexpr float MAX_RANGE = 230.0;
constexpr uint16_t THREE_SIXTY = 360;
constexpr size_t NUM_BLOCKS = 2;

constexpr size_t UNUSED_INT = 0;

#pragma pack(push, 1)
struct Header
{  // 12 bytes
  // Pre header
  uint16_t SOP;
  uint8_t ProtocolMajor;
  uint8_t ProtocolMinor;
  uint16_t Reserved;
  // Header
  uint8_t LaserNum;
  uint8_t BlockNum;
  uint8_t FirstBlockReturn;
  uint8_t DistanceUnitMm;
  uint8_t ReturnNum;
  uint8_t Flags;
};

struct BlockExtended
{  // 4 bytes
  uint16_t distance;
  uint8_t reflectivity;
  uint8_t reserved;
};

struct Block
{  // 3 bytes
  uint16_t distance;
  uint8_t reflectivity;
};

struct Body
{  // 776 bytes
  uint16_t azimuth_1;
  Block block_01[LASER_COUNT];  // 384 bytes
  uint16_t azimuth_2;
  Block block_02[LASER_COUNT];  // 384 bytes
  uint32_t crc_1;
};

struct BodyExtended
{  // 1032 bytes
  uint16_t azimuth_1;
  BlockExtended block_01[LASER_COUNT];  // 512 bytes
  uint16_t azimuth_2;
  BlockExtended block_02[LASER_COUNT];  // 512 bytes
  uint32_t crc_1;
};

struct FunctionalSafety
{  // 17 bytes
  uint8_t reserved[17];
};

struct DateTime
{  // 6 bytes
  uint8_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
};

struct Tail
{  // 56
  uint8_t reserved_01[9];
  uint16_t azimuth_state;
  uint8_t operational_state;
  uint8_t return_mode;
  uint16_t motor_speed_rpm;
  DateTime date_time;
  uint32_t timestamp_us;
  uint8_t factory_info;
  uint32_t udp_sequence;
  uint8_t reserved_02[22];
  uint32_t crc_32;
};

struct Packet
{
  Header header;
  Body body;
  FunctionalSafety functional_safety;
  Tail tail;
};

struct PacketExtended
{
  Header header;
  BodyExtended body;
  FunctionalSafety functional_safety;
  Tail tail;
};
#pragma pack(pop)

}  // namespace pandar_128_e4x
}  // namespace drivers
}  // namespace nebula
