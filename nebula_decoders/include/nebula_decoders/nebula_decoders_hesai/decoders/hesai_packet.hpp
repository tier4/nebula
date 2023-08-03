#pragma once

#include "nebula_decoders/nebula_decoders_hesai/decoders/angle_corrector_calibration_based.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/angle_corrector_correction_based.hpp"

#include <cstddef>
#include <cstdint>
#include <ctime>

namespace nebula
{
namespace drivers
{
namespace hesai_packet
{

namespace return_mode
{
enum ReturnMode {
  SINGLE_FIRST = 0x33,
  SINGLE_SECOND = 0x34,
  SINGLE_STRONGEST = 0x37,
  SINGLE_LAST = 0x38,
  DUAL_LAST_STRONGEST = 0x39,
  DUAL_FIRST_SECOND = 0x3a,
  DUAL_FIRST_LAST = 0x3b,
  DUAL_FIRST_STRONGEST = 0x3c,
  TRIPLE_FIRST_LAST_STRONGEST = 0x3d,
  DUAL_STRONGEST_SECONDSTRONGEST = 0x3,
};
}  // namespace return_mode

#pragma pack(push, 1)

/// @brief DateTime struct for Hesai packets
/// @tparam YearOffset like std::tm, the Hesai format has a year offset that is applied to the raw
/// year value. For most protocol versions it is 1900 (like std::tm), for some it is 2000.
template <int YearOffset>
struct DateTime
{
  /// @brief Year - YearOffset (e.g. for YearOffset=1900 value 100 corresponds to year 2000)
  uint8_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;

  /// @brief Get seconds since epoch
  /// @return Whole seconds since epoch
  uint64_t get_seconds() const
  {
    std::tm tm{};
    tm.tm_year = year - 1900 + YearOffset;
    tm.tm_mon = month - 1;  // starts from 0 in C
    tm.tm_mday = day;
    tm.tm_hour = hour;
    tm.tm_min = minute;
    tm.tm_sec = second;
    return std::mktime(&tm);
  }
};

struct SecondsSinceEpoch
{
  uint8_t zero;
  /// @brief Seconds since epoch, in big-endian format
  uint8_t seconds[5];

  /// @brief Get seconds since epoch
  /// @return Whole seconds since epoch
  uint64_t get_seconds() const
  {
    uint64_t seconds = 0;
    for (int i = 0; i < 5; ++i) {
      seconds = (seconds << 8) | this->seconds[i];
    }
    return seconds;
  }
};

struct FunctionalSafety
{
  uint8_t fs_version;
  uint8_t lidar_state;
  uint8_t fault_code_id;
  uint16_t fault_code;
  uint8_t reserved1[8];
  uint32_t crc_fs;
};

struct Header12B
{
  uint16_t sop;
  uint8_t protocol_major;
  uint8_t protocol_minor;
  uint8_t reserved1[2];

  uint8_t laser_num;
  uint8_t block_num;
  uint8_t reserved2;
  uint8_t dis_unit;
  uint8_t return_num;
  uint8_t flags;
};

struct Header8B
{
  /// @brief Start of Packet, 0xEEFF
  uint16_t sop;

  uint8_t laser_num;
  uint8_t block_num;
  uint8_t reserved1;
  uint8_t dis_unit;
  uint8_t reserved2[2];
};

struct Unit3B
{
  uint16_t distance;
  uint8_t reflectivity;
};

struct Unit4B
{
  uint16_t distance;
  uint8_t reflectivity;
  uint8_t confidence_or_reserved;
};

template <typename UnitT, size_t UnitN>
struct Block
{
  uint16_t azimuth;
  UnitT units[UnitN];
  typedef UnitT unit_t;

  uint32_t get_azimuth() const { return azimuth; }
};

template <typename UnitT, size_t UnitN>
struct FineAzimuthBlock
{
  typedef UnitT unit_t;
  uint16_t azimuth;
  uint8_t fine_azimuth;
  UnitT units[UnitN];

  uint32_t get_azimuth() const { return (azimuth << 8) + fine_azimuth; }
};

template <typename UnitT, size_t UnitN>
struct SOBBlock
{
  typedef UnitT unit_t;

  /// @brief Start of Block, 0xFFEE
  uint16_t sob;
  uint16_t azimuth;
  UnitT units[UnitN];

  uint32_t get_azimuth() const { return azimuth; }
};

template <typename BlockT, size_t BlockN>
struct Body
{
  typedef BlockT block_t;
  BlockT blocks[BlockN];
};

struct Tail128E3X
{
  uint8_t reserved1[9];
  uint16_t azimuth_state;
  uint8_t operational_state;
  uint8_t return_mode;
  uint16_t motor_speed;
  DateTime<1900> date_time;
  uint32_t timestamp;
  uint8_t factory_information;

  /* Ignored optional fields */

  //uint32_t udp_sequence;

  //uint16_t imu_temperature;
  //uint16_t imu_acceleration_unit;
  //uint16_t imu_angular_velocity_unit;
  //uint32_t imu_timestamp;
  //uint16_t imu_x_axis_acceleration;
  //uint16_t imu_y_axis_acceleration;
  //uint16_t imu_z_axis_acceleration;
  //uint16_t imu_x_axis_angular_velocity;
  //uint16_t imu_y_axis_angular_velocity;
  //uint16_t imu_z_axis_angular_velocity;

  //uint32_t crc_tail;
};

struct TailXT32
{
  uint8_t reserved1[10];
  uint8_t return_mode;
  uint16_t motor_speed;
  DateTime<1900> date_time;
  uint32_t timestamp;
  uint8_t factory_information;
};

struct Tail40P
{
  uint8_t reserved1[5];
  uint8_t high_temperature_shutdown_flag;
  uint8_t reserved2[2];
  uint16_t motor_speed;
  uint32_t timestamp;
  uint8_t return_mode;
  uint8_t factory_information;
  DateTime<2000> date_time;

  /* Ignored optional fields */

  //uint32_t udp_sequence;
};

struct TailQT128C2X
{
  uint8_t reserved1[5];
  uint8_t mode_flag;
  uint8_t reserved2[6];
  uint8_t return_mode;
  uint16_t motor_speed;
  DateTime<1900> date_time;
  uint32_t timestamp;
  uint8_t factory_information;
  
  /* Ignored optional fields */

  //uint32_t udp_sequence;
  //uint32_t crc_tail;
};

struct TailAT128E2X
{
  uint8_t reserved1[6];
  uint8_t high_temperature_shutdown_flag;
  uint8_t reserved2[11];
  uint16_t motor_speed;
  uint32_t timestamp;
  uint8_t return_mode;
  uint8_t factory_information;
  SecondsSinceEpoch date_time;
  
  /* Ignored optional fields */

  //uint32_t udp_sequence;
  //uint32_t crc_tail;
};

struct TailQT64
{
  uint8_t reserved1[10];
  uint16_t motor_speed;
  uint32_t timestamp;
  uint8_t return_mode;
  uint8_t factory_information;
  DateTime<1900> date_time;
};

/// @brief Base struct for all Hesai packets. This struct is not allowed to have any non-static
/// members, otherwise memory layout is not guaranteed for the derived structs.
/// @tparam nBlocks The number of blocks in the packet
/// @tparam nChannels The number of channels per block
/// @tparam maxReturns The maximum number of returns, e.g. 2 for dual return
/// @tparam degreeSubdivisions The resolution of the azimuth angle in the packet, e.g. 100 if packet
/// azimuth is given in 1/100th of a degree
template <size_t nBlocks, size_t nChannels, size_t maxReturns, size_t degreeSubdivisions>
struct PacketBase
{
  static constexpr size_t N_BLOCKS = nBlocks;
  static constexpr size_t N_CHANNELS = nChannels;
  static constexpr size_t MAX_RETURNS = maxReturns;
  static constexpr size_t DEGREE_SUBDIVISIONS = degreeSubdivisions;
};

struct Packet128E3X : public PacketBase<2, 128, 2, 100>
{
  typedef Body<Block<Unit3B, Packet128E3X::N_CHANNELS>, Packet128E3X::N_BLOCKS> body_t;
  typedef AngleCorrectorCalibrationBased<
    Packet128E3X::N_CHANNELS, Packet128E3X::DEGREE_SUBDIVISIONS>
    angle_decoder_t;
  Header12B header;
  body_t body;
  uint32_t crc_body;
  FunctionalSafety fs;
  Tail128E3X tail;

  // FIXME(mojomex) uint8_t cyber_security[32]; is added optionally
};

struct PacketXT32 : public PacketBase<8, 32, 2, 100>
{
  typedef Body<Block<Unit4B, PacketXT32::N_CHANNELS>, PacketXT32::N_BLOCKS> body_t;
  typedef AngleCorrectorCalibrationBased<PacketXT32::N_CHANNELS, PacketXT32::DEGREE_SUBDIVISIONS>
    angle_decoder_t;
  Header12B header;
  body_t body;
  TailXT32 tail;
  uint32_t udp_sequence;
};

typedef TailXT32 TailXT32M2X;
struct PacketXT32M2X : public PacketBase<6, 32, 3, 100>
{
  typedef Body<Block<Unit4B, PacketXT32M2X::N_CHANNELS>, PacketXT32M2X::N_BLOCKS> body_t;
  typedef AngleCorrectorCalibrationBased<
    PacketXT32M2X::N_CHANNELS, PacketXT32M2X::DEGREE_SUBDIVISIONS>
    angle_decoder_t;
  Header12B header;
  body_t body;
  TailXT32M2X tail;
  uint32_t udp_sequence;
};

struct PacketQT128C2X : public PacketBase<2, 128, 2, 100>
{
  typedef Body<Block<Unit4B, PacketQT128C2X::N_CHANNELS>, PacketQT128C2X::N_BLOCKS> body_t;
  typedef AngleCorrectorCalibrationBased<
    PacketQT128C2X::N_CHANNELS, PacketQT128C2X::DEGREE_SUBDIVISIONS>
    angle_decoder_t;
  Header12B header;
  body_t body;
  uint32_t crc_body;
  FunctionalSafety fs;
  TailQT128C2X tail;
  /// @brief Always present, all zeros by default
  uint8_t cyber_security[32];
};

struct PacketAT128E2X : public PacketBase<2, 128, 2, 100 * 256>
{
  typedef Body<FineAzimuthBlock<Unit4B, PacketAT128E2X::N_CHANNELS>, PacketAT128E2X::N_BLOCKS>
    body_t;
  typedef AngleCorrectorCorrectionBased<
    PacketAT128E2X::N_CHANNELS, PacketAT128E2X::DEGREE_SUBDIVISIONS>
    angle_decoder_t;
  Header12B header;
  body_t body;
  uint32_t crc_body;
  TailAT128E2X tail;
  /// @brief Always present, not supported according to datasheet
  uint8_t cyber_security[32];
};

struct PacketQT64 : public PacketBase<4, 64, 2, 100>
{
  typedef Body<Block<Unit4B, PacketQT64::N_CHANNELS>, PacketQT64::N_BLOCKS> body_t;
  typedef AngleCorrectorCalibrationBased<PacketQT64::N_CHANNELS, PacketQT64::DEGREE_SUBDIVISIONS>
    angle_decoder_t;
  Header12B header;
  body_t body;
  TailQT64 tail;
  uint32_t udp_sequence;
};

typedef Tail40P Tail64;
struct Packet64 : public PacketBase<6, 64, 2, 100>
{
  typedef Body<Block<Unit3B, Packet64::N_CHANNELS>, Packet64::N_BLOCKS> body_t;
  typedef AngleCorrectorCalibrationBased<Packet64::N_CHANNELS, Packet64::DEGREE_SUBDIVISIONS>
    angle_decoder_t;
  Header8B header;
  body_t body;
  Tail64 tail;
};

struct Packet40P : public PacketBase<10, 40, 2, 100>
{
  typedef Body<SOBBlock<Unit3B, Packet40P::N_CHANNELS>, Packet40P::N_BLOCKS> body_t;
  typedef AngleCorrectorCalibrationBased<Packet40P::N_CHANNELS, Packet40P::DEGREE_SUBDIVISIONS>
    angle_decoder_t;
  body_t body;
  Tail40P tail;
};

#pragma pack(pop)

/// @brief Get the number of returns for a given return mode
/// @param return_mode The return mode
/// @return The number of returns
int get_n_returns(uint8_t return_mode);

/// @brief Get timestamp from packet in nanoseconds
/// @tparam PacketT The packet type
/// @param packet The packet to get the timestamp from
/// @return The timestamp in nanoseconds
template <typename PacketT>
uint64_t get_timestamp_ns(const PacketT & packet);

template <typename PacketT>
uint8_t get_dis_unit(const PacketT & packet);

}  // namespace hesai_packet
}  // namespace drivers
}  // namespace nebula
