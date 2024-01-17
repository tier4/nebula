#pragma once

#include "nebula_common/robosense/robosense_common.hpp"
#include "nebula_decoders/nebula_decoders_common/sensor_mixins/timestamp.hpp"

#include "boost/endian/buffers.hpp"

#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <string>

using namespace boost::endian;

namespace nebula
{
namespace drivers
{
namespace robosense_packet
{

#pragma pack(push, 1)

struct RobosensePacket
{
};

struct Timestamp
{
  big_uint48_buf_t seconds;
  big_uint32_buf_t microseconds;

  uint64_t get_time_in_ns() const
  {
    constexpr uint64_t NS_IN_SECOND = 1000000000ULL;
    constexpr uint64_t NS_IN_MICROSECOND = 1000ULL;

    uint64_t total_nanoseconds = seconds.value() * NS_IN_SECOND;
    total_nanoseconds += microseconds.value() * NS_IN_MICROSECOND;

    return total_nanoseconds;
  }
};

template <typename PacketT>
struct RobosensePacketTimestampMixin : public sensor_mixins::PacketTimestampMixin<PacketT>
{
  uint64_t getPacketTimestamp(const PacketT & packet) const override
  {
    return packet.header.timestamp.get_time_in_ns();
  }
};

struct Unit
{
  big_uint16_buf_t distance;
  big_uint8_buf_t reflectivity;
};

template <typename UnitT, size_t UnitN>
struct Block
{
  big_uint16_buf_t flag;
  big_uint16_buf_t azimuth;
  UnitT units[UnitN];
  typedef UnitT unit_t;

  [[nodiscard]] uint16_t get_azimuth() const { return azimuth.value(); }
};

template <typename BlockT, size_t BlockN>
struct Body
{
  typedef BlockT block_t;
  BlockT blocks[BlockN];
};

/// @brief Base struct for all Robosense packets. This struct is not allowed to have any non-static
/// members, otherwise memory layout is not guaranteed for the derived structs.
/// @tparam nBlocks The number of blocks in the packet
/// @tparam nChannels The number of channels per block
/// @tparam maxReturns The maximum number of returns, e.g. 2 for dual return
/// @tparam degreeSubdivisions The resolution of the azimuth angle in the packet, e.g. 100 if packet
/// @tparam returnsStridedPacket Whether returns within a return group are strided across packets
/// @tparam returnsStridedBlock Whether returns are strided across blocks
/// @tparam returnsStridedChannel Whether returns are strided across channels
/// azimuth is given in 1/100th of a degree
template <
  size_t nBlocks, size_t nChannels, size_t maxReturns, size_t degreeSubdivisions,
  bool returnsStridedPacket = 0, bool returnsStridedBlock = 1, bool returnsStridedChannel = 0>
struct PacketBase
{
  static constexpr size_t N_BLOCKS = nBlocks;
  static constexpr size_t N_CHANNELS = nChannels;
  static constexpr size_t MAX_RETURNS = maxReturns;
  static constexpr size_t DEGREE_SUBDIVISIONS = degreeSubdivisions;

  static constexpr std::array<bool, 3> RETURN_GROUP_STRIDE = {
    returnsStridedPacket, returnsStridedBlock, returnsStridedChannel};
};

struct IpAddress
{
  big_uint8_buf_t first_octet;
  big_uint8_buf_t second_octet;
  big_uint8_buf_t third_octet;
  big_uint8_buf_t fourth_octet;

  [[nodiscard]] std::string to_string() const
  {
    return std::to_string(first_octet.value()) + "." + std::to_string(second_octet.value()) + "." +
           std::to_string(third_octet.value()) + "." + std::to_string(fourth_octet.value());
  }
};

struct MacAddress
{
  big_uint8_buf_t first_octet;
  big_uint8_buf_t second_octet;
  big_uint8_buf_t third_octet;
  big_uint8_buf_t fourth_octet;
  big_uint8_buf_t fifth_octet;
  big_uint8_buf_t sixth_octet;

  [[nodiscard]] std::string to_string() const
  {
    std::stringstream ss;
    ss << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(first_octet.value())
       << ":" << std::setw(2) << static_cast<int>(second_octet.value()) << ":" << std::setw(2)
       << static_cast<int>(third_octet.value()) << ":" << std::setw(2)
       << static_cast<int>(fourth_octet.value()) << ":" << std::setw(2)
       << static_cast<int>(fifth_octet.value()) << ":" << std::setw(2)
       << static_cast<int>(sixth_octet.value());
    return ss.str();
  }
};

struct Ethernet
{
  IpAddress lidar_ip;
  IpAddress dest_pc_ip;
  MacAddress mac_addr;
  big_uint16_buf_t lidar_out_msop_port;
  big_uint16_buf_t pc_dest_msop_port;
  big_uint16_buf_t lidar_out_difop_port;
  big_uint16_buf_t pc_dest_difop_port;
};

struct FovSetting
{
  big_uint16_buf_t fov_start;
  big_uint16_buf_t fov_end;
};

constexpr uint8_t ANGLE_SIGN_FLAG = 0x00;

struct ChannelAngleCorrection
{
  big_uint8_buf_t sign;
  big_uint16_buf_t angle;

  [[nodiscard]] float getAngle() const
  {
    return sign.value() == ANGLE_SIGN_FLAG ? static_cast<float>(angle.value()) / 100.0f
                                           : static_cast<float>(angle.value()) / -100.0f;
  }
};

struct CorrectedVerticalAngle
{
  ChannelAngleCorrection angles[32];
};

struct CorrectedHorizontalAngle
{
  ChannelAngleCorrection angles[32];
};

struct SensorCalibration
{
  CorrectedVerticalAngle corrected_vertical_angle;
  CorrectedHorizontalAngle corrected_horizontal_angle;

  RobosenseCalibrationConfiguration getCalibration() const
  {
    RobosenseCalibrationConfiguration calibration;
    for (size_t i = 0; i < 32; ++i) {
      ChannelCorrection channel_correction;
      channel_correction.azimuth = corrected_horizontal_angle.angles[i].getAngle();
      channel_correction.elevation = corrected_vertical_angle.angles[i].getAngle();
      calibration.calibration.push_back(channel_correction);
    }
    return calibration;
  }
};

struct FirmwareVersion
{
  big_uint8_buf_t first_octet;
  big_uint8_buf_t second_octet;
  big_uint8_buf_t third_octet;
  big_uint8_buf_t fourth_octet;
  big_uint8_buf_t fifth_octet;

  [[nodiscard]] std::string to_string() const
  {
    std::stringstream ss;
    ss << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(first_octet.value())
       << std::setw(2) << static_cast<int>(second_octet.value()) << std::setw(2)
       << static_cast<int>(third_octet.value()) << std::setw(2)
       << static_cast<int>(fourth_octet.value()) << std::setw(2)
       << static_cast<int>(fifth_octet.value());
    return ss.str();
  }
};

struct SerialNumber
{
  big_uint8_buf_t first_octet;
  big_uint8_buf_t second_octet;
  big_uint8_buf_t third_octet;
  big_uint8_buf_t fourth_octet;
  big_uint8_buf_t fifth_octet;
  big_uint8_buf_t sixth_octet;

  [[nodiscard]] std::string to_string() const
  {
    std::stringstream ss;
    ss << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(first_octet.value())
       << std::setw(2) << static_cast<int>(second_octet.value()) << std::setw(2)
       << static_cast<int>(third_octet.value()) << std::setw(2)
       << static_cast<int>(fourth_octet.value()) << std::setw(2)
       << static_cast<int>(fifth_octet.value()) << std::setw(2)
       << static_cast<int>(sixth_octet.value());
    return ss.str();
  }
};

#pragma pack(pop)

/// @brief Get the number of returns for a given return mode
/// @param return_mode The return mode
/// @return The number of returns
size_t get_n_returns(ReturnMode return_mode)
{
  if (return_mode == ReturnMode::DUAL) {
    return 2;
  }
  return 1;
}

/// @brief Convert raw angle value from packet to std::string
/// @param raw_angle The raw angle value from the packet
/// @return The angle as std::string
std::string get_float_value(const uint16_t & raw_angle)
{
  return std::to_string(static_cast<float>(raw_angle) / 100.0f);
}

}  // namespace robosense_packet
}  // namespace drivers
}  // namespace nebula