#include "nebula_decoders/nebula_decoders_hesai/decoders/hesai_packet.hpp"

namespace nebula
{
namespace drivers
{
namespace hesai_packet
{

int get_n_returns(uint8_t return_mode)
{
  switch (return_mode) {
    case return_mode::SINGLE_FIRST:
    case return_mode::SINGLE_SECOND:
    case return_mode::SINGLE_STRONGEST:
    case return_mode::SINGLE_LAST:
      return 1;
    case return_mode::DUAL_LAST_STRONGEST:
    case return_mode::DUAL_FIRST_SECOND:
    case return_mode::DUAL_FIRST_LAST:
    case return_mode::DUAL_FIRST_STRONGEST:
    case return_mode::DUAL_STRONGEST_SECONDSTRONGEST:
      return 2;
    case return_mode::TRIPLE_FIRST_LAST_STRONGEST:
      return 3;
    default:
      throw std::runtime_error("Unknown return mode");
  }
}

template <typename PacketT>
uint64_t get_timestamp_ns(const PacketT & packet)
{
  return packet.tail.date_time.get_seconds() * 1000000000 + packet.tail.timestamp * 1000;
}

template <typename PacketT>
uint8_t get_dis_unit(const PacketT & packet)
{
  return packet.header.dis_unit;
}

/// @brief Get the distance unit of the given @ref Packet40P packet in millimeters. This is the only
/// packet type without a header.
/// @param dummy
/// @return The value 4.
template <>
uint8_t get_dis_unit<Packet40P>(const Packet40P & /* packet */)
{
  return 4;
}

// Explicit template instantiation to prevent linker errors
template uint64_t get_timestamp_ns<Packet128E3X>(const Packet128E3X & packet);
template uint64_t get_timestamp_ns<PacketXT32>(const PacketXT32 & packet);
template uint64_t get_timestamp_ns<PacketXT32M2X>(const PacketXT32M2X & packet);
template uint64_t get_timestamp_ns<PacketQT128C2X>(const PacketQT128C2X & packet);
template uint64_t get_timestamp_ns<PacketAT128E2X>(const PacketAT128E2X & packet);
template uint64_t get_timestamp_ns<PacketQT64>(const PacketQT64 & packet);
template uint64_t get_timestamp_ns<Packet64>(const Packet64 & packet);
template uint64_t get_timestamp_ns<Packet40P>(const Packet40P & packet);

template uint8_t get_dis_unit<Packet128E3X>(const Packet128E3X & packet);
template uint8_t get_dis_unit<PacketXT32>(const PacketXT32 & packet);
template uint8_t get_dis_unit<PacketXT32M2X>(const PacketXT32M2X & packet);
template uint8_t get_dis_unit<PacketQT128C2X>(const PacketQT128C2X & packet);
template uint8_t get_dis_unit<PacketAT128E2X>(const PacketAT128E2X & packet);
template uint8_t get_dis_unit<PacketQT64>(const PacketQT64 & packet);
template uint8_t get_dis_unit<Packet64>(const Packet64 & packet);
template uint8_t get_dis_unit<Packet40P>(const Packet40P & packet);

}  // namespace hesai_packet
}  // namespace drivers
}  // namespace nebula
