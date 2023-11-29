#pragma once

#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace nebula
{
namespace drivers
{

template <typename PacketT>
const typename PacketT::body_t::block_t * getBlock(const PacketT & packet, const size_t block_id)
{
  return &packet.body.blocks[block_id];
}

template <typename PacketT>
const typename PacketT::body_t::block_t::unit_t * getUnit(
  const PacketT & packet, const size_t block_id, const size_t unit_id)
{
  return &packet.body.blocks[block_id].units[unit_id];
}

/// @brief Field accessor for Boost endian buffers
/// @return t.value()
template <typename T>
typename T::value_type getFieldValue(T t) {
    return t.value();
}

/// @brief Field accessor for primitive types
/// @return t (as-is)
template <typename T>
std::enable_if_t<std::is_fundamental_v<T>, T> getFieldValue(T t) {
    return t;
}

}  // namespace drivers
}  // namespace nebula