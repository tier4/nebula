#pragma once

#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace nebula
{
namespace drivers
{

/// @brief Get a pointer to the block identified by `block_id` in the given packet.
template <typename PacketT>
const typename PacketT::body_t::block_t * getBlock(const PacketT & packet, const size_t block_id)
{
  return &packet.body.blocks[block_id];
}

/// @brief Get a pointer to the unit identified by `block_id` and `unit_id` in the given packet.
template <typename PacketT>
const typename PacketT::body_t::block_t::unit_t * getUnit(
  const PacketT & packet, const size_t block_id, const size_t unit_id)
{
  return &packet.body.blocks[block_id].units[unit_id];
}

/// @brief Field accessor for Boost endian buffers. Can be used to generically access fields across
/// packet types.
/// @return t.value()
template <typename T>
typename T::value_type getFieldValue(T t)
{
  return t.value();
}

/// @brief Field accessor for primitive types. Can be used to generically access fields across
/// packet types.
/// @return t (as-is)
template <typename T>
std::enable_if_t<std::is_fundamental_v<T>, T> getFieldValue(T t)
{
  return t;
}

}  // namespace drivers
}  // namespace nebula