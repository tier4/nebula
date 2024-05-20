#pragma once

#include "nebula_decoders/nebula_decoders_seyond/decoders/seyond_packet.hpp"
#include "nebula_decoders/nebula_decoders_seyond/decoders/seyond_sensor.hpp"

namespace nebula
{
namespace drivers
{

namespace seyond_packet
{

#pragma pack(push, 1)

struct PacketRobinW : public PacketBase<8, 32, 2, 100>
{

};

#pragma pack(pop)

}  // namespace seyond_packet

class RobinW : public SeyondSensor<seyond_packet::PacketRobinW>
{


};

}  // namespace drivers
}  // namespace nebula