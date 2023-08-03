#pragma once

#include "nebula_decoders/nebula_decoders_hesai/decoders/hesai_packet.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/hesai_sensor.hpp"

namespace nebula
{
namespace drivers
{

namespace hesai_packet
{

#pragma pack(push, 1)

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

struct Packet128E3X : public PacketBase<2, 128, 2, 100>
{
  typedef Body<Block<Unit3B, Packet128E3X::N_CHANNELS>, Packet128E3X::N_BLOCKS> body_t;
  Header12B header;
  body_t body;
  uint32_t crc_body;
  FunctionalSafety fs;
  Tail128E3X tail;

  /* Ignored optional fields */
  
  //uint8_t cyber_security[32];
};

#pragma pack(pop)

}  // namespace hesai_packet

// FIXME(mojomex) support high resolution mode
class Pandar128E3X : public HesaiSensor<hesai_packet::Packet128E3X>
{
public:
  int getChannelTimeOffset(uint32_t channel_id) override
  {
    return 0; //FIXME(mojomex) implement azimuth state & resolution mode support
  }

  int getBlockTimeOffset(uint32_t block_id, uint32_t n_returns) override
  {
    if (n_returns == 1) {
      return 3148 - 27778 * 2 * (2 - block_id - 1);
    }

    return 3148;
  }
};

}  // namespace drivers
}  // namespace nebula