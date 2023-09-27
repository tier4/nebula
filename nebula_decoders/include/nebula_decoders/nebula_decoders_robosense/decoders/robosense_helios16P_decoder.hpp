/*********************************************************************************************************************
Copyright (c) 2020 RoboSense
All rights reserved

By downloading, copying, installing or using the software you agree to this license. If you do not
agree to this license, do not download, install, copy or use the software.

License Agreement
For RoboSense LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are permitted
provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions
and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions
and the following disclaimer in the documentation and/or other materials provided with the
distribution.

3. Neither the names of the RoboSense, nor Suteng Innovation Technology, nor the names of other
contributors may be used to endorse or promote products derived from this software without specific
prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#pragma once
#include "robosense_helios_decoder.hpp"
#include "robosense_mech_decoder.hpp"
namespace nebula
{
namespace drivers
{
namespace robosense_helios16P
{
#pragma pack(push, 1)

#pragma pack(pop)

class RobosenseHelios16PDecoder : public DecoderMech
{
public:
  virtual void decodeDifopPkt(const uint8_t * pkt);
  virtual bool decodeMsopPkt(const uint8_t * pkt);
  virtual ~RobosenseHelios16PDecoder() = default;

  explicit RobosenseHelios16PDecoder(const RSDecoderParam & param);
  std::tuple<drivers::NebulaPointCloudPtr, double> get_pointcloud() override;

#ifndef UNIT_TEST
protected:
#endif

  void calcParam();
  static RSDecoderMechConstParam & getConstParam();
  RSEchoMode getEchoMode(uint8_t mode);

  template <typename T_BlockIterator>
  bool internDecodeMsopPkt(const uint8_t * pkt);
};

inline RSDecoderMechConstParam & RobosenseHelios16PDecoder::getConstParam()
{
  static RSDecoderMechConstParam param = {
    1248  // msop len
    ,
    1248  // difop len
    ,
    4  // msop id len
    ,
    8  // difop id len
    ,
    {0x55, 0xAA, 0x05, 0x5A}  // msop id
    ,
    {0xA5, 0xFF, 0x00, 0x5A, 0x11, 0x11, 0x55, 0x55}  // difop id
    ,
    {0xFF, 0xEE}  // block id
    ,
    16  // laser number
    ,
    12  // blocks per packet
    ,
    32  // channels per block
    ,
    0.1f  // distance min
    ,
    180.0f  // distance max
    ,
    0.0025f  // distance resolution
    ,
    0.0625f  // temperature resolution

    // lens center
    ,
    0.03498f  // RX
    ,
    -0.015f  // RY
    ,
    0.0f  // RZ
    ,
    0.0f  // BLOCK_DURATION
    ,
    {0.0f}  // CHAN_TSS
    ,
    {0.0f}  // CHAN_AZIS
  };

  float blk_ts = 55.56f;
  param.BLOCK_DURATION = blk_ts / 1000000;

  return param;
}  // namespace robosense_helios16P

inline void RobosenseHelios16PDecoder::calcParam()
{
  float blk_ts = 55.56f;
  float firing_tss[] = {0.00f,  3.15f,  6.30f,  9.45f,  13.26f, 17.08f, 20.56f, 23.71f,
                        26.53f, 27.77f, 31.49f, 32.73f, 36.46f, 38.94f, 41.42f, 43.91f,
                        55.56f, 58.70f, 61.85f, 65.00f, 68.82f, 72.64f, 76.12f, 79.27f,
                        82.08f, 83.32f, 87.05f, 88.29f, 92.01f, 94.50f, 96.98f, 99.46f};

  if (this->echo_mode_ == RSEchoMode::ECHO_SINGLE) {
    Rs16SingleReturnBlockIterator<robosense_helios::RSHELIOSMsopPkt>::calcChannel(
      blk_ts, firing_tss, this->mech_const_param_.CHAN_AZIS, this->mech_const_param_.CHAN_TSS);
  } else {
    Rs16DualReturnBlockIterator<robosense_helios::RSHELIOSMsopPkt>::calcChannel(
      blk_ts, firing_tss, this->mech_const_param_.CHAN_AZIS, this->mech_const_param_.CHAN_TSS);
  }
}

inline RSEchoMode RobosenseHelios16PDecoder::getEchoMode(uint8_t mode)
{
  switch (mode) {
    case 0x00:  // dual return
      first_return_type_ = static_cast<uint8_t>(ReturnType::STRONGEST);
      second_return_type_ = static_cast<uint8_t>(ReturnType::SECONDSTRONGEST);
      return RSEchoMode::ECHO_DUAL;
    case 0x04:  // strongest return
      return_type_ = static_cast<uint8_t>(ReturnType::STRONGEST);
      return RSEchoMode::ECHO_SINGLE;
    case 0x05:  // last return
      return_type_ = static_cast<uint8_t>(ReturnType::LAST);
      return RSEchoMode::ECHO_SINGLE;
    case 0x06:  // nearest return
      return_type_ = static_cast<uint8_t>(ReturnType::FIRST);
      return RSEchoMode::ECHO_SINGLE;
    default:
      return_type_ = static_cast<uint8_t>(ReturnType::STRONGEST);
      return RSEchoMode::ECHO_SINGLE;
  }
}

inline RobosenseHelios16PDecoder::RobosenseHelios16PDecoder(const RSDecoderParam & param)
: DecoderMech(getConstParam(), param)
{
  this->packet_duration_ =
    this->mech_const_param_.BLOCK_DURATION * this->const_param_.BLOCKS_PER_PKT * 2;

  calcParam();
  this->current_point_cloud_.reset(new NebulaPointCloud);
  this->next_point_cloud_.reset(new NebulaPointCloud);
}

inline void RobosenseHelios16PDecoder::decodeDifopPkt(const uint8_t * packet)
{
  const robosense_helios::RSHELIOSDifopPkt & pkt =
    *(const robosense_helios::RSHELIOSDifopPkt *)(packet);
  this->template decodeDifopCommon<robosense_helios::RSHELIOSDifopPkt>(pkt);

  RSEchoMode echo_mode = getEchoMode(pkt.return_mode);
  if (this->echo_mode_ != echo_mode) {
    this->echo_mode_ = echo_mode;
    this->split_blks_per_frame_ = (this->echo_mode_ == RSEchoMode::ECHO_DUAL)
                                    ? this->blks_per_frame_
                                    : (this->blks_per_frame_ >> 1);
    calcParam();
  }

  this->device_info_.rpm = ntohs(pkt.diagnosis.real_rpm);
  this->device_info_.bot_fpga_temperature = ntohs(pkt.diagnosis.bot_fpga_temperature);
  this->device_info_.recv_A_temperature = ntohs(pkt.diagnosis.recv_A_temperature);
  this->device_info_.recv_B_temperature = ntohs(pkt.diagnosis.recv_B_temperature);
  this->device_info_.main_fpga_temperature = ntohs(pkt.diagnosis.main_fpga_temperature);
  this->device_info_.main_fpga_core_temperature = ntohs(pkt.diagnosis.main_fpga_core_temperature);
  this->device_info_.lane_up = pkt.diagnosis.lane_up;
  this->device_info_.lane_up_cnt = ntohs(pkt.diagnosis.lane_up_cnt);
  this->device_info_.main_status = ntohs(pkt.diagnosis.main_status);
  this->device_info_.gps_status = pkt.diagnosis.gps_status;
}
std::tuple<drivers::NebulaPointCloudPtr, double> RobosenseHelios16PDecoder::get_pointcloud()
{
  return std::make_tuple(this->current_point_cloud_, this->cloudTs());
}
inline bool RobosenseHelios16PDecoder::decodeMsopPkt(const uint8_t * pkt)
{
  if (this->echo_mode_ == RSEchoMode::ECHO_SINGLE) {
    return internDecodeMsopPkt<SingleReturnBlockIterator<robosense_helios::RSHELIOSMsopPkt>>(pkt);
  } else {
    return internDecodeMsopPkt<DualReturnBlockIterator<robosense_helios::RSHELIOSMsopPkt>>(pkt);
  }
}

template <typename T_BlockIterator>
inline bool RobosenseHelios16PDecoder::internDecodeMsopPkt(const uint8_t * packet)
{
  if (isSpliteFrame_) {
    this->current_point_cloud_ = this->next_point_cloud_;
    this->next_point_cloud_.reset(new NebulaPointCloud);
    isSpliteFrame_ = false;
  }
  const robosense_helios::RSHELIOSMsopPkt & pkt =
    *(const robosense_helios::RSHELIOSMsopPkt *)(packet);

  this->temperature_ = parseTempInLe(&(pkt.header.temp)) * this->const_param_.TEMPERATURE_RES;

  double pkt_ts = 0;
  if (this->param_.use_lidar_clock) {
    pkt_ts = parseTimeUTCWithUs(&pkt.header.timestamp) * 1e-6;
  } else {
    uint64_t ts = getTimeHost();

    // roll back to first block to approach lidar ts as near as possible.
    pkt_ts = ts * 1e-6 - this->getPacketDuration();

    if (this->write_pkt_ts_) {
      createTimeUTCWithUs(ts, (RSTimestampUTC *)&pkt.header.timestamp);
    }
  }
  T_BlockIterator iter(
    pkt, this->const_param_.BLOCKS_PER_PKT, this->mech_const_param_.BLOCK_DURATION,
    this->block_az_diff_, this->fov_blind_ts_diff_);

  for (uint16_t blk = 0; blk < this->const_param_.BLOCKS_PER_PKT; blk++) {
    const robosense_helios::RSHELIOSMsopBlock & block = pkt.blocks[blk];
    if (memcmp(this->const_param_.BLOCK_ID, block.id, 2) != 0) {
      std::cout << "ERRCODE_WRONGMSOPBLKID" << std::endl;
      break;
    }
    int32_t block_az_diff;
    double block_ts_off;
    iter.get(blk, block_az_diff, block_ts_off);

    double block_ts = pkt_ts + block_ts_off;
    int32_t block_az = ntohs(block.azimuth);
    if (this->split_strategy_->newBlock(block_az)) {
      this->first_point_ts_ = block_ts;
      isSpliteFrame_ = true;
    }
    NebulaPointCloudPtr block_pc(new NebulaPointCloud);
    for (uint16_t chan = 0; chan < this->const_param_.CHANNELS_PER_BLOCK; chan++) {
      const RSChannel & channel = block.channels[chan];
      double chan_ts = block_ts + this->mech_const_param_.CHAN_TSS[chan];
      int32_t angle_horiz =
        block_az + (int32_t)((float)block_az_diff * this->mech_const_param_.CHAN_AZIS[chan]);
      uint16_t laser = chan % 16;
      int32_t angle_vert = this->chan_angles_.vertAdjust(laser);
      int32_t angle_horiz_final = this->chan_angles_.horizAdjust(laser, angle_horiz);
      float distance = ntohs(channel.distance) * this->const_param_.DISTANCE_RES;

      if (this->distance_section_.in(distance) && this->scan_section_.in(angle_horiz_final)) {
        float x = distance * COS(angle_vert) * COS(angle_horiz_final) +
                  this->mech_const_param_.RX * COS(angle_horiz);
        float y = -distance * COS(angle_vert) * SIN(angle_horiz_final) -
                  this->mech_const_param_.RX * SIN(angle_horiz);
        float z = distance * SIN(angle_vert) + this->mech_const_param_.RZ;
        NebulaPoint point{};
        point.x = x;
        point.y = y;
        point.z = z;
        point.intensity = channel.intensity;
        point.channel = this->chan_angles_.toUserChan(laser);
        point.azimuth = angle_horiz_final;
        point.distance = distance;
        point.elevation = angle_vert;
        if (this->echo_mode_ == RSEchoMode::ECHO_SINGLE) {
          point.return_type = this->return_type_;
        } else {
          if (chan < 16) {
            point.return_type = this->first_return_type_;
          } else {
            point.return_type = this->second_return_type_;
          }
        }

        point.time_stamp = chan_ts;

        block_pc->points.emplace_back(point);
      } else if (!this->param_.dense_points) {
        NebulaPoint point{};
        block_pc->points.emplace_back(point);
      }
      this->prev_point_ts_ = chan_ts;
    }
    if (!isSpliteFrame_) {
      *this->current_point_cloud_ += *block_pc;
    } else {
      *this->next_point_cloud_ += *block_pc;
    }
  }
  this->prev_pkt_ts_ = pkt_ts;
  return isSpliteFrame_;
}
}  // namespace robosense_helios16P
}  // namespace drivers
}  // namespace nebula
