#pragma once

#include "nebula_decoders/nebula_decoders_seyond/decoders/seyond_packet.hpp"
#include "nebula_decoders/nebula_decoders_seyond/decoders/seyond_scan_decoder.hpp"
#include "nebula_decoders/nebula_decoders_seyond/decoders/nps_adjustment.hpp"
#include "nebula_decoders/nebula_decoders_seyond/decoders/robin_nps_adjustment.hpp"

#include <rclcpp/rclcpp.hpp>

#include "nebula_msgs/msg/nebula_packet.hpp"
#include "nebula_msgs/msg/nebula_packets.hpp"
namespace nebula
{
namespace drivers
{
namespace seyond_packet
{
class SeyondBlockAngles {
 public:
  int16_t h_angle;
  int16_t v_angle;
};

class SeyondBlockFullAngles {
 public:
  SeyondBlockAngles angles[kSeyondChannelNumber];
};

template<class Block, class Point>
class SeyondDataPacketPointsCallbackParams {
  public:
   SeyondDataPacket pkt;
   Block block;
   Point pt;
   SeyondBlockFullAngles angle;
   uint16_t channel;
   uint16_t multi_return;
};

class SeyondDecoder : public SeyondScanDecoder
{
protected:
  /// @brief Configuration for this decoder
  const std::shared_ptr<drivers::SeyondSensorConfiguration> sensor_configuration_;
  const std::shared_ptr<drivers::SeyondCalibrationConfiguration> calibration_configuration_;

  /// @brief The sensor definition, used for return mode and time offset handling
  /// @brief The point cloud new points get added to
  NebulaPointCloudPtr decode_pc_;
  /// @brief The point cloud that is returned when a scan is complete
  NebulaPointCloudPtr output_pc_;
  /// @brief The timestamp of the last completed scan in nanoseconds
  uint64_t output_scan_timestamp_ns_;
  rclcpp::Logger logger_;
  std::vector<uint8_t> xyz_from_sphere_;

private:
  static int init_;
  static int v_angle_offset_[SEYOND_ITEM_TYPE_MAX][kSeyondChannelNumber];
  static const uint32_t kTableShift_ = 9;
  static const uint32_t kTableStep_ = 1 << kTableShift_;
  static const uint32_t kTableHalfStep_ = 1 << (kTableShift_ - 1);
  static const uint32_t kTableMask_ = kTableStep_ - 1;
  static const uint32_t kVTableSizeBits_ = 4;
  static const uint32_t kHTableSizeBits_ = 6;
  static const uint32_t kVTableSize_ = 1 << kVTableSizeBits_;
  static const uint32_t kHTableSize_ = 1 << kHTableSizeBits_;
  static const uint32_t kVTableEffeHalfSize_ = 6;
  static const uint32_t kHTableEffeHalfSize_ = 22;
  static const uint32_t kXZSize_ = 2;
  static const double kAdjustmentUnitInMeter_;
  static const double kAdjustmentUnitInMeterRobin_;
  static const uint32_t kHRobinTableEffeHalfSize_ = 27;
  static const uint32_t kXYZSize_ = 3;
  static const uint32_t kRobinScanlines_ = 192;
  static const uint32_t RobinWTDCChannelNumber = 48;
  static const uint32_t RobinETDCChannelNumber = 32;
  static const uint8_t robinw_channel_mapping[48];
  // use first 32 entries for robine
  static const uint8_t robine_channel_mapping[48];
  static const int32_t kAngleTableSize = 2 * kSeyondAngleUnitPerPiRad;
  static const int32_t kASinTableSize = 10000;
  static const int32_t kATanTableSize = 45000;
  static const int32_t kAtanTableScale = 10000;
  static const int32_t kAsinTableScale = 10000;

  static float sin_table_[];
  static float cos_table_[];
  static int32_t asin_table_[];
  static int32_t atan_table_[];
  static int8_t robin_nps_adjustment_[kRobinScanlines_][kHTableSize_][kXYZSize_];
  static int8_t nps_adjustment_[kVTableSize_][kHTableSize_][kSeyondChannelNumber][kXZSize_];
  double current_ts_start_ = 0.0;
  static constexpr double us_in_second_c = 1000000.0;
  static constexpr double ten_us_in_second_c = 100000.0;

public:
  /// @brief Constructor
  /// @param sensor_configuration SensorConfiguration for this decoder
  /// @param calibration_configuration Calibration for this decoder
  explicit SeyondDecoder(
    const std::shared_ptr<SeyondSensorConfiguration> & sensor_configuration,
    const std::shared_ptr<SeyondCalibrationConfiguration> & calibration_configuration);

  int unpack(const std::vector<uint8_t> & packet) override;

  std::tuple<drivers::NebulaPointCloudPtr, double> getPointcloud() override;

  static double lookup_cos_table_in_unit(int i);

  static double lookup_sin_table_in_unit(int i);

  static inline bool is_xyz_data(uint32_t packet_type) {
    SeyondItemType inno_type = static_cast<SeyondItemType>(packet_type);
    bool bSuccess = false;
    if(inno_type == SEYOND_ITEM_TYPE_XYZ_POINTCLOUD || inno_type == SEYOND_ROBINE_ITEM_TYPE_XYZ_POINTCLOUD
        ||inno_type == SEYOND_ROBINW_ITEM_TYPE_XYZ_POINTCLOUD||inno_type== SEYOND_FALCONII_DOT_1_ITEM_TYPE_XYZ_POINTCLOUD) {
      bSuccess =  true;
    }
    return bSuccess;
  }

  static inline bool is_sphere_data(uint32_t packet_type) {
    SeyondItemType inno_type = static_cast<SeyondItemType>(packet_type);
    bool bSuccess = false;
    if(inno_type == SEYOND_ITEM_TYPE_SPHERE_POINTCLOUD || inno_type == SEYOND_ROBINE_ITEM_TYPE_SPHERE_POINTCLOUD
        ||inno_type == SEYOND_ROBINW_ITEM_TYPE_SPHERE_POINTCLOUD||inno_type== SEYOND_FALCONII_DOT_1_ITEM_TYPE_SPHERE_POINTCLOUD) {
      bSuccess =  true;
    }
    return bSuccess;
  }

  static inline bool is_en_xyz_data(uint32_t packet_type) {
    SeyondItemType inno_type = static_cast<SeyondItemType>(packet_type);
    bool bSuccess = false;
    if(inno_type == SEYOND_ROBINE_ITEM_TYPE_XYZ_POINTCLOUD
        ||inno_type == SEYOND_ROBINW_ITEM_TYPE_XYZ_POINTCLOUD||inno_type== SEYOND_FALCONII_DOT_1_ITEM_TYPE_XYZ_POINTCLOUD) {
      bSuccess =  true;
    }
    return bSuccess;
  }

  static inline bool is_en_sphere_data(uint32_t packet_type) {
    SeyondItemType inno_type = static_cast<SeyondItemType>(packet_type);
    bool bSuccess = false;
    if(inno_type == SEYOND_ROBINE_ITEM_TYPE_SPHERE_POINTCLOUD || inno_type == SEYOND_ROBINW_ITEM_TYPE_SPHERE_POINTCLOUD
        ||inno_type == SEYOND_FALCONII_DOT_1_ITEM_TYPE_SPHERE_POINTCLOUD) {
      bSuccess =  true;
    }
    return bSuccess;
  }

  inline static void lookup_xz_adjustment_(const SeyondBlockAngles &angles, uint32_t ch, double *x,
                                                        double *z) {
    uint32_t v = angles.v_angle / 512;
    uint32_t h = angles.h_angle / 512;
    v += kVTableEffeHalfSize_;
    h += kHTableEffeHalfSize_;
    // avoid index out-of-bound
    v = v & (kVTableSize_ - 1);
    h = h & (kHTableSize_ - 1);
    int8_t *addr_x = &nps_adjustment_[v][h][ch][0];
    int8_t *addr_z = addr_x + 1;
    *x = *addr_x * kAdjustmentUnitInMeter_;
    *z = *addr_z * kAdjustmentUnitInMeter_;
    return;
  }

  inline static void lookup_xyz_adjustment_(const SeyondBlockAngles &angles, uint32_t scan_id, double adj[]) {
    int h_angle = angles.h_angle + (kHRobinTableEffeHalfSize_ << kTableShift_);
    int h_index = h_angle >> kTableShift_;
    // avoid index out-of-bound
    h_index = h_index & (kHTableSize_ - 1);
    if (h_index > kHTableSize_ - 2) {
      h_index = kHTableSize_ - 2;
    }

    int h_offset = h_angle & kTableMask_;
    int h_offset2 = kTableStep_ - h_offset;

    for (int i = 0; i < 3; i++) {
      int u = robin_nps_adjustment_[scan_id][h_index][i];
      int v = robin_nps_adjustment_[scan_id][h_index + 1][i];
      int t = u * h_offset2 + v * h_offset + kTableHalfStep_;
      int w = t >> kTableShift_;
      adj[i] = w * kAdjustmentUnitInMeterRobin_;
    }

    return;
  }

  static inline size_t get_data_packet_size(SeyondItemType type, uint32_t item_count, SeyondMultipleReturnMode mode) {
    size_t unit_size = 0;
    switch (type) {
    case SEYOND_ITEM_TYPE_SPHERE_POINTCLOUD:
      if (mode == SEYOND_MULTIPLE_RETURN_MODE_SINGLE) {
        unit_size = sizeof(SeyondBlock1);
      } else if (mode == SEYOND_MULTIPLE_RETURN_MODE_2_STRONGEST ||
                 mode == SEYOND_MULTIPLE_RETURN_MODE_2_STRONGEST_FURTHEST) {
        unit_size = sizeof(SeyondBlock2);
      } else {
        return 0;
      }
      break;
    case SEYOND_ITEM_TYPE_XYZ_POINTCLOUD:
      unit_size = sizeof(SeyondXyzPoint);
      break;
    case SEYOND_ROBINE_ITEM_TYPE_SPHERE_POINTCLOUD:
    case SEYOND_ROBINW_ITEM_TYPE_SPHERE_POINTCLOUD:
    case SEYOND_FALCONII_DOT_1_ITEM_TYPE_SPHERE_POINTCLOUD:
      if (mode == SEYOND_MULTIPLE_RETURN_MODE_SINGLE) {
        unit_size = sizeof(SeyondEnBlock1);
      } else if (mode == SEYOND_MULTIPLE_RETURN_MODE_2_STRONGEST ||
                 mode == SEYOND_MULTIPLE_RETURN_MODE_2_STRONGEST_FURTHEST) {
        unit_size = sizeof(SeyondEnBlock2);
      } else {
        return 0;
      }
      break;
    case SEYOND_ROBINE_ITEM_TYPE_XYZ_POINTCLOUD:
    case SEYOND_ROBINW_ITEM_TYPE_XYZ_POINTCLOUD:
    case SEYOND_FALCONII_DOT_1_ITEM_TYPE_XYZ_POINTCLOUD:
      unit_size = sizeof(SeyondEnXyzPoint);
      break;
    default:
      break;
    }
    return sizeof(SeyondDataPacket) + item_count * unit_size;
  }

  static void init_f(void);

  static int init_f_robin(void);

  static int init_f_falcon(void);


  template<typename BlockHeader>
  static inline void get_block_full_angles(SeyondBlockFullAngles *full, const BlockHeader &b, SeyondItemType type) {
    full->angles[0].h_angle = b.h_angle;
    full->angles[0].v_angle = b.v_angle;
    full->angles[1].h_angle = b.h_angle + b.h_angle_diff_1;
    full->angles[1].v_angle = b.v_angle + b.v_angle_diff_1 + v_angle_offset_[type][1];
    full->angles[2].h_angle = b.h_angle + b.h_angle_diff_2;
    full->angles[2].v_angle = b.v_angle + b.v_angle_diff_2 + v_angle_offset_[type][2];
    full->angles[3].h_angle = b.h_angle + b.h_angle_diff_3;
    full->angles[3].v_angle = b.v_angle + b.v_angle_diff_3 + v_angle_offset_[type][3];
  }

  static bool check_data_packet(const SeyondDataPacket &pkt, size_t size);

  static void get_xyzr_meter(const SeyondBlockAngles angles, const uint32_t radius_unit, const uint32_t channel,
                             SeyondXyzrD *result, SeyondItemType type = SEYOND_ITEM_TYPE_SPHERE_POINTCLOUD);

  static inline void get_xyz_point(const SeyondBlockHeader &block, const SeyondChannelPoint &cp,
                                   const SeyondBlockAngles angles, const uint32_t channel, SeyondXyzPoint *pt) {
    SeyondXyzrD xyzr;
    get_xyzr_meter(angles, cp.radius, channel, &xyzr);
    pt->x = xyzr.x;
    pt->y = xyzr.y;
    pt->z = xyzr.z;
    pt->radius = xyzr.radius;
    pt->ts_10us = block.ts_10us;
    pt->scan_idx = block.scan_idx;
    pt->scan_id = block.scan_id;
    pt->in_roi = block.in_roi;
    pt->facet = block.facet;
    pt->reserved_flags = block.reserved_flags;
    pt->refl = cp.refl;
    pt->type = cp.type;
    pt->elongation = cp.elongation;
    pt->channel = channel;
    pt->is_2nd_return = cp.is_2nd_return;
  }

  static inline void get_xyz_point(const SeyondEnBlockHeader &block, const SeyondEnChannelPoint &cp,
                                   const SeyondBlockAngles angles, const uint32_t channel, SeyondEnXyzPoint *pt,
                                   SeyondItemType type) {
    SeyondXyzrD xyzr;
    uint32_t scan_id = 0;
    if (type == SEYOND_FALCONII_DOT_1_ITEM_TYPE_SPHERE_POINTCLOUD) {
      scan_id = block.scan_id;
      get_xyzr_meter(angles, cp.radius, channel, &xyzr, type);
    } else if (type == SEYOND_ROBINE_ITEM_TYPE_SPHERE_POINTCLOUD || type == SEYOND_ROBINW_ITEM_TYPE_SPHERE_POINTCLOUD) {
      const uint8_t *channel_mapping;
      int tdc_channel_number;
      if (type == SEYOND_ROBINE_ITEM_TYPE_SPHERE_POINTCLOUD) {
        channel_mapping = &robine_channel_mapping[0];
        tdc_channel_number = RobinETDCChannelNumber;
      } else {
        channel_mapping = &robinw_channel_mapping[0];
        tdc_channel_number = RobinWTDCChannelNumber;
      }
      int index = block.scan_id * 4 + channel;
      scan_id = channel_mapping[index] + block.facet * tdc_channel_number;
      get_xyzr_meter(angles, cp.radius, scan_id, &xyzr, type);
    }
    pt->x = xyzr.x;
    pt->y = xyzr.y;
    pt->z = xyzr.z;
    pt->radius = xyzr.radius;
    pt->ts_10us = block.ts_10us;
    pt->scan_idx = block.scan_idx;
    pt->scan_id = scan_id;
    pt->in_roi = block.in_roi;
    pt->facet = block.facet;
    pt->reflectance = cp.reflectance;
    pt->intensity = cp.intensity;
    pt->type = cp.type;
    pt->elongation = cp.elongation;
    pt->channel = channel;
    pt->is_2nd_return = cp.is_2nd_return;
    pt->firing = cp.firing;
  }

  static inline void get_block_size_and_number_return(const SeyondDataPacket &pkt, uint32_t *block_size_in_byte,
                                                      uint32_t *number_return) {
    if (is_en_sphere_data(pkt.type)) {
      if (pkt.multi_return_mode == SEYOND_MULTIPLE_RETURN_MODE_2_STRONGEST ||
          pkt.multi_return_mode == SEYOND_MULTIPLE_RETURN_MODE_2_STRONGEST_FURTHEST) {
        *block_size_in_byte = sizeof(SeyondEnBlock2);
        *number_return = 2;
      } else if (pkt.multi_return_mode == SEYOND_MULTIPLE_RETURN_MODE_SINGLE) {
        *block_size_in_byte = sizeof(SeyondEnBlock1);
        *number_return = 1;
      } else {
        std::cout << "invalid return mode " << pkt.multi_return_mode << std::endl;
      }
    } else {
      if (pkt.multi_return_mode == SEYOND_MULTIPLE_RETURN_MODE_2_STRONGEST ||
          pkt.multi_return_mode == SEYOND_MULTIPLE_RETURN_MODE_2_STRONGEST_FURTHEST) {
        *block_size_in_byte = sizeof(SeyondBlock2);
        *number_return = 2;
      } else if (pkt.multi_return_mode == SEYOND_MULTIPLE_RETURN_MODE_SINGLE) {
        *block_size_in_byte = sizeof(SeyondBlock1);
        *number_return = 1;
      } else {
        std::cout << "invalid return mode " << pkt.multi_return_mode << std::endl;
      }
    }
    return;
  }

  static inline int get_return_times(const SeyondMultipleReturnMode mode) {
    return mode >= SEYOND_MULTIPLE_RETURN_MODE_2_STRONGEST ? 2 : 1;
  }

  static inline uint32_t get_points_count(const SeyondDataPacket &pkt) {
    if (is_xyz_data(pkt.type)) {
      return pkt.item_number;
    } else if (pkt.type == SEYOND_ITEM_TYPE_SPHERE_POINTCLOUD) {
      uint32_t item_count = 0;
      auto count_callback = [&](const SeyondDataPacketPointsCallbackParams<SeyondBlock, SeyondChannelPoint> &in_params) {
        if (in_params.pt.radius > 0) {
          item_count++;
        }
      };

      if (SeyondDecoder::iterate_inno_data_packet_cpoints<SeyondBlock, SeyondBlockHeader, SeyondBlock1, SeyondBlock2,
                                                              SeyondChannelPoint>(pkt, count_callback) == 0) {
        std::cerr << "iterate_inno_data_packet_cpoints failed" << std::endl;
      }

      return item_count;
    } else if (is_en_sphere_data(pkt.type)) {
      uint32_t item_count = 0;
      auto count_callback = [&](const SeyondDataPacketPointsCallbackParams<SeyondEnBlock, SeyondEnChannelPoint> &in_params) {
        if (in_params.pt.radius > 0) {
          item_count++;
        }
      };

      (void)SeyondDecoder::iterate_inno_data_packet_cpoints<SeyondEnBlock, SeyondEnBlockHeader, SeyondEnBlock1,
                                                                SeyondEnBlock2, SeyondEnChannelPoint>(pkt, count_callback);
      return item_count;
    } else {
      std::cout << "invalid type " << pkt.type << std::endl;
      return -1;
    }
  }

  template <typename Block, typename BlockHeader, typename Block1, typename Block2, typename Point, typename Callback>
  static uint32_t iterate_inno_data_packet_cpoints(const SeyondDataPacket &in_pkt, Callback in_callback) {
    uint32_t out_count{0};
    uint32_t unit_size{0};
    uint32_t mr = SeyondDecoder::get_return_times(static_cast<SeyondMultipleReturnMode>(in_pkt.multi_return_mode));

    if (mr == 2) {
        unit_size = sizeof(Block2);
    } else if (mr == 1) {
        unit_size = sizeof(Block1);
    } else {
      std::cerr<<"return times of return mode "<<in_pkt.multi_return_mode<<" is "<<mr<<std::endl;
    }
    const Block *block = nullptr;
    uint32_t tmp_idx = 0;
    for (; tmp_idx < in_pkt.item_number; tmp_idx++) {
      if (tmp_idx == 0) {
        const SeyondBlock1 * const block_ptr = &in_pkt.inno_block1s[0];
        std::memcpy(&block, &block_ptr, sizeof(Block*));
      } else {
        int8_t *byte_ptr = nullptr;
        std::memcpy(&byte_ptr, &block, sizeof(const Block *));
        byte_ptr = (int8_t*)(uintptr_t)(byte_ptr + static_cast<uint32_t>(sizeof(int8_t)) * unit_size);
        std::memcpy(&block, &byte_ptr, sizeof(const Block *));
      }
      if (block == nullptr) {
        std::cerr<<"bad block"<<std::endl;
      }
      SeyondBlockFullAngles full_angles;
      SeyondDecoder::get_block_full_angles<BlockHeader>(&full_angles, block->header, static_cast<SeyondItemType>(in_pkt.type));
      uint32_t ch1 = 0;
      for (; ch1 < kSeyondChannelNumber; ch1++) {
        uint32_t m1 = 0;
        for (; m1 < mr; m1++) {
          const Point &pt = block->points[ch1 + (m1 << kSeyondChannelNumberBit)];
          const SeyondDataPacketPointsCallbackParams<Block, Point> in_params{
              in_pkt, *block, pt, full_angles, static_cast<uint16_t>(ch1), static_cast<uint16_t>(m1)};
          in_callback(in_params);
          out_count++;
        }
      }
    }
    return out_count;
  }

private:
  void setup_table_(double inno_angle_unit);
  static bool convert_to_xyz_pointcloud(const SeyondDataPacket &src, SeyondDataPacket *dest, size_t dest_size, bool append);
  void data_packet_parse_(const SeyondDataPacket *pkt);
  template <typename PointType>
  void point_xyz_data_parse_(bool is_en_data, bool is_use_refl, uint32_t point_num, PointType point_ptr);
};
}  // namespace seyond_packet
}  // namespace drivers
}  // namespace nebula