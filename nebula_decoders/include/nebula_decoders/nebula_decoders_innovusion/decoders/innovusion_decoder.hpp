#pragma once

#include "nebula_decoders/nebula_decoders_innovusion/decoders/innovusion_data_packet.hpp"
#include "nebula_decoders/nebula_decoders_innovusion/decoders/innovusion_scan_decoder.hpp"
#include "nebula_decoders/nebula_decoders_innovusion/decoders/nps_adjustment.hpp"
#include "nebula_decoders/nebula_decoders_innovusion/decoders/robin_nps_adjustment.hpp"

#include <rclcpp/rclcpp.hpp>

#include "innovusion_msgs/msg/innovusion_packet.hpp"
#include "innovusion_msgs/msg/innovusion_scan.hpp"
namespace nebula
{
namespace drivers
{
namespace innovusion_packet
{
class InnoBlockAngles {
 public:
  int16_t h_angle;
  int16_t v_angle;
};

class InnoBlockFullAngles {
 public:
  InnoBlockAngles angles[kInnoChannelNumber];
};

template<class Block, class Point>
class InnoDataPacketPointsCallbackParams {
  public:
   InnoDataPacket pkt;
   Block block;
   Point pt;
   InnoBlockFullAngles angle;
   uint16_t channel;
   uint16_t multi_return;
};

class InnovusionDecoder : public InnovusionScanDecoder
{
protected:
  /// @brief Configuration for this decoder
  const std::shared_ptr<drivers::InnovusionSensorConfiguration> sensor_configuration_;
  const std::shared_ptr<drivers::InnovusionCalibrationConfiguration> calibration_configuration_;

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
  static int v_angle_offset_[INNO_ITEM_TYPE_MAX][kInnoChannelNumber];
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
  static const int32_t kAngleTableSize = 2 * kInnoAngleUnitPerPiRad;
  static const int32_t kASinTableSize = 10000;
  static const int32_t kATanTableSize = 45000;
  static const int32_t kAtanTableScale = 10000;
  static const int32_t kAsinTableScale = 10000;

  static float sin_table_[];
  static float cos_table_[];
  static int32_t asin_table_[];
  static int32_t atan_table_[];
  static int8_t robin_nps_adjustment_[kRobinScanlines_][kHTableSize_][kXYZSize_];
  static int8_t nps_adjustment_[kVTableSize_][kHTableSize_][kInnoChannelNumber][kXZSize_];
  std::uint64_t frame_point_number_ = 0;

public:
  /// @brief Constructor
  /// @param sensor_configuration SensorConfiguration for this decoder
  /// @param calibration_configuration Calibration for this decoder
  explicit InnovusionDecoder(
    const std::shared_ptr<InnovusionSensorConfiguration> & sensor_configuration,
    const std::shared_ptr<InnovusionCalibrationConfiguration> & calibration_configuration);

  int unpack(const innovusion_msgs::msg::InnovusionPacket & packet) override;

  std::tuple<drivers::NebulaPointCloudPtr, double> getPointcloud() override;

  static double lookup_cos_table_in_unit(int i);

  static double lookup_sin_table_in_unit(int i);

  static inline bool is_xyz_data(uint32_t packet_type) {
    InnoItemType inno_type = static_cast<InnoItemType>(packet_type);
    bool bSuccess = false;
    if(inno_type == INNO_ITEM_TYPE_XYZ_POINTCLOUD || inno_type == INNO_ROBINE_ITEM_TYPE_XYZ_POINTCLOUD
        ||inno_type == INNO_ROBINW_ITEM_TYPE_XYZ_POINTCLOUD||inno_type== INNO_FALCONII_DOT_1_ITEM_TYPE_XYZ_POINTCLOUD) {
      bSuccess =  true;
    }
    return bSuccess;
  }

  static inline bool is_sphere_data(uint32_t packet_type) {
    InnoItemType inno_type = static_cast<InnoItemType>(packet_type);
    bool bSuccess = false;
    if(inno_type == INNO_ITEM_TYPE_SPHERE_POINTCLOUD || inno_type == INNO_ROBINE_ITEM_TYPE_SPHERE_POINTCLOUD
        ||inno_type == INNO_ROBINW_ITEM_TYPE_SPHERE_POINTCLOUD||inno_type== INNO_FALCONII_DOT_1_ITEM_TYPE_SPHERE_POINTCLOUD) {
      bSuccess =  true;
    }
    return bSuccess;
  }

  static inline bool is_en_xyz_data(uint32_t packet_type) {
    InnoItemType inno_type = static_cast<InnoItemType>(packet_type);
    bool bSuccess = false;
    if(inno_type == INNO_ROBINE_ITEM_TYPE_XYZ_POINTCLOUD
        ||inno_type == INNO_ROBINW_ITEM_TYPE_XYZ_POINTCLOUD||inno_type== INNO_FALCONII_DOT_1_ITEM_TYPE_XYZ_POINTCLOUD) {
      bSuccess =  true;
    }
    return bSuccess;
  }

  static inline bool is_en_sphere_data(uint32_t packet_type) {
    InnoItemType inno_type = static_cast<InnoItemType>(packet_type);
    bool bSuccess = false;
    if(inno_type == INNO_ROBINE_ITEM_TYPE_SPHERE_POINTCLOUD || inno_type == INNO_ROBINW_ITEM_TYPE_SPHERE_POINTCLOUD
        ||inno_type == INNO_FALCONII_DOT_1_ITEM_TYPE_SPHERE_POINTCLOUD) {
      bSuccess =  true;
    }
    return bSuccess;
  }

  inline static void lookup_xz_adjustment_(const InnoBlockAngles &angles, uint32_t ch, double *x,
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

  inline static void lookup_xyz_adjustment_(const InnoBlockAngles &angles, uint32_t scan_id, double adj[]) {
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

  static inline size_t get_data_packet_size(InnoItemType type, uint32_t item_count, InnoMultipleReturnMode mode) {
    size_t unit_size = 0;
    switch (type) {
    case INNO_ITEM_TYPE_SPHERE_POINTCLOUD:
      if (mode == INNO_MULTIPLE_RETURN_MODE_SINGLE) {
        unit_size = sizeof(InnoBlock1);
      } else if (mode == INNO_MULTIPLE_RETURN_MODE_2_STRONGEST ||
                 mode == INNO_MULTIPLE_RETURN_MODE_2_STRONGEST_FURTHEST) {
        unit_size = sizeof(InnoBlock2);
      } else {
        return 0;
      }
      break;
    case INNO_ITEM_TYPE_XYZ_POINTCLOUD:
      unit_size = sizeof(InnoXyzPoint);
      break;
    case INNO_ROBINE_ITEM_TYPE_SPHERE_POINTCLOUD:
    case INNO_ROBINW_ITEM_TYPE_SPHERE_POINTCLOUD:
    case INNO_FALCONII_DOT_1_ITEM_TYPE_SPHERE_POINTCLOUD:
      if (mode == INNO_MULTIPLE_RETURN_MODE_SINGLE) {
        unit_size = sizeof(InnoEnBlock1);
      } else if (mode == INNO_MULTIPLE_RETURN_MODE_2_STRONGEST ||
                 mode == INNO_MULTIPLE_RETURN_MODE_2_STRONGEST_FURTHEST) {
        unit_size = sizeof(InnoEnBlock2);
      } else {
        return 0;
      }
      break;
    case INNO_ROBINE_ITEM_TYPE_XYZ_POINTCLOUD:
    case INNO_ROBINW_ITEM_TYPE_XYZ_POINTCLOUD:
    case INNO_FALCONII_DOT_1_ITEM_TYPE_XYZ_POINTCLOUD:
      unit_size = sizeof(InnoEnXyzPoint);
      break;
    default:
      break;
    }
    return sizeof(InnoDataPacket) + item_count * unit_size;
  }

  static void init_f(void);

  static int init_f_robin(void);

  static int init_f_falcon(void);


  template<typename BlockHeader>
  static inline void get_block_full_angles(InnoBlockFullAngles *full, const BlockHeader &b, InnoItemType type) {
    full->angles[0].h_angle = b.h_angle;
    full->angles[0].v_angle = b.v_angle;
    full->angles[1].h_angle = b.h_angle + b.h_angle_diff_1;
    full->angles[1].v_angle = b.v_angle + b.v_angle_diff_1 + v_angle_offset_[type][1];
    full->angles[2].h_angle = b.h_angle + b.h_angle_diff_2;
    full->angles[2].v_angle = b.v_angle + b.v_angle_diff_2 + v_angle_offset_[type][2];
    full->angles[3].h_angle = b.h_angle + b.h_angle_diff_3;
    full->angles[3].v_angle = b.v_angle + b.v_angle_diff_3 + v_angle_offset_[type][3];
  }

  static bool check_data_packet(const InnoDataPacket &pkt, size_t size);

  static void get_xyzr_meter(const InnoBlockAngles angles, const uint32_t radius_unit, const uint32_t channel,
                             InnoXyzrD *result, InnoItemType type = INNO_ITEM_TYPE_SPHERE_POINTCLOUD);

  static inline void get_xyz_point(const InnoBlockHeader &block, const InnoChannelPoint &cp,
                                   const InnoBlockAngles angles, const uint32_t channel, InnoXyzPoint *pt) {
    InnoXyzrD xyzr;
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

  static inline void get_xyz_point(const InnoEnBlockHeader &block, const InnoEnChannelPoint &cp,
                                   const InnoBlockAngles angles, const uint32_t channel, InnoEnXyzPoint *pt,
                                   InnoItemType type) {
    InnoXyzrD xyzr;
    uint32_t scan_id = 0;
    if (type == INNO_FALCONII_DOT_1_ITEM_TYPE_SPHERE_POINTCLOUD) {
      scan_id = block.scan_id;
      get_xyzr_meter(angles, cp.radius, channel, &xyzr, type);
    } else if (type == INNO_ROBINE_ITEM_TYPE_SPHERE_POINTCLOUD || type == INNO_ROBINW_ITEM_TYPE_SPHERE_POINTCLOUD) {
      const uint8_t *channel_mapping;
      int tdc_channel_number;
      if (type == INNO_ROBINE_ITEM_TYPE_SPHERE_POINTCLOUD) {
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

  static inline void get_block_size_and_number_return(const InnoDataPacket &pkt, uint32_t *block_size_in_byte,
                                                      uint32_t *number_return) {
    if (is_en_sphere_data(pkt.type)) {
      if (pkt.multi_return_mode == INNO_MULTIPLE_RETURN_MODE_2_STRONGEST ||
          pkt.multi_return_mode == INNO_MULTIPLE_RETURN_MODE_2_STRONGEST_FURTHEST) {
        *block_size_in_byte = sizeof(InnoEnBlock2);
        *number_return = 2;
      } else if (pkt.multi_return_mode == INNO_MULTIPLE_RETURN_MODE_SINGLE) {
        *block_size_in_byte = sizeof(InnoEnBlock1);
        *number_return = 1;
      } else {
        std::cout << "invalid return mode " << pkt.multi_return_mode << std::endl;
      }
    } else {
      if (pkt.multi_return_mode == INNO_MULTIPLE_RETURN_MODE_2_STRONGEST ||
          pkt.multi_return_mode == INNO_MULTIPLE_RETURN_MODE_2_STRONGEST_FURTHEST) {
        *block_size_in_byte = sizeof(InnoBlock2);
        *number_return = 2;
      } else if (pkt.multi_return_mode == INNO_MULTIPLE_RETURN_MODE_SINGLE) {
        *block_size_in_byte = sizeof(InnoBlock1);
        *number_return = 1;
      } else {
        std::cout << "invalid return mode " << pkt.multi_return_mode << std::endl;
      }
    }
    return;
  }

  static inline int get_return_times(const InnoMultipleReturnMode mode) {
    return mode >= INNO_MULTIPLE_RETURN_MODE_2_STRONGEST ? 2 : 1;
  }

  static inline uint32_t get_points_count(const InnoDataPacket &pkt) {
    if (is_xyz_data(pkt.type)) {
      return pkt.item_number;
    } else if (pkt.type == INNO_ITEM_TYPE_SPHERE_POINTCLOUD) {
      uint32_t item_count = 0;
      auto count_callback = [&](const InnoDataPacketPointsCallbackParams<InnoBlock, InnoChannelPoint> &in_params) {
        if (in_params.pt.radius > 0) {
          item_count++;
        }
      };

      if (InnovusionDecoder::iterate_inno_data_packet_cpoints<InnoBlock, InnoBlockHeader, InnoBlock1, InnoBlock2,
                                                              InnoChannelPoint>(pkt, count_callback) == 0) {
        std::cerr << "iterate_inno_data_packet_cpoints failed" << std::endl;
      }

      return item_count;
    } else if (is_en_sphere_data(pkt.type)) {
      uint32_t item_count = 0;
      auto count_callback = [&](const InnoDataPacketPointsCallbackParams<InnoEnBlock, InnoEnChannelPoint> &in_params) {
        if (in_params.pt.radius > 0) {
          item_count++;
        }
      };

      (void)InnovusionDecoder::iterate_inno_data_packet_cpoints<InnoEnBlock, InnoEnBlockHeader, InnoEnBlock1,
                                                                InnoEnBlock2, InnoEnChannelPoint>(pkt, count_callback);
      return item_count;
    } else {
      std::cout << "invalid type " << pkt.type << std::endl;
    }
  }

  template <typename Block, typename BlockHeader, typename Block1, typename Block2, typename Point, typename Callback>
  static uint32_t iterate_inno_data_packet_cpoints(const InnoDataPacket &in_pkt, Callback in_callback) {
    uint32_t out_count{0};
    uint32_t unit_size{0};
    uint32_t mr = InnovusionDecoder::get_return_times(static_cast<InnoMultipleReturnMode>(in_pkt.multi_return_mode));

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
        const InnoBlock1 * const block_ptr = &in_pkt.inno_block1s[0];
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
      InnoBlockFullAngles full_angles;
      InnovusionDecoder::get_block_full_angles<BlockHeader>(&full_angles, block->header, static_cast<InnoItemType>(in_pkt.type));
      uint32_t ch1 = 0;
      for (; ch1 < kInnoChannelNumber; ch1++) {
        uint32_t m1 = 0;
        for (; m1 < mr; m1++) {
          const Point &pt = block->points[ch1 + (m1 << kInnoChannelNumberBit)];
          const InnoDataPacketPointsCallbackParams<Block, Point> in_params{
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
  static bool convert_to_xyz_pointcloud(const InnoDataPacket &src, InnoDataPacket *dest, size_t dest_size, bool append);
  void data_packet_parse_(const InnoDataPacket *pkt);
  template <typename PointType>
  void point_xyz_data_parse_(bool is_en_data, bool is_use_refl, uint32_t point_num, PointType point_ptr);
};
}  // namespace innovusion_packet
}  // namespace drivers
}  // namespace nebula
