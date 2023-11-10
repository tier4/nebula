#include "nebula_decoders/nebula_decoders_innovusion/decoders/innovusion_decoder.hpp"

namespace nebula
{
namespace drivers
{
namespace innovusion_packet
{

float InnovusionDecoder::sin_table_[kAngleTableSize + 1];
float InnovusionDecoder::cos_table_[kAngleTableSize + 1];
int32_t InnovusionDecoder::asin_table_[2 * kASinTableSize];
int32_t InnovusionDecoder::atan_table_[2 * kATanTableSize];

// for robinW
const uint8_t InnovusionDecoder::robinw_channel_mapping[48] = {
  0, 4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 44,
  1, 5, 9, 13, 17, 21, 25, 29, 33, 37, 41, 45,
  2, 6, 10, 14, 18, 22, 26, 30, 34, 38, 42, 46,
  3, 7, 11, 15, 19, 23, 27, 31, 35, 39, 43, 47,
};

const uint8_t InnovusionDecoder::robine_channel_mapping[48] = {
  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11,
  12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23,
  24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35,
  36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47
};

int InnovusionDecoder::v_angle_offset_[INNO_ITEM_TYPE_MAX][kInnoChannelNumber];
int8_t InnovusionDecoder::nps_adjustment_[kVTableSize_][kHTableSize_][kInnoChannelNumber][kXZSize_];  // NOLINT
const double InnovusionDecoder::kAdjustmentUnitInMeter_ = 0.0025;
const double InnovusionDecoder::kAdjustmentUnitInMeterRobin_ = 0.001;
int8_t InnovusionDecoder::robin_nps_adjustment_[kRobinScanlines_][kHTableSize_][kXYZSize_];  // NOLINT

InnovusionDecoder::InnovusionDecoder(
    const std::shared_ptr<InnovusionSensorConfiguration> & sensor_configuration,
    const std::shared_ptr<InnovusionCalibrationConfiguration> & calibration_configuration)
  : sensor_configuration_(sensor_configuration),
    calibration_configuration_(calibration_configuration),
    logger_(rclcpp::get_logger("InnovusionDecoder"))
{
  logger_.set_level(rclcpp::Logger::Level::Debug);
  RCLCPP_INFO_STREAM(logger_, sensor_configuration_);

  decode_pc_.reset(new NebulaPointCloud);
  output_pc_.reset(new NebulaPointCloud);

  xyz_from_sphere_.resize(kConvertSize);
  setup_table_(kRadPerInnoAngleUnit);
  init_f();
}

template <typename PointType>
void InnovusionDecoder::point_xyz_data_parse_(bool is_en_data, bool is_use_refl, uint32_t point_num, PointType point_ptr) {
  for (uint32_t i = 0; i < point_num; ++i, ++point_ptr) {
    drivers::NebulaPoint point;
    if (point_ptr->channel >= kInnoChannelNumber) {
      RCLCPP_ERROR_STREAM(logger_, "bad channel " << point_ptr->channel);
      continue;
    }

    if ((point_ptr->radius < sensor_configuration_->cloud_min_range) ||
        (point_ptr->radius > sensor_configuration_->cloud_max_range)) {
      continue;
    }

    if constexpr (std::is_same<PointType, const InnoEnXyzPoint *>::value) {
      if (is_use_refl) {
        point.intensity = point_ptr->reflectance;
      } else {
        point.intensity = point_ptr->intensity;
      }
    } else if constexpr (std::is_same<PointType, const InnoXyzPoint *>::value) {
      point.intensity = point_ptr->refl;
    }
  
    point.time_stamp = point_ptr->ts_10us / ten_us_in_second_c + current_ts_start_;
    point.distance = point_ptr->radius;
    point.x = point_ptr->x;
    point.y = point_ptr->y;
    point.z = point_ptr->z;
    decode_pc_->points.emplace_back(point);
  }
}

void InnovusionDecoder::data_packet_parse_(const InnoDataPacket *pkt) {
  current_ts_start_ = pkt->common.ts_start_us / us_in_second_c;
  // adapt different data structures form different lidar
  if (is_en_xyz_data(pkt->type)) {
    const InnoEnXyzPoint *pt =
      reinterpret_cast<const InnoEnXyzPoint *>(reinterpret_cast<const char *>(pkt) + sizeof(InnoDataPacket));
    point_xyz_data_parse_<const InnoEnXyzPoint *>(true, pkt->use_reflectance, pkt->item_number, pt);
  } else {
    const InnoXyzPoint *pt =
      reinterpret_cast<const InnoXyzPoint *>(reinterpret_cast<const char *>(pkt) + sizeof(InnoDataPacket));
    point_xyz_data_parse_<const InnoXyzPoint *>(false, pkt->use_reflectance, pkt->item_number, pt);
  }
  output_scan_timestamp_ns_ = pkt->common.ts_start_us * 1000;
}

int InnovusionDecoder::unpack(const innovusion_msgs::msg::InnovusionPacket & packet)
{
  const InnoDataPacket *inno_pkt =
    reinterpret_cast<const InnoDataPacket *>(reinterpret_cast<const char *>(&packet.data[0]));
  if (is_sphere_data(inno_pkt->type)) {
    // convert sphere to xyz
    bool ret_val =
      convert_to_xyz_pointcloud(*inno_pkt, reinterpret_cast<InnoDataPacket *>(&xyz_from_sphere_[0]), kConvertSize, false);
    if (!ret_val) {
      RCLCPP_ERROR_STREAM(logger_, "convert_to_xyz_pointcloud failed");
      return -1;
    }
    data_packet_parse_(reinterpret_cast<InnoDataPacket *>(&xyz_from_sphere_[0]));
  } else if (is_xyz_data(inno_pkt->type)) {
    data_packet_parse_(inno_pkt);
  } else {
    RCLCPP_ERROR_STREAM(logger_, "cframe type" <<  inno_pkt->type << "is not supported");
  }

  return 0;
}

std::tuple<drivers::NebulaPointCloudPtr, double> InnovusionDecoder::getPointcloud() {
  double scan_timestamp_s = static_cast<double>(output_scan_timestamp_ns_) * 1e-9;
  std::swap(decode_pc_, output_pc_);
  decode_pc_->clear();
  return std::make_pair(output_pc_, scan_timestamp_s);
}

double InnovusionDecoder::lookup_cos_table_in_unit(int i) {
  return cos_table_[i];
}

double InnovusionDecoder::lookup_sin_table_in_unit(int i) {
  return sin_table_[i];
}

void InnovusionDecoder::setup_table_(double inno_angle_unit) {
  for (int32_t i = 0; i <= kAngleTableSize; ++i) {
    double angle = i * kRadPerInnoAngleUnit;
    cos_table_[i] = cos(angle);
    sin_table_[i] = sin(angle);
  }

  for (int32_t i = -kASinTableSize; i < kASinTableSize; i++) {
    asin_table_[i + kASinTableSize] =
        static_cast<int32_t>(asin(i/static_cast<double>(kAsinTableScale))
                             / inno_angle_unit);
  }

  for (int32_t i = -kATanTableSize; i < kATanTableSize; i++) {
    atan_table_[i + kATanTableSize] =
        static_cast<int32_t>(atan(i/static_cast<double>(kAtanTableScale))
                             / inno_angle_unit);
  }
}

void InnovusionDecoder::init_f(void) {
  init_f_falcon();
  init_f_robin();
}

int InnovusionDecoder::init_f_falcon(void) {
  for (uint32_t ich = 0; ich < kInnoChannelNumber; ich++) {
    v_angle_offset_[INNO_ITEM_TYPE_SPHERE_POINTCLOUD][ich] = ich * kInnoFaconVAngleDiffBase;
    // falconII NT3
    v_angle_offset_[INNO_FALCONII_DOT_1_ITEM_TYPE_SPHERE_POINTCLOUD][ich] = ich * kInnoFaconVAngleDiffBase;
  }
  // init the nps_adjustment_
  size_t input_size = (kVTableEffeHalfSize_ * 2 + 1) * (kHTableEffeHalfSize_ * 2 + 1) * 2 * kInnoChannelNumber;
  memset(nps_adjustment_, 0, sizeof(nps_adjustment_));
  static double k_max[2] = {-100, -100};
  static double k_min[2] = {100, 100};
  for (uint32_t v = 0; v < kVTableEffeHalfSize_ * 2 + 1; v++) {
    for (uint32_t h = 0; h < kHTableEffeHalfSize_ * 2 + 1; h++) {
      for (uint32_t ich = 0; ich < kInnoChannelNumber; ich++) {
        for (uint32_t xz = 0; xz < kXZSize_; xz++) {
          double k = kInnoPs2Nps[xz][ich][v][h];
          double u = k / kAdjustmentUnitInMeter_;
          double q = std::floor(u + 0.5);
          nps_adjustment_[v][h][ich][xz] = q;
          k_max[xz] = std::max(k_max[xz], k);
          k_min[xz] = std::min(k_min[xz], k);
        }
      }
    }
  }
  return 0;
}

int InnovusionDecoder::init_f_robin(void) {
  for (uint32_t ich = 0; ich < kInnoChannelNumber; ich++) {
    v_angle_offset_[INNO_ROBINE_ITEM_TYPE_SPHERE_POINTCLOUD][ich] = ich * kInnoRobinEVAngleDiffBase;
    v_angle_offset_[INNO_ROBINW_ITEM_TYPE_SPHERE_POINTCLOUD][ich] = ich * kInnoRobinWVAngleDiffBase;
  }

  // init the nps_adjustment_
  size_t input_size = kRobinScanlines_ * (kHRobinTableEffeHalfSize_ * 2 + 1) * kXYZSize_;
  memset(robin_nps_adjustment_, 0, sizeof(robin_nps_adjustment_));
  static double k_max[3] = {-200, -200, -200};
  static double k_min[3] = {200, 200, 200};
  for (uint32_t scan_id = 0; scan_id < kRobinScanlines_; scan_id++) {
    for (uint32_t h = 0; h < kHRobinTableEffeHalfSize_ * 2 + 1; h++) {
      for (uint32_t xyz = 0; xyz < kXYZSize_; xyz++) {
        double k = robinW_kInnoPs2Nps[xyz][scan_id][h];
        double u = k * 0.001 / kAdjustmentUnitInMeterRobin_;
        double q = std::floor(u + 0.5);
        robin_nps_adjustment_[scan_id][h][xyz] = q;
        k_max[xyz] = std::max(k_max[xyz], k);
        k_min[xyz] = std::min(k_min[xyz], k);
      }
    }
  }
  return 0;
}

void InnovusionDecoder::get_xyzr_meter(const InnoBlockAngles angles, const uint32_t radius_unit,
                                         const uint32_t channel, InnoXyzrD *result, InnoItemType type) {
  if (type == INNO_ITEM_TYPE_SPHERE_POINTCLOUD) {
    result->radius = radius_unit * kMeterPerInnoDistanceUnit200;
  } else {
    result->radius = radius_unit * kMeterPerInnoDistanceUnit400;
  }
  double t;
  if (angles.v_angle >= 0) {
    t = result->radius * lookup_cos_table_in_unit(angles.v_angle);
    result->x = result->radius * lookup_sin_table_in_unit(angles.v_angle);
  } else {
    t = result->radius * lookup_cos_table_in_unit(-angles.v_angle);
    result->x = -result->radius * lookup_sin_table_in_unit(-angles.v_angle);
  }
  if (angles.h_angle >= 0) {
    result->y = t * lookup_sin_table_in_unit(angles.h_angle);
    result->z = t * lookup_cos_table_in_unit(angles.h_angle);
  } else {
    result->y = -t * lookup_sin_table_in_unit(-angles.h_angle);
    result->z = t * lookup_cos_table_in_unit(-angles.h_angle);
  }

  // don't do nps for robinE, no data yet
  if (type == INNO_ROBINE_ITEM_TYPE_SPHERE_POINTCLOUD) {
    return;
  }

  // falconI & falconII
  if (type == INNO_ITEM_TYPE_SPHERE_POINTCLOUD || type == INNO_FALCONII_DOT_1_ITEM_TYPE_SPHERE_POINTCLOUD) {
    double x_adj, z_adj;
    lookup_xz_adjustment_(angles, channel, &x_adj, &z_adj);
    result->x += x_adj;
    result->z += z_adj;
  } else if (type == INNO_ROBINW_ITEM_TYPE_SPHERE_POINTCLOUD) {
    double adj[3];
    lookup_xyz_adjustment_(angles, channel, adj);
    result->x += adj[0];
    result->y += adj[1];
    result->z += adj[2];
  }

  return;
}

bool InnovusionDecoder::check_data_packet(const InnoDataPacket &pkt, size_t size) {
  if (pkt.common.version.magic_number != kInnoMagicNumberDataPacket) {
    std::cout << "bad magic " << pkt.common.version.magic_number << std::endl;
    return false;
  }
  if (size && (pkt.common.size > size)) {
    std::cout << "bad size " << size << " " << pkt.common.size << std::endl;
    return false;
  }
  bool is_data = true;
  switch (pkt.type) {
  case INNO_ITEM_TYPE_SPHERE_POINTCLOUD:
  if (pkt.multi_return_mode == INNO_MULTIPLE_RETURN_MODE_SINGLE) {
    if (pkt.item_size != sizeof(InnoBlock1)) {
      std::cout << "bad block1 item size " << pkt.item_size << " " << sizeof(InnoBlock1) << std::endl;
      return false;
    }
  } else if (pkt.multi_return_mode == INNO_MULTIPLE_RETURN_MODE_2_STRONGEST ||
              pkt.multi_return_mode == INNO_MULTIPLE_RETURN_MODE_2_STRONGEST_FURTHEST) {
    if (pkt.item_size != sizeof(InnoBlock2)) {
      std::cout << "bad block2 item size " << pkt.item_size << " " << sizeof(InnoBlock2) << std::endl;
      return false;
    }
  } else {
    std::cout << "bad return_mode " << pkt.multi_return_mode << std::endl;
    return false;
  }
  break;
  case INNO_ROBINE_ITEM_TYPE_SPHERE_POINTCLOUD:
  case INNO_ROBINW_ITEM_TYPE_SPHERE_POINTCLOUD:
  case INNO_FALCONII_DOT_1_ITEM_TYPE_SPHERE_POINTCLOUD:
    if (pkt.multi_return_mode == INNO_MULTIPLE_RETURN_MODE_SINGLE) {
      if (pkt.item_size != sizeof(InnoEnBlock1)) {
        std::cout << "bad block1 item size " << pkt.item_size << " " << sizeof(InnoEnBlock1) << std::endl;
        return false;
      }
    } else if (pkt.multi_return_mode == INNO_MULTIPLE_RETURN_MODE_2_STRONGEST ||
               pkt.multi_return_mode == INNO_MULTIPLE_RETURN_MODE_2_STRONGEST_FURTHEST) {
      if (pkt.item_size != sizeof(InnoEnBlock2)) {
        std::cout << "bad block2 item size " << pkt.item_size << " " << sizeof(InnoEnBlock2) << std::endl;
        return false;
      }
    } else {
      std::cout << "bad return_mode " << pkt.multi_return_mode << std::endl;
      return false;
    }
    break;
  case INNO_ITEM_TYPE_XYZ_POINTCLOUD:
    if (pkt.item_size != sizeof(InnoXyzPoint)) {
      std::cout << "bad InnoXyzPoint item size " << pkt.item_size << std::endl;
      return false;
    }
    break;
  case INNO_ROBINE_ITEM_TYPE_XYZ_POINTCLOUD:
  case INNO_ROBINW_ITEM_TYPE_XYZ_POINTCLOUD:
  case INNO_FALCONII_DOT_1_ITEM_TYPE_XYZ_POINTCLOUD:
    if (pkt.item_size != sizeof(InnoEnXyzPoint)) {
      std::cout << "bad InnoEnXyzPoint item size " << pkt.item_size << std::endl;
      return false;
    }
    break;
  default:
    is_data = false;
    break;
  }

  if (is_data) {
    size_t s =
        get_data_packet_size(InnoItemType(pkt.type), pkt.item_number, InnoMultipleReturnMode(pkt.multi_return_mode));
    if (pkt.common.size != s) {
      std::cout << "bad size " << s << " " << pkt.common.size << std::endl;
      return false;
    }
    if (pkt.common.version.major_version > kInnoMajorVersionDataPacket) {
      std::cout << "please upgrade client sdk, lidar protocol major version:" << pkt.common.version.major_version
                << " sdk major version:" << kInnoMajorVersionDataPacket << std::endl;
      return false;
    }
    return true;
  } else if (pkt.type == INNO_ITEM_TYPE_MESSAGE || pkt.type == INNO_ITEM_TYPE_MESSAGE_LOG) {
    if (pkt.item_number != 1) {
      std::cout << "bad item_number " << pkt.item_number << std::endl;
      return false;
    }
    const InnoMessage *messages = reinterpret_cast<const InnoMessage *>(pkt.payload);
    if (static_cast<uint32_t>(pkt.item_size) != messages[0].size || pkt.item_size <= sizeof(InnoMessage)) {
      std::cout << "bad message size " << pkt.item_size << " " << messages[0].size << std::endl;
      return false;
    }
    if (pkt.common.size != pkt.item_size + sizeof(InnoDataPacket)) {
      std::cout << "bad message size " << pkt.common.size << " " << pkt.item_size + sizeof(InnoDataPacket) << std::endl;
      return false;
    }
    return true;
  } else {
    std::cout << "bad type " << pkt.type << std::endl;
    return false;
  }
}

bool InnovusionDecoder::convert_to_xyz_pointcloud(const InnoDataPacket &src, InnoDataPacket *dest, size_t dest_size,
                                                    bool append) {
  if (!is_sphere_data(src.type)) {
    std::cout << "invalid type: " << src.type << std::endl;
    return false;
  }
  if (!check_data_packet(src, 0)) {
    std::cout << "invalid src datapacket" << std::endl;
    return false;
  }

  uint32_t item_count = 0;
  uint32_t dummy_count = 0;
  item_count = get_points_count(src);

  if (append) {
    if (!check_data_packet(*dest, dest_size)) {
      std::cout << "invalid dest datapacket" << std::endl;
      return false;
    }
    item_count += dest->item_number;
  }

  size_t required_size = 0;
  uint16_t time_adjust_10us = 0;
  if (!append) {
    required_size = sizeof(InnoDataPacket);
    if (required_size > dest_size) {
      std::cout << "not enough size " << "required_size " <<required_size << "dest_size " << dest_size;
      return false;
    }
    memcpy(dest, &src, sizeof(InnoDataPacket));
    if (src.type == INNO_ITEM_TYPE_SPHERE_POINTCLOUD) {
      dest->type = INNO_ITEM_TYPE_XYZ_POINTCLOUD;
      dest->item_size = sizeof(InnoXyzPoint);
    } else {
      dest->type += 1;  // robin & falconIII InnoItemType xyz = sphere+1
      dest->item_size = sizeof(InnoEnXyzPoint);
    }
    dest->item_number = 0;
  } else {
    required_size = dest->common.size;
    if (src.common.ts_start_us < dest->common.ts_start_us) {
      std::cout << "cannot merge earlier packet " << src.common.ts_start_us << " " << dest->common.ts_start_us << std::endl;
      return false;
    }
    time_adjust_10us = (src.common.ts_start_us - dest->common.ts_start_us) / 10;
  }

  {
    if (src.type == INNO_ITEM_TYPE_SPHERE_POINTCLOUD) {
      (void)iterate_inno_data_packet_cpoints<InnoBlock, InnoBlockHeader, InnoBlock1, InnoBlock2, InnoChannelPoint>(
          src, [&](const InnoDataPacketPointsCallbackParams<InnoBlock, InnoChannelPoint> &in_params) {
            if (in_params.pt.radius > 0) {
              InnoXyzPoint &ipt = dest->xyz_points[dest->item_number];
              required_size += static_cast<uint32_t>(sizeof(InnoXyzPoint));
              if (required_size > dest_size) {
                std::cerr << "no enough size required_size:" << required_size << ",dest_size:" << dest_size
                          << std::endl;
              } else {
                (void)get_xyz_point(in_params.block.header, in_params.pt, in_params.angle.angles[in_params.channel],
                                    in_params.channel, &ipt);
                ipt.multi_return = in_params.multi_return;
                ipt.is_2nd_return = in_params.pt.is_2nd_return;
                ipt.ts_10us += time_adjust_10us;
                dest->item_number++;
              }
            }
          });
    } else {
      (void)iterate_inno_data_packet_cpoints<InnoEnBlock, InnoEnBlockHeader, InnoEnBlock1, InnoEnBlock2,
                                             InnoEnChannelPoint>(
          src, [&](const InnoDataPacketPointsCallbackParams<InnoEnBlock, InnoEnChannelPoint> &in_params) {
            if (in_params.pt.radius > 0) {
              InnoEnXyzPoint &ipt = dest->en_xyz_points[dest->item_number];
              required_size += static_cast<uint32_t>(sizeof(InnoEnXyzPoint));
              if (required_size > dest_size) {
                std::cerr << "no enough size required_size en:" << required_size << ",dest_size:" << dest_size
                          << std::endl;
              } else {
                (void)get_xyz_point(in_params.block.header, in_params.pt, in_params.angle.angles[in_params.channel],
                                    in_params.channel, &ipt, static_cast<InnoItemType>(in_params.pkt.type));
                ipt.multi_return = in_params.multi_return;
                ipt.is_2nd_return = in_params.pt.is_2nd_return;
                ipt.ts_10us += time_adjust_10us;
                dest->item_number++;
              }
            }
          });
    }
  }

  if (item_count != dest->item_number) {
    std::cout << "item number: " << dest->item_number << " item count " << item_count << std::endl;
  }

  dest->common.size = required_size;

  if (!check_data_packet(*dest, dest_size)) {
    std::cout << "invalid dest datapacket" << std::endl;
    return false;
  }

  return true;
}

}  // namespace innovusion_packet
}  // namespace drivers
}  // namespace nebula
