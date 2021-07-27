#ifndef LIDARDRIVER_LIVOX_DATA_RECV_HPP_
#define LIDARDRIVER_LIVOX_DATA_RECV_HPP_

#include "LidarDriver/livox_common.hpp"

namespace lidar_driver
{
/** Spherical coordinate format. */
#pragma pack(1)
struct LivoxSpherPoint
{
  uint32_t depth;       /**< Depth, Unit: mm */
  uint16_t theta;       /**< Zenith angle[0, 18000], Unit: 0.01 degree */
  uint16_t phi;         /**< Azimuth[0, 36000], Unit: 0.01 degree */
  uint8_t reflectivity; /**< Reflectivity */
};
#pragma pack()

/** Extend cartesian coordinate format. */
#pragma pack(1)
struct LivoxExtendRawPoint
{
  int32_t x;            /**< X axis, Unit:mm */
  int32_t y;            /**< Y axis, Unit:mm */
  int32_t z;            /**< Z axis, Unit:mm */
  uint8_t reflectivity; /**< Reflectivity */
  uint8_t tag;          /**< Tag */
};
#pragma pack()

/** Extend spherical coordinate format. */
#pragma pack(1)
struct LivoxExtendSpherPoint
{
  uint32_t depth;       /**< Depth, Unit: mm */
  uint16_t theta;       /**< Zenith angle[0, 18000], Unit: 0.01 degree */
  uint16_t phi;         /**< Azimuth[0, 36000], Unit: 0.01 degree */
  uint8_t reflectivity; /**< Reflectivity */
  uint8_t tag;          /**< Tag */
};
#pragma pack()

/** Dual extend cartesian coordinate format. */
#pragma pack(1)
struct LivoxDualExtendRawPoint
{
  int32_t x1;            /**< X axis, Unit:mm */
  int32_t y1;            /**< Y axis, Unit:mm */
  int32_t z1;            /**< Z axis, Unit:mm */
  uint8_t reflectivity1; /**< Reflectivity */
  uint8_t tag1;          /**< Tag */
  int32_t x2;            /**< X axis, Unit:mm */
  int32_t y2;            /**< Y axis, Unit:mm */
  int32_t z2;            /**< Z axis, Unit:mm */
  uint8_t reflectivity2; /**< Reflectivity */
  uint8_t tag2;          /**< Tag */
};
#pragma pack()

/** Dual extend spherical coordinate format. */
#pragma pack(1)
struct LivoxDualExtendSpherPoint
{
  uint16_t theta;        /**< Zenith angle[0, 18000], Unit: 0.01 degree */
  uint16_t phi;          /**< Azimuth[0, 36000], Unit: 0.01 degree */
  uint32_t depth1;       /**< Depth, Unit: mm */
  uint8_t reflectivity1; /**< Reflectivity */
  uint8_t tag1;          /**< Tag */
  uint32_t depth2;       /**< Depth, Unit: mm */
  uint8_t reflectivity2; /**< Reflectivity */
  uint8_t tag2;          /**< Tag */
};
#pragma pack()

/** Triple extend cartesian coordinate format. */
#pragma pack(1)
struct LivoxTripleExtendRawPoint
{
  int32_t x1;            /**< X axis, Unit:mm */
  int32_t y1;            /**< Y axis, Unit:mm */
  int32_t z1;            /**< Z axis, Unit:mm */
  uint8_t reflectivity1; /**< Reflectivity */
  uint8_t tag1;          /**< Tag */
  int32_t x2;            /**< X axis, Unit:mm */
  int32_t y2;            /**< Y axis, Unit:mm */
  int32_t z2;            /**< Z axis, Unit:mm */
  uint8_t reflectivity2; /**< Reflectivity */
  uint8_t tag2;          /**< Tag */
  int32_t x3;            /**< X axis, Unit:mm */
  int32_t y3;            /**< Y axis, Unit:mm */
  int32_t z3;            /**< Z axis, Unit:mm */
  uint8_t reflectivity3; /**< Reflectivity */
  uint8_t tag3;          /**< Tag */
};
#pragma pack()

/** Triple extend spherical coordinate format. */
#pragma pack(1)
struct LivoxTripleExtendSpherPoint
{
  uint16_t theta;        /**< Zenith angle[0, 18000], Unit: 0.01 degree */
  uint16_t phi;          /**< Azimuth[0, 36000], Unit: 0.01 degree */
  uint32_t depth1;       /**< Depth, Unit: mm */
  uint8_t reflectivity1; /**< Reflectivity */
  uint8_t tag1;          /**< Tag */
  uint32_t depth2;       /**< Depth, Unit: mm */
  uint8_t reflectivity2; /**< Reflectivity */
  uint8_t tag2;          /**< Tag */
  uint32_t depth3;       /**< Depth, Unit: mm */
  uint8_t reflectivity3; /**< Reflectivity */
  uint8_t tag3;          /**< Tag */
};
#pragma pack()

/** IMU data format. */
#pragma pack(1)
struct LivoxImuPoint
{
  float gyro_x; /**< Gyroscope X axis, Unit:rad/s */
  float gyro_y; /**< Gyroscope Y axis, Unit:rad/s */
  float gyro_z; /**< Gyroscope Z axis, Unit:rad/s */
  float acc_x;  /**< Accelerometer X axis, Unit:g */
  float acc_y;  /**< Accelerometer Y axis, Unit:g */
  float acc_z;  /**< Accelerometer Z axis, Unit:g */
};
#pragma pack()

/** Point data type. */
enum PointDataType {
  kCartesian,             /**< Cartesian coordinate point cloud. */
  kSpherical,             /**< Spherical coordinate point cloud. */
  kExtendCartesian,       /**< Extend cartesian coordinate point cloud. */
  kExtendSpherical,       /**< Extend spherical coordinate point cloud. */
  kDualExtendCartesian,   /**< Dual extend cartesian coordinate  point cloud. */
  kDualExtendSpherical,   /**< Dual extend spherical coordinate point cloud. */
  kImu,                   /**< IMU data. */
  kTripleExtendCartesian, /**< Triple extend cartesian coordinate  point cloud. */
  kTripleExtendSpherical, /**< Triple extend spherical coordinate  point cloud. */
  kMax                    /**< Max Point Data Type. */
};

/** Timestamp sync mode define. */
enum TimestampType {
  kNoSync = 0, /**< No sync signal mode. */
  kPtp = 1,    /**< 1588v2.0 PTP sync mode. */
  kRsvd = 2,   /**< Reserved use. */
  kPpsGps = 3, /**< pps+gps sync mode. */
  kPps = 4,    /**< pps only sync mode. */
  kUnknown = 5 /**< Unknown mode. */
};

const int kPrefixDataSize = 18;

struct DataTypePointInfoPair
{
  int32_t points_per_packet; /**< number of points every packet */
  int32_t packet_length;     /**< length of raw ethenet packet unit:bytes */
  int32_t raw_point_length;  /**< length of point uint:bytes */
  int32_t echo_num;          /**< echo number of current data */
} const g_data_type_info_pair_table[PointDataType::kMax] = {
  // data_type_info_pair_table
  {100, 1318, sizeof(LivoxRawPoint), 1},       // kCartesian
  {100, 918, sizeof(LivoxSpherPoint), 1},      // kSpherical
  {96, 1362, sizeof(LivoxExtendRawPoint), 1},  // kExtendCartesian
  {96, 978, sizeof(LivoxExtendSpherPoint),
   1},  // kExtendSpherical  update 9 -> sizeof(LivoxExtendSpherPoint)
  {48, 1362, sizeof(LivoxDualExtendRawPoint), 2},     // kDualExtendCartesian
  {48, 786, sizeof(LivoxDualExtendSpherPoint), 2},    // kDualExtendSpherical
  {1, 42, sizeof(livox_driver::LivoxImuPoint), 1},    // kImu
  {30, 1278, sizeof(LivoxTripleExtendRawPoint), 3},   // kTripleExtendCartesian
  {30, 678, sizeof(LivoxTripleExtendSpherPoint), 3},  // kTripleExtendSpherical
};

const uint32_t kMaxProductType = 9;
struct ProductTypePointInfoPair
{
  uint32_t points_per_second; /**< number of points per second */
  uint32_t point_interval;    /**< unit:ns */
  uint32_t line_num;          /**< laser line number */
} const g_product_type_info_pair_table[kMaxProductType] = {
  // product_type_info_pair_table
  {100000, 10000, 1}, {100000, 10000, 1}, {240000, 4167, 6},                      /**< tele */
  {240000, 4167, 6},  {100000, 10000, 1}, {100000, 10000, 1}, {100000, 10000, 1}, /**< mid70 */
  {240000, 4167, 6},  {240000, 4167, 6},
};

/// @param product_type : device type
/// @param data_type : receive data type
/// @return packet interval
inline int32_t GetPacketInterval(uint32_t product_type, uint8_t data_type)
{
  return g_product_type_info_pair_table[product_type].point_interval *
         g_data_type_info_pair_table[data_type].points_per_packet;
}

/// @param product_type : device type
/// @param data_type : receive data type
/// @return points per second / points per packet
inline int32_t GetPacketNumPerSec(uint32_t product_type, uint8_t data_type)
{
  return g_product_type_info_pair_table[product_type].points_per_second /
         g_data_type_info_pair_table[data_type].points_per_packet;
}

inline int32_t GetEthPacketLen(uint8_t data_type)
{
  return g_data_type_info_pair_table[data_type].packet_length;
}

inline int32_t GetPointsPerPacket(uint8_t data_type)
{
  return g_data_type_info_pair_table[data_type].points_per_packet;
}

inline int32_t GetRealPacketLen(uint8_t data_type, int point_num)
{
  return (g_data_type_info_pair_table[data_type].raw_point_length * point_num) + kPrefixDataSize;
}

inline uint32_t GetEchoNumPerPoint(uint8_t data_type)
{
  return g_data_type_info_pair_table[data_type].echo_num;
}

/// @brief receive data count.
/// @param data_type : receive data type
/// @param rcv_len : receive byte
/// @return raw point data count.
/// @retval -1 : error
inline int RecvDataCnt(uint8_t data_type, int rcv_len)
{
  int body_size = rcv_len - kPrefixDataSize;
  int cnt;

  if ((body_size < 0) || (data_type >= PointDataType::kMax)) {
    cnt = -1;
  } else {
    cnt = body_size / g_data_type_info_pair_table[data_type].raw_point_length;
  }
  return cnt;
}

}  // namespace lidar_driver
#endif  // LIDARDRIVER_LIVOX_DATA_RECV_HPP_
