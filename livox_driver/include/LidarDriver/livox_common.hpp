#ifndef LIVOX_DRIVER_LIVOX_COMMON_HPP_
#define LIVOX_DRIVER_LIVOX_COMMON_HPP_
#include <string>

namespace livox_driver
{
enum class LivoxEchoMode { UNKNOWN, SINGLE_FIRST, SINGLE_STRONGEST, DUAL };

enum class LivoxCoordinateMode { UNKNOWN, CARTESIAN, SPHERICAL };

enum class LivoxSensorModel { UNKNOWN, HORIZON };

class LivoxSensorConfiguration
{
public:
  LivoxSensorModel sensor_model;
  LivoxCoordinateMode coordinate_model;
  LivoxEchoMode echo_mode;
  std::string host_ip;
  std::string sensor_ip;
  uint16_t data_port;
  uint16_t cmd_port;
  uint16_t imu_port;
  uint16_t frequency_ms;
};

class LivoxCloudConfiguration
{
public:
  LivoxEchoMode echo_mode;
  LivoxCoordinateMode coordinate_model;
  bool intensity;
  bool point_timestamp;

  double cloud_min_range;
  double cloud_max_range;
  bool remove_nans;
};

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

#pragma pack(1)
struct LivoxPointXyzrtl
{
  float x;            /**< X axis, Unit:m */
  float y;            /**< Y axis, Unit:m */
  float z;            /**< Z axis, Unit:m */
  float reflectivity; /**< Reflectivity   */
  uint8_t tag;        /**< Livox point tag   */
  uint8_t line;       /**< Laser line id     */
};
#pragma pack()

class LivoxLidarPacket
{
public:
  uint64_t time_stamp;
  int32_t data_cnt;
  std::vector<uint8_t> data;
};

class LivoxPublishData
{
public:
  uint64_t time;
  uint32_t num;
  std::vector<uint8_t> data;
};

}  // namespace livox_driver

#endif  // LIVOX_DRIVER_LIVOX_COMMON_HPP_
