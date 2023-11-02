#pragma once

#include <cstddef>
#include <cstdint>
#include <ctime>
#include <cmath>

#ifndef DEFINE_INNO_COMPACT_STRUCT
#if !(defined(_MSC_VER))
#define DEFINE_INNO_COMPACT_STRUCT(x) struct __attribute__((packed)) x
#define DEFINE_INNO_COMPACT_STRUCT_END
#else

#define DEFINE_INNO_COMPACT_STRUCT(x) __pragma(pack(push, 1)) struct x
#define DEFINE_INNO_COMPACT_STRUCT_END __pragma(pack(pop))
#endif
#endif

namespace nebula
{
namespace drivers
{
namespace innovusion_packet
{
enum InnoLidarMode {
  INNO_LIDAR_MODE_NONE = 0,
  INNO_LIDAR_MODE_SLEEP = 1,               // falcon & robin
  INNO_LIDAR_MODE_STANDBY = 2,             // falcon & robin
  INNO_LIDAR_MODE_WORK_NORMAL = 3,         // falcon & robin
  INNO_LIDAR_MODE_WORK_SHORT_RANGE = 4,    // falcon
  INNO_LIDAR_MODE_WORK_CALIBRATION = 5,    // falcon & robin
  INNO_LIDAR_MODE_PROTECTION = 6,          // falcon & robin
  INNO_LIDAR_MODE_WORK_QUIET = 7,          // falcon
  INNO_LIDAR_MODE_WORK_INTERNAL_1 = 8,     // falcon
  INNO_LIDAR_MODE_FOTA = 9,                // [UDS upgrade & ADC FOTA]
  INNO_LIDAR_MODE_WORK_MAX = 10,
};

enum InnoLidarStatus {
  INNO_LIDAR_STATUS_NONE = 0,
  INNO_LIDAR_STATUS_TRANSITION = 1,
  INNO_LIDAR_STATUS_NORMAL = 2,
  INNO_LIDAR_STATUS_FAILED = 3,
  INNO_LIDAR_STATUS_MAX = 4,
};

enum InnoTimeSyncConfig {
  INNO_TIME_SYNC_CONFIG_HOST = 0,
  INNO_TIME_SYNC_CONFIG_PTP = 1,
  INNO_TIME_SYNC_CONFIG_GPS = 2,
  INNO_TIME_SYNC_CONFIG_FILE = 3,
  INNO_TIME_SYNC_CONFIG_NTP = 4,
  INNO_TIME_SYNC_CONFIG_MAX = 5,
};

enum InputSource {
  SOURCE_NO = 0,
  SOURCE_FILE,
  SOURCE_TCP,
  SOURCE_UDP,
  SOURCE_PCAP,
  SOURCE_MAX,
};

enum InnoItemType {
  INNO_ITEM_TYPE_NONE = 0,
  // Falcon SPHERE POINTCLOUD, InnoBlock
  INNO_ITEM_TYPE_SPHERE_POINTCLOUD = 1,
  INNO_ITEM_TYPE_MESSAGE = 2,
  INNO_ITEM_TYPE_MESSAGE_LOG = 3,
  // Falcon SPHERE POINTCLOUD, InnoXyzPoint
  INNO_ITEM_TYPE_XYZ_POINTCLOUD = 4,

  // ROBIN_E SPHERE POINTCLOUD, InnoEnBlock
  INNO_ROBINE_ITEM_TYPE_SPHERE_POINTCLOUD = 5,
  // ROBIN_E SPHERE POINTCLOUD, InnoEnXyzPoint
  INNO_ROBINE_ITEM_TYPE_XYZ_POINTCLOUD = 6,

  // ROBIN_W SPHERE POINTCLOUD, InnoEnBlock
  INNO_ROBINW_ITEM_TYPE_SPHERE_POINTCLOUD = 7,
  // ROBIN_W SPHERE POINTCLOUD, InnoEnXyzPoint
  INNO_ROBINW_ITEM_TYPE_XYZ_POINTCLOUD = 8,

  // Falcon2.1 SPHERE POINTCLOUD, InnoEnBlock
  INNO_FALCONII_DOT_1_ITEM_TYPE_SPHERE_POINTCLOUD = 9,
  // Falcon2.1 XYZ POINTCLOUD, InnoEnXyzPoint
  INNO_FALCONII_DOT_1_ITEM_TYPE_XYZ_POINTCLOUD = 10,

  // FalconIII
  INNO_FALCONIII_ITEM_TYPE_SPHERE_POINTCLOUD = 11,
  INNO_FALCONIII_ITEM_TYPE_XYZ_POINTCLOUD = 12,

  INNO_ITEM_TYPE_MAX = 13,
};

enum InnoReflectanceMode {
  INNO_REFLECTANCE_MODE_NONE = 0,
  INNO_REFLECTANCE_MODE_INTENSITY = 1,
  INNO_REFLECTANCE_MODE_REFLECTIVITY = 2,
  INNO_REFLECTANCE_MODE_MAX = 3,
};

enum InnoMultipleReturnMode {
  INNO_MULTIPLE_RETURN_MODE_NONE = 0,
  INNO_MULTIPLE_RETURN_MODE_SINGLE = 1,
  INNO_MULTIPLE_RETURN_MODE_2_STRONGEST = 2,
  // one strongest return and one furthest return
  INNO_MULTIPLE_RETURN_MODE_2_STRONGEST_FURTHEST = 3,
  INNO_MULTIPLE_RETURN_MODE_MAX
};

enum InnoDistanceUnitPerMeter {
  kInnoDistanceUnitPerMeter200 = 200,  // FalconIGK
  kInnoDistanceUnitPerMeter400 = 400   // Robin & falcon2.1
};

enum InnoVAngleDiffBase {
  kInnoFaconVAngleDiffBase = 196,   // FalconIGK,falconII & falcon2.1
  kInnoRobinEVAngleDiffBase = 0,    // RobinE
  kInnoRobinWVAngleDiffBase = 240,  // RobinW
};

/************
 Constants
*************/
typedef double InnoTimestampUs;
#define INNO_CHANNEL_NUMBER_BIT 2
#define INNO_CHANNEL_NUMBER (1 << INNO_CHANNEL_NUMBER_BIT)
#define INNO_MAX_MULTI_RETURN 2
#define INNO_SN_SZIE 16
#define INNO_HW_NUMBER_SIZE 3
static const uint16_t kInnoMagicNumberDataPacket = 0x176A;
static const uint8_t kInnoMajorVersionDataPacket = 2;  // upgrade Major version from 1->2 2023/6/6
static const uint8_t kInnoMinorVersionDataPacket = 1;
static const uint16_t kInnoMagicNumberStatusPacket = 0x186B;
static const uint8_t kInnoMajorVersionStatusPacket = 2;  // upgrade Major version from 1->2 2023/6/6
static const uint8_t kInnoMinorVersionStatusPacket = 1;

static const uint32_t kInnoDistanceUnitPerMeter = 400;
static const double kMeterPerInnoDistanceUnit = 1.0 / kInnoDistanceUnitPerMeter;
static const double kMeterPerInnoDistanceUnit200 = 1.0 / kInnoDistanceUnitPerMeter200;  // falconIGK
static const double kMeterPerInnoDistanceUnit400 = 1.0 / kInnoDistanceUnitPerMeter400;  // robin & falcon2.1
static const uint32_t kInnoDegreePerPiRad = 180;
static const uint32_t kInnoAngleUnitPerPiRad = 32768;
static const double kRadPerInnoAngleUnit = M_PI / kInnoAngleUnitPerPiRad;
static const double kDegreePerInnoAngleUnit =
    180.0 / kInnoAngleUnitPerPiRad;
static const double kInnoAngleUnitPerDegree =
    kInnoAngleUnitPerPiRad / 180.0;
static const int32_t kInnoInvalidAngleInUnit = kInnoAngleUnitPerDegree * 90;
static const uint32_t kInnoChannelNumberBit = INNO_CHANNEL_NUMBER_BIT;
static const uint32_t kInnoChannelNumber = INNO_CHANNEL_NUMBER;
static const uint32_t kInnoMaxMultiReturn = INNO_MAX_MULTI_RETURN;
static const int16_t kInnoVAngleDiffBase = 196;

static const int kInnoBaseFaultEnd = 64;
static const double kInnoNopROI = 10000.0;
static const uint32_t kConvertSize = 1024 * 1024 * 10;

union InputParam {
  struct {
    enum InputSource source_type;
    char lidar_ip[16];
  } base_param;

  struct {
    enum InputSource source_type;
    char lidar_ip[16];
    char filename[256];
    int32_t play_rate;
    int32_t rewind;
    int64_t skip;
  } file_param;

  struct {
    enum InputSource source_type;
    char lidar_ip[16];
    double read_timeout_sec;
    int64_t skip;
  } tcp_param;

  struct {
    enum InputSource source_type;
    char lidar_ip[16];
    uint16_t udp_port;
    double read_timeout_sec;
    int64_t skip;
  } udp_param;

  struct {
    enum InputSource source_type;
    char lidar_ip[16];
    char filename[256];
    int32_t play_rate;
    int32_t rewind;
    int64_t skip;
    uint16_t data_port;
    uint16_t status_port;
    uint16_t message_port;
  } pcap_param;
};

#if defined(__MINGW64__) || !defined(_WIN32)
/* 17 bytes per block header */
typedef DEFINE_INNO_COMPACT_STRUCT(InnoBlockHeader) {
  /* horizontal angle, 0 is straight forward, right is positive,
     unit is kRadPerInnoAngleUnit rad, range (-PI to PI) */
  int16_t h_angle;
  /* vertical angle, 0 is the horizon, up is positive,
     unit is kRadPerInnoAngleUnit rad, range (-PI to PI) */
  int16_t v_angle;
  /* relative timestamp (to ts_start_us) in 10us, 0-655,350us */
  uint16_t ts_10us;
  uint16_t scan_idx;     /* point idx within the scan line */
  uint16_t scan_id: 9;   /* id of the scan line */
  // real angle is h_angle + h_angle_diff_1
  int64_t h_angle_diff_1: 9;
  int64_t h_angle_diff_2: 10;
  int64_t h_angle_diff_3: 11;
  // real angle is v_angle + v_angle_diff_1 + kVAngleDiffBase * channel
  int64_t v_angle_diff_1: 8;  // 196 + [-128, 127]
  int64_t v_angle_diff_2: 9;  // 392 + [-256, 255]
  int64_t v_angle_diff_3: 9;  // 588 + [-256, 255]
  /*   0: in sparse region
    0x01: in vertical slow region
    0x10: in horizontal slow region
    0x11: in center ROI */
  uint64_t in_roi: 2;
  uint64_t facet: 3;
  uint64_t reserved_flags: 2; /* all 0 */
} InnoBlockHeader;
DEFINE_INNO_COMPACT_STRUCT_END
#else
/* 17 bytes per block header */
DEFINE_INNO_COMPACT_STRUCT(InnoBlockHeader) {
  /* horizontal angle, 0 is straight forward, right is positive,
     unit is kRadPerInnoAngleUnit rad, range (-PI to PI) */
  int16_t h_angle;
  /* vertical angle, 0 is the horizon, up is positive,
     unit is kRadPerInnoAngleUnit rad, range (-PI to PI) */
  int16_t v_angle;
  /* relative timestamp (to ts_start_us) in 10us, 0-655,350us */
  uint16_t ts_10us;
  uint16_t scan_idx;    /* point idx within the scan line */
  union {
    uint16_t scan_id : 9; /* id of the scan line */
    struct {
      uint8_t scan_id0;
      int64_t scan_id1 : 1;
      // real angle is h_angle + h_angle_diff_1
      int64_t h_angle_diff_1 : 9;
      int64_t h_angle_diff_2 : 10;
      int64_t h_angle_diff_3 : 11;
      // real angle is v_angle + v_angle_diff_1 + kVAngleDiffBase * channel
      int64_t v_angle_diff_1 : 8;  // 196 + [-128, 127]
      int64_t v_angle_diff_2 : 9;  // 392 + [-256, 255]
      int64_t v_angle_diff_3 : 9;  // 588 + [-256, 255]
      /*   0: in sparse region
        0x01: in vertical slow region
        0x11: in center ROI */
      uint64_t in_roi : 2;
      uint64_t facet : 3;
      uint64_t reserved_flags : 2; /* all 0 */
    };
  };
};
DEFINE_INNO_COMPACT_STRUCT_END
#endif

typedef DEFINE_INNO_COMPACT_STRUCT(InnoXyzrD) {
  double x;
  double y;
  double z;
  double radius;
} InnoXyzrD;
DEFINE_INNO_COMPACT_STRUCT_END

/* compact format, 16 + 8 + 2 = 26 bytes per point */
typedef DEFINE_INNO_COMPACT_STRUCT(InnoXyzPoint) {
  float x;
  float y;
  float z;
  float radius;
  uint16_t ts_10us;
  uint16_t scan_id: 9;   /* id of the scan line */
  uint16_t in_roi: 2;
  uint16_t facet: 3;
  uint16_t multi_return: 1;
  uint16_t reserved_flags: 1; /* all 0 */
  uint32_t is_2nd_return: 1;
  uint32_t scan_idx: 14;   /* point idx within the scan line */
  uint32_t refl: 9;        /* reflectance, 1-254, 255 means a reflector     */
                           /* or intensity, also 1-254 & 255=reflector      */
  uint32_t type: 2;        /* 0: normal, 1: ground, 2: fog                  */
  uint32_t elongation: 4;  /* elongation */
  uint32_t channel: 2;
  uint16_t ring_id;
} InnoXyzPoint;
DEFINE_INNO_COMPACT_STRUCT_END

/* compact format, 4 bytes per point */
typedef DEFINE_INNO_COMPACT_STRUCT(InnoChannelPoint) {
  uint32_t radius: 17;     /* distance in distance unit, range [0, 655.35m] */
  uint32_t refl: 8;        /* reflectance, 1-254, 255 means a reflector     */
                           /* or intensity, also 1-254 & 255=reflector      */
  uint32_t is_2nd_return: 1; /* 0: 1st return, 1: 2nd return                */
  uint32_t type: 2;        /* 0: normal, 1: ground, 2: fog                  */
  uint32_t elongation: 4;  /* elongation */
} InnoChannelPoint;
DEFINE_INNO_COMPACT_STRUCT_END

typedef DEFINE_INNO_COMPACT_STRUCT(InnoBlock) {
  InnoBlockHeader header;
  InnoChannelPoint points[0];
} InnoBlock;
DEFINE_INNO_COMPACT_STRUCT_END

/* 17 + 4 * 4 = 33 bytes */
typedef DEFINE_INNO_COMPACT_STRUCT(InnoBlock1) {
  InnoBlockHeader header;
  InnoChannelPoint points[INNO_CHANNEL_NUMBER];
} InnoBlock1;
DEFINE_INNO_COMPACT_STRUCT_END

/* 17 + 8 * 4 = 49 bytes */
typedef DEFINE_INNO_COMPACT_STRUCT(InnoBlock2) {
  InnoBlockHeader header;
  InnoChannelPoint points[INNO_CHANNEL_NUMBER * INNO_MAX_MULTI_RETURN];
} InnoBlock2;
DEFINE_INNO_COMPACT_STRUCT_END

/* compact format, 8 bytes per point */
typedef DEFINE_INNO_COMPACT_STRUCT(InnoEnChannelPoint) {
  uint16_t reflectance;            /* reflectance, falcon 1-65535,robin 1-4095  */
  uint16_t intensity;           /* intensity, falcon 1-65535,robin 1-4095  */
  uint32_t elongation: 7;      /* elongation unit: 1ns */
  uint32_t is_2nd_return: 1;    /* 0: 1st return, 1: 2nd return                  */
  uint32_t radius : 19;         /* distance in distance unit, distance unit:1/400m, range [0, 655.35m] */
  uint32_t type : 2;            /* 0: normal, 1: ground, 2: fog                  */
  uint32_t firing: 1;           /* 0: weak, 1: strong */
  uint32_t reserved_flags : 2;  /* all 0 */
} InnoEnChannelPoint;
DEFINE_INNO_COMPACT_STRUCT_END

/* compact format, 30 bytes per point */
typedef DEFINE_INNO_COMPACT_STRUCT(InnoEnXyzPoint) {
  float x;
  float y;
  float z;
  float radius;
  uint16_t ts_10us;
  uint16_t scan_id : 10; /* id of the scan line */
  uint16_t in_roi : 2;
  uint16_t facet : 3;
  uint16_t multi_return : 1;    /* multi return mode,true mean the 2nd point*/
  uint16_t scan_idx : 14;       /* point idx within the scan line */
  uint16_t type : 2;            /* 0: normal, 1: ground, 2: fog                 */
  uint16_t reflectance;         /* reflectance, falcon 1-65535,robin 1-4095   */
  uint16_t intensity;           /* intensity, falcon 1-65535,robin 1-4095   */
  uint8_t elongation : 7;       /* elongation, unit: 1ns */
  uint8_t is_2nd_return : 1;
  uint8_t channel : 3;          /* max 8 channel */
  uint8_t firing : 1;
  uint8_t reserved_flags : 4;   /* all 0 */
  uint16_t ring_id;
} InnoEnXyzPoint;
DEFINE_INNO_COMPACT_STRUCT_END

/* 18 bytes per EnBlock header */
typedef DEFINE_INNO_COMPACT_STRUCT(InnoEnBlockHeader) {
  /* horizontal angle, 0 is straight forward, right is positive,
     unit is kRadPerInnoAngleUnit rad, range (-PI to PI) */
  int16_t h_angle;
  /* vertical angle, 0 is the horizon, up is positive,
     unit is kRadPerInnoAngleUnit rad, range (-PI to PI) */
  int16_t v_angle;
  /* relative timestamp (to ts_start_us) in 10us, 0-655,350us */
  uint16_t ts_10us;
  // real angle is h_angle + h_angle_diff_1
  int64_t h_angle_diff_1 : 11;
  int64_t h_angle_diff_2 : 11;
  int64_t h_angle_diff_3 : 12;
  // real angle is v_angle + v_angle_diff_1 + kVAngleDiffBase * channel
  int64_t v_angle_diff_1 : 10;
  int64_t v_angle_diff_2 : 10;
  int64_t v_angle_diff_3 : 10;
  uint16_t scan_idx;            /* point idx within the scan line */
  uint16_t scan_id : 9;         /* id of the scan line */
  /*   0: in sparse region
  0x01: in vertical slow region
  0x11: in center ROI
  only for falcon  */
  uint16_t in_roi : 2;
  uint16_t facet : 3;
  uint16_t reserved_flags : 2; /* all 0 */
} InnoEnBlockHeader;
DEFINE_INNO_COMPACT_STRUCT_END

typedef DEFINE_INNO_COMPACT_STRUCT(InnoEnBlock) {
  InnoEnBlockHeader header;
  InnoEnChannelPoint points[0];
} InnoEnBlock;
DEFINE_INNO_COMPACT_STRUCT_END

/* 18 + 4 * 8 = 50 bytes */
typedef DEFINE_INNO_COMPACT_STRUCT(InnoEnBlock1) {
  InnoEnBlockHeader header;
  InnoEnChannelPoint points[INNO_CHANNEL_NUMBER];
} InnoEnBlock1;
DEFINE_INNO_COMPACT_STRUCT_END

/* 18 + 8 * 8 = 82 bytes */
typedef DEFINE_INNO_COMPACT_STRUCT(InnoEnBlock2) {
  InnoEnBlockHeader header;
  InnoEnChannelPoint points[INNO_CHANNEL_NUMBER * INNO_MAX_MULTI_RETURN];
} InnoEnBlock2;
DEFINE_INNO_COMPACT_STRUCT_END


static inline size_t innoblock_get_idx(size_t channel, size_t r) {
  /* r0_ch0 r0_ch1 r0_ch2 r0_ch3 r1_ch0 r1_ch1 r1_ch2 r1_ch3 */
  return channel + (r << kInnoChannelNumberBit);
}

typedef DEFINE_INNO_COMPACT_STRUCT(InnoMessage) {
  uint32_t size;  // size of the whole InnoMessage,
                  // i.e. size of content + sizeof(InnoMessage)
  uint32_t src;
  uint64_t id;
  uint32_t level;      /* enum InnoMessageLevel */
  uint32_t code;       /* message code          */
  int32_t reserved[4]; /* all 0                 */
  char content[0];     /* 0 end string          */
} InnoMessage;
DEFINE_INNO_COMPACT_STRUCT_END

/*
  Fixed header structure to indicate the firmware/software version.
  This structure won't change during firmware update in the future.
*/
typedef DEFINE_INNO_COMPACT_STRUCT(InnoCommonVersion) {
  /* 2 byte */
  uint16_t magic_number;

  /* 2 byte */
  uint8_t major_version;
  uint8_t minor_version;

  /* 2 byte */
  uint16_t fw_sequence;
} InnoCommonVersion;
DEFINE_INNO_COMPACT_STRUCT_END

typedef DEFINE_INNO_COMPACT_STRUCT(InnoCommonHeader) {
  /* 6 bytes */
  InnoCommonVersion version;

  /* 4 bytes, cover every thing except checksum */
  uint32_t checksum;

  /* 4 bytes */
  uint32_t size;

  /* 2 bytes */
  uint8_t source_id : 4;           /* up to 16 different LiDAR source */
  uint8_t timestamp_sync_type : 4; /* enum InnoTimestampSyncType      */
  uint8_t lidar_type;          /* enum InnoLidarType */
  /* 8 bytes */
  InnoTimestampUs ts_start_us; /* epoch time of start of frame, in micro-sec */

  /* 2 bytes */
  uint8_t lidar_mode;        /* enum InnoLidarMode    */
  uint8_t lidar_status;      /* enum InnoLidarStatus  */
} InnoCommonHeader;
DEFINE_INNO_COMPACT_STRUCT_END

/*
 * Main data packet definition
 * 26 + 12 + 10 + 2 + 4 + 16 = 70 bytes, max overhead is 70/1472 = 4.7%,
 */
typedef DEFINE_INNO_COMPACT_STRUCT(InnoDataPacket) {
  InnoCommonHeader common;

  /* 12 bytes */
  uint64_t idx;         /* frame index, start from 0                     */
  uint16_t sub_idx;     /* sub-frame index, start from 0 for every frame */
  uint16_t sub_seq;     /* sequence within a sub-frame                   */

  /* 10 byte */
  /* type in enum InnoItemType, each type uses independent global idx */
  uint32_t type :8;
  uint32_t item_number :24;        /* max 4 * 1024 * 1024               */
  uint16_t item_size;              /* max 65535, 0 means variable size  */
  uint32_t topic;                  /* reserved                          */

  /* 2 bytes */
  uint16_t scanner_direction :1; /* 0: top->bottom, 1: bottom->top          */
  uint16_t use_reflectance   :1; /* 0: intensity mode, 1: reflectance mode  */
  uint16_t multi_return_mode :3; /* ... */
  uint16_t confidence_level  :2; /* 0: no confidence, 3: higest             */
  uint16_t is_last_sub_frame :1; /* 1: the last sub frame of a frame        */
  uint16_t is_last_sequence  :1; /* 1: the last piece of a sub frame        */
  uint16_t has_tail :1;          /* has additional tail struct after points */
  uint16_t frame_sync_locked :1; /* 1: frame sync has locked                */
  uint16_t is_first_sub_frame :1; /* 1: the first sub frame of a frame      */
  uint16_t last_four_channel :1;
  uint16_t reserved_flag :3;     /* all 0 */

  /* 4 bytes */
  int16_t roi_h_angle;           /* configured ROI in InnoAngleUnit */
  int16_t roi_v_angle;
  uint32_t extend_reserved[4];  /* add more extend reserved area 16 byte */
// MSVC compiler does not support multi-dimensional flexible arrays.
# if !defined(_MSC_VER)
  union {
    char payload[0];
    InnoBlock1 inno_block1s[0];
    InnoBlock2 inno_block2s[0];
    InnoMessage messages[0];
    InnoXyzPoint xyz_points[0];
        // Robin & Falcon2.1
    InnoEnBlock1 inno_en_block1s[0];
    InnoEnBlock2 inno_en_block2s[0];
    InnoEnXyzPoint en_xyz_points[0];
  };
#else
  char payload[0];
#endif
} InnoDataPacket;
DEFINE_INNO_COMPACT_STRUCT_END

}  // namespace innovusion_packet
}  // namespace drivers
}  // namespace nebula
