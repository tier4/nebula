#pragma once

#include <cstdint>

#include "nebula_common/point_types.hpp"
#include "nebula_common/innoviz/innoviz_common.hpp"

#include "innoviz_msgs/msg/innoviz_packet.hpp"

namespace nebula
{
namespace drivers
{

#pragma pack(push, 1)

struct innoviz_network_al_st {
    uint32_t marker;                        // 0xFACE_CAFE
    uint32_t pc_idx;                        // iPCL payload Index for Reassembly
    uint16_t innoviz_network_al_version;    // Default set to 0
    uint16_t protocol_type;                 // The type of protocol adaptation layer - iPCL protocol type is set to 0x1.
    uint32_t data_fregmantaton_type;        // Default set to 0
    uint32_t sequence_number;               // Incremented by 1 for every packet sent
    uint32_t reserved;                      // Should be set to zero
    uint32_t start_idx;                     //First object index for each fragment.
    uint32_t end_idx;                       // End object index.
    uint32_t total_objects;                 // this is the total objects; Currently Object represented by 1 byte
};

struct EventDQ_et{
    uint8_t value;
};

struct ExEventDQ_et{
    uint16_t value;
};

struct PowerMode_et{
    uint8_t value;
};

struct Radian_et{
    float value;
};

struct Meter_et{
    float value;
};

struct Centimeter_st{
    uint16_t value;
};

struct InterfaceVersionId_st{
    uint8_t major;
    uint8_t minor;
    uint16_t patch;
};

struct SensorPose_st{
    uint16_t invalid_flags;
    Meter_et x;
    Meter_et x_std_dev;
    Meter_et y;
    Meter_et y_std_dev;
    Meter_et z;
    Meter_et z_std_dev;
    Radian_et roll;
    Radian_et roll_std_dev;
    Radian_et pitch;
    Radian_et pitch_std_dev;
    Radian_et yaw;
    Radian_et yaw_std_dev;
};

struct StndTimestamp_st{
    uint64_t seconds;
    uint32_t nano_seconds;
    uint32_t lidar_internal_timer; 
	uint8_t  sync_status; // Sync status 
};

struct Classification_et{
    uint16_t value; 
};

struct Sensor_metadata_et{
    uint32_t value[8];
};

struct Pcp_st{ 
    uint8_t ppv; // Positive predicted value
    uint8_t height;
    Classification_et classification; //
};

union Extension_ut{ 
    Pcp_st pcp; // PCP extension 
};

// Structure definitions 
struct Position_st{
	Centimeter_st distance;
    int16_t angle_azimuth; // axis Definition based on ISO23150  
    int16_t angle_elevation; // axis Definition based on ISO23150
};

union Signature_ut{
    uint32_t crc_32_type1;
    uint32_t crc_32_type2;
    uint32_t crc_32_type3;
    uint32_t autentication_128[4];
    uint32_t autentication_256[8]; 
};

// The Pixel Structure - 10 bytes Size - Core , PCP extension - 4 bytes
struct LidarDetectionEntity_st{
    Centimeter_st distance;
    uint8_t confidence;
    uint8_t reflectivity;
    int16_t angle_azimuth; // axis Definition based on ISO23150   (MUST - Need to run Teta/Phi to Cartesian trasformation) 
    int16_t angle_elevation; // axis Definition based on ISO23150 (MUST - Need to run Teta/Phi to Cartesian trasformation)
    uint16_t Invalid_detection_classification; // Invalid classes - Artifacts classes as example (DEFAULR - 0x0 - No artifacts) 
   /*---------------------------------------------------------------------------------*/
    // Extension_ut detection_entity_extension; // Extension – should be align to 2 bytes
   /*---------------------------------------------------------------------------------*/
};

// Point Cloud Header Structure
struct LidarSensorDetectionsHeader_st{    
    InterfaceVersionId_st version_id; // Interface Version ID   (DEFAULT 0.1.0)  
    uint32_t frame_id; // Frame ID from LiDAR raw data (VALID - Taken from INVZ)
    Sensor_metadata_et metadata; // Sensor Metadata For Sensor periodic information (DEFAULT - first 4 bytes - 0x01 (Sensor ID - Raven Center), byte 5-8 Lidar FW App Version, other 24 bytes set to 0x0)   
	uint32_t number_of_detections; // Number of detection points (<= Max Detection points) (VALID from the INVZ - mumber of pixels in the frame) 
    uint8_t pixel_struct_type; // Pixel extension type (DEFAULT - - 0x0 - Basic structure , No extensions)
    EventDQ_et event_data_qualifier; // Qualifiers ENUM (DEFAULT - 0x0)
    ExEventDQ_et extended_qualifier; // Extended Qualifiers ENUM - DEFAULT (0x0)
    PowerMode_et lidar_power_mode; // Power mode ENUM (NPM - 0x0 - No other modes for now) 
    StndTimestamp_st timestamp; // Frame Timestamp (MUST - Wall clock and sync status / Internal timer if out of sync) 
    SensorPose_st sensor_pose; // Sensor position – Calibration information (DEFAUT - Instalation initial coordinates - Hardcoded value)   
} ;

// Point cloud interface structure - iPCL (r1.0)
struct LidarDetectionInterface_st{
    uint32_t detection_interface_type; // To be used for Interface formats (DEFAULT: 0x1 - type 1 - No signature)
	uint32_t length; // Total length in Bytes
	LidarSensorDetectionsHeader_st lidar_sensor_detections_header;
    std::vector<LidarDetectionEntity_st> detections; //
};


#pragma pack(pop)

/// @brief  Base class for Innoviz LiDAR decoder
class InnovizScanDecoder 
{
public:
    InnovizScanDecoder(uint32_t numOfPoints, const std::shared_ptr<drivers::InnovizSensorConfiguration>& sensorConfiguration);

    /// @brief Unpack UDP data
    /// @param buf pointer to UDP payload
    void parsePacket(innoviz_msgs::msg::InnovizPacket& packet); 

    /// @brief Checks if decoder holds a complete frame.
    /// @return True if the decoder holds a complete frame. False otherwise.
    bool isFrameComplete();

    /// @brief Gets a completed pointcloud in Nebula format.
    /// @return Nebula format pointcloud.
    drivers::NebulaPointCloudPtr getPointcloud();

    /// @brief Resets the pointcloud. Should be called between frames.
    void resetPointcloud();

private:
    NebulaPointCloudPtr nebula_scan_pc_;
    LidarDetectionInterface_st innoviz_scan_pc_;

    size_t written_size_           = 0;
    size_t scan_size_              = 0;
    uint32_t last_sequence_number_ = 0;
    std::shared_ptr<drivers::InnovizSensorConfiguration> sensor_configuration_;
    
};

}
}
