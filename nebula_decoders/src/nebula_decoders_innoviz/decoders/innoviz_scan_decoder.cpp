
#include "nebula_decoders/nebula_decoders_innoviz/decoders/innoviz_scan_decoder.hpp"
#include <nebula_common/nebula_common.hpp>
#include <chrono>
#include <algorithm>
#include <iomanip>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define ANGLE_ELEVATION_AZIMUTH_FACTOR (1.f/32768)
#define INNOVIZ_IPCL_MARKER (0xfacecafe)


namespace nebula
{
namespace drivers
{

InnovizScanDecoder::InnovizScanDecoder(uint32_t numOfPoints, const std::shared_ptr<drivers::InnovizSensorConfiguration>& sensorConfiguration)
{
    sensor_configuration_ = sensorConfiguration;
    innoviz_scan_pc_.detections.resize(numOfPoints);
    nebula_scan_pc_.reset(new NebulaPointCloud);
    nebula_scan_pc_->reserve(innoviz_scan_pc_.detections.size());
}


void InnovizScanDecoder::resetPointcloud()
{
    written_size_ = 0;
    scan_size_    = 0;

    innoviz_scan_pc_.detection_interface_type = 0;
    innoviz_scan_pc_.length = 0;
    memset(&innoviz_scan_pc_.lidar_sensor_detections_header, 0, sizeof(LidarSensorDetectionsHeader_st));
    memset(innoviz_scan_pc_.detections.data(), 0, innoviz_scan_pc_.detections.size() * sizeof(LidarDetectionEntity_st));
}

void InnovizScanDecoder::parsePacket(innoviz_msgs::msg::InnovizPacket& packet)
{
    uint32_t marker = packet.header.marker;
    uint32_t sequence_number = packet.header.sequence_number;
    uint32_t start_id = packet.header.start_id;
    uint32_t end_id   = packet.header.end_idx;
    uint32_t total_objects = packet.header.total_objects;
    uint32_t length = end_id - start_id;
    uint32_t offset = 0;

    constexpr uint32_t INVZ_PREFIX_SIZE = sizeof(innoviz_scan_pc_.length) + 
                                    sizeof(innoviz_scan_pc_.detection_interface_type) + 
                                    sizeof(innoviz_scan_pc_.lidar_sensor_detections_header);

    //Check packet has a valid Innoviz marker
    if(marker != INNOVIZ_IPCL_MARKER)
    {
        std::cerr << "InnovizScanDecoder: Invalid marker" << std::endl;
        std::cerr << "Marker = " << marker << std::endl;
        return;
    }

    //Check for missing packet sequence number
    if(start_id != 0 &&last_sequence_number_ != 0 && 
        ((last_sequence_number_ == UINT32_MAX && sequence_number != 0) || 
        last_sequence_number_ + 1 != sequence_number))
    {
        std::cerr << "InnovizScanDecoder: Missing packet" << std::endl;
    }

    //Ensure validity of scan_size
    if(scan_size_ != 0 && scan_size_ != total_objects)
    {
        std::cerr << "InnovizScanDecoder: Scan has inconsistent size" << std::endl;
    }

    if(start_id >= end_id)
    {
        std::cerr << "InnovizScanDecoder: Invalid start/end id" << std::endl;
        return;
    }

    //Check for overflow
    uint32_t totalStructSize = INVZ_PREFIX_SIZE + innoviz_scan_pc_.detections.size() * sizeof(LidarDetectionEntity_st); 
    if((start_id + length) > totalStructSize)
    {
        std::cerr << "InnovizScanDecoder: Scan overflow" << std::endl;
        auto sum = start_id + length;
        std::cerr << "Writing to " + sum << " in struct of size " << totalStructSize << std::endl;
        return;
    }

    //Write to prefix first
    if(start_id < INVZ_PREFIX_SIZE)
    {
        uint32_t prefixLength = std::min(INVZ_PREFIX_SIZE - start_id, length);
        uint8_t* destination = ((uint8_t *)(&innoviz_scan_pc_)) + start_id;
        memcpy(destination, packet.data.data(), prefixLength);    
        start_id += prefixLength;
        length -= prefixLength;
        written_size_ += prefixLength;
        offset += prefixLength;
    }

    //Logically this can only happen if length < INVZ_PREFIX_SIZE.
    // If this happens, we have no more data to write from this packet.
    if(start_id < INVZ_PREFIX_SIZE)
    {
        return;
    }

    //Discount INVZ_PREFIX_SIZE when writing to detections vector
    start_id -= INVZ_PREFIX_SIZE;
    uint8_t* destination = (uint8_t *)(innoviz_scan_pc_.detections.data()) + start_id;
    
    
    memcpy(destination, packet.data.data() + offset, length);

    scan_size_ = total_objects;
    written_size_ += length;
    last_sequence_number_ = sequence_number;

}

bool InnovizScanDecoder::isFrameComplete()
{
    return scan_size_ != 0 && written_size_ == scan_size_;
}

drivers::NebulaPointCloudPtr InnovizScanDecoder::getPointcloud()
{
    if(!isFrameComplete())
    {
        std::cerr << "Missing packets in frame. Returning empty frame" << std::endl;
        nebula_scan_pc_->points.clear();
        return nebula_scan_pc_;
    }

    nebula_scan_pc_->points.clear();
    nebula_scan_pc_->points.reserve(innoviz_scan_pc_.detections.size());
    //Take UTC timestamp from synced LiDAR and convert to micros
    nebula_scan_pc_->header.stamp = (innoviz_scan_pc_.lidar_sensor_detections_header.timestamp.seconds * 1e6 + 
                                    innoviz_scan_pc_.lidar_sensor_detections_header.timestamp.nano_seconds) * 0.001;

    for(uint32_t pixelID = 0; pixelID < innoviz_scan_pc_.detections.size(); pixelID++)
    {
        LidarDetectionEntity_st invzPoint = innoviz_scan_pc_.detections[pixelID];

        bool isValid = invzPoint.distance.value > 0;

        // Filter pixels that are below the configured confidence threshold
        if(invzPoint.confidence < sensor_configuration_->min_confidence) 
        {
            isValid = false;
        }

        // Filter artifact pixels based on the configuration
        if(sensor_configuration_->filter_artifacts && invzPoint.Invalid_detection_classification) 
        {
            isValid = false;
        }

        if(isValid)
        {
            drivers::NebulaPoint nebulaPoint{};
            nebulaPoint.distance  = invzPoint.distance.value * 0.01f; // uint16 cm to float m
            //Convert digital azimuth/elevation values to radians
            nebulaPoint.azimuth   = static_cast<float>(invzPoint.angle_azimuth)* M_PI * (ANGLE_ELEVATION_AZIMUTH_FACTOR); 
            nebulaPoint.elevation = static_cast<float>(invzPoint.angle_elevation)* M_PI * (ANGLE_ELEVATION_AZIMUTH_FACTOR);

            nebulaPoint.x = nebulaPoint.distance * cos(nebulaPoint.azimuth) * cos(nebulaPoint.elevation);
		    nebulaPoint.y = nebulaPoint.distance * sin(nebulaPoint.azimuth) * cos(nebulaPoint.elevation);
		    nebulaPoint.z = nebulaPoint.distance * sin(nebulaPoint.elevation);

            nebulaPoint.intensity = invzPoint.reflectivity;
            nebulaPoint.return_type = static_cast<uint8_t>(drivers::ReturnType::STRONGEST);
            
            //TODO: Inhabit channel and time_stamp

            nebula_scan_pc_->points.emplace_back(nebulaPoint);
        }
    }

    return nebula_scan_pc_;
}

}
}
