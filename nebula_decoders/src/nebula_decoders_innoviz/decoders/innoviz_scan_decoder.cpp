
#include "nebula_decoders/nebula_decoders_innoviz/decoders/innoviz_scan_decoder.hpp"
#include <nebula_common/nebula_common.hpp>
#include <chrono>
#include <algorithm>
#include <iomanip>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define ANGLE_ELEVATION_AZIMUTH_FACTOR (1.f/32768)


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

    if(marker != 0xfacecafe)
    {
        std::cerr << "InnovizScanDecoder: Invalid marker" << std::endl;
        std::cerr << "Marker = " << marker << std::endl;
        return;
    }

    if(last_sequence_number_ != 0 && 
        ((last_sequence_number_ == UINT32_MAX && sequence_number != 0) || 
        last_sequence_number_ + 1 != sequence_number))
    {
        std::cerr << "InnovizScanDecoder: Missing packet" << std::endl;
    }

    if(scan_size_ != 0 && scan_size_ != total_objects)
    {
        std::cerr << "InnovizScanDecoder: Scan has inconsistent size" << std::endl;
    }

    if(start_id >= end_id)
    {
        std::cerr << "InnovizScanDecoder: Invalid start/end id" << std::endl;
        return;
    }

    uint32_t totalStructSize =INVZ_PREFIX_SIZE + innoviz_scan_pc_.detections.size() * sizeof(LidarDetectionEntity_st); 
    if((start_id + length) > totalStructSize)
    {
        std::cerr << "InnovizScanDecoder: Scan overflow" << std::endl;
        auto sum = start_id + length;
        std::cerr << "Writing to " + sum << " in struct of size " << totalStructSize << std::endl;
    }

    //Write to prefix first
    if(start_id < INVZ_PREFIX_SIZE)
    {
        //min
        uint32_t prefixLength = std::min(INVZ_PREFIX_SIZE - start_id, length);
        uint8_t* destination = ((uint8_t *)(&innoviz_scan_pc_)) + start_id;
        memcpy(destination, packet.data.data(), prefixLength);    
        start_id += prefixLength;
        length -= prefixLength;
        written_size_ += prefixLength;
        offset += prefixLength;
    }

    if(start_id < INVZ_PREFIX_SIZE)
    {
        std::cout << "Error writing prefix. Start = " << start_id << std::endl;
    }

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
    nebula_scan_pc_->header.stamp = (innoviz_scan_pc_.lidar_sensor_detections_header.timestamp.seconds * 1e9 + 
                                    innoviz_scan_pc_.lidar_sensor_detections_header.timestamp.nano_seconds) * 0.001;

    if(nebula_scan_pc_->header.stamp == 0)
    {
        auto now = std::chrono::system_clock::now();
        auto now_micros = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();
        nebula_scan_pc_->header.stamp = now_micros;
    }

    for(uint32_t pixelID = 0; pixelID < innoviz_scan_pc_.detections.size(); pixelID++)
    {
        LidarDetectionEntity_st invzPoint = innoviz_scan_pc_.detections[pixelID];

        bool isValid = invzPoint.distance.value > 0;

        if(invzPoint.confidence < sensor_configuration_->min_confidence) // Minimum confidence filter
        {
            isValid = false;
        }

        if(sensor_configuration_->filter_artifacts && invzPoint.Invalid_detection_classification) // Filter artifact pixels
        {
            isValid = false;
        }

        if(isValid)
        {
            drivers::NebulaPoint nebulaPoint{};
            nebulaPoint.distance  = invzPoint.distance.value * 0.01f; // uint16 cm to float m
            nebulaPoint.azimuth   = static_cast<float>(invzPoint.angle_azimuth)* M_PI * (ANGLE_ELEVATION_AZIMUTH_FACTOR); //Decode azimuth/elevation
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
