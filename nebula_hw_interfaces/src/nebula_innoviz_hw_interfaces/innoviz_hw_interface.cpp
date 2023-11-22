
#include "nebula_hw_interfaces/nebula_hw_interfaces_innoviz/innoviz_hw_interface.hpp"


namespace nebula
{
namespace drivers
{
    InnovizHwInterface::InnovizHwInterface()
    :   cloud_io_context_{new ::drivers::common::IoContext(1)},
        cloud_udp_driver_{new ::drivers::udp_driver::UdpDriver(*cloud_io_context_)},
        scan_cloud_ptr_{std::make_unique<innoviz_msgs::msg::InnovizScan>()}
    {}

    Status InnovizHwInterface::SetSensorConfiguration(std::shared_ptr<SensorConfigurationBase> sensor_configuration)
    {
        Status status = Status::OK;
        sensor_configuration_ = sensor_configuration;

        if(sensor_configuration_->sensor_model != SensorModel::INNOVIZ_TWO_CONDOR && 
            sensor_configuration_->sensor_model != SensorModel::INNOVIZ_TWO_RAVEN)
        {
            status = Status::INVALID_SENSOR_MODEL;
        }

        if(sensor_configuration_->return_mode != drivers::ReturnMode::SINGLE_FIRST && 
            sensor_configuration_->return_mode != drivers::ReturnMode::FIRST)
        {
            status = Status::INVALID_ECHO_MODE;
        }
        
        return status;
    }

    Status InnovizHwInterface::GetSensorConfiguration(SensorConfigurationBase & sensor_configuration){
        sensor_configuration = *sensor_configuration_;
        return Status::OK;
    }

    Status InnovizHwInterface::GetCalibrationConfiguration(CalibrationConfigurationBase & /*calibration_configuration*/)
    {
        return Status::ERROR_1;
    }

    Status InnovizHwInterface::CloudInterfaceStart()
    {
        Status status = Status::OK;
        try
        {
            cloud_udp_driver_->init_receiver(sensor_configuration_->host_ip, sensor_configuration_->data_port, UINT16_MAX);
            cloud_udp_driver_->receiver()->open();
            cloud_udp_driver_->receiver()->bind();
            cloud_udp_driver_->receiver()->asyncReceive(std::bind(&InnovizHwInterface::ReceiveCloudPacketCallback, this, std::placeholders::_1));
        }
        catch(const std::exception& e)
        {
            PrintError(e.what());
            status = Status::UDP_CONNECTION_ERROR;     
        }

        return status;
    }

    Status InnovizHwInterface::CloudInterfaceStop() {return Status::ERROR_1;}

    void InnovizHwInterface::ReceiveCloudPacketCallback(const std::vector<uint8_t>& buffer)
    {
        auto now = std::chrono::system_clock::now();
        constexpr uint32_t INVZ_PACKET_HEADER_SIZE = sizeof(innoviz_msgs::msg::InnovizPacketHeader);
        uint32_t bufferSize = buffer.size();

        innoviz_msgs::msg::InnovizPacket innovizPacket;
        
        if(bufferSize > INVZ_PACKET_HEADER_SIZE)
        {
            //Populate packet header
            memcpy(&innovizPacket.header, buffer.data(), INVZ_PACKET_HEADER_SIZE);
            
            //Start of new frame before completion of previous one
            if(innovizPacket.header.pc_idx != last_frame_id_ && scan_cloud_ptr_->packets.size() > 0)
            {
                //Send previous scan data (will be a partial frame. Let decoder handle it)
                scan_reception_callback_(std::move(scan_cloud_ptr_));
                scan_cloud_ptr_ = std::make_unique<innoviz_msgs::msg::InnovizScan>();
                scan_cloud_ptr_->packets.clear();    
                processed_bytes_ = 0;
            }

            uint32_t offset = INVZ_PACKET_HEADER_SIZE;
            uint32_t actualDataSize = innovizPacket.header.end_idx - innovizPacket.header.start_id;

            if(buffer.size() <= offset + actualDataSize)
            {
                //Populate packet data
                innovizPacket.data.resize(actualDataSize);
                memcpy(innovizPacket.data.data(), buffer.data() + offset, actualDataSize);

                scan_cloud_ptr_->packets.emplace_back(innovizPacket);

                processed_bytes_ += actualDataSize;

                //Start of new scan, update scan timestamp with host time of first packet
                if(last_frame_id_ != innovizPacket.header.pc_idx)
                {
                    auto now_nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
                    scan_cloud_ptr_->header.stamp.nanosec = now_nanos % ((uint32_t)1e9);
                    scan_cloud_ptr_->header.stamp.sec = now_nanos / 1e9;
                }
                last_frame_id_ = innovizPacket.header.pc_idx;

                //Check if completed frame
                if(processed_bytes_ == innovizPacket.header.total_objects)
                {
                    //Send completed scan data
                    scan_reception_callback_(std::move(scan_cloud_ptr_));
                    scan_cloud_ptr_ = std::make_unique<innoviz_msgs::msg::InnovizScan>();
                    scan_cloud_ptr_->packets.clear();    
                    processed_bytes_ = 0;
                }
            }
            else
            {
                PrintError("Buffer size too small to contain the size described in the packet header");
            }
            

        }
        else
        {
            PrintError("Invalid packet size smaller than INVZ_PACKET_HEADER_SIZE");
        }

    }

    void InnovizHwInterface::RegisterScanCallback(std::function<void(std::unique_ptr<innoviz_msgs::msg::InnovizScan>)> scan_callback)
    {
        scan_reception_callback_ = std::move(scan_callback);
    }

    void InnovizHwInterface::SetLogger(std::shared_ptr<rclcpp::Logger> logger)
    {
        parent_node_logger = logger;
    }

    void InnovizHwInterface::PrintError(std::string errorMessage)
    {
        if(parent_node_logger)
        {
            RCLCPP_ERROR_STREAM((*parent_node_logger), errorMessage);
        }
        else
        {
            std::cout << errorMessage << '\n';
        }
    }

}
}