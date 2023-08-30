
#include "nebula_decoders/nebula_decoders_innoviz/innoviz_driver.hpp"
#include "nebula_decoders/nebula_decoders_innoviz/decoders/innoviz_two_condor_decoder.hpp"
#include "nebula_decoders/nebula_decoders_innoviz/decoders/innoviz_two_raven_decoder.hpp"


namespace nebula
{
namespace drivers
{
    
InnovizDriver::InnovizDriver(const std::shared_ptr<drivers::InnovizSensorConfiguration>& sensorConfiguration)
{
    driver_status_ = nebula::Status::OK;
    switch(sensorConfiguration->sensor_model)
    {
        case SensorModel::INNOVIZ_TWO_CONDOR: 
        {
            scan_decoder_ = std::make_shared<itwo_condor::InnovizTwoCondor>(sensorConfiguration);
            break;
        }
        case SensorModel::INNOVIZ_TWO_RAVEN:
        {
            scan_decoder_ = std::make_shared<itwo_raven::InnovizTwoRaven>(sensorConfiguration);
            break;
        }
        default:
            driver_status_ = Status::INVALID_SENSOR_MODEL;

    }
}

Status InnovizDriver::GetStatus()
{
    return driver_status_;
}


drivers::NebulaPointCloudPtr InnovizDriver::ConvertScanToPointcloud(const std::shared_ptr<innoviz_msgs::msg::InnovizScan> & innoviz_scan)
{
    drivers::NebulaPointCloudPtr nebulaPCL;
    if(driver_status_ == Status::OK)
    {
        scan_decoder_->resetPointcloud();
        for(innoviz_msgs::msg::InnovizPacket& packet : innoviz_scan->packets)
        {
            scan_decoder_->parsePacket(packet);
        }

        nebulaPCL = scan_decoder_->getPointcloud();
    }
    else
    {
        nebulaPCL = nullptr;
    }

    return nebulaPCL;
}

Status InnovizDriver::SetCalibrationConfiguration(const CalibrationConfigurationBase & /*calibration_configuration*/)
{
    return Status::NOT_IMPLEMENTED;
}

} // namespace drivers    
} // namespace nebula
