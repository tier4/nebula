#include "hesai/hesai_hw_interface.hpp"

namespace nebula
{
namespace drivers
{
HesaiHwInterface::HesaiHwInterface()
: cloud_io_context_{new IoContext(1)},
  cloud_udp_driver_{new ::drivers::udp_driver::UdpDriver(*cloud_io_context_)}
{
}

Status HesaiHwInterface::SetSensorConfiguration(
  std::shared_ptr<SensorConfigurationBase> sensor_configuration)
{
  Status status = Status::OK;
  mtu_size_ = 1500;
  try {
    sensor_configuration_ =
      std::static_pointer_cast<HesaiSensorConfiguration>(sensor_configuration);
    if (
      sensor_configuration_->sensor_model == SensorModel::HESAI_PANDAR40P ||
      sensor_configuration_->sensor_model == SensorModel::HESAI_PANDAR40P) {
      azimuth_index_ = 2;  // 2 + 124 * [0-9]
      is_valid_packet_ = [](size_t packet_size) {
        return (packet_size == 1262 || packet_size == 1266);
      };
    } else if (sensor_configuration_->sensor_model == SensorModel::HESAI_PANDARQT) {
      azimuth_index_ = 12;  // 12 + 258 * [0-3]
      is_valid_packet_ = [](size_t packet_size) { return (packet_size == 1072); };
    } else if (sensor_configuration_->sensor_model == SensorModel::HESAI_PANDARXT32) {
      azimuth_index_ = 12;  // 12 + 130 * [0-7]
      is_valid_packet_ = [](size_t packet_size) { return (packet_size == 1080); };
    } else if (sensor_configuration_->sensor_model == SensorModel::HESAI_PANDAR64) {
      azimuth_index_ = 8;  // 8 + 192 * [0-5]
      is_valid_packet_ = [](size_t packet_size) {
        return (packet_size == 1194 || packet_size == 1198);
      };
    } else if (sensor_configuration_->sensor_model == SensorModel::HESAI_PANDAR128_V14) {
      azimuth_index_ = 12;  // 12 + 386 * [0-1]
      is_valid_packet_ = [](size_t packet_size) { return (packet_size == 893); };  // version 1.4
      mtu_size_ = 1800;
    } else {
      status = Status::INVALID_SENSOR_MODEL;
    }
  } catch (const std::exception & ex) {
    status = Status::SENSOR_CONFIG_ERROR;
    std::cerr << NebulaStatusToString(status) << std::endl;
    return status;
  }
  return status;
}

Status HesaiHwInterface::CloudInterfaceStart()
{
  try {
    std::cout << "Starting UDP server on: " << sensor_configuration_->host_ip << "."
              << sensor_configuration_->data_port << std::endl;
    cloud_udp_driver_->init_receiver(
      sensor_configuration_->host_ip, sensor_configuration_->data_port);
    cloud_udp_driver_->receiver()->open();
    cloud_udp_driver_->receiver()->bind();
    cloud_udp_driver_->receiver()->asyncReceive(
      std::bind(&HesaiHwInterface::ReceiveCloudPacketCallback, this, std::placeholders::_1));
  } catch (const std::exception & ex) {
    Status status = Status::UDP_CONNECTION_ERROR;
    std::cerr << NebulaStatusToString(status) << sensor_configuration_->sensor_ip << ","
              << sensor_configuration_->data_port << std::endl;
    return status;
  }
  return Status::OK;
}

Status HesaiHwInterface::RegisterScanCallback(
  std::function<void(std::unique_ptr<std::vector<std::vector<uint8_t>>>)> scan_callback)
{
  scan_reception_callback_ = std::move(scan_callback);
}

void HesaiHwInterface::ReceiveCloudPacketCallback(const std::vector<uint8_t> & buffer)
{
  int scan_phase = static_cast<int>(sensor_configuration_->scan_phase * 100.0);
  std::unique_ptr<std::vector<std::vector<uint8_t>>> scan_data_ptr(
    new std::vector<std::vector<uint8_t>>);

  for (int prev_phase = 0;;) {  // start forming scan
    while (true) {              // wait for valid lidar packet
      if (is_valid_packet_(buffer.size())) {
        if (buffer.size() < mtu_size_) {
          // make sure the size matches the expected mtu size by the packet
          std::vector<uint8_t> packet(buffer);
          packet.resize(mtu_size_);
          scan_data_ptr->emplace_back(packet);
        } else {
          scan_data_ptr->emplace_back(buffer);
        }

        break;
      }
    }
    int current_phase = 0;
    {
      const auto & data = scan_data_ptr->back();
      current_phase = (data[azimuth_index_] & 0xff) | ((data[azimuth_index_ + 1] & 0xff) << 8);
      current_phase = (static_cast<int>(current_phase) + 36000 - scan_phase) % 36000;
    }
    if (current_phase >= prev_phase || scan_data_ptr->size() < 2) {
      prev_phase = current_phase;
    } else {
      // scan complete
      break;
    }
  }  // finish scan
  // scan formed, send it back through the registered callback
  if (scan_reception_callback_) {
    scan_reception_callback_(std::move(scan_data_ptr));
  }
}
Status HesaiHwInterface::CloudInterfaceStop() { return Status::ERROR_1; }

Status HesaiHwInterface::GetSensorConfiguration(SensorConfigurationBase & sensor_configuration)
{
  return Status::ERROR_1;
}

Status HesaiHwInterface::GetCalibrationConfiguration(
  CalibrationConfigurationBase & calibration_configuration)
{
  return Status::ERROR_1;
}

}  // namespace drivers
}  // namespace nebula
