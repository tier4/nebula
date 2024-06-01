#include "nebula_hw_interfaces/nebula_hw_interfaces_tutorial/tutorial_hw_interface.hpp"

#include <boost/asio.hpp>

namespace nebula
{
namespace drivers
{
TutorialHwInterface::TutorialHwInterface(
  std::shared_ptr<loggers::Logger> logger,
  const std::shared_ptr<const TutorialSensorConfiguration> & sensor_configuration)
: logger_(logger),
  ptc_connection_(
    logger->child("PTC"), sensor_configuration->host_ip, TCP_PORT, sensor_configuration->sensor_ip,
    TCP_PORT),
  udp_receiver_(
    logger->child("UDP"), sensor_configuration->host_ip, sensor_configuration->data_port,
    std::bind(&TutorialHwInterface::onSensorPacket, this, std::placeholders::_1))
{
}

TutorialConfig TutorialHwInterface::getConfig()
{
  return ptc_connection_.get<TutorialConfig>(PTC_COMMAND_GET_CONFIG_INFO);
}

Status TutorialHwInterface::setSpinRate(uint16_t rpm)
{
  std::vector<unsigned char> request_payload;
  request_payload.emplace_back((rpm >> 8) & 0xff);
  request_payload.emplace_back(rpm & 0xff);

  ptc_connection_.set(PTC_COMMAND_SET_SPIN_RATE, request_payload);
  return Status::OK;
}

Status TutorialHwInterface::setSyncAngle(int sync_angle, int angle)
{
  std::vector<unsigned char> request_payload;
  request_payload.emplace_back(sync_angle & 0xff);
  request_payload.emplace_back((angle >> 8) & 0xff);
  request_payload.emplace_back(angle & 0xff);

  ptc_connection_.set(PTC_COMMAND_SET_SYNC_ANGLE, request_payload);
  return Status::OK;
}

Status TutorialHwInterface::setReturnMode(int return_mode)
{
  std::vector<unsigned char> request_payload;
  request_payload.emplace_back(return_mode & 0xff);

  ptc_connection_.set(PTC_COMMAND_SET_RETURN_MODE, request_payload);
  return Status::OK;
}

Status TutorialHwInterface::compareAndSendConfig(const TutorialSensorConfiguration & updated)
{
  using namespace std::chrono_literals;  // NOLINT(build/namespaces)

  auto current = getConfig();
  logger_->info((std::stringstream() << "Current sensor config: " << current).str());

  auto current_return_mode = returnModeFromInt(current.return_mode, updated.sensor_model);
  if (updated.return_mode != current_return_mode) {
    std::stringstream ss;
    ss << current_return_mode;
    logger_->info("Current LiDAR return_mode: " + ss.str());
    std::stringstream ss2;
    ss2 << updated.return_mode;
    logger_->info("Current Configuration return_mode: " + ss2.str());
    auto return_mode_int = intFromReturnMode(updated.return_mode, updated.sensor_model);
    if (return_mode_int < 0) {
      logger_->error(
        "Invalid Return Mode for this sensor. Please check your settings. Falling back to Dual "
        "mode.");
      return_mode_int = 2;
    }
    setReturnMode(return_mode_int);
  }

  auto current_rotation_speed = current.spin_rate;
  if (updated.rotation_speed != current_rotation_speed.value()) {
    logger_->info(
      "current lidar rotation_speed: " +
      std::to_string(static_cast<int>(current_rotation_speed.value())));
    logger_->info(
      "current configuration rotation_speed: " + std::to_string(updated.rotation_speed));
    logger_->info("Setting up spin rate via TCP." + std::to_string(updated.rotation_speed));
    setSpinRate(updated.rotation_speed);
  }

  auto sync_angle = static_cast<int>(current.sync_angle.value() / 100);
  auto scan_phase = static_cast<int>(updated.scan_phase);
  if (scan_phase != sync_angle) {
    logger_->info("current lidar sync: " + std::to_string(current.sync));
    logger_->info("current lidar sync_angle: " + std::to_string(sync_angle));
    logger_->info("current configuration scan_phase: " + std::to_string(scan_phase));
    setSyncAngle(1, scan_phase);
  }

  return Status::OK;
}

}  // namespace drivers
}  // namespace nebula
