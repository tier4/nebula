// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "converters.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nebula_core_common/nebula_common.hpp>
#include <nebula_core_ros/rclcpp_logger.hpp>
#include <nebula_hesai_common/hesai_common.hpp>
#include <nebula_hesai_decoders/hesai_driver.hpp>
#include <nebula_robosense_common/robosense_common.hpp>
#include <nebula_robosense_decoders/robosense_driver.hpp>
#include <nebula_robosense_decoders/robosense_info_driver.hpp>
#include <nebula_velodyne_common/velodyne_common.hpp>
#include <nebula_velodyne_decoders/velodyne_driver.hpp>

#include <filesystem>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace nebula::cli
{
namespace
{

template <typename T>
std::string status_message(const T & status)
{
  std::ostringstream ss;
  ss << status;
  return ss.str();
}

/// @brief Calibration path from `key`, or `<decoders share>/calibration/<model><ext>` if unset
std::string calibration_path(
  const Params & p, const std::string & key, const std::string & decoders_pkg,
  const std::string & model, const std::string & ext)
{
  auto path = p.get_string(
    key,
    ament_index_cpp::get_package_share_directory(decoders_pkg) + "/calibration/" + model + ext);
  if (!std::filesystem::exists(path)) {
    throw std::runtime_error("Calibration file not found: " + path + " (set --" + key + ")");
  }
  return path;
}

/// @brief Fill the fields shared by all LiDAR configs
template <typename Config>
void fill_lidar_config(Config & c, const Params & p)
{
  c.sensor_model = drivers::sensor_model_from_string(p.get_string("sensor_model", ""));
  if (c.sensor_model == drivers::SensorModel::UNKNOWN) {
    throw std::runtime_error("Unknown or missing sensor_model (use --model)");
  }
  c.frame_id = p.get_string("frame_id", "lidar");
  c.min_range = p.get_double("min_range", 0.3);
  c.max_range = p.get_double("max_range", 300.0);
  c.packet_mtu_size = static_cast<uint16_t>(p.get_int("packet_mtu_size", 1500));
}

std::shared_ptr<drivers::loggers::Logger> make_logger()
{
  return std::make_shared<drivers::loggers::RclcppLogger>("nebula_cli");
}

class HesaiConverter : public Converter
{
public:
  HesaiConverter(const Params & p, CloudCallback cb)
  {
    drivers::HesaiSensorConfiguration cfg{};
    fill_lidar_config(cfg, p);
    cfg.return_mode = drivers::return_mode_from_string_hesai(
      p.get_string("return_mode", ""), cfg.sensor_model);
    if (cfg.return_mode == drivers::ReturnMode::UNKNOWN) {
      throw std::runtime_error("Unknown or missing return_mode (use --return-mode)");
    }
    cfg.sync_angle = static_cast<uint16_t>(p.get_int("sync_angle", 0));
    cfg.cut_angle = p.get_double("cut_angle", 0.0);
    cfg.dual_return_distance_threshold = p.get_double("dual_return_distance_threshold", 0.1);
    cfg.rotation_speed = static_cast<uint16_t>(p.get_int("rotation_speed", 600));
    cfg.cloud_min_angle = static_cast<uint16_t>(p.get_int("cloud_min_angle", 0));
    cfg.cloud_max_angle = static_cast<uint16_t>(p.get_int("cloud_max_angle", 360));
    cfg.hires_mode = p.get_string("hires_mode", "false") == "true";

    // AT128 uses a correction file instead of a calibration file
    const bool is_at128 = cfg.sensor_model == drivers::SensorModel::HESAI_PANDARAT128;
    const auto model = p.get_string("sensor_model", "");
    std::shared_ptr<drivers::HesaiCalibrationConfigurationBase> calib;
    if (is_at128) {
      calib = std::make_shared<drivers::HesaiCorrection>();
      cfg.calibration_path =
        calibration_path(p, "correction_file", "nebula_hesai_decoders", model, ".dat");
    } else {
      calib = std::make_shared<drivers::HesaiCalibrationConfiguration>();
      cfg.calibration_path =
        calibration_path(p, "calibration_file", "nebula_hesai_decoders", model, ".csv");
    }
    const auto status = calib->load_from_file(cfg.calibration_path);
    if (status != Status::OK) {
      throw std::runtime_error(
        "Could not load " + cfg.calibration_path + ": " + status_message(status));
    }
    calib->calibration_file = cfg.calibration_path;

    driver_ = std::make_shared<drivers::HesaiDriver>(
      std::make_shared<const drivers::HesaiSensorConfiguration>(cfg), calib, make_logger(),
      std::move(cb));
    if (driver_->get_status() != Status::OK) {
      throw std::runtime_error(
        "Hesai driver init failed: " + status_message(driver_->get_status()));
    }
  }

  void feed_scan(const Packet & packet) override
  {
    if (!driver_->parse_cloud_packet(packet.data).metadata_or_error.has_value()) ++dropped_;
  }
  uint64_t dropped_packets() const override { return dropped_; }

private:
  std::shared_ptr<drivers::HesaiDriver> driver_;
  uint64_t dropped_{0};
};

class VelodyneConverter : public Converter
{
public:
  VelodyneConverter(const Params & p, CloudCallback cb) : cb_(std::move(cb))
  {
    drivers::VelodyneSensorConfiguration cfg{};
    fill_lidar_config(cfg, p);
    // Accept both the short (Strongest, Last) and the long (SingleStrongest, SingleLast) names
    const auto mode_name = p.get_string("return_mode", "");
    cfg.return_mode = drivers::return_mode_from_string_velodyne(mode_name);
    if (cfg.return_mode == drivers::ReturnMode::UNKNOWN) {
      cfg.return_mode = drivers::return_mode_from_string(mode_name);
    }
    if (cfg.return_mode == drivers::ReturnMode::UNKNOWN) {
      throw std::runtime_error("Unknown or missing return_mode (use --return-mode)");
    }
    cfg.scan_phase = p.get_double("scan_phase", 0.0);
    cfg.rotation_speed = static_cast<uint16_t>(p.get_int("rotation_speed", 600));
    cfg.cloud_min_angle = static_cast<uint16_t>(p.get_int("cloud_min_angle", 0));
    cfg.cloud_max_angle = static_cast<uint16_t>(p.get_int("cloud_max_angle", 360));

    auto calib = std::make_shared<drivers::VelodyneCalibrationConfiguration>();
    const auto path = calibration_path(
      p, "calibration_file", "nebula_velodyne_decoders", p.get_string("sensor_model", ""), ".yaml");
    const auto status = calib->load_from_file(path);
    if (status != Status::OK) {
      throw std::runtime_error("Could not load " + path + ": " + status_message(status));
    }
    calib->calibration_file = path;

    driver_ = std::make_shared<drivers::VelodyneDriver>(
      std::make_shared<const drivers::VelodyneSensorConfiguration>(cfg), calib, make_logger());
    if (driver_->get_status() != Status::OK) {
      throw std::runtime_error(
        "Velodyne driver init failed: " + status_message(driver_->get_status()));
    }
  }

  void feed_scan(const Packet & packet) override
  {
    auto [cloud, timestamp_s] = driver_->parse_cloud_packet(packet.data, packet.stamp_ns * 1e-9);
    if (cloud) cb_(cloud, timestamp_s);
  }
  uint64_t dropped_packets() const override { return 0; }

private:
  std::shared_ptr<drivers::VelodyneDriver> driver_;
  CloudCallback cb_;
};

class RobosenseConverter : public Converter
{
public:
  RobosenseConverter(const Params & p, CloudCallback cb) : cb_(std::move(cb))
  {
    drivers::RobosenseSensorConfiguration cfg{};
    fill_lidar_config(cfg, p);
    cfg.return_mode = drivers::return_mode_from_string_robosense(p.get_string("return_mode", ""));
    cfg.scan_phase = p.get_double("scan_phase", 0.0);
    cfg.dual_return_distance_threshold = p.get_double("dual_return_distance_threshold", 0.1);
    cfg_ = std::make_shared<drivers::RobosenseSensorConfiguration>(cfg);
    info_driver_.emplace(cfg_, make_logger());
  }

  /// Robosense sensors send their calibration and return mode in info (DIFOP) packets.
  /// The scan driver is created from the first valid one.
  void feed_info(const Packet & packet) override
  {
    if (driver_) return;
    if (info_driver_->decode_info_packet(packet.data) != Status::OK) return;

    auto cfg = *cfg_;
    cfg.return_mode = info_driver_->get_return_mode();
    cfg.use_sensor_time = info_driver_->get_sync_status();
    auto calib = info_driver_->get_sensor_calibration();
    calib.create_corrected_channels();
    driver_ = std::make_shared<drivers::RobosenseDriver>(
      std::make_shared<const drivers::RobosenseSensorConfiguration>(cfg),
      std::make_shared<const drivers::RobosenseCalibrationConfiguration>(std::move(calib)),
      make_logger());
    if (driver_->get_status() != Status::OK) {
      throw std::runtime_error(
        "Robosense driver init failed: " + status_message(driver_->get_status()));
    }
  }

  void feed_scan(const Packet & packet) override
  {
    if (!driver_) {
      ++dropped_;  // no info packet seen yet, so no calibration
      return;
    }
    auto [cloud, timestamp_s] = driver_->parse_cloud_packet(packet.data);
    if (cloud) cb_(cloud, timestamp_s);
  }
  uint64_t dropped_packets() const override { return dropped_; }

private:
  std::shared_ptr<drivers::RobosenseSensorConfiguration> cfg_;
  std::optional<drivers::RobosenseInfoDriver> info_driver_;
  std::shared_ptr<drivers::RobosenseDriver> driver_;
  CloudCallback cb_;
  uint64_t dropped_{0};
};

}  // namespace

std::unique_ptr<Converter> make_converter(
  const std::string & vendor, const Params & params, Converter::CloudCallback callback)
{
  if (vendor == "hesai") return std::make_unique<HesaiConverter>(params, std::move(callback));
  if (vendor == "velodyne") return std::make_unique<VelodyneConverter>(params, std::move(callback));
  if (vendor == "robosense") {
    return std::make_unique<RobosenseConverter>(params, std::move(callback));
  }
  throw std::runtime_error("Unknown vendor '" + vendor + "' (hesai, velodyne, robosense)");
}

}  // namespace nebula::cli
