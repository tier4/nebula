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

// Python bindings for the Hesai decoder. Messages cross the boundary as CDR bytes
// (rclpy.serialization on the Python side). All decoding logic stays in the decoder.

#include <builtin_interfaces/msg/time.hpp>
#include <nebula_core_common/loggers/logger.hpp>
#include <nebula_core_common/nebula_status.hpp>
#include <nebula_core_common/point_types.hpp>
#include <nebula_core_common/util/string_conversions.hpp>
#include <nebula_core_ros/point_cloud_conversions.hpp>
#include <nebula_hesai_common/hesai_common.hpp>
#include <nebula_hesai_decoders/decoders/packet_loss_detector.hpp>
#include <nebula_hesai_decoders/hesai_decoder_settings.hpp>
#include <nebula_hesai_decoders/hesai_driver.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/time.hpp>

#include <nebula_msgs/msg/nebula_packet.hpp>
#include <nebula_msgs/msg/nebula_packets.hpp>
#include <pandar_msgs/msg/pandar_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <chrono>
#include <cstdint>
#include <cstring>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace py = pybind11;

namespace
{
using nebula::drivers::HesaiCalibrationConfigurationBase;
using nebula::drivers::HesaiDriver;
using nebula::drivers::HesaiSensorConfiguration;
using nebula_msgs::msg::NebulaPacket;

template <typename MsgT>
MsgT deserialize(const py::bytes & data)
{
  const std::string_view buffer = data;
  rclcpp::SerializedMessage serialized(buffer.size());
  auto & rcl_msg = serialized.get_rcl_serialized_message();
  std::memcpy(rcl_msg.buffer, buffer.data(), buffer.size());
  rcl_msg.buffer_length = buffer.size();

  MsgT msg;
  rclcpp::Serialization<MsgT>().deserialize_message(&serialized, &msg);
  return msg;
}

template <typename MsgT>
py::bytes serialize(const MsgT & msg)
{
  rclcpp::SerializedMessage serialized;
  rclcpp::Serialization<MsgT>().serialize_message(&msg, &serialized);
  const auto & rcl_msg = serialized.get_rcl_serialized_message();
  return py::bytes(reinterpret_cast<const char *>(rcl_msg.buffer), rcl_msg.buffer_length);
}

std::chrono::nanoseconds to_nanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return std::chrono::nanoseconds(rclcpp::Time(stamp).nanoseconds());
}

builtin_interfaces::msg::Time to_stamp(double timestamp_s)
{
  return rclcpp::Time(
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(timestamp_s))
      .count());
}

/// Forwards to Python's `logging`. Only used while the GIL is held, i.e. from within calls from
/// Python.
class PythonLogger final : public nebula::drivers::loggers::Logger
{
public:
  explicit PythonLogger(std::string name)
  : name_(std::move(name)), logger_(py::module_::import("logging").attr("getLogger")(name_))
  {
  }

  void debug(const std::string & message) override { logger_.attr("debug")(message); }
  void info(const std::string & message) override { logger_.attr("info")(message); }
  void warn(const std::string & message) override { logger_.attr("warning")(message); }
  void error(const std::string & message) override { logger_.attr("error")(message); }

  std::shared_ptr<Logger> child(const std::string & name) override
  {
    return std::make_shared<PythonLogger>(name_ + "." + name);
  }

private:
  std::string name_;
  py::object logger_;
};

std::shared_ptr<const HesaiCalibrationConfigurationBase> load_calibration(
  nebula::drivers::SensorModel sensor_model, const std::string & path)
{
  auto calibration = nebula::drivers::make_calibration(sensor_model);
  if (calibration->load_from_file(path) != nebula::Status::OK) {
    throw std::invalid_argument("Could not load calibration file: " + path);
  }
  calibration->calibration_file = path;
  return calibration;
}

// Owns the decoder, and converts serialized messages to and from its C++ types.
class HesaiDecoderWrapper
{
public:
  HesaiDecoderWrapper(
    const HesaiSensorConfiguration & sensor_configuration, const std::string & calibration_file)
  : sensor_configuration_(std::make_shared<const HesaiSensorConfiguration>(sensor_configuration))
  {
    const auto validation = nebula::drivers::validate_decoder_settings(*sensor_configuration_);
    if (!validation.has_value()) {
      throw std::invalid_argument(validation.error());
    }

    calibration_ = load_calibration(sensor_configuration_->sensor_model, calibration_file);
    logger_ = std::make_shared<PythonLogger>("nebula_py.hesai");
    reset();
  }

  /// Discard all state, as if the decoder had just been created.
  void reset()
  {
    results_ = py::list();
    errors_.clear();
    last_stamp_.reset();
    lost_packets_.reset();

    nebula::drivers::PacketLossDetectorBase::lost_cb_t lost_cb;
    if (nebula::drivers::supports_packet_loss_detection(sensor_configuration_->sensor_model)) {
      lost_packets_ = 0;
      lost_cb = [this](uint64_t n_lost) { *lost_packets_ += n_lost; };
    }

    driver_.emplace(
      sensor_configuration_, calibration_, logger_,
      [this](const nebula::drivers::NebulaPointCloudPtr & pointcloud, double timestamp_s) {
        on_pointcloud(pointcloud, timestamp_s);
      },
      nullptr, nullptr, nullptr, std::move(lost_cb));

    if (const auto status = driver_->get_status(); status != nebula::Status::OK) {
      throw std::runtime_error("Error instantiating decoder: " + nebula::util::to_string(status));
    }
  }

  py::tuple decode_pandar_scan(const py::bytes & scan)
  {
    const auto msg = deserialize<pandar_msgs::msg::PandarScan>(scan);

    // As in the ROS wrapper, the whole buffer is passed on. The decoder checks the size itself.
    std::vector<NebulaPacket> packets(msg.packets.size());
    for (size_t i = 0; i < msg.packets.size(); ++i) {
      packets[i].stamp = msg.packets[i].stamp;
      packets[i].data.assign(msg.packets[i].data.begin(), msg.packets[i].data.end());
    }
    return decode(packets);
  }

  py::tuple decode_nebula_packets(const py::bytes & packets)
  {
    return decode(deserialize<nebula_msgs::msg::NebulaPackets>(packets).packets);
  }

  py::tuple decode_nebula_packet(const py::bytes & packet)
  {
    return decode({deserialize<NebulaPacket>(packet)});
  }

private:
  py::tuple decode(const std::vector<NebulaPacket> & packets)
  {
    check_stamps(packets);
    for (const auto & packet : packets) {
      last_stamp_ = to_nanoseconds(packet.stamp);

      const auto result = driver_->parse_cloud_packet(packet.data);
      if (!result.metadata_or_error.has_value()) {
        ++errors_[static_cast<int>(result.metadata_or_error.error())];
      }
    }
    return take_result();
  }

  /// Reject the whole batch before any of it is processed, so that a rejected call leaves the
  /// decoder as it was.
  void check_stamps(const std::vector<NebulaPacket> & packets) const
  {
    auto previous = last_stamp_;
    for (const auto & packet : packets) {
      const auto stamp = to_nanoseconds(packet.stamp);
      if (previous && stamp < previous.value()) {
        throw std::invalid_argument(
          "Packet timestamps went backwards from " + std::to_string(previous->count()) + " ns to " +
          std::to_string(stamp.count()) +
          " ns. Feed packets in the order they were received, or call reset() to start over.");
      }
      previous = stamp;
    }
  }

  /// Packet loss is detected before the packet's points are decoded, so losses that a packet
  /// reveals count towards the scan that it completes.
  void on_pointcloud(const nebula::drivers::NebulaPointCloudPtr & pointcloud, double timestamp_s)
  {
    auto msg = nebula::ros::to_ros_msg(*pointcloud);
    msg.header.frame_id = sensor_configuration_->frame_id;
    msg.header.stamp = to_stamp(timestamp_s);

    results_.append(py::make_tuple(serialize(msg), lost_packets_));
    if (lost_packets_) {
      lost_packets_ = 0;
    }
  }

  py::tuple take_result()
  {
    auto result = py::make_tuple(std::move(results_), errors_);
    results_ = py::list();
    errors_.clear();
    return result;
  }

  std::shared_ptr<const HesaiSensorConfiguration> sensor_configuration_;
  std::shared_ptr<const HesaiCalibrationConfigurationBase> calibration_;
  std::shared_ptr<PythonLogger> logger_;

  std::optional<std::chrono::nanoseconds> last_stamp_;
  /// The packets lost since the last scan. Unset for sensor models without packet loss detection.
  std::optional<uint64_t> lost_packets_;
  py::list results_;
  std::map<int, uint64_t> errors_;
  /// Declared last, since its callbacks use the members above
  std::optional<HesaiDriver> driver_;
};
}  // namespace

PYBIND11_MODULE(_hesai_pybind, m)
{
  m.doc() = "Python bindings for the Nebula Hesai decoder.";

  // Only the settings that decoding uses are exposed. They are filled from the parameters on the
  // Python side, and checked by the decoder wrapper.
  py::class_<HesaiSensorConfiguration>(m, "HesaiSensorConfiguration")
    .def(py::init<>())
    .def_readwrite("frame_id", &HesaiSensorConfiguration::frame_id)
    .def_property(
      "sensor_model",
      [](const HesaiSensorConfiguration & config) {
        return nebula::drivers::sensor_model_to_string(config.sensor_model);
      },
      [](HesaiSensorConfiguration & config, const std::string & sensor_model) {
        config.sensor_model = nebula::drivers::sensor_model_from_string(sensor_model);
      })
    .def_readwrite("min_range", &HesaiSensorConfiguration::min_range)
    .def_readwrite("max_range", &HesaiSensorConfiguration::max_range)
    .def_readwrite(
      "dual_return_distance_threshold", &HesaiSensorConfiguration::dual_return_distance_threshold)
    .def_readwrite("rotation_speed", &HesaiSensorConfiguration::rotation_speed)
    .def_readwrite("cloud_min_angle", &HesaiSensorConfiguration::cloud_min_angle)
    .def_readwrite("cloud_max_angle", &HesaiSensorConfiguration::cloud_max_angle)
    .def_readwrite("cut_angle", &HesaiSensorConfiguration::cut_angle)
    .def_readwrite("downsample_mask_path", &HesaiSensorConfiguration::downsample_mask_path);

  m.def("calibration_parameter_name", [](const HesaiSensorConfiguration & config) {
    return nebula::drivers::calibration_parameter_name(config.sensor_model);
  });

  py::class_<HesaiDecoderWrapper>(m, "HesaiDecoder")
    .def(
      py::init<const HesaiSensorConfiguration &, const std::string &>(),
      py::arg("sensor_configuration"), py::arg("calibration_file"))
    .def("reset", &HesaiDecoderWrapper::reset)
    .def("decode_pandar_scan", &HesaiDecoderWrapper::decode_pandar_scan, py::arg("scan"))
    .def("decode_nebula_packets", &HesaiDecoderWrapper::decode_nebula_packets, py::arg("packets"))
    .def("decode_nebula_packet", &HesaiDecoderWrapper::decode_nebula_packet, py::arg("packet"));
}
