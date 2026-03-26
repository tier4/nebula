// Copyright 2026 TIER IV, Inc.

#include <nebula_seyond_common/seyond_configuration.hpp>
#include <nebula_seyond_hw_interfaces/seyond_hw_interface.hpp>

#include <iostream>
#include <string>

int main(int argc, char ** argv)
{
  nebula::drivers::SeyondSensorConfiguration config{};
  config.sensor_model = nebula::drivers::SeyondSensorModel::FALCON_K;
  config.connection.sensor_ip = argc > 1 ? argv[1] : "172.168.1.10";
  config.connection.host_ip = argc > 2 ? argv[2] : "172.168.1.100";
  config.connection.netmask = "255.255.255.0";
  config.connection.gateway = "0.0.0.0";
  config.connection.udp_port = 8010;
  config.connection.udp_message_port = 8010;
  config.connection.udp_status_port = 8010;
  config.setup_sensor = false;

  nebula::drivers::SeyondHwInterface hw_interface(config);

  auto print_attribute = [&](const std::string & name) {
    std::string response;
    auto status = hw_interface.get_attribute(name, response);
    if (status == nebula::Status::OK) {
      std::cout << name << ": " << response << "\n";
    } else {
      std::cout << name << ": <unavailable>\n";
    }
  };

  std::cout << "sensor_ip: " << config.connection.sensor_ip << "\n";
  print_attribute("udp_ports_ip");
  print_attribute("return_mode");
  print_attribute("reflectance_mode");
  print_attribute("frame_rate");
  print_attribute("v_angle_offset");

  auto calibration_or_error = hw_interface.get_calibration();
  if (calibration_or_error.has_value()) {
    std::cout << "calibration.v_angle_offset: " << calibration_or_error.value().v_angle_offset
              << "\n";
    std::cout << "calibration.angle_hv_table_bytes: "
              << calibration_or_error.value().angle_hv_table.size() << "\n";
  } else {
    std::cout << "calibration: <unavailable>\n";
  }

  return 0;
}
