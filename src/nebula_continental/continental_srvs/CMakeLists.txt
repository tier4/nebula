cmake_minimum_required(VERSION 3.5)
project(continental_srvs)

find_package(autoware_cmake REQUIRED)
autoware_package()

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "srv/ContinentalArs548SetSensorMounting.srv"
  "srv/ContinentalArs548SetVehicleParameters.srv"
  "srv/ContinentalArs548SetRadarParameters.srv"
  "srv/ContinentalArs548SetNetworkConfiguration.srv"
  "srv/ContinentalSrr520SetRadarParameters.srv"
  DEPENDENCIES
  std_msgs)

ament_package()
