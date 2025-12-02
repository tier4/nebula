cmake_minimum_required(VERSION 3.8)
project(robosense_msgs)

# find dependencies
find_package(autoware_cmake REQUIRED)
autoware_package()

rosidl_generate_interfaces(
  ${PROJECT_NAME} "msg/RobosensePacket.msg" "msg/RobosenseInfoPacket.msg"
  "msg/RobosenseScan.msg" DEPENDENCIES std_msgs)

ament_package()
