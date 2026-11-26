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

// Stand-in for a Nebula ROS wrapper: a component that publishes point clouds without hardware.

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <chrono>
#include <stdexcept>
#include <string>

namespace nebula::ros2_control::test
{

class TestDriver : public rclcpp::Node
{
public:
  explicit TestDriver(const rclcpp::NodeOptions & options) : rclcpp::Node("test_driver", options)
  {
    if (declare_parameter<bool>("fail_on_start", false)) {
      throw std::runtime_error("sensor unreachable");
    }
    frame_id_ = declare_parameter<std::string>("frame_id", "default_frame");
    stamp_sec_ = declare_parameter<int>("stamp_sec", 0);
    const auto rate_hz = declare_parameter<double>("rate_hz", 50.0);

    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("points", rclcpp::SensorDataQoS());
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate_hz), [this]() {
      sensor_msgs::msg::PointCloud2 cloud;
      cloud.header.frame_id = frame_id_;
      cloud.header.stamp.sec = stamp_sec_;
      cloud.header.stamp.nanosec = 500000000U;
      publisher_->publish(cloud);
    });
  }

private:
  std::string frame_id_;
  int stamp_sec_{0};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace nebula::ros2_control::test

RCLCPP_COMPONENTS_REGISTER_NODE(nebula::ros2_control::test::TestDriver)
