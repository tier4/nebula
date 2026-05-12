// Copyright 2024 TIER IV, Inc.
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

#pragma once

#include <rosbag2_storage/serialized_bag_message.hpp>

#include <cstdint>

namespace nebula::ros
{

#ifdef ROS_DISTRO_HUMBLE
uint64_t get_timestamp_ns(const rosbag2_storage::SerializedBagMessage & bag_message)
{
  return bag_message.time_stamp;
}
#else
uint64_t get_timestamp_ns(const rosbag2_storage::SerializedBagMessage & bag_message)
{
  return bag_message.recv_timestamp;
}
#endif

}  // namespace nebula::ros
