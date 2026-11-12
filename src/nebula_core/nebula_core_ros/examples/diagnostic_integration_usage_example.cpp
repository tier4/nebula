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

// # --8<-- [start:include]
#include "nebula_core_ros/diagnostics/diagnostic_task.hpp"
#include "nebula_core_ros/rclcpp_clock.hpp"

#include <nebula_core_common/diagnostics/liveness_monitor.hpp>
#include <nebula_core_common/diagnostics/rate_bound_status.hpp>
// # --8<-- [end:include]

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <nebula_core_common/util/clock.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>

namespace nebula::ros::examples
{

using diagnostics::LivenessMonitor;
using diagnostics::RateBoundStatus;
using diagnostics::RateBoundStatusParam;
using std::chrono_literals::operator""ms;

// # --8<-- [start:usage]
class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("diagnostic_integration_usage_example")
  {
    add_diagnostic_task(updater_, publish_rate_diag_);
    add_diagnostic_task(updater_, liveness_diag_);
  }

  void on_packet_received(const std::array<uint8_t, 1500> & packet)
  {
    liveness_diag_.tick();

    if (packet[0] == 1 /* some condition indicating we should publish */) {
      // publish something here
      publish_rate_diag_.tick();
    }
  }

private:
  diagnostic_updater::Updater updater_{this, 0.1};
  std::shared_ptr<const util::Clock> clock_{make_diagnostics_clock(*this)};
  RateBoundStatus publish_rate_diag_{
    clock_, RateBoundStatusParam{9, 11}, RateBoundStatusParam{8, 12}};
  LivenessMonitor liveness_diag_{"packet_receive", clock_, 10ms};
};
// # --8<-- [end:usage]

}  // namespace nebula::ros::examples
