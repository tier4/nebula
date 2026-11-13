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

#pragma once

#include <diagnostic_updater/diagnostic_status_wrapper.hpp>
#include <nebula_core_common/diagnostics/diagnostic_task.hpp>

namespace nebula::ros
{

/**
 * @brief Register a ROS-independent diagnostic task with a diagnostic updater.
 *
 * @param updater A `diagnostic_updater::Updater` or `agnocast_wrapper::diagnostic_updater::Updater`
 * @param task The task to run on every update. It has to outlive `updater`.
 */
template <typename UpdaterT>
void add_diagnostic_task(UpdaterT & updater, diagnostics::DiagnosticTask & task)
{
  updater.add(task.name(), [&task](diagnostic_updater::DiagnosticStatusWrapper & status) {
    task.run(status);
  });
}

}  // namespace nebula::ros
