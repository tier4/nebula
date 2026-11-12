# Copyright 2025 TIER IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

function(nebula_agnocast_wrapper_setup target)
  if(DEFINED ENV{ENABLE_AGNOCAST} AND "$ENV{ENABLE_AGNOCAST}" STREQUAL "1")
    target_compile_definitions(${target} PUBLIC USE_AGNOCAST_ENABLED)
  endif()
endfunction()

# Set template path for node_main_switchable
set(nebula_agnocast_wrapper_NODE_TEMPLATE
  "${nebula_core_ros_DIR}/../templates/node_main_switchable.cpp.in")
if(NOT EXISTS "${nebula_agnocast_wrapper_NODE_TEMPLATE}")
  message(FATAL_ERROR
    "nebula_agnocast_wrapper: template file not found: "
    "${nebula_agnocast_wrapper_NODE_TEMPLATE}")
endif()

# Include the register_node macro
include("${nebula_core_ros_DIR}/nebula_agnocast_wrapper_register_node.cmake")
