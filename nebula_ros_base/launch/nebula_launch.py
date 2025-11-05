# Copyright 2024 TIER IV, Inc.
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

import os

import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

SENSOR_MODELS_VELODYNE = ["VLP16", "VLP32", "VLS128"]
SENSOR_MODELS_HESAI = [
    "Pandar40P",
    "Pandar64",
    "Pandar128E4X",
    "PandarAT128",
    "PandarQT64",
    "PandarQT128",
    "PandarXT16",
    "PandarXT32",
    "PandarXT32M",
]
SENSOR_MODELS_ROBOSENSE = ["Bpearl", "Helios"]
SENSOR_MODELS_CONTINENTAL = ["ARS548", "SRR520"]

SENSOR_MODELS = (
    SENSOR_MODELS_VELODYNE
    + SENSOR_MODELS_HESAI
    + SENSOR_MODELS_ROBOSENSE
    + SENSOR_MODELS_CONTINENTAL
)


def launch_setup(context: launch.LaunchContext, *args, **kwargs):
    sensor_model = LaunchConfiguration("sensor_model").perform(context)
    launch_args = {
        "sensor_model": sensor_model,
    }

    if sensor_model in SENSOR_MODELS_VELODYNE:
        vendor_launch_file = "velodyne_launch_all_hw.xml"
    elif sensor_model in SENSOR_MODELS_HESAI:
        vendor_launch_file = "hesai_launch_all_hw.xml"
    elif sensor_model in SENSOR_MODELS_ROBOSENSE:
        vendor_launch_file = "robosense_launch_all_hw.xml"
    elif sensor_model in SENSOR_MODELS_CONTINENTAL:
        vendor_launch_file = "continental_launch_all_hw.xml"
    else:
        raise KeyError(f"Sensor model {sensor_model} does not have an associated launch file")

    vendor_launch_file = os.path.join(ThisLaunchFileDir().perform(context), vendor_launch_file)
    if not os.path.isfile(vendor_launch_file):
        raise FileNotFoundError(f"Could not find launch file {vendor_launch_file}")

    # Any launch arguments not specified here are implicitly forwarded to the included launch description.
    # Thus, passing arguments like `config_file` still works, it is just not appearing here.
    # If it would be here, even if empty and not passed in `launch_arguments`,
    # it would cause the default substitution in the included launch file to fail.
    return [
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(vendor_launch_file),
            launch_arguments=launch_args.items(),
        )
    ]


def generate_launch_description():
    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                "sensor_model",
                choices=SENSOR_MODELS,
                description="The sensor model for which to launch Nebula",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
