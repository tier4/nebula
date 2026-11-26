# Copyright 2026 TIER IV, Inc.
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

"""Run the Nebula sensors declared in a URDF through ros2_control.

Starts robot_state_publisher (TF of the sensor frames) and the ros2_control controller manager,
which loads one nebula_ros2_control/NebulaSensorInterface per <ros2_control> block and runs the
corresponding Nebula driver.

    ros2 launch nebula_ros2_control nebula_ros2_control.launch.py \
        description_file:=/path/to/robot.urdf.xacro xacro_args:="lidar_model:=PandarQT128"
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_file = LaunchConfiguration("description_file")
    xacro_args = LaunchConfiguration("xacro_args")
    controller_manager_config = LaunchConfiguration("controller_manager_config")

    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    PathJoinSubstitution([FindExecutable(name="xacro")]),
                    " ",
                    description_file,
                    " ",
                    xacro_args,
                ]
            ),
            value_type=str,
        )
    }

    controller_manager_parameters = [controller_manager_config]
    if os.environ.get("ROS_DISTRO", "") == "humble":
        # Humble's controller manager reads the URDF from its parameters; later distributions
        # subscribe to the robot_description topic of robot_state_publisher instead.
        controller_manager_parameters.append(robot_description)

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "description_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("nebula_ros2_control"),
                        "urdf",
                        "examples",
                        "nebula_sensors.urdf.xacro",
                    ]
                ),
                description="URDF/xacro file declaring the Nebula sensors in <ros2_control> blocks",
            ),
            DeclareLaunchArgument(
                "xacro_args",
                default_value="",
                description="Extra 'name:=value' arguments for xacro",
            ),
            DeclareLaunchArgument(
                "controller_manager_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("nebula_ros2_control"),
                        "config",
                        "controller_manager.param.yaml",
                    ]
                ),
                description="Parameter file of the controller manager",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[robot_description],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=controller_manager_parameters,
                remappings=[("~/robot_description", "/robot_description")],
                output="screen",
            ),
        ]
    )
