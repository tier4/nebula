import os
import warnings

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import OpaqueFunction
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
import yaml


def get_lidar_make(sensor_name):
    if sensor_name[:6].lower() == "pandar":
        return "Hesai", ".csv"
    elif sensor_name[:3].lower() in ["hdl", "vlp", "vls"]:
        return "Velodyne", ".yaml"
    return "unrecognized_sensor_model"


def launch_setup(context, *args, **kwargs):
    # Model and make
    sensor_model = LaunchConfiguration("sensor_model").perform(context)
    sensor_make, sensor_extension = get_lidar_make(sensor_model)
    nebula_decoders_share_dir = get_package_share_directory("nebula_decoders")
    nebula_ros_share_dir = get_package_share_directory("nebula_ros")

    # Config
    sensor_params_fp = LaunchConfiguration("config_file").perform(context)
    if sensor_params_fp == "":
        warnings.warn("No config file provided, using sensor model default", RuntimeWarning)
        sensor_params_fp = os.path.join(nebula_ros_share_dir, "config", sensor_make.lower(), sensor_model + ".yaml")
    sensor_calib_fp = os.path.join(nebula_decoders_share_dir, "calibration", sensor_make.lower(), sensor_model + sensor_extension)
    if not os.path.exists(sensor_params_fp):
        sensor_params_fp = os.path.join(nebula_ros_share_dir, "config", "BaseParams.yaml")
    assert os.path.exists(sensor_params_fp), "Sensor params yaml file under config/ was not found: {}".format(sensor_params_fp)
    assert os.path.exists(sensor_calib_fp), "Sensor calib file under calibration/ was not found: {}".format(sensor_calib_fp)
    with open(sensor_params_fp, "r") as f:
            sensor_params = yaml.safe_load(f)["/**"]["ros__parameters"]
    nodes = []
    if LaunchConfiguration("launch_hw").perform(context) == "true":
        nodes.append(
            # HwInterface
            ComposableNode(
                package="nebula_ros",
                plugin=sensor_make+"HwInterfaceRosWrapper",
                name=sensor_make.lower()+"_hw_interface_ros_wrapper_node",
                parameters=[
                    sensor_params,
                    {
                        "sensor_model": LaunchConfiguration("sensor_model"),
                        "sensor_ip": LaunchConfiguration("sensor_ip"),
                        "return_mode": LaunchConfiguration("return_mode"),
                        "calibration_file": sensor_calib_fp,
                        "setup_sensor": LaunchConfiguration("setup_sensor"),
                    },
                ],
            ),
        )
        nodes.append(
            # HwMonitor
            ComposableNode(
                package="nebula_ros",
                plugin=sensor_make+"HwMonitorRosWrapper",
                name=sensor_make.lower()+"_hw_monitor_ros_wrapper_node",
                parameters=[
                    sensor_params,
                    {
                        "sensor_model": sensor_model,
                        "sensor_ip": LaunchConfiguration("sensor_ip"),
                        "return_mode": LaunchConfiguration("return_mode"),
                        "calibration_file": sensor_calib_fp,
                    },
                ],
            )
        )
    nodes.append(
        ComposableNode(
            package="nebula_ros",
            plugin=sensor_make+"DriverRosWrapper",
            name=sensor_make.lower()+"_driver_ros_wrapper_node",
            parameters=[
                sensor_params,
                {
                    "sensor_model": sensor_model,
                    "sensor_ip": LaunchConfiguration("sensor_ip"),
                    "return_mode": LaunchConfiguration("return_mode"),
                    "calibration_file": sensor_calib_fp,
                },
            ],
        ),
    )

    loader = LoadComposableNodes(
        condition=LaunchConfigurationNotEquals("container", ""),
        composable_node_descriptions=nodes,
        target_container=LaunchConfiguration("container"),
    )

    container_kwargs = {}
    if LaunchConfiguration("debug_logging").perform(context) == "true":
        container_kwargs["ros_arguments"] = ['--log-level', 'debug']
    
    container = ComposableNodeContainer(
        name="nebula_ros_node",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=nodes,
        output="screen",
        condition=LaunchConfigurationEquals("container", ""),
        **container_kwargs,
    )

    group = GroupAction(
        [
            container,
            loader,
        ]
    )

    return [group]


def generate_launch_description():
    def add_launch_arg(name: str, default_value=None):
        return DeclareLaunchArgument(name, default_value=default_value)

    return launch.LaunchDescription(
        [
            add_launch_arg("container", ""),
            add_launch_arg("config_file", ""),
            add_launch_arg("sensor_model", ""),
            add_launch_arg("sensor_ip", "192.168.1.201"),
            add_launch_arg("return_mode", "Dual"),
            add_launch_arg("launch_hw", "true"),
            add_launch_arg("setup_sensor", "true"),
            add_launch_arg("debug_logging", "false"),
        ]
        + [OpaqueFunction(function=launch_setup)]
    )
