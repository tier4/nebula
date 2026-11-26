# Running Nebula through ros2_control

The `nebula_ros2_control` package provides `nebula_ros2_control/NebulaSensorInterface`, a
[ros2_control](https://control.ros.org) sensor hardware interface that runs any Nebula driver
inside the controller manager. Sensors are declared in the robot's URDF next to the links they are
mounted on, instead of in a separate launch file per sensor.

The hardware interface loads the existing Nebula ROS wrapper (`HesaiRosWrapper`,
`VelodyneRosWrapper`, ...) as a component, so topics, parameters, diagnostics and services are
exactly the same as when the driver is launched on its own.

## Building

`nebula_ros2_control` is not part of the `nebula` meta-package, so ros2_control stays an optional
dependency. Build it together with the vendor packages you need:

```bash
colcon build --symlink-install --packages-up-to nebula_ros2_control nebula_hesai
```

## Declaring a sensor in URDF

Use the xacro macro shipped with the package:

```xml
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:include filename="$(find nebula_ros2_control)/urdf/nebula_sensor.ros2_control.xacro"/>

  <link name="base_link"/>
  <link name="lidar_top"/>
  <joint name="base_link_to_lidar_top" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_top"/>
    <origin xyz="0 0 1.8" rpy="0 0 0"/>
  </joint>

  <xacro:nebula_sensor_ros2_control
      name="lidar_top"
      plugin="HesaiRosWrapper"
      package="nebula_hesai"
      frame_id="lidar_top"
      namespace="sensing/lidar/top"
      params_file="$(find nebula_hesai)/config/Pandar64.param.yaml"
      monitor_topic="pandar_points">
    <params>
      <param name="ros.sensor_model">Pandar64</param>
      <param name="ros.sensor_ip">192.168.1.201</param>
    </params>
  </xacro:nebula_sensor_ros2_control>
</robot>
```

which expands to the plain ros2_control description below. Writing it by hand works just as well:

```xml
<ros2_control name="lidar_top" type="sensor">
  <hardware>
    <plugin>nebula_ros2_control/NebulaSensorInterface</plugin>
    <param name="plugin">HesaiRosWrapper</param>
    <param name="package">nebula_hesai</param>
    <param name="params_file">/opt/ros/.../nebula_hesai/config/Pandar64.param.yaml</param>
    <param name="namespace">sensing/lidar/top</param>
    <param name="monitor_topic">pandar_points</param>
    <param name="monitor_topic_type">sensor_msgs/msg/PointCloud2</param>
    <param name="executor_threads">1</param>
    <param name="ros.sensor_model">Pandar64</param>
    <param name="ros.sensor_ip">192.168.1.201</param>
  </hardware>
  <sensor name="lidar_top">
    <param name="frame_id">lidar_top</param>
    <state_interface name="driver_active"/>
    <state_interface name="message_count"/>
    <state_interface name="message_rate"/>
    <state_interface name="message_age"/>
    <state_interface name="header_stamp"/>
  </sensor>
</ros2_control>
```

Declare one `<ros2_control>` block per sensor.

### Hardware parameters

| Parameter                 | Required | Description                                                                                                                             |
| ------------------------- | -------- | --------------------------------------------------------------------------------------------------------------------------------------- |
| `plugin`                  | yes      | Driver component, as listed by `ros2 component types`. See the table below.                                                             |
| `package`                 | no       | Package registering `plugin`. Without it, all packages are searched.                                                                    |
| `library`                 | no       | Shared library providing `plugin`, bypassing the lookup. Rarely needed.                                                                 |
| `node_name`               | no       | Name of the driver node. Default: `<ros2_control name>_driver`.                                                                         |
| `namespace`               | no       | Namespace of the driver node.                                                                                                           |
| `params_file`             | no       | Parameter file(s) of the driver, `;`-separated, applied in order.                                                                       |
| `allow_substs`            | no       | Resolve `$(find-pkg-share pkg)`, `$(var name)` and `$(env NAME)` in the parameter files (default `true`), like launch's `allow_substs`. |
| `remappings`              | no       | `;`-separated `from:=to` remappings of the driver node.                                                                                 |
| `use_intra_process_comms` | no       | Enable intra-process communication for the driver node (default `false`).                                                               |
| `executor_threads`        | no       | Number of threads spinning the driver node (default `1`).                                                                               |
| `monitor_topic`           | no       | Output topic of the driver (relative to `namespace`) feeding the `message_*` and `header_stamp` state interfaces.                       |
| `monitor_topic_type`      | no       | Message type of `monitor_topic` (default `sensor_msgs/msg/PointCloud2`).                                                                |
| `ros.<name>`              | no       | Sets the driver's ROS parameter `<name>`, overriding the parameter files.                                                               |

`ros.<name>` values are parsed as YAML, like `ros2 run ... -p name:=value`: write floating point
parameters with a decimal point (`10.0`, not `10`) and quote strings that look like numbers.

`$(var name)` in a parameter file resolves to the value of the `ros.<name>` parameter. This makes
the stock parameter files usable as they are, e.g.
`calibration_file: $(find-pkg-share nebula_hesai_decoders)/calibration/$(var sensor_model).csv`.

The `<sensor>` tag accepts `frame_id` (forwarded as the driver's `frame_id` parameter) and
`ros.<name>` parameters, which take precedence over those of `<hardware>`.

### Driver plugins

| Vendor      | `plugin`                                                     | `package`            |
| ----------- | ------------------------------------------------------------ | -------------------- |
| Hesai       | `HesaiRosWrapper`                                            | `nebula_hesai`       |
| Velodyne    | `VelodyneRosWrapper`                                         | `nebula_velodyne`    |
| Robosense   | `RobosenseRosWrapper`                                        | `nebula_robosense`   |
| Continental | `ContinentalARS548RosWrapper`, `ContinentalSRR520RosWrapper` | `nebula_continental` |

### State interfaces

| Interface       | Description                                                                         |
| --------------- | ----------------------------------------------------------------------------------- |
| `driver_active` | `1.0` while the driver node is running, `0.0` otherwise.                            |
| `message_count` | Messages received on `monitor_topic` since activation.                              |
| `message_rate`  | Rate of `monitor_topic` in Hz, averaged over about one second.                      |
| `message_age`   | Seconds since the last message on `monitor_topic` (NaN before the first one).       |
| `header_stamp`  | `header.stamp` of the last message on `monitor_topic`, in seconds (NaN before one). |

All interfaces except `driver_active` require `monitor_topic`. Monitoring subscribes to the topic
with a generic (serialized) subscription and reads only the header, but the message is still
delivered to the controller manager process, so only enable it if you need it.

## Lifecycle

| ros2_control transition | Effect                                                                     |
| ----------------------- | -------------------------------------------------------------------------- |
| configure               | The driver library is located and loaded.                                  |
| activate                | The driver node is created (connects to the sensor) and starts publishing. |
| deactivate              | The driver node is destroyed, releasing the sensor and its UDP/TCP ports.  |
| cleanup / shutdown      | The driver library is unloaded.                                            |

Sensors can therefore be stopped and restarted at runtime, e.g.

```bash
ros2 control set_hardware_component_state lidar_top inactive
ros2 control set_hardware_component_state lidar_top active
```

## Launching

```bash
ros2 launch nebula_ros2_control nebula_ros2_control.launch.py \
    description_file:=/path/to/my_robot.urdf.xacro
```

starts `robot_state_publisher` (publishing the sensor frames to TF) and the controller manager with
`config/controller_manager.param.yaml`. No controller is needed for the drivers to run; inspect
them with

```bash
ros2 control list_hardware_components
ros2 control list_hardware_interfaces
```

The package also contains an example URDF, `urdf/examples/nebula_sensors.urdf.xacro`, with a Hesai
LiDAR and a Continental radar, which the launch file uses by default.
