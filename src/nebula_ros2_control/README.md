# nebula_ros2_control

ros2_control sensor hardware interface (`nebula_ros2_control/NebulaSensorInterface`) that runs Nebula
drivers declared in the robot's URDF inside the controller manager.

```xml
<xacro:include filename="$(find nebula_ros2_control)/urdf/nebula_sensor.ros2_control.xacro"/>

<xacro:nebula_sensor_ros2_control
    name="lidar_top" plugin="HesaiRosWrapper" package="nebula_hesai" frame_id="lidar_top"
    params_file="$(find nebula_hesai)/config/Pandar64.param.yaml" monitor_topic="pandar_points">
  <params>
    <param name="ros.sensor_ip">192.168.1.201</param>
  </params>
</xacro:nebula_sensor_ros2_control>
```

```bash
ros2 launch nebula_ros2_control nebula_ros2_control.launch.xml description_file:=/path/to/robot.urdf.xacro
```

See [docs/ros2_control.md](../../docs/ros2_control.md) for all parameters, state interfaces and the
lifecycle mapping.
