# Running Nebula

## Launch with default parameters

To launch Nebula as a ROS 2 node with default parameters for your sensor model:

```bash
ros2 launch nebula nebula_launch.py sensor_model:=*sensor_model_name*
```

For example, for a Hesai Pandar40P sensor:

```bash
ros2 launch nebula nebula_launch.py sensor_model:=Pandar40P
```

You can also use vendor-specific launch files directly:

```bash
ros2 launch nebula_<vendor> <vendor>_launch_all_hw.xml sensor_model:=*sensor_model_name*
```

For example:

```bash
ros2 launch nebula_hesai hesai_launch_all_hw.xml sensor_model:=Pandar40P
```

Refer to the list of [supported sensors](supported_sensors.md) for more information on the available sensors and configuration options.

## Sensor configuration

WIP
