# Nebula

[![build-and-test](https://github.com/tier4/nebula/actions/workflows/build-and-test.yaml/badge.svg)](https://github.com/tier4/nebula/actions/workflows/build-and-test.yaml)
[![documentation](https://github.com/tier4/nebula/actions/workflows/documentation.yml/badge.svg)](https://github.com/tier4/nebula/actions/workflows/documentation.yml)
[![codecov](https://codecov.io/gh/tier4/nebula/branch/main/graph/badge.svg)](https://codecov.io/gh/tier4/nebula)

## Welcome to Nebula, the universal sensor driver

Nebula is a sensor driver platform that is designed to provide a unified framework for as wide a variety of devices as possible.
While it primarily targets Ethernet-based LiDAR sensors, it aims to be easily extendable to support new sensors and interfaces.
Nebula works with ROS 2 and is the recommended sensor driver for the [Autoware](https://autoware.org/) project.

## Documentation

We recommend you get started with the [Nebula Documention](https://tier4.github.io/nebula/).
Here you will find information about the background of the project, how to install and use with ROS 2, and also how to add new sensors to the Nebula driver.

- [About Nebula](https://tier4.github.io/nebula/about)
- [Design](https://tier4.github.io/nebula/design)
- [Supported Sensors](https://tier4.github.io/nebula/supported_sensors)
- [Installation](https://tier4.github.io/nebula/installation)
- [Launching with ROS 2](https://tier4.github.io/nebula/usage)
- [Parameters](https://tier4.github.io/nebula/parameters)
- [Point cloud types](https://tier4.github.io/nebula/point_types)
- [Contributing](https://tier4.github.io/nebula/contribute)
- [Tutorials](https://tier4.github.io/nebula/tutorials)

## Quick start

Nebula builds with ROS 2 Galactic and Humble.

> **Note**
>
> Boost version 1.74.0 or later is required. A manual install may be required in Ubuntu versions earlier than 22.04.
>
> **Note**
>
> A [TCP enabled version of ROS' Transport Driver](https://github.com/mojomex/transport_drivers/tree/mutable-buffer-in-udp-callback) is required to use Nebula.
> It is installed automatically into your workspace using the below commands. However, if you already have ROS transport driver binaries installed, you will have to uninstall them to avoid conflicts (replace `humble` with your ROS distribution):
> `sudo apt remove ros-humble-udp-driver ros-humble-io-context`

To build Nebula run the following commands in your workspace:

```bash
# In workspace
mkdir src
git clone https://github.com/tier4/nebula.git src
# Import dependencies
vcs import src < src/build_depends.repos
rosdep install --from-paths src --ignore-src -y -r
# Build Nebula
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

To launch Nebula as a ROS 2 node with default parameters for your sensor model:

```bash
ros2 launch nebula_ros *sensor_vendor_name*_launch_all_hw.xml sensor_model:=*sensor_model_name*
```

For example, for a Hesai Pandar40P sensor:

```bash
ros2 launch nebula_ros hesai_launch_all_hw.xml sensor_model:=Pandar40P
```
