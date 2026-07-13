# Nebula

[![build-and-test](https://github.com/tier4/nebula/actions/workflows/build-and-test.yaml/badge.svg)](https://github.com/tier4/nebula/actions/workflows/build-and-test.yaml)
[![documentation](https://github.com/tier4/nebula/actions/workflows/documentation.yml/badge.svg)](https://github.com/tier4/nebula/actions/workflows/documentation.yml)
[![codecov](https://codecov.io/gh/tier4/nebula/branch/main/graph/badge.svg)](https://codecov.io/gh/tier4/nebula)

## Welcome to Nebula, the universal sensor driver

Nebula is a sensor driver platform that is designed to provide a unified framework for as wide a variety of devices as possible.
While it primarily targets Ethernet-based LiDAR sensors, it aims to be easily extendable to support new sensors and interfaces.
Nebula works with ROS 2 and is the recommended sensor driver for the [Autoware](https://autoware.org/) project.

## Table of Contents

- [Nebula](#nebula)
  - [Welcome to Nebula, the universal sensor driver](#welcome-to-nebula-the-universal-sensor-driver)
  - [Table of Contents](#table-of-contents)
  - [Documentation](#documentation)
  - [Quick start](#quick-start)
  - [Agnocast](#agnocast)
  - [Building only specific vendors](#building-only-specific-vendors)
  - [Ouster](#ouster)
  - [Migration to Nebula 0.3.0](#migration-to-nebula-030)

## Documentation

We recommend you get started with the [Nebula Documention](https://tier4.github.io/nebula/).
Here you will find information about the background of the project, how to install and use with ROS 2, and also how to add new sensors to the Nebula driver.

- [Design](https://tier4.github.io/nebula/design)
- [Supported Sensors](https://tier4.github.io/nebula/supported_sensors)
- [Installation](https://tier4.github.io/nebula/installation)
- [Launching with ROS 2](https://tier4.github.io/nebula/usage)
- [Parameters](https://tier4.github.io/nebula/parameters)
- [Point cloud types](https://tier4.github.io/nebula/point_types)
- [Contributing](https://tier4.github.io/nebula/contributing)
- [Tutorials](https://tier4.github.io/nebula/tutorials)

To build and serve the documentation locally, see the build steps further below.

## Quick start

Nebula officially supports the following ROS 2 distros:

- [Humble](https://docs.ros.org/en/humble/Installation.html)
- [Jazzy](https://docs.ros.org/en/jazzy/Installation.html)

Other ROS 2 distros, such as [Rolling](https://docs.ros.org/en/rolling/Installation.html), might
work, but are not officially supported at this time.

> **Note**
>
> Boost version 1.74.0 or later is required. A manual install may be required in Ubuntu versions earlier than 22.04.

To build Nebula run the following commands in your workspace:

```bash
# In workspace
git clone https://github.com/tier4/nebula.git
cd nebula
# Import dependencies
vcs import < build_depends-${ROS_DISTRO}.repos
rosdep install --from-paths . --ignore-src -y
# Build Nebula
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=1
```

To build with support for [Agnocast](https://github.com/tier4/agnocast), TIER IV's zero-copy
middleware, refer to the Agnocast section below.

_(optional)_ To build and serve the documentation locally (including API reference), run:

```shell
# Install system dependencies
sudo apt install doxygen

# Install Python dependencies
pip3 install -r docs/requirements.txt

# Generate API reference from C++ sources
mkdocs build --config-file scripts/mkdoxy_gen.yml

# Preprocess docs (expands macros, merges API reference)
python3 scripts/preprocess_zensical_docs.py

# Serve locally at http://localhost:8000
zensical serve --config-file .zensical.toml
```

To launch Nebula as a ROS 2 node with default parameters for your sensor model:

```bash
ros2 launch nebula nebula_launch.py sensor_model:=*sensor_model_name*
```

For example, for a Hesai Pandar40P sensor:

```bash
ros2 launch nebula nebula_launch.py sensor_model:=Pandar40P
```

## Agnocast

Nebula supports [Agnocast](https://github.com/tier4/agnocast) features as follows:

| Feature                                                                                                                                            | Hesai | Continental |
| -------------------------------------------------------------------------------------------------------------------------------------------------- | ----- | ----------- |
| IPC zero-copy                                                                                                                                      | Yes   | No          |
| [CallbackIsolatedAgnocastExecutor (CIE)](https://github.com/autowarefoundation/agnocast/blob/main/docs/callback_isolated_executor_for_agnocast.md) | Yes   | Yes         |

To build with support for Agnocast, execute the above `colcon build` command with the environment
variable `ENABLE_AGNOCAST=1` set.

### IPC zero-copy

Agnocast IPC zero-copy is used for pointcloud and blockage mask outputs on Hesai sensors.

The following apt dependencies are required at run time:

```bash
sudo add-apt-repository ppa:t4-system-software/agnocast
sudo apt-get update
sudo apt-get install agnocast-heaphook-v2.3.1 agnocast-kmod-v2.3.1
```

Nebula binaries that have been compiled with Agnocast support require the following environment
variables to be set at runtime:

```bash
export LD_PRELOAD=/opt/ros/humble/lib/libagnocast_heaphook.so

# This depends on the sensor model used and the number of Nebula topics subscribed.
# A few maximum-size pointclouds worth of memory shall be enough. The below threshold is a
# reasonable default.
export AGNOCAST_MEMPOOL_SIZE=134217728 # 128MB
```

In addition, the Agnocast kernel module must be loaded at runtime:

```bash
sudo modprobe agnocast
```

To confirm that Agnocast IPC zero-copy is enabled, run:

```bash
$ ros2 topic list_agnocast
[...]
/pandar_packets
/pandar_points (Agnocast enabled)
[...]
```

Please note that the `packets` topics do not support Agnocast, as they are purely used for
data recording and tools like `ros2 bag` do not have Agnocast support yet.

### CallbackIsolatedAgnocastExecutor (CIE)

CIE is a real-time Executor that enables per-callback-group Scheduling Attributes configuration. This is enabled for the following nodes when `ENABLE_AGNOCAST=1`:

- `nebula_hesai`: `HesaiRosWrapper`
- `nebula_continental`: `ContinentalARS548RosWrapper`

## Building only specific vendors

By default, building the `nebula` package will build all supported vendors. To build only
specific vendors, use the `--packages-up-to` flag:

```bash
# Build only Hesai support
colcon build --packages-up-to nebula_hesai

# Build multiple vendors
colcon build --packages-up-to nebula_hesai nebula_velodyne
```

Available vendor packages are:

- `nebula_hesai` - Hesai LiDARs (Pandar series, AT128, OT128, etc.)
- `nebula_velodyne` - Velodyne LiDARs (VLP-16, VLP-32, VLS-128)
- `nebula_robosense` - Robosense LiDARs (Bpearl, Helios)
- `nebula_ouster` - Ouster LiDARs (OS0, OS1, OS2 — all beam counts; native decoder, no external SDK)
- `nebula_continental` - Continental radars (ARS548, SRR520)

## Ouster

The `nebula_ouster` package is a native Nebula driver for Ouster OS-0 / OS-1 / OS-2 sensors
(any beam count, including OS-128). It decodes Ouster UDP packets directly and does not depend
on `ouster-sdk`.

Setup steps:

1. Build the package:

   ```bash
   colcon build --packages-up-to nebula_ouster --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

2. Edit `src/nebula_ouster/nebula_ouster/config/ouster_sensor.param.yaml` and set
   `connection.sensor_ip` and `connection.host_ip` to match your environment. Update
   `frame_id` if needed.

3. Launch:

   ```bash
   ros2 launch nebula_ouster ouster_launch_all_hw.xml
   ```

The decoder publishes point clouds on `/points`, IMU samples on `/imu`, and raw packets on
`/packets`. For offline rosbag replay without the sensor, set `metadata_file` to a cached
metadata JSON path and launch with `launch_hw:=false`.

## Migration to Nebula 0.3.0

Version 0.3.0 separates vendor-specific functionality into individual packages. This allows users
to only build the functionality they need.

Since packages have been split and renamed, some changes are required to existing launch files:

- the `nebula_ros` package has been renamed to `nebula`.
- the `nebula_sensor_driver` meta package has been removed. Depend on the `nebula` package, or
  on a `nebula_<vendor>` package instead. `nebula` includes all vendor packages.
- Calibration files have moved from `nebula_decoders` to `nebula_<vendor>_decoders`.
- Vendor-specific launch files have moved from `nebula_ros` to `nebula_<vendor>`.
- `nebula_tests` and `nebula_examples` have been absorbed into the `nebula_<vendor>` packages.

Below are the changes you most likely need to make:

- `ros2 launch nebula_ros nebula_launch.py ...` ➡️ `ros2 launch nebula nebula_launch.py ...`
- `ros2 launch nebula_ros <vendor>_launch_all_hw.xml ...` ➡️ `ros2 launch nebula_<vendor> <vendor>_launch_all_hw.xml ...`
- `$(find-pkg-share nebula_decoders)/calibration/<vendor>/...` ➡️ `$(find-pkg-share nebula_<vendor>_decoders)/calibration/...`
- `$(find-pkg-share nebula_ros)/config/lidar/<vendor>/...` ➡️ `$(find-pkg-share nebula_<vendor>)/config/...`
- `<depend>nebula_ros</depend>` ➡️ `<depend>nebula</depend>` or `<depend>nebula_<vendor></depend>`
- `<depend>nebula_sensor_driver</depend>` ➡️ `<depend>nebula</depend>` or `<depend>nebula_<vendor></depend>`
