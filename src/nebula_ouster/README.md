# Nebula ouster sensor package

A module extends the support Nebula driver framework to Ouster lidars and sensors.

## Purpose

This package integrates support for Ouster lidars into the Nebula framework.

The ouster decoder receive packets from the sensor converts them into a LidarFrame and then into
a pointcloud of Nebula point type before publishing them as a rostopic.

## Package structure

The ouster sensor consists of four packages:

- **nebula_ouster_common** - Common definitions and configuration structures
- **nebula_ouster_decoders** - Packet decoder and driver implementation
- **nebula_ouster_hw_interfaces** - Hardware interface for sensor communication
- **nebula_ouster** - ROS 2 wrapper and launch files
- **vendor/ouster-sdk** Contains ouster-sdk with minimal dependencies

## Building

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to nebula_ouster
```

## Running

```bash
# Online mode (with hardware)
ros2 launch nebula_ouster ouster_launch_all_hw.xml
# Offline mode (replay from rosbag)
ros2 launch nebula_ouster ouster_launch_all_hw.xml launch_hw:=false
```

## Maintenance

The vendored ouster-sdk is currently version 1.0.1 (see `vendor/ouster-sdk/VERSION`).

To update ouster-sdk to the latest release, monitor the repo <http://github.com/ouster-lidar/ouster-sdk>
for any new releases. When a new release is published, download the new sdk into a separate external
folder, then go to the folder `nebula_ouster/vendor` in **nebula** and delete the entire contents of
`ouster-sdk/*` except the `thirdparty` folder, then selectively copy the following list of files and folders:

- ouster-sdk/ouster_core/
- ouster-sdk/ouster_sensor/
- ouster-sdk/cmake/
- CHANGELOG.rst
- CMakeLists.txt
- CMakeSettings.json
- COPYRIGHT
- LICENSE
- LICENSE-bin
- README.rst
- Security.md
- vcpkg.json
- VERSION

Under `thirdparty`, refresh `nmea` and `robin-map` (only its `CMakeLists.txt`, `cmake/`, `include/`,
`LICENSE` and `README.md`) from the new release, and keep `ouster_json.hpp`. The other ouster-sdk
modules (`ouster_algorithm`, `ouster_perception`, `ouster_pcap`, `ouster_osf`, `ouster_viz`,
`ouster_mapping`, ...) and third-party libraries (jsoncons, spdlog, zpng, sophus, glad) are not needed by
**nebula_ouster** and are not vendored.

After copying, re-apply the following changes to the vendored sources (each is marked with a `nebula:`
comment):

- `CMakeLists.txt`: guard `add_subdirectory(ouster_algorithm)` and `add_subdirectory(ouster_perception)`
  with an `if(EXISTS ...)` check, since those modules are not vendored.
- Replace every `#include <jsoncons/...>` / `#include <jsoncons_ext/...>` in `ouster_core` and
  `ouster_sensor` with a single `#include <ouster_json.hpp>`. This header is a drop-in replacement for the
  subset of jsoncons used by ouster-sdk, backed by the `nlohmann/json` library already used by nebula.
  Extend it if a new release uses jsoncons APIs that it does not cover yet.
- `ouster_core/src/logging.cpp`: wrap all spdlog usage in `#if defined(ENABLE_SPDLOG)` and provide no-op
  fallbacks in the `#else` branches, so that spdlog is not needed.
- `COPYRIGHT`: remove the jsoncons entry. `LICENSE-bin`: replace the jsoncons entry with nlohmann/json.

Compiler warnings raised by the vendored sources are suppressed for the `ouster_core` target in the
`CMakeLists.txt` of each package that adds the SDK via `add_subdirectory`.

## Key components

### Configuration (`*_common`)

- `OusterSensorConfiguration` - Minimal sensor-specific settings for an IP-based sensor

### Decoder (`*_decoders`)

- `OusterDecoder` - Decodes packets into Ouster LidarScan object before converting them into pointclouds
- `PacketDecodeResult` - Decoder output containing metadata/error and performance counters
- `DecodeError` - Decoder error codes for packet handling failures

### Hardware interface (`*_hw_interfaces`)

- `OusterHwInterface` - Communicates with Ouster sensors via their API

### ROS wrapper

- `OusterRosWrapper` - ROS 2 node wrapping the driver
- Point cloud publisher on `/points`
- Packet publish/replay topic on `/packets` (`nebula_msgs/msg/NebulaPackets`) depending on
  runtime mode (`launch_hw` parameter)
