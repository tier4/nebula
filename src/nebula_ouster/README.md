# Nebula ouster sensor package

A module extends the support Nebula driver framework to Ouster lidars and sensors.

## Purpose

This package integrates support for Ouster lidars into the Nebula framework.

The ouster decoder receive packets from the sensor converts them into a Lidar scan and then into
a pointcloud of Nebula point type before publishing them as a rostopic.

## Package structure

The ouster sensor consists of four packages:

- **nebula_ouster_common** - Common definitions and configuration structures
- **nebula_ouster_decoders** - Packet decoder and driver implementation
- **nebula_ouster_hw_interfaces** - Hardware interface for sensor communication
- **nebula_ouster** - ROS 2 wrapper and launch files

## Building

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to nebula_ouster
```

## Running

```bash
# Online mode (with hardware)
ros2 launch nebula_ouster nebula_ouster.launch.xml
# Offline mode (replay from rosbag)
ros2 launch nebula_ouster nebula_ouster.launch.xml launch_hw:=false
```

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
