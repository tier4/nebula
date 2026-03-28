# Nebula sample sensor package

A minimal template sensor package for the Nebula LiDAR driver framework.

## Purpose

This package is a starting point for adding new sensor support to Nebula. It compiles, launches,
and exercises the ROS packet/pointcloud pipeline with intentionally minimal behavior.

The sample decoder does not invent fake sensor geometry. It only counts packets, reports a scan
boundary every 10 packets, and emits an empty pointcloud for that scan. Replace that logic with
real packet parsing and scan-cutting for your sensor.

## Package structure

The sample sensor consists of four packages:

- **nebula_sample_common** - Common definitions and configuration structures
- **nebula_sample_decoders** - Packet decoder and driver implementation
- **nebula_sample_hw_interfaces** - Hardware interface for sensor communication
- **nebula_sample** - ROS 2 wrapper and launch files

## Building

```bash
colcon build --packages-up-to nebula_sample
```

## Running

```bash
# Online mode (with hardware)
ros2 launch nebula_sample nebula_sample.launch.xml
# Offline mode (replay from rosbag)
ros2 launch nebula_sample nebula_sample.launch.xml launch_hw:=false
```

## Using as a template

For detailed instructions on how to use this package as a template for adding a new sensor, please refer to the [Integration guide](../../docs/integration_guide.md).

The guide covers:

1. Cloning and renaming the package
2. Implementing sensor-specific logic
3. Verifying the new implementation

## Key components

### Configuration (`*_common`)

- `SampleSensorConfiguration` - Minimal sensor-specific settings for an IP-based sensor

### Decoder (`*_decoders`)

- `SampleDecoder` - Minimal decoder stub with packet counting and scan-boundary callbacks
- `PacketDecodeResult` - Decoder output containing metadata/error and performance counters
- `DecodeError` - Decoder error codes for packet handling failures

### Hardware interface (`*_hw_interfaces`)

- `SampleHwInterface` - Sensor communication interface

### ROS wrapper

- `SampleRosWrapper` - ROS 2 node wrapping the driver
- Point cloud publisher on `/points`
- Packet publish/replay topic on `/packets` (`nebula_msgs/msg/NebulaPackets`) depending on
  runtime mode (`launch_hw` parameter)

## Reference implementation

This package provides a template structure for adding new sensor support. For complete examples,
refer to existing sensor packages like `nebula_hesai` or `nebula_velodyne`.

**For detailed integration instructions, see the [Integration guide](../../docs/integration_guide.md) in the documentation.**
