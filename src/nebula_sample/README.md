# Nebula sample sensor package

A template sensor package for the Nebula LiDAR driver framework. This package provides a minimal, working example that developers can copy and modify to add support for new sensors.

## Purpose

This package serves as a starting point for adding new sensor support to Nebula. It includes all necessary components with empty/stub implementations that compile and run without errors.

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
ros2 launch nebula_sample nebula_sample.launch.xml
```

## Using as a template

For detailed instructions on how to use this package as a template for adding a new sensor, please refer to the [Integration guide](../../docs/integration_guide.md).

The guide covers:

1. Cloning and renaming the package
2. Implementing sensor-specific logic
3. Verifying the new implementation

## Key components

### Configuration (`*_common`)

- `SampleSensorConfiguration` - Sensor-specific settings

### Decoder (`*_decoders`)

- `SampleScanDecoder` - Base decoder interface
- `SampleDecoder` - Packet decoder implementation
- `SampleDriver` - Main driver class

### Hardware interface (`*_hw_interfaces`)

- `SampleHwInterface` - Sensor communication interface

### ROS wrapper

- `SampleRosWrapper` - ROS 2 node wrapping the driver
- Point cloud publisher on `/points_raw`

## Reference implementation

This package provides a template structure for adding new sensor support. For complete examples, refer to existing sensor packages like `nebula_hesai` or `nebula_velodyne`.

**For detailed integration instructions, see the [Integration guide](../../docs/integration_guide.md) in the documentation.**
