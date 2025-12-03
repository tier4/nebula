# Nebula Sample Sensor Package

A template sensor package for the Nebula LiDAR driver framework. This package provides a minimal, working example that developers can copy and modify to add support for new sensors.

## Purpose

This package serves as a starting point for adding new sensor support to Nebula. It includes all necessary components with empty/stub implementations that compile and run without errors.

## Package Structure

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

## Using as a Template

To create support for a new sensor:

1. Copy this package structure:

   ```bash
   cp -r src/nebula_sample src/nebula_<your_vendor>
   ```

2. Rename all occurrences:

   - Files: `sample_*` → `<your_vendor>_*`
   - Classes: `Sample*` → `<YourVendor>*`
   - Packages: `nebula_sample*` → `nebula_<your_vendor>*`

3. Implement sensor-specific logic:

   - **Common**: Add sensor configuration parameters
   - **Decoders**: Implement packet parsing and point cloud generation
   - **HW Interfaces**: Implement UDP/TCP communication with sensor
   - **ROS Wrapper**: Add ROS parameters and topics

4. Update dependencies in `package.xml` and `CMakeLists.txt`

## Key Components

### Configuration (`*_common`)

- `SampleSensorConfiguration` - Sensor-specific settings
- `SampleCalibrationConfiguration` - Calibration data structure

### Decoder (`*_decoders`)

- `SampleScanDecoder` - Base decoder interface
- `SampleDecoder` - Packet decoder implementation
- `SampleDriver` - Main driver class

### Hardware Interface (`*_hw_interfaces`)

- `SampleHwInterface` - Sensor communication interface

### ROS Wrapper

- `SampleRosWrapper` - ROS 2 node wrapping the driver
- Point cloud publisher on `/points_raw`

## Reference Implementation

This package is based on the Hesai implementation structure but with all vendor-specific code removed. For a complete example, refer to `nebula_hesai`.
