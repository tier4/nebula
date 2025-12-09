# New Sensor Integration Guide

This guide outlines the process of adding support for a new LiDAR sensor to Nebula using the `nebula_sample` package as a template.

## Workflow Overview

The integration process involves four main steps:

1. **Clone**: Copy the sample package structure.
2. **Rename**: Update all file names, class names, and namespaces to match your sensor vendor.
3. **Implement**: Fill in the stub methods with your sensor-specific logic.
4. **Verify**: Build and test with your sensor hardware or PCAP data.

## 1. Cloning the Template

Copy the `nebula_sample` directory to a new directory named after your sensor vendor (e.g., `nebula_myvendor`).

```bash
cp -r src/nebula_sample src/nebula_myvendor
```

## 2. Renaming Components

You must rename all occurrences of "sample" and "Sample" to your vendor name. This includes:

- **Directory Names**: `nebula_sample_*` -> `nebula_myvendor_*`
- **File Names**: `sample_*` -> `myvendor_*`
- **Class Names**: `SampleDriver` -> `MyVendorDriver`
- **Namespaces**: `nebula::drivers::sample` -> `nebula::drivers::myvendor`
- **CMake & Package Definitions**: Update `CMakeLists.txt` and `package.xml` in all sub-packages.

## 3. Implementation Details

The Nebula architecture is modular. You will need to implement specific classes in each sub-package.

### A. Common Package (`nebula_myvendor_common`)

**Purpose**: Defines data structures for configuration and calibration.

- **`MyVendorSensorConfiguration`**:

  - Add fields for sensor-specific settings (e.g., return mode, frequency, IP address).
  - This struct is passed to both the driver and hardware interface.

- **`MyVendorCalibrationConfiguration`**:
  - Define how calibration data (e.g., angle corrections) is stored.
  - Implement `load_from_file()` to parse your vendor's calibration file format (csv, dat, xml, etc.) if required.

### B. Decoders Package (`nebula_myvendor_decoders`)

**Purpose**: Handles packet parsing and point cloud generation.

- **`MyVendorScanDecoder`** (Interface):

  - Inherits from `ScanDecoder`.
  - Defines the contract for parsing packets.

- **`MyVendorDecoder`** (Implementation):

  - **`unpack(packet)`**: The core method. It takes raw bytes (UDP packet) and returns a `PacketDecodeResult` containing the decoded points or an error.
  - **`get_pointcloud()`**: (Optional) If you buffer points, this returns the constructed point cloud.
  - **Key Task**: Parse the raw byte stream according to your sensor's user manual.

- **`MyVendorDriver`**:
  - The high-level manager.
  - Initializes the decoder and hardware interface.
  - **`parse_cloud_packet(packet)`**: Called when a packet is received. Delegates to `decoder_->unpack()`.

### C. Hardware Interfaces Package (`nebula_myvendor_hw_interfaces`)

**Purpose**: Handles network communication (UDP/TCP).

- **`MyVendorHwInterface`**:
  - Inherits from `HwInterfaceBase`.
  - **`sensor_interface_start()`**: Setup UDP sockets and start receiving data.
  - **`sensor_interface_stop()`**: Close sockets.
  - **`register_scan_callback()`**: Register the function to call when a packet is received (usually `MyVendorDriver::parse_cloud_packet`).
  - **UDP Receiver Example**: Refer to `src/nebula_core/include/nebula_core/hw/hw_interface_base.hpp` for the base interface definition. Concrete implementations are found in sensor-specific packages.

### D. ROS Wrapper Package (`nebula_myvendor`)

**Purpose**: Bridges the C++ driver with ROS 2.

- **`MyVendorRosWrapper`**:
  - Inherits from `rclcpp::Node`.
  - **`initialize_driver()`**: Instantiates your `MyVendorDriver` and `MyVendorHwInterface`.
  - **`receive_cloud_packet_callback()`**: The bridge callback. Receives data from HW interface, passes to Driver, gets PointCloud, converts to ROS message, and publishes.
  - **Parameters**: Declare ROS parameters that map to your `MyVendorSensorConfiguration`.

## 4. Verification

1. **Build**:

   ```bash
   colcon build --packages-up-to nebula_myvendor
   ```

2. **Launch**:

   ```bash
   ros2 launch nebula_myvendor nebula_myvendor.launch.xml
   ```

3. **Test**:
   - Verify topics: `ros2 topic list`
   - Visualize: `rviz2` (add PointCloud2 display)

## Checklist

- [ ] Renamed all directories and files.
- [ ] Updated `CMakeLists.txt` and `package.xml` dependencies.
- [ ] Implemented `SensorConfiguration` struct.
- [ ] Implemented `unpack()` in Decoder.
- [ ] Implemented UDP reception in HW Interface.
- [ ] Mapped ROS parameters in Wrapper.
- [ ] Added copyright headers.
- [ ] Verified build.
