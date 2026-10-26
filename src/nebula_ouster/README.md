# Nebula Ouster sensor package

Native Nebula driver for Ouster LiDAR sensors (OS-0 / OS-1 / OS-2 at 32, 64, or 128 beams).
The driver decodes Ouster UDP packets directly — it does **not** depend on `ouster-sdk`.

## Features

- Native packet parsing for the `RNG19_RFL8_SIG16_NIR16` (single return),
  `RNG19_RFL8_SIG16_NIR16_DUAL` (dual return), and `LEGACY` UDP profiles.
- Dual return support — points from both returns are published with the correct `return_type`
  field (`FIRST` and `LAST`).
- IMU output — Ouster IMU packets (~100 Hz) are decoded and published as `sensor_msgs/Imu`.
- Metadata caching — sensor metadata JSON can be cached to a file so offline rosbag replay
  does not require the sensor to be reachable.
- No external SDK dependencies; uses only Nebula's built-in HTTP and UDP clients.

## Package structure

The driver is split into the four standard Nebula sub-packages:

- **nebula_ouster_common** — sensor configuration structs
- **nebula_ouster_decoders** — packet parsing, XYZ lookup, metadata JSON parsing
- **nebula_ouster_hw_interfaces** — UDP receive socket wrapper
- **nebula_ouster** — ROS 2 wrapper node, launch files, diagnostics

## Building

```bash
colcon build --packages-up-to nebula_ouster
```

## Running

Edit `config/ouster_sensor.param.yaml` and set `connection.sensor_ip` and `connection.host_ip`
to match your network. Then:

```bash
# Live hardware
ros2 launch nebula_ouster ouster_launch_all_hw.xml

# Offline replay (uses cached metadata_file; subscribes to NebulaPackets on the 'packets' topic)
ros2 launch nebula_ouster ouster_launch_all_hw.xml launch_hw:=false
```

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `points` | `sensor_msgs/PointCloud2` | Decoded point cloud (`PointXYZIRCAEDT`) |
| `imu` | `sensor_msgs/Imu` | IMU sample (~100 Hz) |
| `packets` | `nebula_msgs/NebulaPackets` | Raw packet stream for rosbag recording |

## Parameters

See `config/ouster_sensor.param.yaml` for the full list. Key parameters:

- `connection.sensor_ip` / `connection.host_ip` / `connection.data_port`
- `connection.imu_port` — dedicated UDP socket for IMU packets (set to 0 to disable)
- `launch_hw` — `true` for live hardware, `false` for rosbag replay via `NebulaPackets`
- `metadata_file` — optional cache path; enables offline replay without the sensor
- `fov.azimuth.min_deg` / `max_deg`, `fov.elevation.min_deg` / `max_deg` — FoV crop
- `frame_id` — TF frame used for the point cloud and IMU topics
