# nebula_cli

Offline tool: decode sensor packets recorded in a rosbag with the Nebula drivers and write
`sensor_msgs/PointCloud2` to a new bag.

```bash
ros2 run nebula_cli nebula_cli convert --vendor hesai \
  --input my_bag --topic /pandar_packets --output out_bag --output-topic /points \
  --params-file src/nebula_hesai/nebula_hesai/config/Pandar64.param.yaml
```

Several topics (any mix of vendors/models) convert in one pass: start each with `--sensor`.
Sensor flags apply to the latest `--sensor`; `--output-topic` is then required and must be unique.

```bash
ros2 run nebula_cli nebula_cli convert --input my_bag --output out_bag \
  --sensor --vendor hesai --topic /lidar_front/packets --output-topic /lidar_front/points \
  --sensor --vendor velodyne --topic /lidar_rear/packets --output-topic /lidar_rear/points
```

All other topics of the input bag are copied unchanged unless `--drop-other-topics` is given.

Sensors are decoded in parallel, one thread per sensor, up to `--threads N` (default 4). A progress bar per input topic is shown on stderr.

Run `nebula_cli --help` for all options. Supported vendors: `hesai`, `velodyne`, `robosense`.
Input topic types: `PandarScan`, `VelodyneScan`, `RobosenseScan`, `NebulaPackets`.

- `--model` defaults to `auto`: Hesai is detected from the packet size, Velodyne from the product ID
  byte (VLP16, VLP32, VLS128; others need `--model`). Robosense cannot be detected, pass `--model`.
  `--return-mode` defaults to `auto` too: read from the packet (Hesai tail, Velodyne factory byte);
  Robosense takes it from its info packets. Detection uses the first scan packet of each topic;
  explicit `--model` / `--return-mode` always win. Detection runs in a pre-pass before the output
  bag is created, so a bad configuration or an empty scan topic fails without leaving a bag behind.
- Robosense needs `--info-topic` (`RobosenseInfoPacket`): calibration and return mode come from it.
  Scan packets before the first info packet are dropped.
- Calibration files default to those shipped with the vendor `_decoders` package.
  For PandarAT128 pass the correction file with `-p correction_file:=<path>`.
- Continental radars are not supported (they do not produce point clouds).

## Output point fields

The fields of each output `PointCloud2` depend on `--point-format` (per sensor):

| Format             | Fields (type)                                                                                                                                                  |
| ------------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `full` (default)   | `x`, `y`, `z` (f32), `intensity` (u8), `return_type` (u8), `channel` (u16), `azimuth` (f32), `elevation` (f32), `distance` (f32), `time_stamp` (u32)           |
| `xyzir`            | `x`, `y`, `z` (f32), `intensity` (f32), `ring` (u16)                                                                                                           |
| `xyziradt`         | `x`, `y`, `z` (f32), `intensity` (f32), `ring` (u16), `azimuth` (f32), `distance` (f32), `return_type` (u8), `time_stamp` (f64)                                |

- `full` is the native Nebula point type (`PointXYZIRCAEDT`); its `time_stamp` is the offset in
  nanoseconds from the cloud header stamp.
- In `xyziradt`, `time_stamp` is absolute time in seconds (header stamp plus the point offset).
- Header: `stamp` is the cloud timestamp, `frame_id` is the `--frame-id` / `frame_id` param (default `lidar`).
