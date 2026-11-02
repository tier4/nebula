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

Run `nebula_cli --help` for all options. Supported vendors: `hesai`, `velodyne`, `robosense`.
Input topic types: `PandarScan`, `VelodyneScan`, `RobosenseScan`, `NebulaPackets`.

- `--model` defaults to `auto`: Hesai is detected from the packet size, Velodyne from the product ID
  byte (VLP16, VLP32, VLS128; others need `--model`). Robosense cannot be detected, pass `--model`.
  `--return-mode` defaults to `auto` too: read from the packet (Hesai tail, Velodyne factory byte);
  Robosense takes it from its info packets. Detection uses the first scan packet of each topic;
  explicit `--model` / `--return-mode` always win.
- Robosense needs `--info-topic` (`RobosenseInfoPacket`): calibration and return mode come from it.
  Scan packets before the first info packet are dropped.
- Calibration files default to those shipped with the vendor `_decoders` package.
  For PandarAT128 pass the correction file with `-p correction_file:=<path>`.
- Continental radars are not supported (they do not produce point clouds).
