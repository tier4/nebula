# ROS parameters for supported sensors

> **Note**
>
> The information on this page may be out of date.
> Please refer to the configuration in the relevant `*sensor_model*.param.yaml` file for you sensor, to confirm what parameters are available,

## Common ROS parameters

Parameters shared by all supported models:

| Parameter    | Type   | Default          | Accepted values            | Description      |
| ------------ | ------ | ---------------- | -------------------------- | ---------------- |
| sensor_model | string |                  | See supported models       |                  |
| return_mode  | string |                  | See supported return modes |                  |
| frame_id     | string | Sensor dependent |                            | ROS frame ID     |
| scan_phase   | double | 0.0              | degrees [0.0, 360.0]       | Scan start angle |

## Hesai specific parameters

### Supported return modes per model

| Sensor model | return_mode    | Mode   |
| ------------ | -------------- | ------ |
| Pandar XT32M | Last           | Single |
| Pandar XT32M | Strongest      | Single |
| Pandar XT32M | LastStrongest  | Dual   |
| Pandar XT32M | First          | Single |
| Pandar XT32M | LastFirst      | Dual   |
| Pandar XT32M | FirstStrongest | Dual   |
| Pandar XT32M | Dual           | Dual   |
| ---          | ---            | ---    |
| Pandar AT128 | Last           | Single |
| Pandar AT128 | Strongest      | Single |
| Pandar AT128 | LastStrongest  | Dual   |
| Pandar AT128 | First          | Single |
| Pandar AT128 | LastFirst      | Dual   |
| Pandar AT128 | FirstStrongest | Dual   |
| Pandar AT128 | Dual           | Dual   |
| ---          | ---            | ---    |
| Pandar QT128 | Last           | Single |
| Pandar QT128 | Strongest      | Single |
| Pandar QT128 | LastStrongest  | Dual   |
| Pandar QT128 | First          | Single |
| Pandar QT128 | LastFirst      | Dual   |
| Pandar QT128 | FirstStrongest | Dual   |
| Pandar QT128 | Dual           | Dual   |
| ---          | ---            | ---    |
| Pandar QT64  | Last           | Single |
| Pandar QT64  | Dual           | Dual   |
| Pandar QT64  | First          | Single |
| ---          | ---            | ---    |
| Pandar 40P   | Last           | Single |
| Pandar 40P   | Strongest      | Single |
| Pandar 40P   | Dual           | Dual   |
| ---          | ---            | ---    |
| Pandar 64    | Last           | Single |
| Pandar 64    | Strongest      | Single |
| Pandar 64    | Dual           | Dual   |

### Hardware interface parameters

| Parameter                      | Type   | Default         | Accepted values   | Description                    |
| ------------------------------ | ------ | --------------- | ----------------- | ------------------------------ |
| frame_id                       | string | hesai           |                   | ROS frame ID                   |
| sensor_ip                      | string | 192.168.1.201   |                   | Sensor IP                      |
| host_ip                        | string | 255.255.255.255 |                   | Host IP                        |
| data_port                      | uint16 | 2368            |                   | Sensor port                    |
| gnss_port                      | uint16 | 2369            |                   | GNSS port                      |
| frequency_ms                   | uint16 | 100             | milliseconds, > 0 | Time per scan                  |
| packet_mtu_size                | uint16 | 1500            |                   | Packet MTU size                |
| rotation_speed                 | uint16 | 600             |                   | Rotation speed                 |
| cloud_min_angle                | uint16 | 0               | degrees [0, 360]  | FoV start angle                |
| cloud_max_angle                | uint16 | 359             | degrees [0, 360]  | FoV end angle                  |
| dual_return_distance_threshold | double | 0.1             |                   | Dual return distance threshold |
| diag_span                      | uint16 | 1000            | milliseconds, > 0 | Diagnostic span                |
| setup_sensor                   | bool   | True            | True, False       | Configure sensor settings      |

### Driver parameters

| Parameter        | Type   | Default | Accepted values | Description            |
| ---------------- | ------ | ------- | --------------- | ---------------------- |
| frame_id         | string | hesai   |                 | ROS frame ID           |
| calibration_file | string |         |                 | LiDAR calibration file |
| correction_file  | string |         |                 | LiDAR correction file  |

## Velodyne specific parameters

### Supported return modes

| return_mode     | Mode               |
| --------------- | ------------------ |
| SingleFirst     | Single (First)     |
| SingleStrongest | Single (Strongest) |
| SingleLast      | Single (Last)      |
| Dual            | Dual               |

### Hardware interface parameters

| Parameter       | Type   | Default         | Accepted values   | Description     |
| --------------- | ------ | --------------- | ----------------- | --------------- |
| frame_id        | string | velodyne        |                   | ROS frame ID    |
| sensor_ip       | string | 192.168.1.201   |                   | Sensor IP       |
| host_ip         | string | 255.255.255.255 |                   | Host IP         |
| data_port       | uint16 | 2368            |                   | Sensor port     |
| gnss_port       | uint16 | 2369            |                   | GNSS port       |
| frequency_ms    | uint16 | 100             | milliseconds, > 0 | Time per scan   |
| packet_mtu_size | uint16 | 1500            |                   | Packet MTU size |
| cloud_min_angle | uint16 | 0               | degrees [0, 360]  | FoV start angle |
| cloud_max_angle | uint16 | 359             | degrees [0, 360]  | FoV end angle   |

### Driver parameters

| Parameter        | Type   | Default  | Accepted values  | Description                   |
| ---------------- | ------ | -------- | ---------------- | ----------------------------- |
| frame_id         | string | velodyne |                  | ROS frame ID                  |
| calibration_file | string |          |                  | LiDAR calibration file        |
| min_range        | double | 0.3      | meters, >= 0.3   | Minimum point range published |
| max_range        | double | 300.0    | meters, <= 300.0 | Maximum point range published |
| cloud_min_angle  | uint16 | 0        | degrees [0, 360] | FoV start angle               |
| cloud_max_angle  | uint16 | 359      | degrees [0, 360] | FoV end angle                 |
