# Nebula Lidar Driver

## How to build

Builds on ROS Galactic and Humble.

A [TCP enabled version of ROS' Transport Driver](https://github.com/MapIV/transport_drivers/tree/tcp) is required to use Nebula. You can install it manually or pull it inside this repository using vcs:

`vcs import . < build_depends.repos`

Once you have either installed or added the source of the above Transport Drivers package to your workspace, be sure to install dependencies using `rosdep`:

`rosdep install --from-paths src --ignore-src -y -r`

Then compile with colcon, optionally enabling symlink:

`colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`

## How to run tests

Run tests:

```
colcon test --event-handlers console_cohesion+ --packages-above nebula_common
```

Show results:

```
colcon test-result --all
```

## Generic launch file

You can easily run the sensor hardware interface, the sensor hardware monitor and sensor driver using (e.g. Pandar64):

`ros2 launch nebula_ros nebula_launch.py sensor_model:=Pandar64`

If you don't want to launch the hardware (i.e. working on rosbag) set the `launch_hw` flag to false:

`ros2 launch nebula_ros nebula_launch.py sensor_model:=Pandar64 launch_hw:=false`

If you don't want the hardware driver to perform the sensor configuration communication (i.e. limited number of connections) set the `setup_sensor` flag to false:

`ros2 launch nebula_ros nebula_launch.py sensor_model:=Pandar64 setup_sensor:=false`

You should ideally provide a config file for your specific sensor, but default ones are provided `nebula_drivers/config`

`ros2 launch nebula_ros nebula_launch.py sensor_model:=Pandar64 config_file:=your_sensor.yaml`

## Supported sensors

Supported models, where sensor_model is the ROS param to be used at launch:

| Manufacturer | Model         | sensor_model | Configuration file | Test status     |
|--------------| ------------- | ------------ | ------------------ | --------------- |
| Hesai        | Pandar 64     | Pandar64     | Pandar64.yaml      | Complete        |
| Hesai        | Pandar 40P    | Pandar40P    |                    | Complete        |
| Hesai        | Pandar XT32   | PandarXT32   | PandarXT32.yaml    | Complete        |
| Hesai        | Pandar XT32M  | PandarXT32M  | PandarXT32M.yaml   | Limited         |
| Hesai        | Pandar QT64   | PandarQT64   |                    | Limited         |
| Hesai        | Pandar QT128  | PandarQT128  | PandarQT128.yaml   | Limited         |
| Hesai        | Pandar AT128  | PandarAT128  | PandarAT128.yaml   | Complete        |
| Hesai        | Pandar 128E4X | Pandar128E4X |                    | Limited         |
| Velodyne     | VLP-16        | VLP16        | VLP16.yaml         | Limited         |
| Velodyne     | VLP-16-HiRes  | VLP16        |                    | Limited         |
| Velodyne     | VLP-32        | VLP32        | VLP32.yaml         | Limited         |
| Velodyne     | VLS-128       | VLS128       | VLS128.yaml        | Limited         |

The test status indicates if all aspects of the sensor driver have been tested (including setting parameters over TCP and diagnostic data support).

## ROS parameters

### Common ROS parameters

Parameters shared by all supported models: 

| Parameter    | Type   | Default | Accepted Values            | Description      |
| ------------ | ------ | ------- | -------------------------- | ---------------- |
| sensor_model | string |         | See supported models       |                  |
| return_mode  | string |         | See supported return modes |                  |
| frame_id     | string | pandar  |                            | ROS frame ID     |
| scan_phase   | double | 0.0     | degrees [0.0, 360.0]       | Scan start angle |

### Hesai specific parameters

#### Supported return modes per model:

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

#### Hardware interface parameters:

| Parameter                      | Type   | Default         | Accepted Values   | Description                    |
| ------------------------------ | ------ | --------------- | ------------------| ------------------------------ |
| sensor_ip                      | string | 192.168.1.201   |                   | Sensor IP                      |
| host_ip                        | string | 255.255.255.255 |                   | Host IP                        |
| data_port                      | uint16 | 2368            |                   | Sensor port                    |
| gnss_port                      | uint16 | 2369            |                   | GNSS port                      |
| frequency_ms                   | uint16 | 100             | milliseconds, > 0 | Time per scan                  |
| packet_mtu_size                | uint16 | 1500            |                   | Packet MTU size                |
| rotation_speed                 | uint16 | 600             |                   | Rotation speed                 |
| rotation_speed                 | uint16 | 600             |                   | Rotation speed                 |
| cloud_min_angle                | uint16 | 0               | degrees [0, 360]  | FoV start angle                |
| cloud_max_angle                | uint16 | 360             | degrees [0, 360]  | FoV end angle                  |
| dual_return_distance_threshold | double | 0.1             |                   | Dual return distance threshold |
| diag_span                      | uint16 | 1000            | milliseconds, > 0 | Diagnostic span                |
| setup_sensor                   | bool   | True            | True, False       | Configure sensor settings      |

#### Driver parameters:

| Parameter        | Type   | Default | Accepted Values | Description            |
| ---------------- | ------ | ------- | --------------- | ---------------------- |
| calibration_file | string |         |                 | LiDAR calibration file |
| correction_file  | string |         |                 | LiDAR correction file  |

### Velodyne specific parameters

#### Supported return modes:

| return_mode     | Mode               |
| --------------- | ------------------ | 
| SingleFirst     | Single (First)     |
| SingleStrongest | Single (Strongest) |
| SingleLast      | Single (Last)      |
| Dual            | Dual               |

#### Hardware interface parameters:

| Parameter       | Type   | Default         | Accepted Values   | Description     |
| --------------- | ------ | --------------- | ----------------- | --------------- |
| sensor_ip       | string | 192.168.1.201   |                   | Sensor IP       |
| host_ip         | string | 255.255.255.255 |                   | Host IP         |
| data_port       | uint16 | 2368            |                   | Sensor port     |
| gnss_port       | uint16 | 2369            |                   | GNSS port       |
| frequency_ms    | uint16 | 100             | milliseconds, > 0 | Time per scan   |
| packet_mtu_size | uint16 | 1500            |                   | Packet MTU size |

#### Driver parameters:

| Parameter        | Type   | Default | Accepted Values      | Description                             |
| ---------------- | ------ | ------- | -------------------- | --------------------------------------- |
| calibration_file | string |         |                      | LiDAR calibration file                  |
| min_range        | double | 0.3     | meters, >= 0.3       | Minimum point range published           |
| max_range        | double | 300.0   | meters, <= 300.0     | Maximum point range published           |
| view_width       | double | 360.0   | degrees [0.0, 360.0] | Horizontal FOV centered at `scan_phase` |

## Diagrams

![DriverOrganization](docs/diagram.png)