# Nebula Sensor Driver

Nebula is a sensor driver platform that is designed to provide a unified framework for as wide a variety of devices as possible.
While it primarily targets Ethernet-based LiDAR sensors, it aims to be easily extendable to support new sensors and interfaces.
Nebula provides the following features:

- Support for Velodyne and Hesai sensors, with other LiDAR vendor support under development
- ROS 2 interface implementations
- TCP/IP and UDP communication implementations
- Abstraction of sensor decoders and hardware interfaces available as libraries
- Handling of standard LiDAR functionality, including but not limited to:
  - Configuration of communication settings such as sensor and host IP addresses and communication ports
  - Configuration of scan speed, synchronization settings, scan phase, and field of view
  - Receiving and conversion of UDP packet data into point clouds in Cartesian co-ordinates
  - Receiving and interpretation of diagnostics information from the sensor
  - Support for multiple return modes and labelling of return types for each point

With a rapidly increasing number of sensor types and models becoming available, and varying levels of vendor and third-party driver support, Nebula creates a centralized driver methodology. We hope that this project will be used to facilitate active collaboration and efficiency in development projects by providing a platform that reduces the need to re-implement and maintain many different sensor drivers. Contributions to extend the supported devices and features of Nebula are always welcome.

## How to build

Nebula builds on ROS Galactic and Humble.

> **Note**
>
> A [TCP enabled version of ROS' Transport Driver](https://github.com/MapIV/transport_drivers/tree/tcp) is required to use Nebula.
> It is installed automatically into your workspace using the below commands. However, if you already have ROS transport driver binaries installed, you will have to uninstall them to avoid conflicts (replace `humble` with your ROS distribution):
> `sudo apt remove ros-humble-udp-driver ros-humble-io-context`

To build Nebula run the following commands in your workspace:

```bash
# In workspace
mkdir src
git clone https://github.com/tier4/nebula.git src
# Import dependencies
vcs import src < src/build_depends.repos
rosdep install --from-paths src --ignore-src -y -r
# Build Nebula
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## How to run tests

Run tests:

```bash
colcon test --event-handlers console_cohesion+ --packages-above nebula_common
```

Show results:

```bash
colcon test-result --all
```

## Generic launch file

You can easily run the sensor hardware interface, the sensor hardware monitor and sensor driver using (e.g. Pandar64):

```bash
ros2 launch nebula_ros nebula_launch.py sensor_model:=Pandar64
```

If you don't want to launch the hardware (i.e. when you are working from a rosbag), set the `launch_hw` flag to false:

```bash
ros2 launch nebula_ros nebula_launch.py sensor_model:=Pandar64 launch_hw:=false
```

If you don't want the hardware driver to perform the sensor configuration communication (i.e. limited number of connections) set the `setup_sensor` flag to false:

```bash
ros2 launch nebula_ros nebula_launch.py sensor_model:=Pandar64 setup_sensor:=false
```

You should ideally provide a config file for your specific sensor, but default ones are provided `nebula_drivers/config`:

```bash
ros2 launch nebula_ros nebula_launch.py sensor_model:=Pandar64 config_file:=your_sensor.yaml
```

## Supported sensors

Supported models, where sensor_model is the ROS param to be used at launch:

| Manufacturer | Model         | sensor_model | Configuration file | Test status          |
| ------------ | ------------- | ------------ | ------------------ | -------------------- |
| HESAI        | Pandar 64     | Pandar64     | Pandar64.yaml      | :heavy_check_mark:   |
| HESAI        | Pandar 40P    | Pandar40P    | Pandar40P.yaml     | :heavy_check_mark:   |
| HESAI        | Pandar XT32   | PandarXT32   | PandarXT32.yaml    | :heavy_check_mark:   |
| HESAI        | Pandar XT32M  | PandarXT32M  | PandarXT32M.yaml   | :warning:            |
| HESAI        | Pandar QT64   | PandarQT64   | PandarQT64.yaml    | :heavy_check_mark:   |
| HESAI        | Pandar QT128  | PandarQT128  | PandarQT128.yaml   | :warning:            |
| HESAI        | Pandar AT128  | PandarAT128  | PandarAT128.yaml   | :heavy_check_mark:\* |
| HESAI        | Pandar 128E4X | Pandar128E4X | Pandar128E4X.yaml  | :warning:            |
| Velodyne     | VLP-16        | VLP16        | VLP16.yaml         | :warning:            |
| Velodyne     | VLP-16-HiRes  | VLP16        |                    | :x:                  |
| Velodyne     | VLP-32        | VLP32        | VLP32.yaml         | :warning:            |
| Velodyne     | VLS-128       | VLS128       | VLS128.yaml        | :warning:            |
| Continental  | ARS548        | ARS548       | ARS548.yaml        | :warning:            |

Test status:\
:heavy_check_mark:: complete\
:warning:: some functionality yet to be tested\
:x:: untested\
\*: AT128 needs software version 3.50.8 or newer for the `scan_angle` setting to work correctly.

## ROS parameters

### Common ROS parameters

Parameters shared by all supported models:

| Parameter    | Type   | Default          | Accepted values            | Description      |
| ------------ | ------ | ---------------- | -------------------------- | ---------------- |
| sensor_model | string |                  | See supported models       |                  |
| return_mode  | string |                  | See supported return modes |                  |
| frame_id     | string | Sensor dependent |                            | ROS frame ID     |
| scan_phase   | double | 0.0              | degrees [0.0, 360.0]       | Scan start angle |

### Hesai specific parameters

#### Supported return modes per model

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

#### Hardware interface parameters

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

#### Driver parameters

| Parameter        | Type   | Default | Accepted values | Description            |
| ---------------- | ------ | ------- | --------------- | ---------------------- |
| frame_id         | string | hesai   |                 | ROS frame ID           |
| calibration_file | string |         |                 | LiDAR calibration file |
| correction_file  | string |         |                 | LiDAR correction file  |

### Velodyne specific parameters

#### Supported return modes

| return_mode     | Mode               |
| --------------- | ------------------ |
| SingleFirst     | Single (First)     |
| SingleStrongest | Single (Strongest) |
| SingleLast      | Single (Last)      |
| Dual            | Dual               |

#### Hardware interface parameters

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

#### Driver parameters

| Parameter              | Type   | Default  | Accepted values  | Description                      |
|------------------------|--------|----------|------------------|----------------------------------|
| frame_id               | string | velodyne |                  | ROS frame ID                     |
| calibration_file       | string |          |                  | LiDAR calibration file           |
| min_range              | double | 0.3      | meters, >= 0.3   | Minimum point range published    |
| max_range              | double | 300.0    | meters, <= 300.0 | Maximum point range published    |
| cloud_min_angle        | uint16 | 0        | degrees [0, 360] | FoV start angle                  |
| cloud_max_angle        | uint16 | 359      | degrees [0, 360] | FoV end angle                    |
| invalid_point_remove   | bool   | false    | true, false      | Enable ring based filter*        |
| invalid_regions        | string |          |                  | Invalid point regions to remove* |

*`invalid_point_remove` activates the ring based filter which removes points if they are within specified angle ranges. 

*The format for an invalid region is [ring_id, start_angle, end_angle]

*Angles are given in degrees and multiplied by 100. For instance, 34.44 degrees is represented as 3444. 

*Invalid regions are specified as a string containing a list of invalid regions. Ensure that you have quotation marks to make it string. For example:
```xml
<node pkg="nebula_ros" exec="velodyne_driver_ros_wrapper_node"
          name="velodyne_cloud" output="screen">
        ...
        <param name="invalid_point_remove" value="true"/>
        <param name="invalid_regions" value="'[[0, 3500, 6900], [1, 3400, 6500], [2, 3200, 4600], [3, 3200, 4600]]'"/>
    </node>
```

*Multiple invalid regions are possible for the same ring.

## Software design overview

![DriverOrganization](docs/diagram.png)

## Hesai Sensor Setup

New Hesai sensors do not provide a Web UI to verify and set up the sensor parameters. Instead, these offer a TCP-based protocol to obtain and set the configuration. Nebula sets these sensors at launch. However, settings such as Destination IP, Sensor IP, IP Mask, and Data Port might cause undesired sensor functioning if the driver gets mistakenly launched with inappropriate values. To overcome this problem, Nebula provides an additional setup script for these settings to avoid such scenarios.

The script requires the installation of dependencies via pip:

`$ pip3 install scripts/requirements.txt # first-time setup`

Once the dependencies are installed, the setup script can be invoked using the following command:

`$ python3 scripts/hesai_config --sensor-ip X.X.X.X`

To set the parameters, please use the corresponding arguments as defined below:

```
usage: hesai_config.py [-h] --sensor-ip SENSOR_IP [--destination-ip DESTINATION_IP]
                       [--data-port DATA_PORT] [--new-sensor-ip NEW_SENSOR_IP] [--mask MASK]

options:
  -h, --help            Show this help message and exit
  --sensor-ip SENSOR_IP
                        The current sensor IP address
  --destination-ip DESTINATION_IP
                        Change the current destination IP address to the given one
  --data-port DATA_PORT
                        Change the current destination LiDAR data port to the given one
  --new-sensor-ip NEW_SENSOR_IP
                        Change the current sensor IP address to the given one
  --mask MASK           Change the current net mask to the given one. You can pass it in either a
                        CIDR notation or dotted-decimal notation.
```

## How to evaluate performance

You can evaluate Nebula performance on a given rosbag and sensor model using the below tools.
The profiling runner is most accurate when assigning isolated cores via the `-c <core_id>`.
CPU frequencies are locked/unlocked automatically by the runner to increase repeatability.

Run profiling for each version you want to compare:

```bash
./scripts/profiling_runner.bash baseline -m Pandar64 -b ~/my_rosbag -c 2 -t 20 -n 3
git checkout my_improved_branch
./scripts/profiling_runner.bash improved -m Pandar64 -b ~/my_rosbag -c 2 -t 20 -n 3
```

Show results:

```bash
pip3 install scripts/requirements.txt  # first-time setup
python3 scripts/plot_times.py baseline improved
```
