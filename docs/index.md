# Nebula introduction

The project is separated into four main parts:
プロジェクトは主となる 4 つのパートに分かれている：
Lidar ドライバ、ROS ラッパー、HWI インターフェイスだ。

- Common: `nebula_common`. This packages contains the structures, data types, calibration and configuration definitions used among all the packages.
- Drivers: `nebula_decoders`. The Drivers take care of all the data parsing and conversion. Lidar ドライバは、全てのセンサ通信とデータパーシングを管理している。
- ROS Nodes Wrappers: `nebula_ros`, The ROSWrappers are a lightweight layer responsible for the data conversion between the LidarDriver point cloud and the corresponding ROS counterparts. The ROSWrapper also provides methods for configuration and the obtention of the status information of the Lidar. ROS ラッパーは、Lidar ドライバ点群と対応する ROS のカウンターパーツのデータ変換を掌る軽量のレイヤである。 ROS ラッパーは、構成方法、Lidar のステータス情報取得の方法を提供している。
- HWInterface: `nebula_hw_interfaces`. The HWInterface offers an abstraction layer between the parser and the sensor communication. HW インターフェイスは、パーサとセンサ間の抽象化レイヤを提供している。

# Nebula Common

The Nebula common package contains structure definition such as configuration, calibration, point types.
It also contains other common status strings and conversions used among all the packages.

## Point Types

Nebula supports three point cloud output types.
However, it can easily be extended to support other custom point cloud types.

These definitions can be found in the `nebula_common/include/point_types.hpp`.

### PointXYZIR

| Field         | Type    | Units | Description                                                          |
| ------------- | ------- | ----- | -------------------------------------------------------------------- |
| `x`           | `float` | `m`   | Contains the abscissa member of the point in cartesian coordinates.  |
| `y`           | `float` | `m`   | Contains the ordinate member of the point in cartesian coordinates.  |
| `z`           | `float` | `m`   | Contains the applicate member of the point in cartesian coordinates. |
| `intensity`   | `uint8` |       | Contains the laser energy return value as reported by the sensor.    |
| `return type` | `uint8` |       | Contains the lase return type according to the sensor configuration. |

### PointXYZICAETR

| Field         | Type    | Units | Description                                                          |
| ------------- | ------- | ----- | -------------------------------------------------------------------- |
| `x`           | `float` | `m`   | Contains the abscissa member of the point in cartesian coordinates.  |
| `y`           | `float` | `m`   | Contains the ordinate member of the point in cartesian coordinates.  |
| `z`           | `float` | `m`   | Contains the applicate member of the point in cartesian coordinates. |
| `intensity`   | `uint8` |       | Contains the laser energy return value as reported by the sensor.    |
| `channel`     | `uint8` |       | Contains the laser channel id.                                       |
| `azimuth`     | `float` | `rad` | Contains the azimuth of the current point.                           |
| `elevation`   | `float` | `rad` | Contains the elevation of the current point.                         |
| `timestamp`   | `float` | `ns`  | Contains the relative time to the triggered scan time.               |
| `return type` | `uint8` |       | Contains the lase return type according to the sensor configuration. |

### PointXYZICATR

| Field         | Type    | Units     | Description                                                          |
| ------------- | ------- | --------- | -------------------------------------------------------------------- |
| `x`           | `float` | `m`       | Contains the abscissa member of the point in cartesian coordinates.  |
| `y`           | `float` | `m`       | Contains the ordinate member of the point in cartesian coordinates.  |
| `z`           | `float` | `m`       | Contains the applicate member of the point in cartesian coordinates. |
| `intensity`   | `uint8` |           | Contains the laser energy return value as reported by the sensor.    |
| `channel`     | `uint8` |           | Contains the laser channel id.                                       |
| `azimuth`     | `float` | `degrees` | Contains the azimuth of the current point.                           |
| `timestamp`   | `float` | `ns`      | Contains the relative time to the triggered scan time.               |
| `return type` | `uint8` |           | Contains the lase return type according to the sensor configuration. |

### PointXYZIRADT

| Field         | Type    | Units     | Description                                                                |
| ------------- | ------- | --------- | -------------------------------------------------------------------------- |
| `x`           | `float` | `m`       | Contains the abscissa member of the point in cartesian coordinates.        |
| `y`           | `float` | `m`       | Contains the ordinate member of the point in cartesian coordinates.        |
| `z`           | `float` | `m`       | Contains the applicate member of the point in cartesian coordinates.       |
| `intensity`   | `uint8` |           | Contains the laser energy return value as reported by the sensor.          |
| `return type` | `uint8` |           | Contains the lase return type according to the sensor configuration.       |
| `azimuth`     | `float` | `degrees` | Contains the azimuth of the current point.                                 |
| `distance`    | `float` | `m`       | Contains the distance from the sensor origin to this echo on the XY plane. |
| `timestamp`   | `float` | `ns`      | Contains the relative time to the triggered scan time.                     |
