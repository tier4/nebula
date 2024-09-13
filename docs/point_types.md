# Nebula point cloud types

Nebula currently supports the below point cloud output types.
However, it can easily be extended to support other custom point cloud types.

These definitions can be found in the `nebula_common/include/point_types.hpp`.

## NebulaPoint = PointXYZIRCAEDT

| Field         | Type     | Units | Description                                             |
| ------------- | -------- | ----- | ------------------------------------------------------- |
| `x`           | `float`  | `m`   | Cartesian x coordinate.                                 |
| `y`           | `float`  | `m`   | Cartesian y coordinate.                                 |
| `z`           | `float`  | `m`   | Cartesian z coordinate.                                 |
| `intensity`   | `uint8`  |       | Intensity of the return as reported by the sensor.      |
| `return type` | `uint8`  |       | Return (echo) type.                                     |
| `channel`     | `uint16` |       | Laser channel ID.                                       |
| `azimuth`     | `float`  | `rad` | Azimuth in polar coordinates.                           |
| `elevation`   | `float`  | `rad` | Elevation in polar coordinates.                         |
| `distance`    | `float`  | `m`   | Distance from the sensor origin.                        |
| `timestamp`   | `uint32` | `ns`  | Time of detection relative to the pointcloud timestamp. |

## [Deprecated] PointXYZIR

| Field       | Type     | Units | Description                                        |
| ----------- | -------- | ----- | -------------------------------------------------- |
| `x`         | `float`  | `m`   | Cartesian x coordinate.                            |
| `y`         | `float`  | `m`   | Cartesian y coordinate.                            |
| `z`         | `float`  | `m`   | Cartesian z coordinate.                            |
| padding     | 4 bytes  |       |                                                    |
| `intensity` | `float`  |       | Intensity of the return as reported by the sensor. |
| `ring`      | `uint16` |       | Ring ID - only defined for rotational LiDARs.      |

## [Deprecated] PointXYZICATR

| Field         | Type     | Units     | Description                                             |
| ------------- | -------- | --------- | ------------------------------------------------------- |
| `x`           | `float`  | `m`       | Cartesian x coordinate.                                 |
| `y`           | `float`  | `m`       | The point's cartesian y coordinate.                     |
| `z`           | `float`  | `m`       | Cartesian z coordinate.                                 |
| padding       | 4 bytes  |           |                                                         |
| `intensity`   | `uint8`  |           | Intensity of the return as reported by the sensor.      |
| `channel`     | `uint16` |           | The ID of the laser channel that produced the point.    |
| `azimuth`     | `float`  | `degrees` | Azimuth in polar coordinates.                           |
| `timestamp`   | `uint32` | `ns`      | Time of detection relative to the pointcloud timestamp. |
| `return type` | `uint8`  |           | Return (echo) type.                                     |

## [Deprecated] PointXYZIRADT

| Field         | Type     | Units     | Description                                             |
| ------------- | -------- | --------- | ------------------------------------------------------- |
| `x`           | `float`  | `m`       | Cartesian x coordinate.                                 |
| `y`           | `float`  | `m`       | Cartesian y coordinate.                                 |
| `z`           | `float`  | `m`       | Cartesian z coordinate.                                 |
| padding       | 4 bytes  |           |                                                         |
| `intensity`   | `float`  |           | Intensity of the return as reported by the sensor.      |
| `return type` | `uint8`  |           | Return (echo) type.                                     |
| `azimuth`     | `float`  | `degrees` | Azimuth in polar coordinates.                           |
| `distance`    | `float`  | `m`       | Distance from the sensor origin.                        |
| `timestamp`   | `double` | `ns`      | Time of detection relative to the pointcloud timestamp. |
