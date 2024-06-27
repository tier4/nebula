# Nebula point cloud types

Nebula currently supports the below point cloud output types.
However, it can easily be extended to support other custom point cloud types.

These definitions can be found in the `nebula_common/include/point_types.hpp`.

## PointXYZIR

| Field       | Type     | Units | Description                                                                  |
| ----------- | -------- | ----- | ---------------------------------------------------------------------------- |
| `x`         | `float`  | `m`   | The point's cartesian x coordinate.                                          |
| `y`         | `float`  | `m`   | The point's cartesian y coordinate.                                          |
| `z`         | `float`  | `m`   | The point's cartesian z coordinate.                                          |
| padding     | 4 bytes  |       |                                                                              |
| `intensity` | `float`  |       | The intensity of the return as reported by the sensor.                       |
| `ring`      | `uint16` |       | The ID of the ring the point is part of. Only defined for rotational LiDARs. |

## PointXYZICATR

| Field         | Type     | Units     | Description                                                                 |
| ------------- | -------- | --------- | --------------------------------------------------------------------------- |
| `x`           | `float`  | `m`       | The point's cartesian x coordinate.                                         |
| `y`           | `float`  | `m`       | The point's cartesian y coordinate.                                         |
| `z`           | `float`  | `m`       | The point's cartesian z coordinate.                                         |
| padding       | 4 bytes  |           |                                                                             |
| `intensity`   | `uint8`  |           | The intensity of the return as reported by the sensor.                      |
| `channel`     | `uint16` |           | The ID of the laser channel that produced the point.                        |
| `azimuth`     | `float`  | `degrees` | azimuth in polar coordinates.                                               |
| `timestamp`   | `uint32` | `ns`      | The time the point was detected relative to the pointcloud timestamp.       |
| `return type` | `uint8`  |           | Whether the point was the first, strongest, last, etc. of multiple returns. |

## PointXYZIRADT

| Field         | Type     | Units     | Description                                                                 |
| ------------- | -------- | --------- | --------------------------------------------------------------------------- |
| `x`           | `float`  | `m`       | The point's cartesian x coordinate.                                         |
| `y`           | `float`  | `m`       | The point's cartesian y coordinate.                                         |
| `z`           | `float`  | `m`       | The point's cartesian z coordinate.                                         |
| padding       | 4 bytes  |           |                                                                             |
| `intensity`   | `float`  |           | The intensity of the return as reported by the sensor.                      |
| `return type` | `uint8`  |           | Whether the point was the first, strongest, last, etc. of multiple returns. |
| `azimuth`     | `float`  | `degrees` | The point's azimuth in polar coordinates.                                   |
| `distance`    | `float`  | `m`       | The point's distance from the sensor origin.                                |
| `timestamp`   | `double` | `ns`      | The time the point was detected relative to the pointcloud timestamp.       |

## NebulaPoint = PointXYZIRCAEDT

| Field         | Type     | Units | Description                                                                 |
| ------------- | -------- | ----- | --------------------------------------------------------------------------- |
| `x`           | `float`  | `m`   | The point's cartesian x coordinate.                                         |
| `y`           | `float`  | `m`   | The point's cartesian y coordinate.                                         |
| `z`           | `float`  | `m`   | The point's cartesian z coordinate.                                         |
| `intensity`   | `uint8`  |       | The intensity of the return as reported by the sensor.                      |
| `return type` | `uint8`  |       | Whether the point was the first, strongest, last, etc. of multiple returns. |
| `channel`     | `uint16` |       | The ID of the laser channel that produced the point.                        |
| `azimuth`     | `float`  | `rad` | The point's azimuth in polar coordinates.                                   |
| `elevation`   | `float`  | `rad` | The point's elevation in polar coordinates.                                 |
| `distance`    | `float`  | `m`   | The point's distance from the sensor origin.                                |
| `timestamp`   | `uint32` | `ns`  | The time the point was detected relative to the pointcloud timestamp.       |
