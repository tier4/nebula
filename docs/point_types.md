# Nebula point cloud types

Nebula supports three point cloud output types.
However, it can easily be extended to support other custom point cloud types.

These definitions can be found in the `nebula_common/include/point_types.hpp`.

## PointXYZIR

| Field         | Type    | Units | Description                                                          |
| ------------- | ------- | ----- | -------------------------------------------------------------------- |
| `x`           | `float` | `m`   | Contains the abscissa member of the point in cartesian coordinates.  |
| `y`           | `float` | `m`   | Contains the ordinate member of the point in cartesian coordinates.  |
| `z`           | `float` | `m`   | Contains the applicate member of the point in cartesian coordinates. |
| `intensity`   | `uint8` |       | Contains the laser energy return value as reported by the sensor.    |
| `return type` | `uint8` |       | Contains the lase return type according to the sensor configuration. |

## PointXYZICAETR

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

## PointXYZICATR

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

## PointXYZIRADT

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

## PointXYZVIRCAEDT

| Field           | Type     | Units | Description                                                                 |
| --------------- | -------- | ----- | --------------------------------------------------------------------------- |
| `x`             | `float`  | `m`   | The point's cartesian x coordinate.                                         |
| `y`             | `float`  | `m`   | The point's cartesian y coordinate.                                         |
| `z`             | `float`  | `m`   | The point's cartesian z coordinate.                                         |
| `distance_rate` | `float`  | `m/s` | The point's velocity component in the direction of the sensor's origin.     |
| `intensity`     | `uint8`  |       | The intensity of the return as reported by the sensor.                      |
| `return type`   | `uint8`  |       | Whether the point was the first, strongest, last, etc. of multiple returns. |
| `channel`       | `uint16` |       | The ID of the laser channel that produced the point.                        |
| `azimuth`       | `float`  | `rad` | The point's azimuth in polar coordinates.                                   |
| `elevation`     | `float`  | `rad` | The point's elevation in polar coordinates.                                 |
| `distance`      | `float`  | `m`   | The point's distance from the sensor origin.                                |
| `timestamp`     | `uint32` | `ns`  | The time the point was detected relative to the pointcloud timestamp.       |
