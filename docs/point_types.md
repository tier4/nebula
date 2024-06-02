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
| `timestamp`   | `float` | `ns`      | Contains the relative time to the triggered scan time.                   