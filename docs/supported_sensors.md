# Supported sensors

Nebula currently supports the sensor models listed below. The test status column indicates how many of the sensors' features are supported.

For all sensors, the respective configuration file is found under `nebula_ros/config/<type>/<vendor>/<filename>` where

- `<type>` is either lidar or radar,
- `<vendor>` is the vendor of the sensor and
- `<filename>` is listed in the table below.

The launch file for a given vendor is called `<vendor>_launch_all_hw.xml`.
The `sensor_model` parameter below decides which sensor driver is launched.

## Hesai LiDARs

| Model        | `sensor_model` | Configuration file      | Test status |
| ------------ | -------------- | ----------------------- | ----------- |
| Pandar64     | Pandar64       | Pandar64.param.yaml     | ⚠️          |
| Pandar 40P   | Pandar40P      | Pandar40P.param.yaml    | ✅          |
| Pandar XT16  | PandarXT16     | PandarXT16.param.yaml   | ⚠️          |
| Pandar XT32  | PandarXT32     | PandarXT32.param.yaml   | ✅          |
| Pandar XT32M | PandarXT32M    | PandarXT32M.param.yaml  | ⚠️          |
| Pandar QT64  | PandarQT64     | PandarQT64.param.yaml   | ✅          |
| Pandar QT128 | PandarQT128    | PandarQT128.param.yaml  | ✅          |
| Pandar AT128 | PandarAT128    | PandarAT128.param.yaml  | ✅\*        |
| Pandar OT128 | Pandar128E4X   | Pandar128E4X.param.yaml | ✅          |

\*: AT128 needs software version 3.50.8 or newer for the `scan_angle` setting to work correctly.

## Velodyne LiDARs

| Model   | `sensor_model` | Configuration file | Test status |
| ------- | -------------- | ------------------ | ----------- |
| VLP-16  | VLP16          | VLP16.param.yaml   | ⚠️          |
| VLP-32  | VLP32          | VLP32.param.yaml   | ⚠️          |
| VLS-128 | VLS128         | VLS128.param.yaml  | ⚠️          |

## Robosense LiDARs

| Model  | `sensor_model` | Configuration file | Test status |
| ------ | -------------- | ------------------ | ----------- |
| Bpearl | Bpearl         | Bpearl.param.yaml  | ⚠️          |
| Helios | Helios         | Helios.param.yaml  | ⚠️          |

## Continental radars

| Model  | `sensor_model` | Configuration file | Test status |
| ------ | -------------- | ------------------ | ----------- |
| ARS548 | ARS548         | ARS548.param.yaml  | ✅          |
| SRR520 | SRR520         | SRR520.param.yaml  | ✅          |

Test status:  
✅: complete  
⚠️: some functionality yet to be tested  
❌: untested
