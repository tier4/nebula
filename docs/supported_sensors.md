# Supported sensors

Nebula currently supports the following sensor models, where `sensor_model` is the ROS parameter to be used at launch:

## Hesai LiDARs

| Model         | `sensor_model` | Configuration file | Test status |
| ------------- | -------------- | ------------------ | ----------- |
| Pandar64      | Pandar64       | Pandar64.yaml      | ✅          |
| Pandar 40P    | Pandar40P      | Pandar40P.yaml     | ✅          |
| Pandar XT32   | PandarXT32     | PandarXT32.yaml    | ✅          |
| Pandar XT32M  | PandarXT32M    | PandarXT32M.yaml   | ⚠️          |
| Pandar QT64   | PandarQT64     | PandarQT64.yaml    | ✅          |
| Pandar QT128  | PandarQT128    | PandarQT128.yaml   | ⚠️          |
| Pandar AT128  | PandarAT128    | PandarAT128.yaml   | ✅\*        |
| Pandar 128E4X | Pandar128E4X   | Pandar128E4X.yaml  | ⚠️          |

## Velodyne LiDARs

| Model        | `sensor_model` | Configuration file | Test status |
| ------------ | -------------- | ------------------ | ----------- |
| VLP-16       | VLP16          | VLP16.yaml         | ⚠️          |
| VLP-16-HiRes | VLP16          |                    | ❌          |
| VLP-32       | VLP32          | VLP32.yaml         | ⚠️          |
| VLS-128      | VLS128         | VLS128.yaml        | ⚠️          |

## Robosense LiDARs

| Model  | `sensor_model` | Configuration file | Test status |
| ------ | -------------- | ------------------ | ----------- |
| Bpearl | Bpearl         | Bpearl.yaml        | ⚠️          |
| Helios | Helios         | Helios.yaml        | ⚠️          |

## Aeva LiDARs

| Model     | `sensor_model` | Configuration file | Test status |
| --------- | -------------- | ------------------ | ----------- |
| Aeries II | Aeries2        | Aeries2.param.yaml | ⚠️          |

## Continental radars

| Model  | `sensor_model` | Configuration file | Test status |
| ------ | -------------- | ------------------ | ----------- |
| ARS548 | ARS548         | ARS548.yaml        | ⚠️          |

Test status:  
✅: complete  
⚠️: some functionality yet to be tested  
❌: untested  
\*: AT128 needs software version 3.50.8 or newer for the `scan_angle` setting to work correctly.
