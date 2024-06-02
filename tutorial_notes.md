# Tutorial Demo Notes

## Agenda

1. Create parameter schema
2. Create sensor enum, config and calibration types
3. Create HW interface
4. (Create decoder)
5. Create sensor class
6. Create HW monitor
7. Create ROS wrappers

## Demo

* Will be re-implementing Hesai's OT128 driver under the name "TutorialSensor"
* For brevity's sake, skipping unit tests (which are mandatory!)

### 1. Create parameter schema

* Specify valid types and value ranges in [TutorialSensor.schema.json](./nebula_ros/schema/TutorialSensor.schema.json)
* Use default values from datasheet [TutorialSensor.param.yaml](./nebula_ros/config/lidar/tutorial/TutorialSensor.param.yaml)
* Create launch file [tutorial.launch.xml](./nebula_ros/launch/tutorial.launch.xml)

### 2. Create sensor enum, config and calibration types

* Add sensor to relevant enums [nebula_common.hpp](./nebula_common/include/nebula_common/nebula_common.hpp)
* We want to pass these params to many points in the code. Use config object. [tutorial_common.hpp](./nebula_common/include/nebula_common/tutorial/tutorial_common.hpp)
* Make config read-only unless changed at the top wrapper
* same parameters from earlier, as an object

### 3. Create HW interface

* We need to get and forward data from sensor and send configuration etc.
* Use pre-defined connection types or implement your own
  * (show [ptc.hpp](./nebula_hw_interfaces/include/nebula_hw_interfaces/nebula_hw_interfaces_tutorial/connections/ptc.hpp), [udp_receiver.hpp](./nebula_hw_interfaces/include/nebula_hw_interfaces/nebula_hw_interfaces_tutorial/connections/udp_receiver.hpp))
* Implement commands/data flows based on those connections [tutorial_hw_interface.hpp](./nebula_hw_interfaces/include/nebula_hw_interfaces/nebula_hw_interfaces_tutorial/tutorial_hw_interface.hpp)

### 4. (Create decoder)

* Convert raw packets to point clouds
* Do not want to copy/paste drivers --> generic decoders
* If you implement a radically different sensor type (i.e. not rotational), might need to implement decoder
* If your sensor is similar to others, use existing decoders (mixins)
* We are using the Hesai decoder [hesai_decoder.hpp](./nebula_decoders/include/nebula_decoders/nebula_decoders_hesai/decoders/hesai_decoder.hpp)

### 5. Create sensor class

* "Copy the datasheet into C++" and provide common decoding API
* (show [pandar_128e4x.hpp](./nebula_decoders/include/nebula_decoders/nebula_decoders_hesai/decoders/pandar_128e4x.hpp)) this is how much code you have to write for a new sensor
* A composable decoder also exists at [nebula#104](https://github.com/tier4/nebula/pull/104) (show)

### 6. Create HW monitor

* We want to monitor sensor state and publish diagnostics [hw_monitor_wrapper.cpp](./nebula_ros/src/tutorial/hw_monitor_wrapper.cpp)
* Receiving data from sensor is omitted here
* On a timer, update and validate diagnostics
* On a watchdog timer, emit warnings if diagnostics die

### 7. Create ROS wrappers

* HW receive, decoding, and HW monitor/ROS executor are on different threads [tutorial_ros_wrapper.cpp](./nebula_ros/src/tutorial/tutorial_ros_wrapper.cpp)
  * Thread-safe queue ensures no packets are lost when pointcloud conversion takes longer
* Launch sub-components (decoder, monitor, interface) only on demand
* Handle parameter updates here

### 8. Demo

* Run
