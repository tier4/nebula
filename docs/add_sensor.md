# How to add your sensor

1. Add your sensor to the `SensorModel` enumeration class located in the `nebula_common.hpp` header inside the `nebula_common` package.
2. Add your sensor model string to the `SensorModelFromString` method in the `nebula_common.hpp` header inside the `nebula_common` package.
3. Write the sensor decoder for your sensor. This class is in charge of converting raw packets to point clouds. This class should implement the abstract class defined in `nebula_driver_base.hpp` inside the `nebula_decoders` package. Add methods as the sensor requires.
4. Write the sensor hardware interface. This class is in charge of obtaining raw packets from the sensor using the `transport_drivers` library, accumulating them to form a scan, and making them available for consumption through callbacks. This class should implement the `nebula_hw_interface_base.hpp` inside the `nebula_hw_interfaces` package. Add methods as the sensor requires.
5. Write the ROS wrappers. The ROS wrappers use the sensor libraries to obtain raw data, decode it and convert it to PointCloud2 ROS messages. The Decoder wrapper receives the configuration and calibration data from files and sends them as structures to the decoder and the hw_interface.
