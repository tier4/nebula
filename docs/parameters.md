# Common Parameters

- Sensor-specific parameters are defined in `nebula_ros/schema/<sensor_model>.schema.json` and are described in the sensor-specific parameter pages in this tab.
- Vendor-specific parameters can be found on the vendor pages in this tab.
- Parameters common to all or most sensors, regardless of vendor, are explained on this page.

## Connection Mode Settings

### `launch_hw`

Whether to connect to a real sensor or to accept replayed packets from a topic like `/<vendor>_packets` or `/nebula_packets`.
These behaviors are mutually exclusive: replayed packets are only accepted if no sensor is connected, and packets are only published on the above topics if a sensor is connected.
If enabled, Nebula connects to the sensor over UDP/TCP/CAN/etc. and publishes packets on the above topics but does not subscribe to them.
If disabled, Nebula subscribes to packets on `/<vendor>_packets` or `/nebula_packets` but does not publish packets on these topics.

### `setup_sensor`

_Only applies if `launch_hw = true`_

Whether to set up the sensor with the values from Nebula's parameters or to just check and warn if they are different.
If enabled, Nebula checks current sensor configuration state, and updates the sensor's parameters where they differ from Nebula's.
If disabled, the current configuration state is downloaded from the sensor, and Nebula warns if its parameters are different than the ones from the sensor but no sensor settings are changed.

## Network Settings

### IP-Based Sensors

#### `sensor_ip`

This parameter is mainly used for TCP communication, such as diagnostics and for setting parameters.
TCP connections will be made to `sensor_ip`, and if `multicast_ip` is supported and set to a multicast group,
Nebula will drop all UDP traffic received via multicast that was not sent from `sensor_ip`.

#### `host_ip`

UDP sockets are bound to this IP, and for sensors supporting it, Nebula will change the sensor's host IP setting to this address.
Set this parameter to the IP address of your host.

!!! warning

    This parameter can be set to `255.255.255.255` to receive packets on any interface. However, this will set the sensor to use IP broadcast.
    IP multicast may also break with this setting, as the UDP socket cannot determine the correct network interface.

#### `multicast_ip`

For sensors with IP multicast support in Nebula, `multicast_ip` can be set to an address in `224.0.0.0/28` (the range from `224.0.0.0` to `239.255.255.255`).
Nebula will then configure the sensor to send its data to that group, and Nebula will join that group and accept only data sent by `sensor_ip`.
Set this parameter to `""` to disable multicast.

#### `data_port`, `gnss_port`, etc

The ports at which data streams from the sensor arrive. If multiple sensors are connected to one machine, make sure that sensor data streams are separated by setting these ports to different values for each sensor.
These settings have to be mirrored in the sensor's settings for sensors where Nebula cannot set them automatically (e.g. Robosense).

### CAN-FD-Based Sensors

#### `interface`

The name of the CAN interface the sensor is connected to. Find available interfaces via `ip link show type can`.

#### `filters`

A string expressing the filters used to accept/reject arriving CAN frames. See [man candump](https://manpages.ubuntu.com/manpages/jammy/man1/candump.1.html) for syntax information.
You can install `candump` via `apt install can-utils`.

## ROS-Specific Settings

### `frame_id`

The TF2 frame ID used for the published point clouds, objects, etc.

## LiDAR-Specific Settings

These settings are common to most LiDARs, but the ranges or options supported by specific sensor models can vary. Please refer to the individual sensor parameter pages for details.

### `rotation_speed`

The revolutions per minute (RPM) setting for the sensor's motor (mechanical LiDARs only). To calculate the resulting frame rate in frames per second (FPS), use `FPS = RPM / 60`.

### `min_range`

The minimum distance in meters for any point. Points closer than this are filtered out.

### `max_range`

The maximum distance in meters for any point. Points farther away than this are filtered out.

### `return_mode`

Each laser beam can result in multiple returns: if there is a semi-transparent object in front of a solid one, a first weak return, and a strong last return will be reported.
Depending on perception requirements, one might be interested in specific returns, e.g. the strongest and last returns, or only the first return.
This parameter is used to set this preference.

### `dual_return_distance_threshold`

For multiple returns that are close together, the points will be fused into one if they are below this threshold (in meters).

### `point_filters`

Filters that are applied while decoding the pointcloud. For the full reference, see [Point filters](filters.md).
