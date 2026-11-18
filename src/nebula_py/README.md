# nebula_py

Python bindings for decoding recorded sensor data with Nebula, without a ROS 2 node.

The decoders take the same parameters as the ROS 2 nodes and the same packet messages that the
nodes publish. They return the point clouds the nodes would publish, each with the number of packets
that were lost while it was received. No ROS 2 node, executor or clock is involved, so the results
only depend on the inputs.

Supported so far: Hesai LiDARs (`PandarScan`, `NebulaPackets` and `NebulaPacket`).

```python
from nebula_py import HesaiDecoder, load_parameters

decoder = HesaiDecoder(load_parameters("Pandar128E4X.param.yaml"))

for message in messages:  # e.g. read with rosbag2_py
    for result in decoder.decode(message):  # one result per completed scan, possibly none
        cloud = result.pointcloud  # sensor_msgs.msg.PointCloud2
        if result.lost_packets:
            ...
```

- Packets are the unit of work, the messages are only containers. Scans do not have to be aligned
  with messages.
- `lost_packets` is the number of packets lost since the previous scan. It is `None` for sensor
  models that do not support packet loss detection.
- Packets have to be fed in the order they were received. Call `decoder.reset()` before feeding
  data from an earlier time.
- `decoder.decode(...).errors` counts the packets that could not be decoded.
- Log messages go to Python's `logging`, under `nebula_py`.

`load_parameters` removes the `/**: ros__parameters:` root of a parameter file and resolves the
`$(find-pkg-share ...)` and `$(var ...)` substitutions that Nebula's parameter files use. A plain
dict with the same layout works as well.
