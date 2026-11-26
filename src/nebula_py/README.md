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

for data in messages:  # serialized messages, e.g. read with rosbag2_py
    # One result per completed scan, possibly none
    for result in decoder.decode(data, "pandar_msgs/msg/PandarScan"):
        cloud = result.pointcloud  # sensor_msgs.msg.PointCloud2
        if result.lost_packets:
            ...
```

[`examples/decode_hesai_bag.py`](examples/decode_hesai_bag.py) is a runnable version that reads a
rosbag and logs the points per scan, packet loss and decode errors. Without arguments, it decodes a
recording from the test resources of `nebula_hesai`:

```bash
python3 src/nebula_py/examples/decode_hesai_bag.py [--bag BAG] [--params PARAMS] [--topic TOPIC]
```

- Messages can be passed serialized, as rosbags store them, together with their type: the type
  name that the bag stores, or the message class. Message objects (`decoder.decode(message)`) work
  as well, but are slower, since the decoder serializes them internally before decoding them.
- Packets are the unit of work, the messages are only containers. Scans do not have to be aligned
  with messages.
- `lost_packets` is the number of packets lost since the previous scan. It is `None` for sensor
  models that do not support packet loss detection.
- Packets have to be fed in the order they were received. Call `decoder.reset()` before feeding
  data from an earlier time.
- `decoder.decode(...).errors` counts the packets that could not be decoded.
- With `HesaiDecoder(parameters, filters_as_mask=True)`, the downsample mask in `parameters` does
  not drop points. Instead, `result.keep_mask`, a NumPy bool array, flags the points that it keeps.
  These are exactly the points that decoding without `filters_as_mask` outputs. The FoV applies as
  usual. Without a downsample mask, `keep_mask` is `None`.
- Log messages go to Python's `logging`, under `nebula_py`.

`load_parameters` removes the `/**: ros__parameters:` root of a parameter file and resolves the
`$(find-pkg-share ...)` and `$(var ...)` substitutions that Nebula's parameter files use. A plain
dict with the same layout works as well.
