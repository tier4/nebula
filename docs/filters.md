# Point Filters

Filters run for every point, as soon as it is fully decoded. This can speed up the later parts of pointcloud processing pipelines by reducing the number of points sent and copied between modules.

## Configuration

Filters are configured via the ROS parameter `point_filters`. The parameter is a stringified JSOn object of the form:

```json
{
  "filter_type1": configuration1,
  "filter_type2": configuration2
}
```

Where the `filter_type` field has to be one of the supported filters below, and where the `configuration` format depends on the filter type.

Filters can also be set during runtime, e.g. via:

```shell
ros2 param set /hesai_ros_wrapper_node point_filters '"{\"filter_type1\": configuration1, ...}"'
```

## Supported Filters

The following filter types are supported:

| Filter Name         | Filter Type Field     |
| ------------------- | --------------------- |
| Ring Section Filter | `ring_section_filter` |

Below, each filter type is documented in detail.

### Ring Section Filter

This filter can remove zero or more contiguous sections per ring.

Configuration is done in the following format:

```json
  [
    [channel_id1, start_deg1, end_deg1],
    [channel_id2, start_deg2, end_deg2],
    ...
  ]
```

Things to keep in mind:

- Not all channels have to be configured (unconfigured channels are not filtered).
- A channel can be configured multiple times, in which case all of the configured sections are filtered (like a logical `OR` operation)
- Sections are allowed to overlap
- Sections can wrap around the `360/0` boundary (e.g. `[270, 90] deg`)
- `start_deg` and `end_deg` are floating-point, so precise angles can be specified

Examples:

- `[50.25, 100.114] deg` on rings `[2, 4]`:

  ```json
  [
    [2, 50.25, 100, 114],
    [3, 50.25, 100, 114],
    [4, 50.25, 100, 114]
  ]
  ```

- complete rings `1, 3, 5`:

  ```json
  [
    [1, 0, 360],
    [3, 0, 360],
    [5, 0, 360]
  ]
  ```

- `[10, 20] deg`, `[30, 40] deg` and `[50, 60] deg` on ring `5`:

  ```json
  [
    [5, 10, 20],
    [5, 30, 40],
    [5, 50, 60]
  ]
  ```

## Compatibility Chart

|           | Ring Section Filter |
| --------- | ------------------- |
| Hesai     | ✅                  |
| Robosense | ❌                  |
| Velodyne  | ❌                  |

Compatibility:  
✅: compatible  
❌: incompatible
