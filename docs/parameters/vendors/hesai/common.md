## Overview

{{ json_to_markdown("nebula_ros/schema/sub/lidar_hesai.json", ["definitions"], True) }}

## PTP Settings

Support varies from sensor to sensor, so check the sensor's parameter page for allowed values.

### `ptp_profile`

The PTP version or profile to use. The supported profiles are `1588v2`, `802.1as` (gPTP), and `automotive` (AutoSAR Time Synchronization Protocol).

### `ptp_domain`

The domain to separate multiple PTP connections over the same link.

### `ptp_transport_type`

Whether to use `UDP` or `L2`. For `802.1as` and `automotive`, only `L2` is allowed.

### `ptp_switch_type`

While this is not found in the PTP standards, this influences how often the sensor performs delay measurements.
Set this to `TSN` if your switch supports gPTP or the AutoSAR protocol, and `NON_TSN` if the switch does not.
In the latter case, the sensor will measure more often.

### `ptp_lock_threshold`

_Only applies to `OT128` and `QT128`_

The maximum difference between the sensor and PTP master that will still be considered `locked` by the sensor, in microseconds.
When this threshold is crossed, the sensor will report its synchronization state to be `tracking`.
Nebula's hardware monitor treats only the `locked` state as `OK`, `tracking` as `WARNING` and `frozen` and `free run` as `ERROR`.

## Scan Cutting and Field of View

Scan cutting influences the time stamps of points and the point cloud headers.
The point cloud header time stamp is always the absolute time of the earliest (theoretical) point in the cloud.
This means that for different parameter combinations, different timing behaviors are observed:

- `cut_angle` is within `(cloud_min_angle, cloud_max_angle)` (both bounds exclusive): the earliest point is the one directly after `cut_angle`
- `cut_angle = cloud_max_angle` and the FoV is not 360 deg: the earliest point is at `cloud_min_angle`

Depending on whether `cut_angle` and `sync_angle` are aligned, the point cloud time stamp will be somewhere in `[ToS, ToS + 100 ms)` assuming a `10 Hz` LiDAR.

The below figure illustrates the various angles at play. Solid lines represent corrected angles, while dotted ones represent encoder angles.
Note that a range of encoder angles can contribute points to the same corrected angle: each channel can have a different azimuth correction term.
The upper bounds of these areas of influence are shown in orange and are auto-calculated by Nebula.

![Explanation of the angles involved in scan cutting and timing](sensor_angles.svg)

<!-- prettier-ignore-start -->
!!! note
    The parts of the correction ranges that are outside of the (corrected) FoV also need to be captured by the sensor as they contribute points to regions within the FoV.
    Nebula automatically sets the sensor's FoV settings to an oversized range compared to the desired corrected FoV.
<!-- prettier-ignore-end -->

The two examples shown below illustrate the behavior of a `cut_angle` inside the FoV (left) and one that coincides with the `cloud_max_angle`.
Scans are always published on `cut_angle`, so the timestamp has a jump from `100 ms` to `0 ms` in the left scan: the points near `0 ms` were inserted into the scan first,
while those near `100 ms` were inserted at the end of the scan.

The example on the right has timestamps ranging from `0 ms` at `90 deg`, and `62.5 ms` at `315 deg`. The cloud is published at `315 deg`, but the timestamp is reset at `90 deg`,
resulting in a range of point times much less than `100 ms`. See the table below for the exact parameters used in this figure.

![Examples of the angles shown above](sensor_angle_examples.svg)

<!-- prettier-ignore-start -->
!!! note
    Setting `cut_angle = cloud_max_angle` for non-360 deg FoVs results in the cloud being published right at the end of the FoV, meaning that there is 0 latency where the decoder has to wait for points to arrive.
    In comparison, when cutting somewhere inside the FoV, the decoder has to wait after `cloud_max_angle` has been passed until the FoV is entered again at `cloud_min_angle` to complete the scan, resulting in a
    latency from first point to scan publication of `100 ms`.

    In general, the scan latency (from the time of the first point until publication) can be expressed as `L = (cloud_max_angle - cloud_min_angle) / 360 * 1000 ms / framerate` where w.l.o.g. `cloud_max_angle >=` `cloud_min_angle`.
<!-- prettier-ignore-end -->

**Examples:**

The last two rows correspond to the examples in the above figure.

| `cloud_min_angle` | `cloud_max_angle` | `cut_angle` | `sync_angle` | Scan timestamp | Scan latency |
| ----------------: | ----------------: | ----------: | -----------: | -------------: | -----------: |
|                 0 |               360 |           0 |            0 |            ToS |       100 ms |
|                 0 |               180 |           0 |            0 |            ToS |       100 ms |
|                 0 |               360 |         180 |            0 |    ToS + 50 ms |       100 ms |
|                90 |               360 |         360 |            0 |    ToS + 25 ms |        75 ms |
|                90 |               315 |         180 |           30 |    ToS + 17 ms |       100 ms |
|                90 |               315 |         315 |           30 |    ToS + 17 ms |        63 ms |

### `cloud_min_angle` and `cloud_max_angle`

The start and end angle of the desired field of view (FoV), in degrees. For the full FoV for 360 deg LiDARs, set these to `0` and `360`.
The angles can also be set such that the 360->0 bound is crossed, e.g. `270` and `90`.

Internally, the sensor handles angle correction effects and sets the hardware parameters with a computed margin to account for them.
The output pointcloud will have exactly the FoV angles set in these parameters.

### `sync_angle`

The encoder angle of the sensor which will be passed at top-of-second (ToS). This is the time where the timestamp will roll over to `0`,
e.g. `X.X0000... s` for a `10 Hz` frame rate.

### `cut_angle`

The corrected angle at which one point cloud ends and the next one starts.
This angle has to be in the range `(cloud_min_angle, cloud_max_angle]`, except for a 360 deg FoV, where the angle has to be `[0, 360)`.
