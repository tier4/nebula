# Hesai-Specific Parameters

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

## Scan Cutting and Field of View

Scan cutting influences the time stamps of points and the point cloud headers.
The point cloud header time stamp is always the absolute time of the earliest (theoretical) point in the cloud.
This means that for different parameter combinations, different timing behaviors are observed:

- `cut_angle` is in `(cloud_min_angle, cloud_max_angle)` (both bounds exclusive): the earliest point is the one directly after `cut_angle`
- `cut_angle = cloud_max_angle` and the FoV is not 360 deg: the earliest point is at `cloud_min_angle`

Depending on whether `cut_angle` and `sync_angle` are aligned, the point cloud time stamp will be somewhere in `[ToS, ToS + 100 ms)` assuming a `10 Hz` LiDAR.

**Examples:**

| `cloud_min_angle` | `cloud_max_angle` | `cut_angle` | `sync_angle` | Resulting cloud timestamp |
| ----------------: | ----------------: | ----------: | -----------: | ------------------------: |
|                 0 |               360 |           0 |            0 |                       ToS |
|                 0 |               180 |           0 |            0 |                       ToS |
|                 0 |               360 |         180 |            0 |               ToS + 50 ms |
|                90 |               360 |         360 |            0 |               ToS + 25 ms |

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
