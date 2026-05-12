This page details the design of the scan cutting logic for sensors with per-channel angle correction.

## Supported Sensors

Precise scan cutting is currently supported for the following vendors:

| Hesai | Robosense | Velodyne |
| :---: | :-------: | :------: |
|  ✅   |    ❌     |    ❌    |

See the [Hesai configuration page](../parameters/vendors/hesai/common.md#scan-cutting-and-field-of-view)
for how to configure this feature.

## Problem Statement

Some sensor models, such as most Hesai sensors, have per-channel azimuth offset terms that result
from the channels' different firing times. This results in a given block with a given encoder
azimuth to have a range of point azimuths. This spread needs to be taken into account when cutting
and publishing scan frames.

<figure markdown="span">
  ![ChannelCorrections](../img/scan_cutting.drawio)
</figure>

In particular, in the region around the configured cut angle, two scan frames are overlapping due to
this spread, and points from one packet are split between the two frames. The timing of when to
initialize the next frame (e.g. set its timestamp) and when to emit the current frame (e.g. publish it)
needs to be carefully defined.

<figure markdown="span">
  ![CutAngleTransitions](../img/scan_cutting.drawio)
</figure>

Additionally, around field of view (FoV) boundaries, a subset of channels may produce points inside
the FoV, while others produce points outside the FoV.

<figure markdown="span">
  ![FovTransitions](../img/scan_cutting.drawio)
</figure>

## Assumptions

- There can be packet loss, but consecutive lost packets span less than $360^\circ - 2 \cdot spread$
  where spread is the maximum difference between the azimuths of any two channels in a block.
- There is at least one packet after all channels have crossed out of the FoV.

Cases that are violating the above assumptions are marked with ⛔ in the below tables and diagrams.

## Action Definitions

| Action | Description                                    | Modified State               |
| ------ | ---------------------------------------------- | ---------------------------- |
| `Ti`   | Reset timestamp of buffer $i$                  | None                         |
| `Ei`   | Emit scan buffer $i$, and change active buffer | $buf \leftarrow buf_{1 - i}$ |

## Cutting in FoV

State definitions:

| State   | Condition                                   |
| ------- | ------------------------------------------- |
| `F0`    | $\forall c . c \in buf_0$                   |
| `C0->1` | $buf = buf_0 \land \exists c . c \in buf_1$ |

Analogously for `F1` and `C1->0`.

Transition actions:

| To \ From | `F0`     | `C0->1` |
| --------- | -------- | ------- |
| `F0`      | -        |         |
| `C0->1`   | `T1`     | -       |
| `F1`      | `T1, E0` | `E0`    |
| `C1->0`   |          | ⛔      |

Analogously for `F1` and `C1->0`.

![FSM-CutInFov](../img/scan_cutting_fsm.drawio)

## Cutting at FoV End

State definitions:

| State   | Condition                                             |
| ------- | ----------------------------------------------------- |
| `O0`    | $\forall c . c \in buf_0 \land c \notin fov$          |
| `F0`    | $\forall c . c \in buf_0 \land \exists c . c \in fov$ |
| `C0->1` | $buf = buf_0 \land \exists c . c \in buf_1$           |

Analogously for `O1`, `F1`, and `C1->0`.

Transition actions:

| To \ From | `O0`     | `F0`     | `C0->1`  |
| --------- | -------- | -------- | -------- |
| `O0`      | -        |          |          |
| `F0`      | `T0`     | -        |          |
| `C0->1`   | `T0`     | -        | -        |
| `O1`      | `T1, E0` | `E0`     | `E0`     |
| `F1`      |          | `T1, E0` | `T1, E0` |
| `C1->0`   |          |          | ⛔       |

Analogously for `O1`, `F1`, and `C1->0`.

![FSM-CutAtFovEnd](../img/scan_cutting_fsm.drawio)
