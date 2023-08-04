# Generic Hesai Decoder

Since sensors from the same vendor often follow similar conventions when it comes to packet structure and data processing steps, a generic decoder can be used for most of the decoding work.
This document outlines the requirements and design of the generic Hesai decoder.

## Potential for generalizaiton

### Packet formats

For all handled Hesai sensors, the packet structure follows this rough format:
1. (optional) header: static sensor info and feature flags
2. body: point data
3. tail and other appendices: timestamp, operation mode info

### Decoding steps

For all handled Hesai sensors, decoding a packet follows these steps:
1. parse (& validate) packet
2. decode and append points to pointcloud, iterating over groups of blocks (corresponding to the same firing for multi-return) or single blocks (single-return)
    1. check if block (group) completes scan and swap output buffers
    2. iterate over units in block and decode
        1. get distance from packet
        2. filter by distance
        3. get and correct azimuth and elevation from packet __*__
        4. compute x, y, z using trigonometry (preferably lookup tables)
        5. compute and correct time offset of point __*__
        6. assign return type to point (handling possible duplicates from multi-return) __*__
        7. append point to pointcloud

The steps marked with __*__ are model-specific:
* angle correction
* timing correction
* return type assignment

### Angle correction

There are two approaches between all the supported sensors:
* Calibration file based
* Correction file based (currently only used by AT128)

For both approaches, the same sin/cos lookup tables can be computed and used.
However, the resolution and calculation of these tables is different.

#### Calibration based

For each laser channel, a fixed elevation angle and azimuth angle offset are defined in the calibration file.
sin/cos lookup tables are computed at 0.001 deg resolution (the resolution observed in the calibration files) for the whole 360 deg range .

This yields a (360 deg / 0.001 deg) * sizeof(float) = 360000 * 4B = 1.4MB memory footprint per lookup table (of which there are two: sin, cos).

#### Correction based

While azimuth and elevation correction also have a per-channel component, an additional component depending on azimuth AND channel is present.

This leads to more individual azimuth/elevation values than the calibration-based approach, and for the case of the AT128, the lookup table resolution is around 0.00004 deg.

This yields a memory footprint of 36.9 MB per lookup table.

### Timing correction

Timing correction for all sensors follows the same underlying formula:
Given a scan start timestamp $T_{s_i}$, packet start timestamp $T_{p_j}$, block offset $o_{b_k}$ within the packet and channel offset $o_{c_l}$ within the block, the point identified by $(i, j, k, l)$ has the relative scan timestamp $t_{i,j,k,l} = T_{p_j} - T_{s_i} + o_{b_k} + o_{c_l}$.

The block offset follows a formula linear in the block index for all sensor models which addidionally depends on the number of returns of the currently active `return_mode`.

The channel offset is given as a formula, table or set of tables for all sensors. A few sensors' formula is influenced by factors such as high resolution mode (128E3X, 128E4X), alternate firing sequences (QT128) and near/farfield firing (128E3X).

### Return types

While there is a wide range of different supported return modes (e.g. single (first), single (strongest), dual (first, last), etc.) their handling is largely the same.
Differences only arise in multi-return (dual or triple) in the output order of the returns, and in the handling of some returns being duplicates (e.g. in dual(first, strongest), the first return coincides with the strongest one).

Here is an exhaustive list of differences:
* For Dual (First, Last) `0x3B`, 128E3X, 128E4X and XT32 reverse the output order (Last, First)
* For Dual (Last, Strongest) `0x39`, all sensors except XT32M place the second strongest return in the even block if last == strongest
* For Dual (First, Strongest) `0x3c`, the same as for `0x39` holds.

For all other return modes, duplicate points are output if the two returns coincide.

## The implementation

[work in progress]
