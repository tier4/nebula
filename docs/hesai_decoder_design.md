# Generic Hesai Decoder

Since sensors from the same vendor often follow similar conventions when it comes to packet structure and data processing steps, a generic decoder can be used for most of the decoding work.
This document outlines the requirements and design of the generic Hesai decoder.

## Requirements

There shall only be one decoder class which makes use of static (template) polymorphism to handle different sensor types.
This way, runtime overhead for this generalization is `0`.

### Packet formats

For all handled Hesai sensors, the packet structure follows this rough format:

1. (optional) header: static sensor info and feature flags
2. body: point data
3. tail and other appendices: timestamp, operation mode info

### Decoding steps

For all handled Hesai sensors, decoding a packet follows these steps:

```python
def unpack(packet):
    parse_and_validate(packet)
    # return group: one (single-return) or more (multi-return)
    # blocks that belong to the same azimuth
    for return_group in packet:
        if is_start_of_new_scan(return_group):
            # swap output buffers etc.
        decode(return_group)

def decode(return_group):
    for unit in return_group:
        filter by:
          distance thresholds
          distance to other returns
        correct azimuth/elevation *
        compute x/y/z using sin/cos lookup tables *
        compute time_offset to scan *
        determine return_type *
        append to pointcloud
```

The steps marked with **\*** are model-specific:

- angle correction
- timing correction
- return type assignment

### Angle correction

There are two approaches between all the supported sensors:

- Calibration file based
- Correction file based (currently only used by AT128)

For both approaches, sin/cos lookup tables can be computed.
However, the resolution and calculation of these tables is different.

#### Calibration based

For each laser channel, a fixed elevation angle and azimuth angle offset are defined in the calibration file.
Thus, sin/cos for elevation are only a function of the laser channel (not dependent on azimuth) while those for azimuth are a function of azimuth AND elevation.

Lookup tables for elevation can thus be sized with `n_channels`, yielding a maximum size of
`128 * sizeof(float) = 512B` each.

For azimuth, the size is `n_channels * n_azimuths = n_channels * 360 * azimuth_resolution <= 128 * 36000`.
This yields a table size of `128 * 36000 * sizeof(float) ≈ 18.4MB`.

#### Correction based

While azimuth and elevation correction also have a per-channel component, an additional component depending on azimuth AND channel is present.
The angular resolution of AT128 is `1 / (100 * 256) deg` and the per-channel correction as well as the additional component are defined as integers with the same or `1 / 100 deg` resolution respectively.
This means that a lookup table of length `360 * 100 * 256` will contain sin/cos for all corrected values, since the resolution of corrections is smaller/equal to the base angular resolution.

The lookup tables (of which there only need to be two: sin, cos; both usable for azimuth/elevation) each have a size of `360 * 100 * 256 * sizeof(float) ≈ 36.9MB`.

### Timing correction

Each sensor features an absolute timestamp per packet and formulae to compute the relative time between a unit or block and the packet.

The desired output features a start timestamp per scan, and a relative timestamp to the scan for each point.

Thus, the scan timestamp must be computed as the timestamp of the earliest point in the scan, and all packet-relative point times need to be converted to scan-relative ones.
The earliest point in a scan is guaranteed to be in the first return group (≈ first 1-3 blocks) of the scan.
Note that a scan can start mid-packet, if the scan phase does not align with packet bounds.

The block offset follows a formula linear in the block index for all sensor models which additionally depends on the number of returns of the currently active `return_mode`. The parametrization is different for each sensor.

The channel offset is given as a formula, table or set of tables for all sensors. A few sensors' formula is influenced by factors such as high resolution mode (128E3X, 128E4X), alternate firing sequences (QT128) and near/farfield firing (128E3X).

### Return types

While there is a wide range of different supported return modes (e.g. single (first), single (strongest), dual (first, last), etc.) their handling is largely the same.
Differences only arise in multi-return (dual or triple) in the output order of the returns, and in the handling of some returns being duplicates (e.g. in dual(first, strongest), the first return coincides with the strongest one).

Here is an exhaustive list of differences:

- For Dual (First, Last) `0x3B`, 128E3X, 128E4X and XT32 reverse the output order (Last, First)
- For Dual (Last, Strongest) `0x39`, all sensors except XT32M place the second strongest return in the even block if last == strongest
- For Dual (First, Strongest) `0x3c`, the same as for `0x39` holds.

For all other return modes, duplicate points are output if the two returns coincide.

## Implementation

### `HesaiPacket`

Packets are defined as **packed** structs to enable parsing via `memcpy`.
The sensor-specific layout for sensor XYZ is defined in `PacketXYZ` and usually employs an own `TailXYZ` struct.
The header formats are largely shared between sensors.
The packet body (i.e. point data) is mainly parameterized by bytes per point, points per block, and blocks per body. Thus, parameterized templated structs are used. A few skews such as fine azimuth blocks and blocks with a start-of-block (SOB) header exist and are implemented as their own structs.

![HesaiPacket diagram](./GenericHesaiDecoder-Packet%20Formats.png)

### `HesaiSensor`

Each sensor model has its own class `PandarXYZ : HesaiSensor<...>` that defines packet type and timing, and return mode handling logic. Angle correction is the same for 90% of sensors and thus outsourced into `AngleCorrector` and subclasses. These are template arguments for `HesaiSensor`.
Return mode handling has a default implementation that is supplemented by additional logic only in 3 sensors.

### `AngleCorrector`

The angle corrector has three main tasks:

- compute corrected azimuth/elevation for given azimuth and channel
- implement `hasScanCompleted()` logic that decides where one scan ends and the next starts
- compute and provide lookup tables for sin/cos/etc.

The two angle correction types are calibration-based and correction-based. In both approaches, a file from the sensor is used to extract the angle correction for each azimuth/channel.
For all approaches, cos/sin lookup tables in the appropriate size are generated (see requirements section above).

### `HesaiDecoder<SensorT>`

The decoder is in charge of the control flow and shared decoding steps of all sensors.
It is a template class taking a sensor type `SensorT` from which packet type, angle correction etc. are deducted at compile time.
Thus, this unified decoder is an almost zero-cost abstraction.

Its tasks are:

- parsing an incoming packet
- managing decode/output point buffers
- converting all points in the packet using the sensor-specific functions of `SensorT` where necessary

`HesaiDecoder<SensorT>` is a subclass of the existing `HesaiScanDecoder` to allow all template instantiations to be assigned to variables of the supertype.

## Supporting a new sensor

To support a new sensor model, first familiarize with the already implemented decoders.
Then, consult the new sensor's datasheet and identify the following parameters:

| Parameter                  | Chapter | Possible values                  | Notes                                                                                                                                                                                                                                           |
| -------------------------- | ------- | -------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Header format              | 3.1     | `Header12B`, `Header8B`, ...     | `Header12B` is the standard and comprises the UDP pre-header and header (6+6B) mentioned in the data sheets                                                                                                                                     |
| Blocks per packet          | 3.1     | `2`, `6`, `10`, ...              |                                                                                                                                                                                                                                                 |
| Number of channels         | 3.1     | `32`, `40`, `64`, ...            |                                                                                                                                                                                                                                                 |
| Unit format                | 3.1     | `Unit3B`, `Unit4B`, ...          |                                                                                                                                                                                                                                                 |
| Angle correction           | App. 3  | `CALIBRATION`, `CORRECTION`, ... | The datasheet usually specifies whether a calibration/correction file is used                                                                                                                                                                   |
| Timing correction          | App. 2  |                                  | There is usually a block and channel component. These come in the form of formulas/lookup tables. For most sensors, these depend on return mode and for some, features like high resolution mode, alternate firing etc. might change the timing |
| Return type handling       | 3.1     |                                  | Return modes are handled identically for most sensors but some re-order the returns or replace returns if there are duplicates                                                                                                                  |
| Bytes per second           | 1.4     |                                  |                                                                                                                                                                                                                                                 |
| Lowest supported frequency | 1.4     | `5 Hz`, `10 Hz`, ...             |                                                                                                                                                                                                                                                 |

| Chapter | Full title                               |
| ------- | ---------------------------------------- |
| 1.4     | Introduction > Specifications            |
| 3.1     | Data Structure > Point Cloud Data Packet |
| App. 2  | Absolute Time of Point Cloud Data        |
| App. 3  | Angle Correction                         |

With this information, create a `PacketMySensor` struct and `SensorMySensor` class.
Reuse already-defined structs as much as possible (c.f. `Packet128E3X` and `Packet128E4X`).

Implement timing correction in `SensorMySensor` and define the class constants `float MIN_RANGE`,
`float MAX_RANGE` and `size_t MAX_SCAN_BUFFER_POINTS`.
The former two are used for filtering out too-close and too-far away points while the latter is used to
allocate pointcloud buffers.
Set `MAX_SCAN_BUFFER_POINTS = bytes_per_second / lowest_supported_frequency` from the parameters found above.

If there are any non-standard features your sensor has, implement them as generically as possible to allow for future sensors to re-use your code.
