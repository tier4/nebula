# Copyright 2026 TIER IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Python interface to the Hesai decoder, for offline use.

Messages are passed serialized, as stored in rosbags, or as rclpy message objects, which this
module serializes internally. The C++ decoder deserializes them. No ROS node, executor or clock is
involved, so results depend only on the inputs.
"""

import enum
from types import MappingProxyType
from typing import Any
from typing import Dict
from typing import Optional
from typing import Type
from typing import Union

from nebula_msgs.msg import NebulaPacket
from nebula_msgs.msg import NebulaPackets
from pandar_msgs.msg import PandarScan
from rclpy.serialization import deserialize_message
from rclpy.serialization import serialize_message
from sensor_msgs.msg import PointCloud2

from . import _hesai_pybind as _ext
from .outputs import DecodeResult
from .outputs import Decoded

__all__ = ["DecodeError", "HesaiDecoder"]


class DecodeError(enum.Enum):
    """Why a packet did not contribute to a scan."""

    PACKET_PARSE_FAILED = 0
    DRIVER_NOT_OK = 1
    INVALID_PACKET_SIZE = 2
    PACKET_IGNORED = 3
    """A valid packet without point cloud data, e.g. an IMU or fault message."""


_MESSAGE_TYPES = {
    "pandar_msgs/msg/PandarScan": PandarScan,
    "nebula_msgs/msg/NebulaPackets": NebulaPackets,
    "nebula_msgs/msg/NebulaPacket": NebulaPacket,
}
"""The supported message types, by the type names that rosbags store."""

_DECODE_FUNCTIONS = {
    PandarScan: _ext.HesaiDecoder.decode_pandar_scan,
    NebulaPackets: _ext.HesaiDecoder.decode_nebula_packets,
    NebulaPacket: _ext.HesaiDecoder.decode_nebula_packet,
}

_MessageType = Union[str, Type[PandarScan], Type[NebulaPackets], Type[NebulaPacket]]


def _decode_function(message_type: _MessageType):
    try:
        return _DECODE_FUNCTIONS[_MESSAGE_TYPES.get(message_type, message_type)]
    except (KeyError, TypeError):
        raise TypeError(
            "message_type has to be one of "
            f"{', '.join(_MESSAGE_TYPES)} or their classes, got {message_type!r}"
        ) from None


_DECODER_SETTINGS = (
    "frame_id",
    "sensor_model",
    "min_range",
    "max_range",
    "dual_return_distance_threshold",
    "rotation_speed",
    "cloud_min_angle",
    "cloud_max_angle",
    "cut_angle",
)


def _parameter(parameters: Dict[str, Any], key: str) -> Any:
    try:
        return parameters[key]
    except KeyError:
        raise ValueError(f"Missing parameter: {key}") from None


def _sensor_configuration(parameters: Dict[str, Any]) -> "_ext.HesaiSensorConfiguration":
    config = _ext.HesaiSensorConfiguration()
    for key in _DECODER_SETTINGS:
        value = _parameter(parameters, key)
        try:
            setattr(config, key, value)
        except TypeError as e:
            raise ValueError(f"Invalid value for parameter {key}: {value!r}") from e

    mask = parameters.get("point_filters", {}).get("downsample_mask", {})
    config.downsample_mask_path = mask.get("path") or None
    return config


class HesaiDecoder:
    """Decodes the packets of one Hesai LiDAR into point clouds.

    Packets are the unit of work. The messages they arrive in are only containers: scans do not
    have to be aligned with messages, and packets are accumulated across calls.
    """

    def __init__(self, parameters: Dict[str, Any]) -> None:
        """Create a decoder.

        Args:
            parameters: The parameters of the sensor, in the layout of Nebula's Hesai parameter
                files without the ``/**: ros__parameters:`` root (see
                :func:`nebula_py.load_parameters`). Parameters that the decoder does not use are
                ignored. ``calibration_file`` (``correction_file`` for AT128 models) has to be the
                path of an existing file.

        Raises:
            ValueError: If a parameter is missing, or has the wrong type or an invalid value.
        """
        config = _sensor_configuration(parameters)
        calibration_file = _parameter(parameters, _ext.calibration_parameter_name(config))
        self._impl = _ext.HesaiDecoder(config, calibration_file)

    def decode(
        self,
        packets: Union[bytes, PandarScan, NebulaPackets, NebulaPacket],
        message_type: Optional[_MessageType] = None,
    ) -> "Decoded[DecodeError]":
        """Decode a batch of packets.

        Every scan that the batch completes results in a :class:`~nebula_py.DecodeResult`. That
        can be any number, including none. The scan that is incomplete when the data ends is never
        output.

        Args:
            packets: The packets, in the order they were received: a serialized message in the
                format that rosbags store (CDR), or a message object. This method serializes a
                message object internally before decoding it, so passing the serialized message is
                faster.
            message_type: The type of a serialized message, as the name that rosbags store
                (``"pandar_msgs/msg/PandarScan"``, ``"nebula_msgs/msg/NebulaPackets"`` or
                ``"nebula_msgs/msg/NebulaPacket"``) or as the message class. It has to match
                ``packets``: bytes of another type may be decoded as garbage instead of raising.
                Not used for message objects.

        Returns:
            The completed scans, and the packets that could not be decoded.

        Raises:
            TypeError: If the message is not one of the types above, or ``message_type`` is
                missing for a serialized message (or given for a message object).
            ValueError: If ``packets`` cannot be deserialized as ``message_type``, or a packet's
                receive timestamp is older than that of the packet before it. No packet of the
                batch is decoded in that case. Call :meth:`reset` before feeding data from an
                earlier time.
        """
        if isinstance(packets, bytes):
            if message_type is None:
                raise TypeError("message_type is required for a serialized message")
            decode_function = _decode_function(message_type)
        elif type(packets) in _DECODE_FUNCTIONS:
            if message_type is not None:
                raise TypeError("message_type is only for serialized messages")
            decode_function = _DECODE_FUNCTIONS[type(packets)]
            packets = serialize_message(packets)
        else:
            raise TypeError(
                "expected a serialized message (bytes), or a PandarScan, NebulaPackets or "
                f"NebulaPacket object, got {type(packets).__name__}"
            )

        results, errors = decode_function(self._impl, packets)
        return Decoded(
            results=tuple(
                DecodeResult(deserialize_message(pointcloud, PointCloud2), lost_packets)
                for pointcloud, lost_packets in results
            ),
            errors=MappingProxyType({DecodeError(code): n for code, n in errors.items()}),
        )

    def reset(self) -> None:
        """Discard all state, as if the decoder had just been created.

        This drops the scan that is being accumulated, and the packet loss counted for it. It is
        required before feeding packets that are older than the ones decoded so far.
        """
        self._impl.reset()
