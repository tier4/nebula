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

"""Outputs that the decoders of all vendors have in common."""

from dataclasses import dataclass
from typing import Generic
from typing import Iterator
from typing import Mapping
from typing import Optional
from typing import Tuple
from typing import TypeVar

from sensor_msgs.msg import PointCloud2

__all__ = ["DecodeResult", "Decoded"]

ErrorT = TypeVar("ErrorT")


@dataclass(frozen=True)
class DecodeResult:
    """A completed scan.

    Attributes:
        pointcloud: The scan, as the ROS 2 node publishes it. Its header stamp is the time of the
            scan according to the sensor.
        lost_packets: The number of packets lost since the previous scan (for the first scan:
            since the decoder was created or reset). A loss is noticed when the packet after it
            arrives, so a loss across the end of a scan counts towards the scan that ends. ``None``
            if the sensor model does not support packet loss detection.
    """

    pointcloud: PointCloud2
    lost_packets: Optional[int]


@dataclass(frozen=True)
class Decoded(Generic[ErrorT]):
    """What decoding a batch of packets resulted in.

    Iterating yields the ``results``.

    Attributes:
        results: The scans that the batch completed, in order.
        errors: The number of packets that did not contribute to a scan, by reason.
    """

    results: Tuple[DecodeResult, ...]
    errors: Mapping[ErrorT, int]

    def __iter__(self) -> Iterator[DecodeResult]:
        return iter(self.results)

    def __len__(self) -> int:
        return len(self.results)
