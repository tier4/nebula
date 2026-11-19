#!/usr/bin/env python3
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

"""Decode the Hesai packets in a rosbag, and log the number of points of each scan.

Without arguments, this decodes a recording of a Pandar128E4X from the test resources of
nebula_hesai.
"""

import argparse
import logging
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from nebula_msgs.msg import NebulaPackets
from pandar_msgs.msg import PandarScan
from rclpy.serialization import deserialize_message
import rosbag2_py

from nebula_py import DecodeError
from nebula_py import HesaiDecoder
from nebula_py import load_parameters

MESSAGE_TYPES = {
    "pandar_msgs/msg/PandarScan": PandarScan,
    "nebula_msgs/msg/NebulaPackets": NebulaPackets,
}

logger = logging.getLogger("decode_hesai_bag")


def main() -> None:
    hesai_share = Path(get_package_share_directory("nebula_hesai"))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--bag",
        type=Path,
        default=hesai_share / "test_resources/decoder_ground_truth/ot128/1730271167765338806",
    )
    parser.add_argument(
        "--params", type=Path, default=hesai_share / "config/Pandar128E4X.param.yaml"
    )
    parser.add_argument("--topic", default="/pandar_packets")
    args = parser.parse_args()

    # The decoder logs to Python's logging as well, under `nebula_py`
    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] [%(name)s]: %(message)s")

    decoder = HesaiDecoder(load_parameters(args.params))

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag)), rosbag2_py.ConverterOptions("cdr", "cdr")
    )
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    message_type = MESSAGE_TYPES[topic_types[args.topic]]
    reader.set_filter(rosbag2_py.StorageFilter(topics=[args.topic]))

    while reader.has_next():
        _, data, _ = reader.read_next()
        decoded = decoder.decode(deserialize_message(data, message_type))

        for error, n_packets in decoded.errors.items():
            if error is DecodeError.PACKET_IGNORED:
                continue  # Packets without points, e.g. IMU or fault messages
            logger.warning("%d packets could not be decoded: %s", n_packets, error.name)

        for result in decoded:
            cloud = result.pointcloud
            stamp = cloud.header.stamp
            logger.info(
                "Scan at %d.%09d: %d points",
                stamp.sec,
                stamp.nanosec,
                cloud.width * cloud.height,
            )

            # None for sensor models that cannot detect packet loss
            if result.lost_packets:
                logger.warning("%d packets lost since the previous scan", result.lost_packets)


if __name__ == "__main__":
    main()
