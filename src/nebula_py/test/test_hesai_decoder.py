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

from pathlib import Path
import subprocess
import sys

from ament_index_python.packages import get_package_share_directory
from nebula_msgs.msg import NebulaPacket
from nebula_msgs.msg import NebulaPackets
from pandar_msgs.msg import PandarScan
import pytest
from rclpy.serialization import deserialize_message
from rclpy.serialization import serialize_message
import rosbag2_py
from sensor_msgs.msg import PointCloud2

from nebula_py import DecodeError
from nebula_py import HesaiDecoder
from nebula_py import load_parameters

HESAI_SHARE = Path(get_package_share_directory("nebula_hesai"))
GROUND_TRUTH = HESAI_SHARE / "test_resources" / "decoder_ground_truth"

XT32_BAG = GROUND_TRUTH / "xt32" / "1673400677802009732"
XT32_PCD = GROUND_TRUTH / "xt32" / "1673400678101968885.pcd"
OT128_BAG = GROUND_TRUTH / "ot128" / "1730271167765338806"
AT128_BAG = GROUND_TRUTH / "at128" / "1679653308406038376"


def read_messages(bag_path):
    """Return the serialized messages of /pandar_packets, each with its type name."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    messages = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == "/pandar_packets":
            messages.append((data, topic_types[topic]))
    return messages


def read_scans(bag_path):
    return [deserialize_message(data, PandarScan) for data, _ in read_messages(bag_path)]


def to_nebula_packets(scan):
    return NebulaPackets(
        header=scan.header,
        packets=[NebulaPacket(stamp=p.stamp, data=bytes(p.data[: p.size])) for p in scan.packets],
    )


def pcd_point_count(pcd_path):
    with open(pcd_path, "rb") as f:
        for line in f:
            if line.startswith(b"POINTS"):
                return int(line.split()[1])
    raise ValueError(f"no POINTS line in {pcd_path}")


def decode_all(decoder, messages):
    return [result for message in messages for result in decoder.decode(message)]


def decode_serialized(decoder, messages):
    return [result for data, type_name in messages for result in decoder.decode(data, type_name)]


def packets_of(scans):
    return [packet for scan in scans for packet in to_nebula_packets(scan).packets]


@pytest.fixture
def xt32_parameters():
    return load_parameters(HESAI_SHARE / "config" / "PandarXT32.param.yaml")


@pytest.fixture
def ot128_parameters():
    return load_parameters(HESAI_SHARE / "config" / "Pandar128E4X.param.yaml")


@pytest.fixture
def at128_parameters():
    return load_parameters(HESAI_SHARE / "config" / "PandarAT128.param.yaml")


def test_load_parameters_resolves_root_and_substitutions(xt32_parameters):
    assert xt32_parameters["sensor_model"] == "PandarXT32"
    assert Path(xt32_parameters["calibration_file"]).is_file()
    assert Path(xt32_parameters["calibration_file"]).name == "PandarXT32.csv"


def test_decodes_pandar_scans_like_the_ground_truth(xt32_parameters):
    results = decode_all(HesaiDecoder(xt32_parameters), read_scans(XT32_BAG))

    # The reference point cloud is the scan that the last message of the bag completes
    assert len(results) == 4
    cloud = results[-1].pointcloud
    assert cloud.width * cloud.height == pcd_point_count(XT32_PCD)
    assert cloud.header.frame_id == xt32_parameters["frame_id"]
    assert [f.name for f in cloud.fields][:4] == ["x", "y", "z", "intensity"]


def test_decodes_sensors_that_use_correction_files(at128_parameters):
    results = decode_all(HesaiDecoder(at128_parameters), read_scans(AT128_BAG))

    assert results
    assert all(r.pointcloud.width * r.pointcloud.height > 0 for r in results)
    assert all(r.lost_packets == 0 for r in results)


def test_results_do_not_depend_on_how_packets_are_batched(ot128_parameters):
    scans = read_scans(OT128_BAG)
    expected = decode_all(HesaiDecoder(ot128_parameters), scans)
    assert expected

    as_nebula_packets = [to_nebula_packets(scan) for scan in scans]
    assert decode_all(HesaiDecoder(ot128_parameters), as_nebula_packets) == expected

    one_by_one = packets_of(scans)
    assert decode_all(HesaiDecoder(ot128_parameters), one_by_one) == expected

    everything_at_once = NebulaPackets(packets=one_by_one)
    assert decode_all(HesaiDecoder(ot128_parameters), [everything_at_once]) == expected


def test_serialized_messages_decode_like_message_objects(ot128_parameters):
    scans = read_scans(OT128_BAG)
    expected = decode_all(HesaiDecoder(ot128_parameters), scans)
    assert expected

    # As rosbags store them, with the type name that the bag stores
    assert decode_serialized(HesaiDecoder(ot128_parameters), read_messages(OT128_BAG)) == expected

    # With the message class, for the other message types
    for messages in ([to_nebula_packets(scan) for scan in scans], packets_of(scans)):
        serialized = [(serialize_message(message), type(message)) for message in messages]
        assert decode_serialized(HesaiDecoder(ot128_parameters), serialized) == expected


def test_a_batch_can_result_in_no_scan(xt32_parameters):
    packet = to_nebula_packets(read_scans(XT32_BAG)[0]).packets[0]

    decoded = HesaiDecoder(xt32_parameters).decode(packet)

    assert len(decoded) == 0
    assert list(decoded) == []
    assert not decoded.errors


def test_lost_packets_follow_the_sensor_model(xt32_parameters, ot128_parameters):
    xt32 = decode_all(HesaiDecoder(xt32_parameters), read_scans(XT32_BAG))
    assert [result.lost_packets for result in xt32] == [None] * len(xt32)

    ot128 = decode_all(HesaiDecoder(ot128_parameters), read_scans(OT128_BAG))
    assert ot128
    assert [result.lost_packets for result in ot128] == [0] * len(ot128)


def drop_from_second_scan(parameters, packets, n_dropped):
    """Drop packets from the middle of the second scan.

    Returns the remaining packets, and the index of the first one after the gap.
    """
    decoder = HesaiDecoder(parameters)
    completing = [i for i, packet in enumerate(packets) for _ in decoder.decode(packet)]
    assert len(completing) >= 2

    gap = (completing[0] + completing[1]) // 2
    return packets[:gap] + packets[gap + n_dropped :], gap


def test_lost_packets_count_towards_the_scan_they_are_lost_from(ot128_parameters):
    packets = packets_of(read_scans(OT128_BAG))
    kept, _ = drop_from_second_scan(ot128_parameters, packets, n_dropped=5)

    expected = decode_all(HesaiDecoder(ot128_parameters), packets)
    actual = decode_all(HesaiDecoder(ot128_parameters), kept)

    assert [result.lost_packets for result in actual] == [
        result.lost_packets + (5 if i == 1 else 0) for i, result in enumerate(expected)
    ]


def test_reset_discards_lost_packets_of_the_incomplete_scan(ot128_parameters):
    packets = packets_of(read_scans(OT128_BAG))
    kept, gap = drop_from_second_scan(ot128_parameters, packets, n_dropped=5)
    expected = decode_all(HesaiDecoder(ot128_parameters), packets)

    decoder = HesaiDecoder(ot128_parameters)
    decode_all(decoder, kept[: gap + 1])
    decoder.reset()

    assert decode_all(decoder, packets) == expected


def test_backwards_timestamps_are_rejected_until_reset(xt32_parameters):
    scans = read_scans(XT32_BAG)
    decoder = HesaiDecoder(xt32_parameters)
    expected = decode_all(decoder, scans)

    with pytest.raises(ValueError, match="backwards"):
        decoder.decode(scans[0])

    decoder.reset()
    assert decode_all(decoder, scans) == expected


def test_a_rejected_batch_leaves_the_decoder_untouched(xt32_parameters):
    scans = read_scans(XT32_BAG)
    expected = decode_all(HesaiDecoder(xt32_parameters), scans)

    shuffled = to_nebula_packets(scans[1])
    shuffled.packets.append(to_nebula_packets(scans[0]).packets[0])

    decoder = HesaiDecoder(xt32_parameters)
    results = list(decoder.decode(scans[0]))
    with pytest.raises(ValueError, match="backwards"):
        decoder.decode(shuffled)
    results += decode_all(decoder, scans[1:])

    assert results == expected


def test_undecodable_packets_are_counted(xt32_parameters):
    garbage = NebulaPackets(packets=[NebulaPacket(data=bytes(10)), NebulaPacket(data=bytes(10))])

    decoded = HesaiDecoder(xt32_parameters).decode(garbage)

    assert len(decoded) == 0
    assert sum(decoded.errors.values()) == 2
    assert all(isinstance(error, DecodeError) for error in decoded.errors)


def test_rejects_invalid_parameters(xt32_parameters):
    with pytest.raises(ValueError, match="FoV"):
        HesaiDecoder({**xt32_parameters, "cloud_max_angle": 400})

    with pytest.raises(ValueError, match="calibration"):
        HesaiDecoder({**xt32_parameters, "calibration_file": "/does/not/exist.csv"})

    missing = dict(xt32_parameters)
    del missing["min_range"]
    with pytest.raises(ValueError, match="min_range"):
        HesaiDecoder(missing)

    with pytest.raises(ValueError, match="rotation_speed"):
        HesaiDecoder({**xt32_parameters, "rotation_speed": "fast"})

    with pytest.raises(ValueError, match="Unknown sensor model"):
        HesaiDecoder({**xt32_parameters, "sensor_model": "NotASensor"})

    with pytest.raises(ValueError, match="mask.png"):
        HesaiDecoder(
            {**xt32_parameters, "point_filters": {"downsample_mask": {"path": "mask.png"}}}
        )


def test_rejects_other_message_types(xt32_parameters):
    decoder = HesaiDecoder(xt32_parameters)
    data, _ = read_messages(XT32_BAG)[0]

    with pytest.raises(TypeError, match="sensor_msgs/msg/PointCloud2"):
        decoder.decode(data, "sensor_msgs/msg/PointCloud2")
    with pytest.raises(TypeError, match="PointCloud2"):
        decoder.decode(data, PointCloud2)
    with pytest.raises(TypeError, match="PointCloud2"):
        decoder.decode(PointCloud2())


def test_message_type_is_only_for_serialized_messages(xt32_parameters):
    decoder = HesaiDecoder(xt32_parameters)
    data, message_type = read_messages(XT32_BAG)[0]

    with pytest.raises(TypeError, match="message_type"):
        decoder.decode(data)
    with pytest.raises(TypeError, match="message_type"):
        decoder.decode(deserialize_message(data, PandarScan), message_type)


def test_rejects_malformed_serialized_messages(xt32_parameters):
    messages = read_messages(XT32_BAG)
    data, message_type = messages[0]
    decoder = HesaiDecoder(xt32_parameters)

    for malformed in (b"", data[:4], data[: len(data) // 2]):
        with pytest.raises(ValueError, match="Could not deserialize pandar_msgs/msg/PandarScan"):
            decoder.decode(malformed, message_type)

    # The rejected calls left the decoder as it was
    expected = decode_serialized(HesaiDecoder(xt32_parameters), messages)
    assert decode_serialized(decoder, messages) == expected


def test_example_runs():
    example = Path(__file__).parents[1] / "examples" / "decode_hesai_bag.py"

    output = subprocess.run(
        [sys.executable, str(example)], check=True, capture_output=True, text=True
    )

    assert "points" in output.stderr
