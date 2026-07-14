#!/usr/bin/env python3
"""Export NebulaPacket topics from a ROS 2 bag to classic PCAP.

The converter is intended for sharing captured Nebula LiDAR packet data with
non-ROS users. It reads one ROS 2 bag topic of type
``nebula_msgs/msg/NebulaPacket`` or ``nebula_msgs/msg/NebulaPackets``, uses each
``NebulaPacket.stamp`` as the PCAP packet timestamp, reconstructs synthetic
Ethernet/IPv4/UDP headers around the raw packet payload, and writes a classic
``.pcap`` file with Ethernet link type (DLT_EN10MB).

Example:
    # Find candidate Nebula packet topics in a bag.
    python3 scripts/nebula_rosbag_to_pcap.py \
      --bag /path/to/rosbag \
      --list-topics

    # Export a selected topic to PCAP with the default LiDAR-oriented network
    # profile.
    python3 scripts/nebula_rosbag_to_pcap.py \
      --bag /path/to/rosbag \
      --topic /sensing/lidar/front/nebula_packets \
      --output /tmp/front_lidar.pcap

Useful options:
    --list-topics             Print bag topics; supported Nebula packet topics
                              are marked with ``*``.
    --src-ip / --dst-ip       Override synthesized IPv4 addresses.
    --src-port / --dst-port   Override synthesized UDP ports.
    --src-mac / --dst-mac     Override synthesized Ethernet MAC addresses.
    --udp-checksum compute    Compute UDP checksums instead of writing zero.

Defaults are deterministic LiDAR-oriented values (192.168.1.10 ->
192.168.1.201, UDP 2368 -> 2368, source MAC 02:00:00:00:00:01, broadcast
destination MAC). Because NebulaPacket payloads do not contain original network
transport metadata, override these values when the downstream PCAP consumer
expects a specific sensor/network profile.

Runtime requirements:
    Source a ROS 2 environment where ``rosbag2_py``, ``rclpy``,
    ``rosidl_runtime_py``, and ``nebula_msgs`` are importable. No Nebula package
    rebuild is required to run this standalone script.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import ipaddress
from pathlib import Path
import struct
import sys
from typing import Iterator
from typing import List
from typing import Sequence
from typing import Tuple

SUPPORTED_TYPES = {
    "nebula_msgs/msg/NebulaPacket",
    "nebula_msgs/msg/NebulaPackets",
}
MAX_UDP_PAYLOAD_SIZE = 65507


@dataclass(frozen=True)
class NetworkProfile:
    src_ip: bytes
    dst_ip: bytes
    src_port: int
    dst_port: int
    src_mac: bytes
    dst_mac: bytes
    udp_checksum_mode: str


@dataclass
class Stats:
    messages_read: int = 0
    packets_exported: int = 0
    empty_payload_packets: int = 0
    dropped_packets: int = 0
    non_monotonic_timestamps: int = 0


def parse_mac(mac: str) -> bytes:
    parts = mac.split(":")
    if len(parts) != 6:
        raise ValueError(f"Invalid MAC '{mac}'")
    try:
        raw = bytes(int(part, 16) for part in parts)
    except ValueError as exc:
        raise ValueError(f"Invalid MAC '{mac}'") from exc
    if len(raw) != 6:
        raise ValueError(f"Invalid MAC '{mac}'")
    return raw


def ip4_bytes(ip: str) -> bytes:
    return ipaddress.IPv4Address(ip).packed


def checksum16(data: bytes) -> int:
    if len(data) % 2:
        data += b"\x00"
    total = 0
    for i in range(0, len(data), 2):
        total += (data[i] << 8) + data[i + 1]
        total = (total & 0xFFFF) + (total >> 16)
    return (~total) & 0xFFFF


def build_udp_segment(profile: NetworkProfile, payload: bytes) -> bytes:
    length = 8 + len(payload)
    checksum = 0
    udp_header = struct.pack("!HHHH", profile.src_port, profile.dst_port, length, checksum)
    if profile.udp_checksum_mode == "compute":
        pseudo_header = profile.src_ip + profile.dst_ip + struct.pack("!BBH", 0, 17, length)
        checksum = checksum16(pseudo_header + udp_header + payload)
        if checksum == 0:
            checksum = 0xFFFF
        udp_header = struct.pack("!HHHH", profile.src_port, profile.dst_port, length, checksum)
    return udp_header + payload


def build_ipv4_packet(profile: NetworkProfile, udp_segment: bytes, identification: int) -> bytes:
    version_ihl = 0x45
    dscp_ecn = 0
    total_length = 20 + len(udp_segment)
    flags_fragment = 0
    ttl = 64
    protocol = 17
    header_checksum = 0
    header = struct.pack(
        "!BBHHHBBH4s4s",
        version_ihl,
        dscp_ecn,
        total_length,
        identification & 0xFFFF,
        flags_fragment,
        ttl,
        protocol,
        header_checksum,
        profile.src_ip,
        profile.dst_ip,
    )
    header_checksum = checksum16(header)
    header = struct.pack(
        "!BBHHHBBH4s4s",
        version_ihl,
        dscp_ecn,
        total_length,
        identification & 0xFFFF,
        flags_fragment,
        ttl,
        protocol,
        header_checksum,
        profile.src_ip,
        profile.dst_ip,
    )
    return header + udp_segment


def build_ethernet_frame(profile: NetworkProfile, ip_packet: bytes) -> bytes:
    eth_type_ipv4 = 0x0800
    return profile.dst_mac + profile.src_mac + struct.pack("!H", eth_type_ipv4) + ip_packet


class PcapWriter:
    def __init__(self, path: Path) -> None:
        self._fh = path.open("wb")
        # Classic PCAP little-endian, microsecond resolution, DLT_EN10MB.
        self._fh.write(struct.pack("<IHHIIII", 0xA1B2C3D4, 2, 4, 0, 0, 65535, 1))

    def write_packet(self, ts_sec: int, ts_nsec: int, frame: bytes) -> None:
        ts_usec = ts_nsec // 1000
        incl_len = len(frame)
        self._fh.write(struct.pack("<IIII", ts_sec, ts_usec, incl_len, incl_len))
        self._fh.write(frame)

    def close(self) -> None:
        self._fh.close()


class BagReaderError(RuntimeError):
    pass


def import_ros_deps():
    try:
        from rclpy.serialization import deserialize_message  # type: ignore
        import rosbag2_py  # type: ignore
        from rosidl_runtime_py.utilities import get_message  # type: ignore

        return rosbag2_py, deserialize_message, get_message
    except Exception as exc:
        raise BagReaderError(
            "ROS 2 Python dependencies are unavailable. Source your ROS 2 "
            "environment so rosbag2_py, rclpy, and rosidl_runtime_py are importable."
        ) from exc


def get_topics(reader) -> List[Tuple[str, str]]:
    topics = reader.get_all_topics_and_types()
    return sorted((topic.name, topic.type) for topic in topics)


def iter_target_messages(reader, target_topic: str) -> Iterator[Tuple[str, bytes, int]]:
    while reader.has_next():
        topic_name, serialized_data, timestamp = reader.read_next()
        if topic_name == target_topic:
            yield topic_name, serialized_data, timestamp


def packet_items_from_message(msg, topic_type: str):
    if topic_type == "nebula_msgs/msg/NebulaPacket":
        return [msg]
    return msg.packets


def validate_ports(src_port: int, dst_port: int) -> None:
    for name, value in (("src-port", src_port), ("dst-port", dst_port)):
        if value < 1 or value > 65535:
            raise ValueError(f"--{name} must be in [1, 65535], got {value}")


def run(args: argparse.Namespace) -> int:
    rosbag2_py, deserialize_message, get_message = import_ros_deps()

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=args.bag, storage_id="")
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader.open(storage_options, converter_options)

    topics = get_topics(reader)

    if args.list_topics:
        print("Topics in bag:")
        for name, topic_type in topics:
            marker = "*" if topic_type in SUPPORTED_TYPES else " "
            print(f" {marker} {name}: {topic_type}")
        return 0

    if not args.topic or not args.output:
        raise ValueError("--topic and --output are required unless --list-topics is specified")

    topic_map = dict(topics)
    if args.topic not in topic_map:
        available = "\n".join(f"- {name} ({topic_type})" for name, topic_type in topics)
        raise ValueError(f"Topic '{args.topic}' not found in bag. Available topics:\n{available}")

    topic_type = topic_map[args.topic]
    if topic_type not in SUPPORTED_TYPES:
        raise ValueError(
            f"Unsupported topic type for {args.topic}: {topic_type}. "
            f"Supported: {sorted(SUPPORTED_TYPES)}"
        )

    msg_type = get_message(topic_type)
    profile = NetworkProfile(
        src_ip=ip4_bytes(args.src_ip),
        dst_ip=ip4_bytes(args.dst_ip),
        src_port=args.src_port,
        dst_port=args.dst_port,
        src_mac=parse_mac(args.src_mac),
        dst_mac=parse_mac(args.dst_mac),
        udp_checksum_mode=args.udp_checksum,
    )

    stats = Stats()
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    writer = PcapWriter(output_path)
    identification = 0
    last_stamp_ns = None

    try:
        for _, serialized_data, _ in iter_target_messages(reader, args.topic):
            stats.messages_read += 1
            msg = deserialize_message(serialized_data, msg_type)
            for packet in packet_items_from_message(msg, topic_type):
                payload = bytes(packet.data)
                if len(payload) > MAX_UDP_PAYLOAD_SIZE:
                    stats.dropped_packets += 1
                    continue
                if len(payload) == 0:
                    stats.empty_payload_packets += 1

                ts_sec = int(packet.stamp.sec)
                ts_nsec = int(packet.stamp.nanosec)
                stamp_ns = ts_sec * 1_000_000_000 + ts_nsec
                if last_stamp_ns is not None and stamp_ns < last_stamp_ns:
                    stats.non_monotonic_timestamps += 1
                last_stamp_ns = stamp_ns

                udp_segment = build_udp_segment(profile, payload)
                ip_packet = build_ipv4_packet(profile, udp_segment, identification)
                frame = build_ethernet_frame(profile, ip_packet)
                writer.write_packet(ts_sec, ts_nsec, frame)
                identification = (identification + 1) & 0xFFFF
                stats.packets_exported += 1
    finally:
        writer.close()

    if stats.packets_exported == 0:
        raise RuntimeError("No packets exported. Check topic contents and message timestamps.")

    print("Conversion summary")
    print(f"- bag: {args.bag}")
    print(f"- topic: {args.topic}")
    print(f"- topic_type: {topic_type}")
    print(f"- messages_read: {stats.messages_read}")
    print(f"- packets_exported: {stats.packets_exported}")
    print(f"- dropped_packets: {stats.dropped_packets}")
    print(f"- empty_payload_packets: {stats.empty_payload_packets}")
    print(f"- non_monotonic_timestamps: {stats.non_monotonic_timestamps}")
    print(f"- output: {output_path}")
    return 0


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, help="Path to ROS 2 bag directory")
    parser.add_argument("--topic", help="Target topic name")
    parser.add_argument("--output", help="Output .pcap path")
    parser.add_argument("--list-topics", action="store_true", help="List topics and exit")
    parser.add_argument("--src-ip", default="192.168.1.10")
    parser.add_argument("--dst-ip", default="192.168.1.201")
    parser.add_argument("--src-port", type=int, default=2368)
    parser.add_argument("--dst-port", type=int, default=2368)
    parser.add_argument("--src-mac", default="02:00:00:00:00:01")
    parser.add_argument("--dst-mac", default="ff:ff:ff:ff:ff:ff")
    parser.add_argument("--udp-checksum", choices=("zero", "compute"), default="zero")
    args = parser.parse_args(argv)
    validate_ports(args.src_port, args.dst_port)
    return args


if __name__ == "__main__":
    try:
        raise SystemExit(run(parse_args(sys.argv[1:])))
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
