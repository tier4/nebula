from __future__ import annotations

from argparse import ArgumentParser
from dataclasses import dataclass
from ipaddress import IPv4Address
from ipaddress import IPv4Network
from ipaddress import ip_interface
import socket
import struct
import sys
from typing import Any

from rich import print  # noqa: A004
from rich.syntax import Syntax
from rich.table import Table

GET_CONFIG_INFO_COMMAND = 0x08
GET_LIDAR_STATUS_COMMAND = 0x09
GET_PTP_CONFIG_COMMAND = 0x26
SET_DESTINATION_IP_COMMAND = 0x20
SET_CONTROL_PORT_COMMAND = 0x21


def unpack(struct_fields: str, buffer: bytes) -> Any:
    return struct.unpack(struct_fields, buffer)[0]


class PtcCommandBase:
    def parse(self) -> dict[str, str]:
        raise NotImplementedError()

    def print(self, title: str) -> None:  # noqa: A003
        table = Table("Parameter", "Value", title=title, highlight=True, min_width=45)
        for key, value in self.parse().items():
            table.add_row(key, str(value))

        print(table)

    def __str__(self):
        dst = ""

        for key, value in self.parse().items():
            dst += f"{key}: {value}\n"

        return dst


RETURN_MODE_MAP = {
    0: "Last",
    1: "Strongest",
    2: "Last + Strongest",
    3: "First",
    4: "Last + First",
    5: "First + Strongest",
}


@dataclass
class PtcCommandGetConfigInfo(PtcCommandBase):
    ipaddr: bytes
    mask: bytes
    gateway: bytes
    dest_ipaddr: bytes
    dest_lidar_udp_port: bytes
    dest_gps_udp_port: bytes
    spin_rate: bytes
    sync: bytes
    sync_angle: bytes
    start_angle: bytes
    stop_angle: bytes
    clock_source: bytes
    udp_seq: bytes
    trigger_method: bytes
    return_mode: bytes
    standby_mode: bytes
    motor_status: bytes
    vlan_flag: bytes
    vlan_id: bytes
    clock_data_fmt: bytes
    noise_filtering: bytes
    reflectivity_mapping: bytes

    def parse(self) -> dict[str, str]:
        return {
            "ipaddr": socket.inet_ntoa(self.ipaddr),
            "mask": socket.inet_ntoa(self.mask),
            "gateway": socket.inet_ntoa(self.gateway),
            "dest_ipaddr": socket.inet_ntoa(self.dest_ipaddr),
            "dest_lidar_udp_port": unpack("!H", self.dest_lidar_udp_port),
            "dest_gps_udp_port": unpack("!H", self.dest_gps_udp_port),
            "spin_rate": f'{unpack("!H", self.spin_rate)} RPM',
            "sync": "Enabled" if unpack("!B", self.sync) else "Disabled",
            "sync_angle": f'{unpack("!H", self.sync_angle) * 0.01} deg',
            "start_angle": unpack("!H", self.start_angle),
            "stop_angle": unpack("!H", self.stop_angle),
            "clock_source": "PTP" if unpack("!B", self.clock_source) else "Unknown",
            "udp_seq": "ON" if unpack("!B", self.udp_seq) else "OFF",
            "trigger_method": "Time-based" if unpack("!B", self.trigger_method) else "Angle-based",
            "return_mode": RETURN_MODE_MAP[unpack("!B", self.return_mode)],
            "standby_mode": "Standby" if unpack("!B", self.standby_mode) else "In operation",
            "motor_status": unpack("!B", self.motor_status),
            "vlan_flag": unpack("!B", self.vlan_flag),
            "vlan_id": unpack("!H", self.vlan_id),
            "clock_data_fmt": unpack("!B", self.clock_data_fmt),
            "noise_filtering": unpack("!B", self.noise_filtering),
            "reflectivity_mapping": unpack("!B", self.reflectivity_mapping),
        }


PTP_STATUS_MAP = {
    0: "free run",
    1: "tracking",
    2: "locked",
    3: "frozen",
}


@dataclass
class PtcCommandGetLidarStatus(PtcCommandBase):
    system_uptime: bytes
    motor_speed: bytes
    temperature_0: bytes
    temperature_1: bytes
    temperature_2: bytes
    temperature_3: bytes
    temperature_4: bytes
    temperature_5: bytes
    temperature_6: bytes
    temperature_7: bytes
    gps_pps_lock: bytes
    gps_gprmc_status: bytes
    startup_times: bytes
    total_operation_time: bytes
    ptp_status: bytes
    humidity: bytes

    def parse(self) -> dict[str, str]:
        return {
            "system_uptime": f'{unpack("!I", self.system_uptime)} s',
            "motor_speed": f'{unpack("!H", self.motor_speed)} RPM',
            "temperature_0": f'{unpack("!i", self.temperature_0) * 0.01:.1f} deg C',
            "temperature_1": f'{unpack("!i", self.temperature_1) * 0.01:.1f} deg C',
            "temperature_2": f'{unpack("!i", self.temperature_2) * 0.01:.1f} deg C',
            "temperature_3": f'{unpack("!i", self.temperature_3) * 0.01:.1f} deg C',
            "temperature_4": f'{unpack("!i", self.temperature_4) * 0.01:.1f} deg C',
            "temperature_5": f'{unpack("!i", self.temperature_5) * 0.01:.1f} deg C',
            "temperature_6": f'{unpack("!i", self.temperature_6) * 0.01:.1f} deg C',
            "temperature_7": f'{unpack("!i", self.temperature_7) * 0.01:.1f} deg C',
            "gps_pps_lock": unpack("!B", self.gps_pps_lock),
            "gps_gprmc_status": unpack("!B", self.gps_gprmc_status),
            "startup_times": unpack("!I", self.startup_times),
            "total_operation_time": unpack("!I", self.total_operation_time),
            "ptp_status": PTP_STATUS_MAP[unpack("!B", self.ptp_status)],
            "humidity": f'{unpack("!I", self.humidity) * 0.1:.1f} %',
        }


@dataclass
class PtcCommandGetPtpConfig(PtcCommandBase):
    status: bytes
    profile: bytes
    domain: bytes
    network: bytes
    tsn_switch: bytes

    def parse(self) -> dict[str, str]:
        return {
            "status": "Enabled" if unpack("!B", self.status) else "Disabled",
            "profile": "802.1AS (AutoSAR)" if unpack("!B", self.profile) == 3 else "Unknown",
            "domain": unpack("!B", self.domain),
            "network": "L2" if unpack("!B", self.network) == 1 else "Unknown",
            "tsn_switch": "TSN" if unpack("!B", self.tsn_switch) else "Non-TSN",
        }


def compose_packet(command: str, payload_length: int = 0, payload: bytes = b"") -> bytes:
    struct_fields = ">B B B B I {}s".format(payload_length)

    return struct.pack(struct_fields, 0x47, 0x74, command, 0x00, payload_length, payload)


def get_config_info(sock: socket.socket):
    sock.sendall(compose_packet(GET_CONFIG_INFO_COMMAND))

    response = list(sock.recv(8))
    payload_length = sum(response[4:])
    payload = sock.recv(payload_length)

    fields = struct.unpack("4s 4s 4s 4s 2s 2s 2s s 2s 2s 2s s s s s s s s 2s s s s", payload[:41])

    return PtcCommandGetConfigInfo(*fields)


def get_lidar_status(sock: socket.socket):
    sock.sendall(compose_packet(GET_LIDAR_STATUS_COMMAND))

    response = list(sock.recv(8))
    payload_length = sum(response[4:])
    payload = sock.recv(payload_length)

    fields = struct.unpack("4s 2s 4s 4s 4s 4s 4s 4s 4s 4s s s 4s 4s s 4s", payload[:53])

    return PtcCommandGetLidarStatus(*fields)


def get_ptp_config(sock: socket.socket):
    sock.sendall(compose_packet(GET_PTP_CONFIG_COMMAND))

    response = list(sock.recv(8))
    payload_length = sum(response[4:])
    payload = sock.recv(payload_length)

    fields = struct.unpack("s s s s s", payload[:5])

    return PtcCommandGetPtpConfig(*fields)


def is_valid_ip(ip: str | None) -> bool:
    if ip is None:
        return True

    try:
        IPv4Address(ip)
    except ValueError:
        return False
    else:
        return True


def is_valid_netmask(mask: str) -> bool:
    mask = mask.replace("/", "")

    try:
        IPv4Network("0.0.0.0/" + mask)
    except ValueError:
        return False
    else:
        return True


def normalize_netmask(mask: str) -> str:
    mask = mask.replace("/", "")

    return str(IPv4Network("0.0.0.0/" + mask).netmask)


def is_same_subnet(destination_ip: str, sensor_ip: str, mask: str) -> bool:
    if destination_ip == "255.255.255.255":
        return True

    return (
        ip_interface(f"{destination_ip}/{mask}").network
        == ip_interface(f"{sensor_ip}/{mask}").network
    )


def yesno(question):
    prompt = f"{question} [y/n]: "
    answer = input(prompt).strip().lower()
    if answer not in ("y", "n"):
        print(f"{answer} is invalid, please try again...")
        return yesno(question)

    return answer == "y"


if __name__ == "__main__":

    class NameSpace:
        sensor_ip: str
        destination_ip: str | None
        data_port: int | None
        new_sensor_ip: str | None
        mask: str | None

    parser = ArgumentParser()
    parser.add_argument(
        "--sensor-ip",
        type=str,
        required=True,
        help="The current sensor IP address",
    )
    parser.add_argument(
        "--destination-ip",
        type=str,
        default=None,
        help="Change the current destination IP address to the given one",
    )
    parser.add_argument(
        "--data-port",
        type=int,
        default=None,
        help="Change the current destination LiDAR data port to the given one",
    )
    parser.add_argument(
        "--new-sensor-ip",
        type=str,
        default=None,
        help="Change the current sensor IP address to the given one",
    )
    parser.add_argument(
        "--mask",
        type=str,
        default=None,
        help=(
            "Change the current net mask to the given one. "
            "You can pass it in either a CIDR notation or dotted-decimal notation."
        ),
    )

    args = parser.parse_args(namespace=NameSpace)

    if not is_valid_ip(args.sensor_ip):
        raise ValueError(f"Invalid sensor IP {args.sensor_ip}")

    if not is_valid_ip(args.destination_ip):
        raise ValueError(f"Invalid destination IP {args.destination_ip}")

    if not is_valid_ip(args.new_sensor_ip):
        raise ValueError(f"Invalid new sensor IP {args.new_sensor_ip}")

    if args.mask is not None:
        if is_valid_netmask(args.mask):
            args.mask = normalize_netmask(args.mask)
        else:
            raise ValueError(
                f"Invalid net mask {args.mask}. "
                "It must be in the range of 0.0.0.0 (/0) to 255.255.255.255 (/32)."
            )

    if args.data_port is not None:
        if args.data_port < 0 or 65535 < args.data_port:
            raise ValueError(
                f"Invalid data port {args.data_port}. " "It must be in the range of 0 to 65535."
            )

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5)
    sock.connect((args.sensor_ip, 9347))

    try:
        config_info = get_config_info(sock)
        config_info.print(title="Config Info (0x08)")

        print()

        lidar_status = get_lidar_status(sock)
        lidar_status.print(title="Lidar Status (0x09)")

        print()

        ptp_config = get_ptp_config(sock)
        ptp_config.print(title="PTP Config (0x26)")

        print()

        parsed_config_info = config_info.parse()
        current_destination_ip = parsed_config_info["dest_ipaddr"]
        current_data_port = parsed_config_info["dest_lidar_udp_port"]
        current_mask = parsed_config_info["mask"]

        destination_ip = (
            current_destination_ip if args.destination_ip is None else args.destination_ip
        )
        data_port = current_data_port if args.data_port is None else args.data_port
        new_sensor_ip = args.sensor_ip if args.new_sensor_ip is None else args.new_sensor_ip
        mask = current_mask if args.mask is None else args.mask

        # Set Destination IP (0x20)
        if args.destination_ip is not None or args.data_port is not None:
            if not is_same_subnet(destination_ip, args.sensor_ip, mask):
                raise ValueError(
                    f"Destination IP {destination_ip} is not in the same subnet as the sensor IP {args.sensor_ip}."
                    f"The net mask is {mask}."
                )

            print(
                f"Changing destination IP:Port from {current_destination_ip}:{current_data_port} to {destination_ip}:{data_port}...",
                end=" ",
                flush=True,
            )

            payload = socket.inet_aton(destination_ip)
            payload += struct.pack("!H", data_port)
            payload += struct.pack("!H", 2369)

            sock.sendall(compose_packet(SET_DESTINATION_IP_COMMAND, len(payload), payload))

            print("Done")

        # Set Control Port (0x21)
        if args.new_sensor_ip is not None or args.mask is not None:
            if not yesno(
                "Are you sure you want to change the sensor IP address from "
                f"{args.sensor_ip}/{current_mask} to {new_sensor_ip}/{mask}?"
            ):
                print("Aborted")
                sys.exit(0)

            payload = socket.inet_aton(new_sensor_ip)
            payload += socket.inet_aton(mask)
            payload += socket.inet_aton("192.168.1.1")
            payload += struct.pack("!B", 0)
            payload += struct.pack("!H", 0)

            sock.sendall(compose_packet(SET_CONTROL_PORT_COMMAND, len(payload), payload))

            print("Done")

            print()
            print(
                f"Make sure the new sensor IP is successfully set to {new_sensor_ip}/{mask} by the following command"
            )
            print(Syntax(f"python3 {sys.argv[0]} --sensor-ip {new_sensor_ip}", "console"))

        if (
            args.destination_ip is not None
            or args.data_port is not None
            or args.new_sensor_ip is not None
            or args.mask is not None
        ):
            table = Table(
                "Parameter", "Old", "New", title="What's Changed", highlight=True, min_width=50
            )

            if new_sensor_ip != args.sensor_ip:
                table.add_row("sensor_ip", str(args.sensor_ip), str(new_sensor_ip))
            if destination_ip != current_destination_ip:
                table.add_row("destination_ip", str(current_destination_ip), str(destination_ip))
            if data_port != current_data_port:
                table.add_row("data_port", str(current_data_port), str(data_port))
            if mask != current_mask:
                table.add_row("mask", str(current_mask), str(mask))

            print()
            print(table)
    finally:
        sock.close()
