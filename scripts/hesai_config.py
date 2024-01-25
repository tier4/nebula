import argparse
import socket
import struct
from typing import List

from rich import print
from rich.table import Table


class PtcCommandGetConfigInfo:
    def __init__(
        self,
        ipaddr: bytes,
        mask: bytes,
        gateway: bytes,
        dest_ipaddr: bytes,
        dest_lidar_udp_port: bytes,
        dest_gps_udp_port: bytes,
        spin_rate: bytes,
        sync: bytes,
        sync_angle: bytes,
        start_angle: bytes,
        stop_angle: bytes,
        clock_source: bytes,
        udp_seq: bytes,
        trigger_method: bytes,
        return_mode: bytes,
        standby_mode: bytes,
        motor_status: bytes,
        vlan_flag: bytes,
        vlan_id: bytes,
        clock_data_fmt: bytes,
        noise_filtering: bytes,
        reflectivity_mapping: bytes,
    ):
        self.ipaddr = ipaddr
        self.mask = mask
        self.gateway = gateway
        self.dest_ipaddr = dest_ipaddr
        self.dest_lidar_udp_port = dest_lidar_udp_port
        self.dest_gps_udp_port = dest_gps_udp_port
        self.spin_rate = spin_rate
        self.sync = sync
        self.sync_angle = sync_angle
        self.start_angle = start_angle
        self.stop_angle = stop_angle
        self.clock_source = clock_source
        self.udp_seq = udp_seq
        self.trigger_method = trigger_method
        self.return_mode = return_mode
        self.standby_mode = standby_mode
        self.motor_status = motor_status
        self.vlan_flag = vlan_flag
        self.vlan_id = vlan_id
        self.clock_data_fmt = clock_data_fmt
        self.noise_filtering = noise_filtering
        self.reflectivity_mapping = reflectivity_mapping

    def prettyprint(self):
        table = Table(
            "Parameter", "Value"
        )
        table.add_row("ipaddr", socket.inet_ntoa(self.ipaddr))
        table.add_row("mask", socket.inet_ntoa(self.mask))
        table.add_row("gateway", socket.inet_ntoa(self.gateway))
        table.add_row("dest_ipaddr", socket.inet_ntoa(self.dest_ipaddr))
        table.add_row("dest_lidar_udp_port", str(struct.unpack("!H", self.dest_lidar_udp_port)[0]))
        table.add_row("dest_gps_udp_port", str(struct.unpack("!H", self.dest_gps_udp_port)[0]))
        table.add_row("spin_rate", str(struct.unpack("!H", self.spin_rate)[0]))
        table.add_row("sync", str(struct.unpack("!B", self.sync)[0]))
        table.add_row("sync_angle", str(struct.unpack("!H", self.sync_angle)[0]))
        table.add_row("start_angle", str(struct.unpack("!H", self.start_angle)[0]))
        table.add_row("stop_angle", str(struct.unpack("!H", self.stop_angle)[0]))
        table.add_row("clock_source", str(struct.unpack("!B", self.clock_source)[0]))
        table.add_row("udp_seq", str(struct.unpack("!B", self.udp_seq)[0]))
        table.add_row("trigger_method", str(struct.unpack("!B", self.trigger_method)[0]))
        table.add_row("return_mode", str(struct.unpack("!B", self.return_mode)[0]))
        table.add_row("standby_mode", str(struct.unpack("!B", self.standby_mode)[0]))
        table.add_row("motor_status", str(struct.unpack("!B", self.motor_status)[0]))
        table.add_row("vlan_flag", str(struct.unpack("!B", self.vlan_flag)[0]))
        table.add_row("vlan_id", str(struct.unpack("!H", self.vlan_id)[0]))
        table.add_row("clock_data_fmt", str(struct.unpack("!B", self.clock_data_fmt)[0]))
        table.add_row("noise_filtering", str(struct.unpack("!B", self.noise_filtering)[0]))
        table.add_row("reflectivity_mapping", str(struct.unpack("!B", self.reflectivity_mapping)[0]))
        print(table)

    def __str__(self):
        return f'ipaddr: {socket.inet_ntoa(self.ipaddr)}\nmask: {socket.inet_ntoa(self.mask)}\n' \
               f'gateway: {socket.inet_ntoa(self.gateway)}\ndest_ipaddr: {socket.inet_ntoa(self.dest_ipaddr)}\n' \
               f'dest_LiDAR_udp_port: {str(struct.unpack("!H", self.dest_lidar_udp_port)[0])}\n' \
               f'dest_gps_udp_port: {str(struct.unpack("!H", self.dest_gps_udp_port)[0])}\n' \
               f'spin_rate: {str(struct.unpack("!H", self.spin_rate)[0])}\n' \
               f'sync: {str(struct.unpack("!B", self.sync)[0])}\n' \
               f'sync_angle: {str(struct.unpack("!H", self.sync_angle)[0])}\n' \
               f'start_angle: {str(struct.unpack("!H", self.start_angle)[0])}\n' \
               f'stop_angle: {str(struct.unpack("!H", self.stop_angle)[0])}'


class CommandPacket:
    def __init__(self, command_code: int, payload: List):
        self._command_code = command_code
        self._payload_length = len(payload)
        self._payload = payload
        self._pre_header = 0x47
        self._post_header = 0x74
        self._return_code = 1

    def get_command(self) -> bytes:
        command = struct.pack(">B B B B I",
                              self._pre_header,
                              self._post_header,
                              self._command_code,
                              self._return_code,
                              self._payload_length)
        return command


class ResponsePacket:
    def __init__(self):
        self.command_code = 0
        self.payload_length = 0
        self.payload = 0


class GetConfigInfoCommand(CommandPacket):
    def __init__(self):
        super().__init__(0x08, [])


class Client:
    def __init__(self, sensor_model: str, sensor_ip: str = '192.168.225.201'):
        self._host = sensor_ip
        self._sensor_model = sensor_model
        self._ptc_port = 9347
        self._msg_length = 1024

    def receive(self, sock):
        chunks = []
        bytes_recd = 0
        while bytes_recd < self._msg_length:
            #chunk = sock.recv(min(self._msg_length - bytes_recd, 2048))
            print("Receive start")
            chunk, _ = sock.recv(65535)
            print("Receive end")
            if chunk == b'':
                raise RuntimeError("socket connection broken")
            chunks.append(chunk)
            bytes_recd = bytes_recd + len(chunk)
        return b''.join(chunks)

    def start(self, packet_data: bytes):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((self._host, self._ptc_port))
            sock.sendall(packet_data)
            response = list(sock.recv(8))
            payload_length = sum(response[4:])
            payload = sock.recv(payload_length)

            fields = struct.unpack(
                "4s 4s 4s 4s 2s 2s 2s s 2s 2s 2s s s s s s s s 2s s s s", payload[:41]
            )
            info = PtcCommandGetConfigInfo(*fields)
            info.prettyprint()
        finally:
            sock.close()


# Argument Parser
parser = argparse.ArgumentParser(description="Hesai Lidar TCP Config Script")
parser.add_argument('--sensor_model', type=str, help='The Sensor Model.')
parser.add_argument('--sensor_ip', type=str, default='192.168.225.201', help='The Sensor Current IP Address.')

args = parser.parse_args()

# Create a client
client = Client(args.sensor_model, args.sensor_ip)
info_cmd = GetConfigInfoCommand()
client.start(info_cmd.get_command())


#############################
# command_code = 0x08
# payload_length = 0
#
# packet = struct.pack(">B B B B I", 0x47, 0x74, command_code, 0x00, payload_length)
# print("Sent:", len(packet))
#
# sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# sock.connect(("192.168.225.201", 9347))

# try:
#     sock.sendall(packet)
#
#     response = list(sock.recv(8))
#     payload_length = sum(response[4:])
#     payload = sock.recv(payload_length)
#
#     fields = struct.unpack(
#         "4s 4s 4s 4s 2s 2s 2s s 2s 2s 2s s s s s s s s 2s s s s", payload[:41]
#     )
#     info = PtcCommandGetConfigInfo(*fields)
#     print(str(info))
#     info.prettyprint()
#
#
# finally:
#     # Close the socket
#     sock.close()
