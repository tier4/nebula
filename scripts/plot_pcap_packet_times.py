#!/usr/bin/python3

import socket
import dpkt
import time
import argparse
import os
import tqdm

from dpkt.udp import UDP
from dpkt.tcp import TCP
from dpkt.arp import ARP
from dpkt.ip import IP

from matplotlib import pyplot as plt

Y_ARP = 0
Y_TCP1 = 2
Y_TCP2 = 3
Y_TCP3 = 4
Y_TCP4 = 5
Y_UDP = 7

parser = argparse.ArgumentParser(
    description="Replay a PCAP file containing UDP packets while rewriting the target IP address"
)
parser.add_argument("input", help="The PCAP file to read from. Supported formats: .pcap, .pcapng")

args = parser.parse_args()

file_type = os.path.splitext(args.input)[1]

t0 = None

packets = {
    Y_ARP: [],
    Y_TCP1: [],
    Y_TCP2: [],
    Y_TCP3: [],
    Y_TCP4: [],
    Y_UDP: [],
}

tcp_streams = {}

with open(args.input, "rb") as f:
    if file_type == ".pcap":
        pcap = dpkt.pcap.Reader(f)
    elif file_type == ".pcapng":
        pcap = dpkt.pcapng.Reader(f)
    else:
        print(f"Unknown file type: {file_type}. Expected .pcap or .pcapng.")
        exit(1)

    for i, (timestamp_s, buf) in enumerate(pcap):
        # This automatically discards everything that is not a UDP packet
        try:
            eth = dpkt.ethernet.Ethernet(buf)

            if t0 is None:
                t0 = timestamp_s

            t = timestamp_s - t0

            if type(eth.data) == ARP:
                packets[Y_ARP].append(t)
                continue
            
            ip: IP = eth.data
            
            if type(ip.data) == UDP:
                packets[Y_UDP].append(t)
                continue
            
            if type(ip.data) != TCP:
                print(f"Skipped {type(ip)}")
                continue
            
            tcp: TCP = ip.data
            stream_tup = (ip.src, ip.dst, tcp.sport, tcp.dport)
            if stream_tup not in tcp_streams:
                tcp_streams[stream_tup] = len(tcp_streams)
            
            stream_id = tcp_streams[stream_tup]
            packets[Y_TCP1 + stream_id].append(t)
        except AttributeError:
            continue
        
fig, ax = plt.subplots()
ax: plt.Axes
for y in [Y_ARP, Y_TCP1, Y_TCP2, Y_TCP3, Y_TCP4, Y_UDP]:
  ax.scatter(packets[y], [y] * len(packets[y]))

plt.show()