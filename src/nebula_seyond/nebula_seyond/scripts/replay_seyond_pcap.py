#!/usr/bin/env python3

import argparse
import socket
import struct
import time
import sys
import os

class FastPcapReader:
    """
    A minimal, high-performance PCAP reader in pure Python.
    """
    def __init__(self, filename):
        self.f = open(filename, "rb")
        global_header = self.f.read(24)
        if len(global_header) < 24:
            raise ValueError("Invalid PCAP file")
        
        # Check magic number for endianness
        magic = struct.unpack("I", global_header[0:4])[0]
        if magic == 0xA1B2C3D4: # Native endian
            self.fmt_header = "IIII" 
            self.fmt_global = "IHHIIII" # magic, major, minor, gmt, sigfigs, snaplen, network
        elif magic == 0xD4C3B2A1: # Swapped endian
            self.fmt_header = "<IIII"
            self.fmt_global = "<IHHIIII"
        else:
            raise ValueError("Unsupported PCAP format (e.g. PCAP-NG or corruption)")

        # Link type is at offset 20 of global header
        _, _, _, _, _, _, self.link_type = struct.unpack(self.fmt_global, global_header)
        print(f"PCAP Link Type: {self.link_type}")

    def __iter__(self):
        while True:
            header_data = self.f.read(16)
            if len(header_data) < 16:
                break
            
            ts_sec, ts_usec, incl_len, orig_len = struct.unpack(self.fmt_header, header_data)
            packet_data = self.f.read(incl_len)
            if len(packet_data) < incl_len:
                break
            
            yield (ts_sec * 1_000_000 + ts_usec, packet_data)

    def close(self):
        self.f.close()

def extract_seyond_payload(packet_data, link_type, fragments):
    """
    Strips Link/IP/UDP headers to find Seyond payload, reassembling if necessary.
    """
    offset = 0
    if link_type == 1: # ETHERNET
        offset = 14
        if len(packet_data) < 14: return None
        proto = struct.unpack(">H", packet_data[12:14])[0]
        while proto == 0x8100: # Handle VLAN tags
            offset += 4
            if len(packet_data) < offset + 2: return None
            proto = struct.unpack(">H", packet_data[offset-2:offset])[0]
        if proto != 0x0800: return None
    elif link_type == 113: # SLL
        offset = 16
        if len(packet_data) < 16: return None
        proto = struct.unpack(">H", packet_data[14:16])[0]
        if proto != 0x0800: return None
    else: return None

    # IPv4 Header
    if len(packet_data) < offset + 20: return None
    ihl = (packet_data[offset] & 0x0F) * 4
    ip_id = struct.unpack(">H", packet_data[offset+4:offset+6])[0]
    frag_info = struct.unpack(">H", packet_data[offset+6:offset+8])[0]
    
    mf = bool(frag_info & 0x2000)
    frag_offset = (frag_info & 0x1FFF) * 8
    
    # We use (src, dst, proto, id) as reassembly key
    # For speed, we just use the IP ID here assuming a single LiDAR source
    src_ip = packet_data[offset+12:offset+16]
    dst_ip = packet_data[offset+16:offset+20]
    key = (src_ip, dst_ip, ip_id)
    
    ip_payload = packet_data[offset + ihl:]
    
    if frag_offset == 0:
        # First fragment: contains UDP header
        if len(ip_payload) < 8: return None
        udp_payload = ip_payload[8:]
        if not mf: # Not fragmented
            # Seyond Magic check
            if len(udp_payload) >= 2 and udp_payload[0:2] == b'\x6a\x17':
                return udp_payload
            return None
        else:
            # Start of a fragmented packet
            fragments[key] = {
                'data': {0: udp_payload},
                'total_len': 0,
                'received_len': len(udp_payload)
            }
            return None
    else:
        # Subsequent fragment
        if key in fragments:
            fragments[key]['data'][frag_offset - 8] = ip_payload
            fragments[key]['received_len'] += len(ip_payload)
            if not mf:
                # Last fragment! Reassemble
                frags = fragments[key]['data']
                sorted_offsets = sorted(frags.keys())
                full_payload = b"".join(frags[o] for o in sorted_offsets)
                del fragments[key]
                
                # Seyond Magic check
                if len(full_payload) >= 2 and full_payload[0:2] == b'\x6a\x17':
                    return full_payload
            return None
    return None

def replay_seyond(pcap_file, target_ip, target_port, rate=1.0, loop=False, max_gap_ms=100.0, use_pcap_time=False):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    try:
        while True:
            reader = FastPcapReader(pcap_file)
            print(f"Replaying {pcap_file} (Mode: {'PCAP' if use_pcap_time else 'Internal'} time)")
            
            fragments = {} # (src, dst, id) -> {'data': {offset: data}, 'mf': bool}
            
            start_wall_time = None
            start_packet_ts_us = None
            last_ts_us = None
            packet_count = 0
            
            for pcap_ts_us, pkt_data in reader:
                payload = extract_seyond_payload(pkt_data, reader.link_type, fragments)
                if not payload:
                    # Clean up old fragments to avoid memory leak if some packets are lost
                    if len(fragments) > 100:
                        fragments.clear()
                    continue
                
                # Determine which timestamp to use for pacing
                if use_pcap_time:
                    ts_us = pcap_ts_us
                else:
                    # Extract ts_start_us (offset 16, double) from LiDAR payload
                    ts_us = struct.unpack("<d", payload[16:24])[0]
                
                if start_wall_time is None:
                    start_wall_time = time.perf_counter()
                    start_packet_ts_us = ts_us
                    last_ts_us = ts_us
                    print(f"First timestamp: {ts_us}")
                
                # Handle large gaps
                if max_gap_ms > 0 and (ts_us - last_ts_us) > max_gap_ms * 1000:
                    start_wall_time -= ((ts_us - last_ts_us) - max_gap_ms * 1000) / 1_000_000.0 / rate

                last_ts_us = ts_us

                # Calculate expected wall time
                elapsed_packet_us = ts_us - start_packet_ts_us
                if elapsed_packet_us < 0: # Handle rollover
                    start_packet_ts_us = ts_us
                    start_wall_time = time.perf_counter()
                    elapsed_packet_us = 0
                
                target_wall_time = start_wall_time + (elapsed_packet_us / 1_000_000.0) / rate
                
                # High-precision wait
                while True:
                    now = time.perf_counter()
                    remaining = target_wall_time - now
                    if remaining <= 0:
                        break
                    if remaining > 0.002:
                        time.sleep(remaining - 0.001)
                
                sock.sendto(payload, (target_ip, target_port))
                packet_count += 1
                if packet_count % 1000 == 0:
                    print(f"Sent {packet_count} packets...", end='\r')
                
            print(f"\nFinished replaying {packet_count} packets.")
            reader.close()
            if not loop:
                break
            print("Looping PCAP...")
            
    except KeyboardInterrupt:
        print("\nReplay stopped.")
    except Exception as e:
        import traceback
        traceback.print_exc()
        print(f"Error: {e}")
    finally:
        sock.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="High-performance Seyond UDP PCAP Replayer")
    parser.add_argument("pcap", help="Path to PCAP file")
    parser.add_argument("--ip", default="127.0.0.1", help="Target IP address (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=8010, help="Target UDP port (default: 8010)")
    parser.add_argument("--rate", type=float, default=1.0, help="Playback rate (default: 1.0)")
    parser.add_argument("--loop", action="store_true", help="Loop the PCAP")
    parser.add_argument("--max-gap-ms", type=float, default=100.0, help="Maximum gap between packets in ms")
    parser.add_argument("--pcap-time", action="store_true", help="Use PCAP recorded timestamps instead of LiDAR internal timestamps")
    
    args = parser.parse_args()
    replay_seyond(args.pcap, args.ip, args.port, args.rate, args.loop, args.max_gap_ms, args.pcap_time)
