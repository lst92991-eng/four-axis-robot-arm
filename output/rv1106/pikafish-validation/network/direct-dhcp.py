"""Serve one reserved DHCP lease on the Raspberry Pi's direct Ethernet link."""

import argparse
import ipaddress
import json
import logging
from pathlib import Path
import socket
import struct
import time


def options(packet):
    result = {}
    offset = 240
    while offset < len(packet):
        tag = packet[offset]
        offset += 1
        if tag == 255:
            break
        if tag == 0:
            continue
        if offset >= len(packet):
            return {}
        size = packet[offset]
        offset += 1
        if offset + size > len(packet):
            return {}
        result[tag] = packet[offset:offset + size]
        offset += size
    return result


def reply(request, message_type, server, address):
    packet = bytearray(240)
    packet[0:4] = bytes([2, 1, 6, 0])
    packet[4:8] = request[4:8]
    packet[10:12] = b"\x80\x00"
    if message_type != 6:
        packet[16:20] = address
    packet[20:24] = server
    packet[28:44] = request[28:44]
    packet[236:240] = b"\x63\x82\x53\x63"
    packet.extend(bytes([53, 1, message_type, 54, 4]) + server)
    if message_type != 6:
        # No router or DNS option: this cable is a management-only network.
        packet.extend(bytes([1, 4, 255, 255, 255, 0]))
        for tag, seconds in ((51, 86400), (58, 43200), (59, 75600)):
            packet.extend(bytes([tag, 4]) + struct.pack("!I", seconds))
    packet.append(255)
    return bytes(packet).ljust(300, b"\x00")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface", type=int, required=True)
    parser.add_argument("--mac", required=True)
    parser.add_argument("--server", default="192.168.55.1")
    parser.add_argument("--address", default="192.168.55.2")
    args = parser.parse_args()
    directory = Path(__file__).resolve().parent
    logging.basicConfig(filename=directory / "rpi5-dhcp.log", level=logging.INFO,
                        format="%(asctime)s %(message)s", encoding="utf-8")
    target_mac = bytes.fromhex(args.mac.replace(":", "").replace("-", ""))
    if len(target_mac) != 6:
        raise ValueError("Expected one six-byte client MAC")
    server = ipaddress.IPv4Address(args.server).packed
    address = ipaddress.IPv4Address(args.address).packed
    broadcast = str(ipaddress.IPv4Network(args.server + "/24", strict=False).broadcast_address)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    # Windows IP_UNICAST_IF takes an interface index in network byte order.
    sock.setsockopt(socket.IPPROTO_IP, 31, struct.pack("!I", args.interface))
    sock.bind(("0.0.0.0", 67))
    sock.settimeout(2)
    logging.info("START interface=%s MAC=%s server=%s reserved=%s",
                 args.interface, args.mac, args.server, args.address)
    (directory / "dhcp-ready.json").write_text(json.dumps({"ready": True}), encoding="utf-8")
    try:
        while not (directory / "dhcp-stop.request").exists():
            try:
                packet, peer = sock.recvfrom(4096)
            except socket.timeout:
                continue
            if (len(packet) < 240 or packet[:3] != b"\x01\x01\x06"
                    
                    or packet[236:240] != b"\x63\x82\x53\x63"
                    or packet[24:28] != b"\x00" * 4):
                continue
            opts = options(packet)
            kind = opts.get(53, b"")
            client_mac = ":".join(f"{b:02x}" for b in packet[28:34])
            logging.info("CLIENT mac=%s type=%s hostname=%r", client_mac, kind.hex(), opts.get(12, b""))
            known = {"e6:ad:54:47:81:fa": "192.168.55.6", "2c:cf:67:f4:b6:6b": "192.168.55.2", "26:91:dc:d2:f6:f2": "192.168.55.6", "a2:ad:df:37:a2:0f": "192.168.55.6", "de:e1:8d:c8:68:0e": "192.168.55.6"}
            if client_mac not in known:
                continue
            address = ipaddress.IPv4Address(known[client_mac]).packed
            hostname = opts.get(12, b"").decode("utf-8", errors="replace")
            logging.info("RECEIVE type=%s hostname=%r peer=%s", kind.hex(), hostname, peer)
            if kind == b"\x01":
                answer_type = 2
            elif kind == b"\x03":
                if 54 in opts and opts[54] != server:
                    continue
                requested = opts.get(50, packet[12:16])
                answer_type = 5 if requested in (address, b"\x00" * 4) else 6
            else:
                continue
            try:
                sock.sendto(reply(packet, answer_type, server, address), (broadcast, 68))
            except OSError as error:
                # A just-restarted Ethernet link may not have its route yet.
                logging.warning("SEND failed while link is changing: %s", error)
                continue
            logging.info("SEND type=%s reserved=%s", answer_type, args.address)
            if answer_type == 5:
                lease = {"address": str(ipaddress.IPv4Address(address)), "mac": client_mac, "hostname": hostname,
                         "acknowledged_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                         "lease_seconds": 86400}
                (directory / "rpi5-lease.json").write_text(json.dumps(lease, indent=2), encoding="utf-8")
    finally:
        sock.close()
        logging.info("STOP")


if __name__ == "__main__":
    main()


