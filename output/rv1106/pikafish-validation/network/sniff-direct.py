import ctypes as c
import json
import os
import socket
import struct
import time

os.add_dll_directory(r'C:\Windows\System32\Npcap')
p = c.CDLL(r'C:\Windows\System32\Npcap\wpcap.dll')
class Header(c.Structure):
    _fields_ = [('sec', c.c_long), ('usec', c.c_long), ('caplen', c.c_uint), ('length', c.c_uint)]
class Filter(c.Structure):
    _fields_ = [('length', c.c_uint), ('insns', c.c_void_p)]
p.pcap_open_live.argtypes = [c.c_char_p, c.c_int, c.c_int, c.c_int, c.c_char_p]
p.pcap_open_live.restype = c.c_void_p
p.pcap_compile.argtypes = [c.c_void_p, c.POINTER(Filter), c.c_char_p, c.c_int, c.c_uint]
p.pcap_setfilter.argtypes = [c.c_void_p, c.POINTER(Filter)]
p.pcap_next_ex.argtypes = [c.c_void_p, c.POINTER(c.POINTER(Header)), c.POINTER(c.POINTER(c.c_ubyte))]
p.pcap_close.argtypes = [c.c_void_p]
p.pcap_geterr.argtypes = [c.c_void_p]
p.pcap_geterr.restype = c.c_char_p
p.pcap_setnonblock.argtypes = [c.c_void_p, c.c_int, c.c_char_p]
p.pcap_sendpacket.argtypes = [c.c_void_p, c.c_char_p, c.c_int]
err = c.create_string_buffer(512)
handle = p.pcap_open_live(b'\\Device\\NPF_{9FD3AC88-884B-41CD-AC6B-B86ACC170F87}', 1024, 0, 1000, err)
if not handle:
    raise RuntimeError(err.value.decode(errors='replace'))
flt = Filter()
if p.pcap_compile(handle, c.byref(flt), b'arp or (udp and (port 67 or port 68))', 1, 0xffffffff) != 0:
    raise RuntimeError(p.pcap_geterr(handle))
p.pcap_setfilter(handle, c.byref(flt))
p.pcap_setnonblock(handle, 1, err)
print('Listening only to ARP/DHCP on direct Ethernet interface 13', flush=True)
source_mac = bytes.fromhex('d853bc3ac11f')
for target in ('192.168.55.6', '192.168.55.2', '169.254.110.6', '169.254.126.121', '172.32.0.70'):
    frame = (b'\xff' * 6 + source_mac + b'\x08\x06' + struct.pack('!HHBBH', 1, 0x0800, 6, 4, 1)
             + source_mac + socket.inet_aton('192.168.55.1') + b'\x00' * 6 + socket.inet_aton(target))
    p.pcap_sendpacket(handle, frame, len(frame))
deadline = time.monotonic() + 35
try:
    while time.monotonic() < deadline:
        hdr = c.POINTER(Header)()
        data = c.POINTER(c.c_ubyte)()
        if p.pcap_next_ex(handle, c.byref(hdr), c.byref(data)) != 1:
            time.sleep(0.05)
            continue
        packet = c.string_at(data, hdr.contents.caplen)
        if len(packet) < 42:
            continue
        mac = ':'.join(f'{b:02x}' for b in packet[6:12])
        record = {'mac': mac}
        if packet[12:14] == b'\x08\x06':
            record.update(kind='ARP', op=int.from_bytes(packet[20:22], 'big'), source=socket.inet_ntoa(packet[28:32]), target=socket.inet_ntoa(packet[38:42]))
        elif packet[12:14] == b'\x08\x00':
            ihl = (packet[14] & 15) * 4
            start = 14 + ihl
            payload = packet[start + 8:]
            record.update(kind='DHCP', source=socket.inet_ntoa(packet[26:30]), ports=struct.unpack('!HH', packet[start:start+4]))
            if len(payload) >= 240:
                record['client_mac'] = ':'.join(f'{b:02x}' for b in payload[28:34])
                record['options_hex'] = payload[240:].hex()
        print(json.dumps(record), flush=True)
finally:
    p.pcap_close(handle)
