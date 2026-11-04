# -*- coding: gbk -*-
"""Candlelight 2.5 legacy GS USB transport (classic CAN and CAN FD).

Wire definitions: Elmue CANable-2.5 Firmware/Candlelight/candlelight_def.h.
No adapter firmware update or USB driver replacement is performed.
"""
import struct
import time
from dataclasses import dataclass

LENGTHS = (0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64)


@dataclass
class Frame:
    identifier: int
    data: bytes
    fd: bool = False
    extended: bool = False
    brs: bool = False
    error: bool = False
    echo: bool = False


def pack_frame(frame, marker=0):
    if len(frame.data) not in LENGTHS or (not frame.fd and len(frame.data) > 8):
        raise ValueError('\u6570\u636e\u957f\u5ea6\u4e0d\u662f\u5408\u6cd5\u7684 CAN DLC')
    if not 0 <= frame.identifier <= (0x1fffffff if frame.extended else 0x7ff):
        raise ValueError('CAN ID \u8d85\u51fa\u8303\u56f4')
    if frame.brs and not frame.fd:
        raise ValueError('BRS \u9700\u8981 CAN FD')
    ident = frame.identifier | (0x80000000 if frame.extended else 0)
    flags = (2 if frame.fd else 0) | (4 if frame.brs else 0)
    return struct.pack('<IIBBBB', marker, ident, LENGTHS.index(len(frame.data)),
                       0, flags, 0) + frame.data.ljust(64 if frame.fd else 8, b'\0')


def unpack_frame(raw):
    if len(raw) < 12:
        raise ValueError('\u4e0d\u5b8c\u6574\u7684 USB \u5e27\u5934')
    echo, ident, dlc, channel, flags, _ = struct.unpack_from('<IIBBBB', raw)
    fd = bool(flags & 2)
    expected = 76 if fd else 20
    if channel != 0 or dlc > 15 or (not fd and dlc > 8) or len(raw) not in (expected, expected + 4):
        raise ValueError('\u65e0\u6548\u7684 GS USB \u5e27: '+f'length={len(raw)}, channel={channel}, dlc={dlc}, flags={flags}, raw={bytes(raw).hex()}')
    return Frame(ident & (0x1fffffff if ident & 0x80000000 else 0x7ff),
                 bytes(raw[12:12 + LENGTHS[dlc]]), fd, bool(ident & 0x80000000),
                 bool(flags & 4), bool(ident & 0x20000000), echo != 0xffffffff)




def is_candlelight_status_report(frame):
    # Elmue buffer.c emits counter-only/back-to-active reports in the error
    # envelope. Accept only known active-state layouts below warning counts.
    return (frame.error and not frame.echo and not frame.fd and not frame.extended
            and len(frame.data) == 8 and frame.identifier in (0, 4)
            and frame.data[0] == 0 and frame.data[1] in (0, 0x40)
            and not any(frame.data[2:6]) and max(frame.data[6:8]) < 96
            and ((frame.identifier == 4) == (frame.data[1] == 0x40)))


def describe_bus_error(frame, bitrate_hint="500k/2M"):
    """Decode standard GS USB / SocketCAN error classes without clearing them."""
    labels = []
    for bit, label in ((0x01,'发送超时'),(0x02,'仲裁丢失'),(0x04,'控制器状态'),
                       (0x08,'协议错误'),(0x10,'收发器错误'),(0x20,'无应答 ACK'),
                       (0x40,'Bus-off'),(0x80,'总线错误'),(0x100,'控制器重启'),(0x200,'收发错误计数')):
        if frame.identifier & bit: labels.append(label)
    if frame.identifier & 0x08 and len(frame.data) >= 3:
        for bit, label in ((0x01,'位错误'),(0x02,'格式错误'),(0x04,'位填充错误'),
                           (0x08,'无法发送显性位'),(0x10,'无法发送隐性位')):
            if frame.data[2] & bit: labels.append(label)
    if frame.identifier & 0x04 and len(frame.data) >= 2:
        for bit, label in ((0x01,'接收缓冲溢出'),(0x02,'发送缓冲溢出'),
                           (0x04,'接收错误告警'),(0x08,'发送错误告警'),
                           (0x10,'接收错误被动'),(0x20,'发送错误被动'),
                           (0x40,'恢复错误主动状态')):
            if frame.data[1] & bit: labels.append(label)
    if frame.identifier & 0x200 and len(frame.data) >= 8:
        labels.append(f'TX={frame.data[6]}、RX={frame.data[7]}')
    detail = '、'.join(labels) or '未知 CAN 错误'
    return (detail+f'（ID=0x{frame.identifier:X}，原始={frame.data.hex()}）；'
            f'检查各节点供电、CAN H/L、公共地、两端终端电阻及 {bitrate_hint} 位速率')



def bit_timing(clock_hz, bitrate, limits, sample_point=0.75):
    s1_min, s1_max, s2_min, s2_max, sjw_max, brp_min, brp_max, brp_inc = limits
    candidates = []
    for brp in range(brp_min, brp_max + 1, max(1, brp_inc)):
        if clock_hz % (bitrate * brp):
            continue
        total = clock_hz // (bitrate * brp)
        for seg2 in range(s2_min, min(s2_max, total - 2) + 1):
            seg1 = total - 1 - seg2
            if s1_min <= seg1 <= s1_max:
                candidates.append((abs((1 + seg1) / total - sample_point),
                                   -total, (0, seg1, seg2, min(seg2, sjw_max), brp)))
    if not candidates:
        raise ValueError('\u9002\u914d\u5668\u65e0\u6cd5\u751f\u6210\u6307\u5b9a\u6bd4\u7279\u7387')
    return min(candidates)[2]


def usb_backend():
    import libusb_package
    import usb.backend.libusb1
    backend = usb.backend.libusb1.get_backend(find_library=libusb_package.find_library)
    if backend is None:
        raise RuntimeError('\u672a\u627e\u5230 libusb\uff0c\u8bf7\u5b89\u88c5 requirements-can.txt')
    return backend


def usb_identity(dev):
    # Topology descriptors do not require opening the exclusive Windows handle.
    ports = getattr(dev, 'port_numbers', None)
    location = '.'.join(map(str, ports)) if ports else str(getattr(dev, 'address', 0))
    return f"{dev.idVendor:04x}:{dev.idProduct:04x}:{getattr(dev, 'bus', 0)}:{location}"


def discover():
    import usb.core
    import usb.util
    result = []
    for dev in usb.core.find(find_all=True, backend=usb_backend()):
        if (dev.idVendor, dev.idProduct) not in ((0x1d50, 0x606f), (0x1209, 0x2323)):
            continue
        try:
            result.append((dev, f'{dev.product} / {dev.serial_number}'))
        except (usb.core.USBError, ValueError):
            result.append((dev, f'{dev.idVendor:04x}:{dev.idProduct:04x}'))
    return result


class Candlelight:
    def __init__(self, serial='', fd=True, internal_loopback=False):
        import usb.util
        devices = discover()
        selected = [(d, label) for d, label in devices if not serial or serial in label]
        for dev, label in devices:
            if len(selected) != 1 or dev is not selected[0][0]:
                usb.util.dispose_resources(dev)
        if len(selected) != 1:
            raise RuntimeError('\u8bf7\u8fde\u63a5\u4e00\u4e2a Candlelight\uff0c\u6216\u586b\u5199\u5e8f\u5217\u53f7\u9009\u62e9\u8bbe\u5907')
        self.dev, self.label = selected[0]
        from usb_owner import UsbOwner
        try:
            self.owner = UsbOwner(usb_identity(self.dev))
        except Exception:
            usb.util.dispose_resources(self.dev)
            raise
        self.fd = fd
        self.marker = 0
        self.rx_buffer = bytearray()
        self.partial_since = None
        self.startup_discarded = []
        self.elmue = False
        self.closed = False
        self.claimed = False
        self.io_stats = dict(reads=0, writes=0, timeouts=0, errors=0)
        try:
            interface = self.dev.get_active_configuration()[(0, 0)]
            usb.util.claim_interface(self.dev, 0)
            self.claimed = True
            endpoints = [e for e in interface if usb.util.endpoint_type(e.bmAttributes) == 2]
            self.ep_in = next(e.bEndpointAddress for e in endpoints if e.bEndpointAddress & 0x80)
            self.ep_out = next(e.bEndpointAddress for e in endpoints if not e.bEndpointAddress & 0x80)
            self.read_size = next(e.wMaxPacketSize for e in endpoints if e.bEndpointAddress == self.ep_in)
            self.write_size = next(e.wMaxPacketSize for e in endpoints if e.bEndpointAddress == self.ep_out)
            self._out(2, struct.pack('<II', 0, 0))
            self.drain_startup()
            self._out(0, struct.pack('<I', 0xbeef))
            caps = struct.unpack('<10I', self._in(4, 40))
            self.elmue = bool(caps[0] & 0x4000)
            self.paced_tx = self.elmue and 'WeActStudioV1' in self.label
            self._out(1, struct.pack('<5I', *bit_timing(caps[1], 500000, caps[2:])))
            if fd:
                if caps[0] & 0x500 != 0x500:
                    raise RuntimeError('\u8be5\u56fa\u4ef6\u4e0d\u652f\u6301 CAN FD \u4f4d\u65f6\u5e8f')
                cap_fd = struct.unpack('<18I', self._in(11, 72))
                self._out(10, struct.pack('<5I', *bit_timing(cap_fd[1], 2000000, cap_fd[10:])))
            if internal_loopback and caps[0] & 3 != 3:
                raise RuntimeError('Internal loopback is not supported')
            flags = (0x100 if fd else 0) | (3 if internal_loopback else 0)
            self._out(2, struct.pack('<II', 1, flags))
        except Exception as exc:
            try:
                self.close()
            except Exception:
                pass
            if getattr(exc, 'errno', None) == 32:
                raise RuntimeError('USB-CAN 初始化失败（Pipe error）；尚未开始本次电机通信。关闭其它 CAN 工具后拔插 USB-CAN，再重新连接；重烧电机或增大电流不能修复此 USB 错误。') from exc
            if getattr(exc, 'errno', None) == 13:
                raise RuntimeError('USB-CAN 被其他程序占用，请关闭其他 CAN 工具或切回已打开的上位机；无需修改电机参数') from exc
            raise

    def drain_startup(self):
        import usb.core
        # CAN is closed. Drain previously queued USB packets before accepting
        # new telemetry or ACKs from this connection. Do not interpret old data.
        deadline = time.monotonic()+1.0
        while time.monotonic()<deadline:
            try:
                raw = self.dev.read(self.ep_in, getattr(self, 'read_size', 64), timeout=50)
            except usb.core.USBTimeoutError:
                return
            if raw:
                self.startup_discarded.append(bytes(raw))
        raise RuntimeError('USB startup receive queue did not become idle')

    def _in(self, request, count):
        return bytes(self.dev.ctrl_transfer(0xc1, request, 0, 0, count, timeout=500))

    def _out(self, request, payload):
        self.dev.ctrl_transfer(0x41, request, 0, 0, payload, timeout=500)
        if self.elmue:
            error = self._in(22, 1)[0]
            # Firmware/settings.h: FBK_Success is 2, errors start at ASCII '1'.
            if error != 2:
                raise RuntimeError(f'Candlelight \u62d2\u7edd\u914d\u7f6e: request={request}, error={error}')

    def send(self, frame):
        if frame.fd and not self.fd:
            raise ValueError('\u8fde\u63a5\u672a\u542f\u7528 CAN FD')
        raw = pack_frame(frame, self.marker)
        self.marker = (self.marker + 1) % 32
        import usb.core
        try:
            self.io_stats['writes'] += 1
            # WeActStudioV1 legacy FD OUT uses two USB packets (64+12).
            # Back-to-back packets reproduce EPIPE in internal loopback.
            # Keep the wire frame intact, but give the endpoint time to rearm.
            chunks = [raw]
            if getattr(self, 'paced_tx', False) and len(raw) > self.write_size:
                chunks = [raw[i:i+self.write_size] for i in range(0, len(raw), self.write_size)]
            written = 0
            for index, chunk in enumerate(chunks):
                if index:
                    time.sleep(.002)
                count = self.dev.write(self.ep_out, chunk, timeout=100)
                if count != len(chunk):
                    self.io_stats['errors'] += 1
                    raise IOError('USB packet write incomplete; command not retried')
                written += count
        except usb.core.USBError as exc:
            self.io_stats['errors'] += 1
            raise RuntimeError(f'USB OUT 0x{self.ep_out:02X}: {exc}; '
                '\u53d1\u9001\u672a\u786e\u8ba4\uff0c\u672a\u91cd\u53d1\u8fd0\u52a8\u6307\u4ee4\u3002\u8bf7\u91cd\u65b0\u8fde\u63a5\uff0c\u7a0b\u5e8f\u5c06\u91cd\u65b0\u63e1\u624b\u786e\u8ba4\u505c\u6b62\uff1b\u5982\u4ecd\u5931\u8d25\uff0c\u9002\u914d\u5668\u5c1a\u672a\u6062\u590d\u3002') from exc
        if written != len(raw):
            raise IOError('USB \u53d1\u9001\u4e0d\u5b8c\u6574')

    def recv(self, timeout=0.01):
        import usb.core
        deadline = time.monotonic() + timeout
        while True:
            if len(self.rx_buffer) >= 12:
                flags = self.rx_buffer[10]
                expected = 76 if flags & 2 else 20
                # This session does not request hardware timestamps. USB transfers
                # may split a 76-byte frame into 64+12 or coalesce several frames.
                if self.rx_buffer[9] != 0 or self.rx_buffer[8] > (15 if flags & 2 else 8):
                    raise ValueError('Invalid buffered GS USB header: ' + self.rx_buffer.hex())
                if len(self.rx_buffer) >= expected:
                    raw = bytes(self.rx_buffer[:expected])
                    del self.rx_buffer[:expected]
                    self.partial_since = time.monotonic() if self.rx_buffer else None
                    return unpack_frame(raw)
            if self.partial_since is not None and time.monotonic()-self.partial_since > .2:
                raise ValueError('Incomplete GS USB frame: ' + self.rx_buffer.hex())
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            try:
                if hasattr(self, 'io_stats'):self.io_stats['reads'] += 1
                # Read one USB packet, then assemble CAN frames above. A full
                # 64-byte packet completes immediately, without waiting for the
                # tail of a 76-byte FD frame in the same timed USB request.
                raw = self.dev.read(self.ep_in, getattr(self, 'read_size', 64), timeout=max(1, int(remaining*1000)))
            except usb.core.USBTimeoutError:
                if hasattr(self, 'io_stats'):self.io_stats['timeouts'] += 1
                return None
            except usb.core.USBError as exc:
                if hasattr(self, 'io_stats'):self.io_stats['errors'] += 1
                raise RuntimeError(f'USB IN 0x{self.ep_in:02X}: {exc}') from exc
            if raw:
                if not self.rx_buffer:
                    self.partial_since = time.monotonic()
                self.rx_buffer.extend(raw)

    def close(self):
        import usb.util
        if self.closed:
            return
        self.closed = True
        try:
            if self.claimed:
                self._out(2, struct.pack('<II', 0, 0))
        finally:
            try:
                usb.util.dispose_resources(self.dev)
            finally:
                self.owner.close()
