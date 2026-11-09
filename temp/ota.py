import argparse
import csv
import secrets
import struct
import time
import zlib
from types import SimpleNamespace
from pathlib import Path

MAGIC = 0x3141544F
APP = 0x08004000
META = 0x0801E000
HEADER = struct.Struct("<4I")


def package(node, image):
    image += b"\xff" * (-len(image) % 4)
    return HEADER.pack(MAGIC, node, len(image), zlib.crc32(image)) + image


def unpack(path):
    magic, node, size, crc = HEADER.unpack_from(path)
    image = path[16:]
    if magic != MAGIC or not 1 <= node <= 4 or size != len(image) or not 512 <= size <= META - APP:
        raise ValueError("无效镜像头")
    if size % 4 or zlib.crc32(image) != crc:
        raise ValueError("镜像 CRC 或对齐错误")
    stack, reset = struct.unpack_from("<2I", image)
    if not (0x20000000 < stack <= 0x20004000 and stack % 8 == 0 and
            reset & 1 and APP < reset < APP + size):
        raise ValueError("镜像未链接到 OTA 应用地址")
    return node, crc, image


def send_image(bus, can, node, crc, image, wait_reset_seconds=25):
    nonce = secrets.randbelow(0xffffffff) + 1

    def request(op, arg, payload=b"", seconds=3):
        data = (HEADER.pack(MAGIC, nonce, op, arg) + payload).ljust(64, b"\0")
        bus.send(can.Message(arbitration_id=0x600 + node, is_extended_id=False,
                             is_fd=True, bitrate_switch=True, data=data), timeout=0.25)
        end = time.monotonic() + seconds
        while (remaining := end - time.monotonic()) > 0:
            reply = bus.recv(remaining)
            if (reply and not reply.is_error_frame and not reply.is_extended_id and
                    reply.is_fd and reply.arbitration_id == 0x680 + node and
                    len(reply.data) == 64 and reply.data[:16] == data[:16]):
                if struct.unpack_from("<I", reply.data, 16)[0]:
                    raise RuntimeError(f"节点拒绝命令 {op}，参数 {arg}")
                return True
        return False

    print(f"等待节点 {node} 的引导程序，请复位该板。", flush=True)
    end = time.monotonic() + wait_reset_seconds
    while not request(1, node, seconds=0.2):
        if time.monotonic() >= end:
            raise TimeoutError("引导无应答，未发擦除命令")
    if not request(2, len(image), struct.pack("<2I", crc, node), seconds=8):
        raise TimeoutError("擦除应答超时，请检查总线后重新上传")
    for offset in range(0, len(image), 48):
        if not request(3, offset, image[offset:offset + 48]):
            raise TimeoutError(f"写入在 {offset} 字节处超时，请重新上传")
    if not request(4, len(image)):
        raise TimeoutError("提交未确认，请检查板端身份")
    print(f"节点 {node} 整镜像 CRC 校验通过，等待应用启动并核对 STATUS。")


def capture(bus, node, path, seconds):
    columns = ("host_s", "board_ms", "encoder_counts", "speed_counts_s",
               "position_mdeg", "target_mdeg", "iq_ref_ma", "iq_ma", "vq_request_mv",
               "bus_mv", "power", "fault", "encoder_valid")
    started = time.monotonic()
    rows = 0
    with path.open("x", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(columns)
        while time.monotonic() - started < seconds:
            frame = bus.recv(0.2)
            if (frame and not frame.is_error_frame and not frame.is_extended_id and frame.is_fd
                    and frame.arbitration_id == 0x300 + node and len(frame.data) == 64):
                values = struct.unpack_from("<IIqf6i3I", frame.data)
                if values[0] == 0x3147545A:
                    writer.writerow((round(time.monotonic() - started, 6), *values[1:]))
                    rows += 1
    print(f"记录 {rows} 帧 → {path}")
    if not rows:
        raise RuntimeError("未收到 Z 轴诊断帧")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--candlelight", action="store_true")
    parser.add_argument("--serial", default="")
    sub = parser.add_subparsers(dest="action", required=True)
    update = sub.add_parser("update")
    update.add_argument("node", type=int, choices=range(1, 5))
    update.add_argument("file", type=Path)
    update.add_argument("--write", action="store_true")
    update.add_argument("--wait-reset-seconds", type=int, default=25)
    record = sub.add_parser("capture")
    record.add_argument("node", type=int, choices=range(1, 5))
    record.add_argument("csv", type=Path)
    record.add_argument("--seconds", type=int, default=60)
    args = parser.parse_args()
    if args.action == "update":
        if not 1 <= args.wait_reset_seconds <= 600:
            parser.error("等待复位时间必须为 1～600 秒")
        node, crc, image = unpack(args.file.read_bytes())
        if node != args.node:
            parser.error("镜像节点与目标节点不一致")
        if not args.write:
            print(f"离线校验通过：节点 {node}，{len(image)} 字节，CRC {crc:08x}")
            return
    if args.candlelight:
        from transport.candle_bus import CandleBus
        connection = CandleBus(args.serial)
        can = SimpleNamespace(Message=SimpleNamespace)
    else:
        import can
        connection = can.Bus(interface="socketcan", channel=args.channel, fd=True)
    with connection as bus:
        if args.action == "update":
            send_image(bus, can, node, crc, image, args.wait_reset_seconds)
        else:
            capture(bus, args.node, args.csv, args.seconds)


if __name__ == "__main__":
    main()

