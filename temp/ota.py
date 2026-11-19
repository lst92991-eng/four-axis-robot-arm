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
TARGET_MAGIC = 0x31465250
TARGET = struct.Struct("<4I")
FOUR_AXIS_COUNT = 4


def package(node, image):
    image += b"\xff" * (-len(image) % 4)
    return HEADER.pack(MAGIC, node, len(image), zlib.crc32(image)) + image


def unpack(payload):
    if len(payload) < HEADER.size:
        raise ValueError("无效镜像头")
    magic, node, size, crc = HEADER.unpack_from(payload)
    image = payload[HEADER.size:]
    if magic != MAGIC or not 0 <= node <= FOUR_AXIS_COUNT or size != len(image) or not 512 <= size <= META - APP:
        raise ValueError("无效镜像头")
    if size % 4 or zlib.crc32(image) != crc:
        raise ValueError("镜像 CRC 或对齐错误")
    stack, reset = struct.unpack_from("<2I", image)
    if not (0x20000000 < stack <= 0x20004000 and stack % 8 == 0 and
            reset & 1 and APP < reset < APP + size):
        raise ValueError("镜像未链接到 OTA 应用地址")
    target = TARGET.unpack_from(image, size - TARGET.size)
    if target[0] == TARGET_MAGIC and (not node or target != (TARGET_MAGIC, FOUR_AXIS_COUNT, node, node)):
        raise ValueError("镜像的轴配置与目标节点不一致")
    return node, crc, image


def image_for_node(payload, node):
    image_node, crc, image = unpack(payload)
    if not 1 <= node <= FOUR_AXIS_COUNT or image_node not in (0, node):
        raise ValueError("镜像节点与目标节点不一致")
    if image_node == 0:
        if len(image) + TARGET.size > META - APP:
            raise ValueError("镜像没有空间存放目标轴记录")
        image += TARGET.pack(TARGET_MAGIC, FOUR_AXIS_COUNT, node, node)
        crc = zlib.crc32(image)
    return node, crc, image


def send_image(bus, can, node, crc, image, wait_reset_seconds=25, *, maintain_boot_window=None):
    if not 1 <= node <= FOUR_AXIS_COUNT:
        raise ValueError("烧录前必须指定四轴节点")
    nonce = secrets.randbelow(0xffffffff) + 1

    def request(op, arg, payload=b"", seconds=3):
        data = (HEADER.pack(MAGIC, nonce, op, arg) + payload).ljust(64, b"\0")
        bus.send(can.Message(arbitration_id=0x600 + node, is_extended_id=False,
                             is_fd=True, bitrate_switch=True, data=data), timeout=0.25)
        end = time.monotonic() + seconds
        while (remaining := end - time.monotonic()) > 0:
            if maintain_boot_window is not None:
                maintain_boot_window()
                remaining = min(remaining, 0.05)
            reply = bus.recv(remaining)
            if (reply and not reply.is_error_frame and not reply.is_extended_id and
                    reply.is_fd and reply.arbitration_id == 0x680 + node and
                    len(reply.data) == 64 and reply.data[:16] == data[:16]):
                if struct.unpack_from("<I", reply.data, 16)[0]:
                    raise RuntimeError(f"节点拒绝命令 {op}，参数 {arg}")
                return True
        return False

    if maintain_boot_window is None:
        print(f"等待节点 {node} 的引导程序，请复位该板。", flush=True)
    else:
        print(f"更新节点 {node}，其余待更新节点保留引导窗口。", flush=True)
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
    print(f"节点 {node} 整镜像 CRC 校验通过，等待应用启动并核对 STATUS。", flush=True)


def send_images(bus, can, images, wait_reset_seconds=25):
    """Keep each target in its existing bootloader while uploading sequentially."""
    nodes = [node for node, _, _ in images]
    if len(nodes) < 2 or len(nodes) != len(set(nodes)):
        raise ValueError("批量升级至少需要两个不同节点的镜像")
    probes = {node: HEADER.pack(MAGIC, secrets.randbelow(0xffffffff) + 1, 1, node)
              for node in nodes}

    def probe(targets):
        for node in targets:
            bus.send(can.Message(arbitration_id=0x600 + node, is_extended_id=False,
                                 is_fd=True, bitrate_switch=True,
                                 data=probes[node].ljust(64, b"\0")), timeout=0.25)

    print(f"等待节点 {nodes} 的引导程序，可接通或重启驱动电源。", flush=True)
    deadline = time.monotonic() + wait_reset_seconds
    while time.monotonic() < deadline:
        probe(nodes)
        remaining_nodes = set(nodes)
        round_deadline = min(deadline, time.monotonic() + 0.2)
        while remaining_nodes and (remaining := round_deadline - time.monotonic()) > 0:
            reply = bus.recv(remaining)
            if not reply or reply.is_error_frame or reply.is_extended_id or not reply.is_fd:
                continue
            node = reply.arbitration_id - 0x680
            if (node in remaining_nodes and len(reply.data) == 64 and
                    reply.data[:16] == probes[node] and reply.data[16:20] == b"\0" * 4):
                remaining_nodes.remove(node)
        if not remaining_nodes:
            break
    else:
        raise TimeoutError("未同时发现全部目标引导程序，未发擦除命令")

    print(f"已确认节点 {nodes} 的引导应答，开始逐轴写入。", flush=True)
    waiting = nodes.copy()
    next_probe = 0.0

    def maintain_boot_window():
        nonlocal next_probe
        if waiting and time.monotonic() >= next_probe:
            probe(waiting)
            next_probe = time.monotonic() + 0.4

    for node, crc, image in images:
        waiting.remove(node)
        send_image(bus, can, node, crc, image, wait_reset_seconds=3,
                   maintain_boot_window=maintain_boot_window)


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
    update.add_argument("node", type=int, choices=range(1, FOUR_AXIS_COUNT + 1))
    update.add_argument("file", type=Path)
    update.add_argument("--write", action="store_true")
    update.add_argument("--wait-reset-seconds", type=int, default=25)
    group = sub.add_parser("update-group", help="一次上电，逐轴更新多个节点")
    group.add_argument("files", nargs="+", type=Path)
    group.add_argument("--nodes", nargs="+", type=int, choices=range(1, FOUR_AXIS_COUNT + 1),
                       help="对一份共用镜像指定待更新节点")
    group.add_argument("--write", action="store_true")
    group.add_argument("--wait-reset-seconds", type=int, default=25)
    record = sub.add_parser("capture")
    record.add_argument("node", type=int, choices=range(1, 5))
    record.add_argument("csv", type=Path)
    record.add_argument("--seconds", type=int, default=60)
    args = parser.parse_args()
    if args.action in ("update", "update-group"):
        if not 1 <= args.wait_reset_seconds <= 600:
            parser.error("等待复位时间必须为 1～600 秒")
        if args.action == "update":
            images = [image_for_node(args.file.read_bytes(), args.node)]
        elif args.nodes is not None:
            if len(args.files) != 1:
                parser.error("--nodes 需使用一份共用镜像")
            payload = args.files[0].read_bytes()
            if unpack(payload)[0] != 0:
                parser.error("--nodes 需使用 joint-app.ota")
            images = [image_for_node(payload, node) for node in args.nodes]
        else:
            images = [unpack(path.read_bytes()) for path in args.files]
            if any(node == 0 for node, _, _ in images):
                parser.error("共用镜像需要 --nodes 指定目标")
        if args.action == "update-group":
            if len(images) < 2 or len({node for node, _, _ in images}) != len(images):
                parser.error("批量升级至少需要两个不同节点的镜像")
        if not args.write:
            for node, crc, image in images:
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
        elif args.action == "update-group":
            send_images(bus, can, images, args.wait_reset_seconds)
        else:
            capture(bus, args.node, args.csv, args.seconds)


if __name__ == "__main__":
    main()

