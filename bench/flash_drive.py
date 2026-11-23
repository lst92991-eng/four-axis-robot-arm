"""学生工作站 SWD 首装。默认只核对固件包，--write 才连接 CMSIS-DAP。"""

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import tempfile
from time import monotonic
from types import SimpleNamespace
import zlib


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "images/common-20261123-9fc6a5de"
FLASH, APP, META = 0x08000000, 0x08004000, 0x0801E000

# 首装沿用实板验证过的 AT32M416CBT7 脚本。写入前后比较最后 4 KiB 标定区。
# OpenOCD 执行的 SWD 流程包含识别芯片、写入与校验，以及对标定区的前后比对。
PROGRAM = """
gdb_port disabled
tcl_port disabled
telnet_port disabled
adapter speed 500
init
reset halt
flash probe 0
if {[mrw 0xe0042000] != 0x700221c8} {error {Unexpected MCU ID}}
mem2array before 32 0x0801f000 1024
flash write_image erase drive.hex
verify_image drive.hex
mem2array after 32 0x0801f000 1024
for {set i 0} {$i < 1024} {incr i} {
    if {$before($i) != $after($i)} {error {Calibration region changed}}
}
reset run
shutdown
"""


# ---------首装镜像校验区---------
def first_image(folder, node):
    name = f"J{node}-first-install.hex"
    manifest = json.loads((folder / "joint-app-manifest.json").read_text(encoding="utf-8"))
    source = (folder / name).read_bytes()
    if hashlib.sha256(source).hexdigest() != manifest["files"][name]:
        raise ValueError(f"{name} 与固件清单的 SHA256 不符")

    # 发布工具输出地址扩展、数据、结束三种 HEX 记录。只接受程序区和校验记录。
    bank, memory, ended = 0, {}, False
    for line in source.splitlines():
        if not line.startswith(b":"):
            raise ValueError("无效 HEX 行")
        block = bytes.fromhex(line[1:].decode("ascii"))
        if ended or len(block) < 5 or len(block) != block[0] + 5 or sum(block) % 256:
            raise ValueError("HEX 长度、校验和或结束位置有误")
        address, kind, data = int.from_bytes(block[1:3], "big"), block[3], block[4:-1]
        if kind == 4 and len(data) == 2 and address == 0:
            bank = int.from_bytes(data, "big") << 16
        elif kind == 1 and not data and address == 0:
            ended = True
        elif kind == 0 and FLASH <= bank + address < bank + address + len(data) <= META + 12:
            for offset, value in enumerate(data, bank + address):
                if offset in memory:
                    raise ValueError("HEX 地址重复")
                memory[offset] = value
        else:
            raise ValueError("HEX 含超出首装区域的地址或记录")
    # 核对本节点的应用长度、CRC、轴编号和启动记录，再把 HEX 字节交给写入入口。
    target = manifest["targets"][str(node)]
    size, crc = target["installed_bytes"], int(target["installed_crc32"], 16)
    if not ended or not 528 <= size <= META - APP or size % 4:
        raise ValueError("无效应用长度或缺少 HEX 结束记录")
    app = bytes(memory[a] for a in range(APP, APP + size))
    meta = bytes(memory[a] for a in range(META, META + 12))
    if (
        FLASH not in memory
        or zlib.crc32(app) != crc
        or app[-16:] != struct.pack("<4I", 0x31465250, 4, node, node)
        or meta != struct.pack("<3I", 0x3141544F, size, crc)
    ):
        raise ValueError("启动记录或轴编号与发布清单不符")
    return source, manifest["build_id"]


# ---------单轴 SWD 首装区---------
def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("axis", type=str.upper, choices=["J1", "J2", "J3", "J4"])
    p.add_argument("--package", type=Path, default=PACKAGE)
    p.add_argument("--openocd", default="openocd", help="雅特力 OpenOCD 可执行文件")
    p.add_argument("--write", action="store_true")
    args = p.parse_args(argv)
    source, build = first_image(args.package, int(args.axis[1:]))
    print(f"{args.axis}  固件 {build}  首装镜像和标定区边界已校验", flush=True)
    # 到这里镜像已经校验。传入 --write 后才启动 OpenOCD 写板。
    if not args.write:
        return
    program = shutil.which(args.openocd)
    if not program:
        raise FileNotFoundError("请安装雅特力 OpenOCD，或用 --openocd 指定可执行文件")
    program = Path(program).resolve()
    scripts = program.parent.parent / "scripts"
    search = ["-s", str(scripts)] if scripts.is_dir() else []
    with tempfile.TemporaryDirectory(prefix="scara-swd-") as tmp:
        work = Path(tmp)
        (work / "drive.hex").write_bytes(source)
        (work / "first.cfg").write_text(PROGRAM, encoding="ascii")
        subprocess.run(
            [
                str(program),
                *search,
                "-f",
                "interface/cmsis-dap.cfg",
                "-c",
                "transport select swd",
                "-f",
                "target/at32m416xx.cfg",
                "-f",
                "first.cfg",
            ],
            cwd=work,
            check=True,
        )
    print(f"{args.axis} SWD 写入、读回和标定区比对完成。")


# ---------CAN 在线更新区---------
def update(argv):
    """带有本项目引导程序的板子，共用一份 CAN OTA 镜像。"""
    import os

    sys.path.insert(0, str(ROOT / "temp"))
    from ota import image_for_node, send_image, send_images

    p = argparse.ArgumentParser(description=update.__doc__)
    p.add_argument("axes", nargs="+", type=str.upper, choices=["J1", "J2", "J3", "J4"])
    p.add_argument("--package", type=Path, default=PACKAGE)
    p.add_argument("--serial", default=os.environ.get("SCARA_CAN_SERIAL", ""))
    p.add_argument("--write", action="store_true")
    args = p.parse_args(argv)
    nodes = [int(a[1:]) for a in args.axes]
    if len(set(nodes)) != len(nodes):
        p.error("节点重复")
    manifest = json.loads((args.package / "joint-app-manifest.json").read_text(encoding="utf-8"))
    payload = (args.package / "joint-app.ota").read_bytes()
    if hashlib.sha256(payload).hexdigest() != manifest["files"]["joint-app.ota"]:
        raise ValueError("共用镜像 SHA256 与发布清单不符")
    # 把共用包转换成各节点实际使用的镜像，按发布清单逐轴核对长度和 CRC。
    images = [image_for_node(payload, n) for n in nodes]
    for n, crc, image in images:
        target = manifest["targets"][str(n)]
        if f"{crc:08x}" != target["installed_crc32"] or len(image) != target["installed_bytes"]:
            raise ValueError(f"J{n} 轴配置与发布清单不符")
        print(f"J{n}  {manifest['build_id']}  CRC {crc:08x}", flush=True)
    if not args.write:
        return
    from transport.candle_bus import CandleBus

    can = SimpleNamespace(Message=SimpleNamespace)
    with CandleBus(args.serial) as bus:
        if len(images) == 1:
            send_image(bus, can, *images[0])
        else:
            send_images(bus, can, images)
        # 等待各目标节点回报新构建号和功率关闭状态，记录仍未确认的轴。
        waiting = set(nodes)
        end = monotonic() + 4
        while waiting and monotonic() < end:
            frame = bus.recv(0.10)
            if (
                not frame
                or not frame.is_fd
                or frame.is_error_frame
                or frame.is_extended_id
                or len(frame.data) != 64
            ):
                continue
            n = frame.arbitration_id - 0x200
            if n not in waiting or frame.data[:4] != bytes((0x40, n, 0, 3)):
                continue
            status, build = struct.unpack_from("<II", frame.data, 44)
            if f"{build:08x}" == manifest["build_id"] and (status >> 8) & 255:
                raise RuntimeError(f"J{n} 已启动 {build:08x}，驱动故障 {(status >> 8) & 255}")
            if f"{build:08x}" == manifest["build_id"] and not (status & (1 << 22)):
                print(f"J{n} 应用版本 {build:08x} 已回读，功率关闭", flush=True)
                waiting.remove(n)
        if waiting:
            raise TimeoutError(f"镜像已提交，仍缺少节点 {sorted(waiting)} 的新版本失能状态帧")


if __name__ == "__main__":
    main()
