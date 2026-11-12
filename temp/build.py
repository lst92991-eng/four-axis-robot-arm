import argparse
import hashlib
import json
import shutil
import struct
import subprocess
import tempfile
from pathlib import Path

from ota import APP, FOUR_AXIS_COUNT, MAGIC, META, TARGET, image_for_node, package, unpack

ROOT = Path(__file__).resolve().parent.parent


def replace_once(text, before, after):
    if text.count(before) != 1:
        raise ValueError(f"构建模板不再匹配：{before}")
    return text.replace(before, after)


def ihex(path, segments):
    def record(address, kind, data):
        body = bytes([len(data)]) + address.to_bytes(2, "big") + bytes([kind]) + data
        return ":" + (body + bytes([-sum(body) % 256])).hex().upper()

    lines = []
    upper = None
    for address, blob in segments:
        for offset in range(0, len(blob), 16):
            pos = address + offset
            if upper != pos >> 16:
                upper = pos >> 16
                lines.append(record(0, 4, upper.to_bytes(2, "big")))
            lines.append(record(pos & 65535, 0, blob[offset:offset + 16]))
    lines.append(record(0, 1, b""))
    path.write_text("\n".join(lines) + "\n", encoding="ascii")


def build(work, gcc, output, nodes):
    fw = work / "firmware"
    shutil.copytree(ROOT / "firmware", fw)
    src, drivers = fw / "src", fw / "drivers"
    vendor = fw / "vendor"
    cm = vendor / "cmsis/cm4"
    ds = cm / "device_support"
    dsp = vendor / "cmsis/dsp"
    original_ld = next(ds.rglob("AT32M416xB_FLASH.ld")).read_text()
    original_system = (ds / "system_at32m412_416.c").read_text()
    startup = next(ds.rglob("startup_at32m412_416.s"))
    boot = src / "boot.c"
    shutil.copyfile(ROOT / "temp/boot.c", boot)
    tag = hashlib.sha256()
    for file in sorted(fw.rglob("*")):
        if file.is_file():
            tag.update(file.relative_to(fw).as_posix().encode() + b"\0" + file.read_bytes())
    (work / "build_id.h").write_text(f"#define JOINT_BUILD_ID 0x{tag.hexdigest()[:8]}u\n", encoding="ascii")

    includes = (fw, src, drivers, work, vendor / "drivers/inc", cm / "core_support", ds,
                dsp / "Include", dsp / "PrivateInclude")
    cpu = ["-mcpu=cortex-m4", "-mthumb", "-mfpu=fpv4-sp-d16", "-mfloat-abi=hard"]
    opts = ["-O2", "-Wall", "-Wextra", "-Werror", "-ffunction-sections", "-fdata-sections",
            "-fno-math-errno", "-fno-common", "-DAT32M416CBT7", "-DAT_START_M416_V1",
            "-DUSE_STDPERIPH_DRIVER"]
    for directory in includes:
        opts.extend(("-I", str(directory)))

    def run(args):
        subprocess.run([str(item) for item in args], check=True, cwd=work)

    def link(name, files, base, length, defines=()):
        ld = work / f"{name}.ld"
        script = replace_once(original_ld, "0x08000000, LENGTH = 124K",
                              f"0x{base:08x}, LENGTH = {length}")
        if base == APP:
            script = replace_once(script, "    *(.rodata*)",
                                  "    KEEP(*(.rodata.six_axis_config))\n    *(.rodata*)")
        ld.write_text(script, encoding="ascii")
        objects = []
        for index, file in enumerate(files):
            obj = work / f"{name}-{index}.o"
            flags = ["-x", "assembler-with-cpp"] if file.suffix.lower() == ".s" else ["-std=c11"]
            run([gcc, *cpu, *opts, *defines, *flags, "-c", file, "-o", obj])
            objects.append(obj)
        elf = work / f"{name}.elf"
        run([gcc, *cpu, *objects, "-T", ld, "-Wl,--gc-sections",
             "-Wl,--defsym=__required_stack_bytes=4096", "--specs=nano.specs",
             "--specs=nosys.specs", "-Wl,--start-group", "-lc", "-lm",
             "-Wl,--end-group", "-o", elf])
        bin_path = work / f"{name}.bin"
        objcopy = gcc.with_name(gcc.name.replace("gcc", "objcopy"))
        run([objcopy, "-O", "binary", elf, bin_path])
        return bin_path.read_bytes()

    peripheral = sorted((vendor / "drivers/src").glob("*.c"))
    app_peripheral = [p for p in peripheral if not p.stem.endswith("_usart")]
    system = ds / "system_at32m412_416.c"
    boot_files = [boot, src / "CAN.c", system, startup]
    boot_files += [p for p in peripheral if p.stem.endswith(("_can", "_gpio", "_crm", "_flash"))]
    bootstraps = {
        node: link(f"J{node}-boot", boot_files, 0x08000000, APP - 0x08000000,
                   defines=(f"-DBOOT_NODE={node}",))
        for node in nodes
    }
    system.write_text(replace_once(original_system, "0x0 /*!< vector table",
                                   "0x4000 /*!< vector table"), encoding="ascii")
    dsp_sources = [next(dsp.rglob(file)) for file in
                   ("arm_sin_f32.c", "arm_cos_f32.c", "arm_common_tables.c")]
    application = link("joint-app", [p for p in sorted(src.glob("*.c")) if p != boot] +
                       sorted(drivers.glob("*.c")) + dsp_sources + app_peripheral + [system, startup],
                       APP, META - APP - TARGET.size)
    shared_ota = package(0, application)
    _, shared_crc, shared_image = unpack(shared_ota)
    artifacts = [output / "joint-app.elf", output / "joint-app.ota"]
    shutil.copyfile(work / "joint-app.elf", artifacts[0])
    artifacts[1].write_bytes(shared_ota)
    targets = {}
    for node, bootstrap in bootstraps.items():
        _, crc, installed = image_for_node(shared_ota, node)
        initial = output / f"J{node}-first-install.hex"
        ihex(initial, [(0x08000000, bootstrap), (APP, installed),
                       (META, struct.pack("<3I", MAGIC, len(installed), crc))])
        artifacts.append(initial)
        targets[node] = dict(installed_bytes=len(installed), installed_crc32=f"{crc:08x}")
        if len(nodes) == 1:
            # 保留单节点构建的 OTA 和清单入口，供现有 J1 更新工具使用。
            node_ota = output / f"J{node}.ota"
            node_ota.write_bytes(package(node, installed))
            (output / f"J{node}-manifest.json").write_text(json.dumps(
                dict(node=node, build_id=tag.hexdigest()[:8], app_bytes=len(installed),
                     app_crc32=f"{crc:08x}",
                     files={node_ota.name: hashlib.sha256(node_ota.read_bytes()).hexdigest()}),
                indent=2) + "\n", encoding="utf-8")
            artifacts.append(node_ota)

    manifest = dict(build_id=tag.hexdigest()[:8], app_bytes=len(shared_image),
                    app_crc32=f"{shared_crc:08x}", targets=targets,
                    parameter_source="firmware/src/config.c",
                    files={p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in artifacts})
    (output / "joint-app-manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(f"共用应用 {manifest['build_id']}，{len(shared_image)} B，节点 {list(nodes)}", flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--gcc", required=True, type=Path)
    parser.add_argument("--node", default="all", choices=("1", "2", "3", "4", "all"))
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--work-root", type=Path, default=Path(tempfile.gettempdir()))
    args = parser.parse_args()
    output, gcc, work_root = args.output.resolve(), args.gcc.resolve(), args.work_root.resolve()
    if not str(work_root).isascii():
        parser.error("--work-root 需要 ASCII 路径，例如 C:/Temp")
    output.mkdir(parents=True, exist_ok=True)
    work_root.mkdir(parents=True, exist_ok=True)
    nodes = range(1, FOUR_AXIS_COUNT + 1) if args.node == "all" else (int(args.node),)
    with tempfile.TemporaryDirectory(prefix="arm4-", dir=work_root) as directory:
        build(Path(directory), gcc, output, nodes)


if __name__ == "__main__":
    main()

