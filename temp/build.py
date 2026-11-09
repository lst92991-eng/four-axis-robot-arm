import argparse
import hashlib
import json
import re
import shutil
import struct
import subprocess
import tempfile
from pathlib import Path

from ota import APP, MAGIC, META, package, unpack

ROOT = Path(__file__).resolve().parent.parent
PROFILES = Path(__file__).with_name("axes.json")


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


def build(work, gcc, output, node, profile):
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
    profile = {"NODE": node, **profile}
    if profile["NODE"] != node:
        raise ValueError("节点号由 axes.json 的键指定")
    conf = src / "config.h"
    cfg = conf.read_text(encoding="utf-8")
    for key, value in profile.items():
        if not re.fullmatch(r"[A-Z_]+", key) or not re.fullmatch(r"\(?-?\d+(\.\d+f?)?\)?", str(value)):
            raise ValueError(f"配置值格式不合法：{key}={value}")
        cfg, n = re.subn(rf"(?m)^#define CONFIG_{key} .+$", f"#define CONFIG_{key} {value}", cfg)
        if n != 1:
            raise ValueError(f"未知配置项：{key}")
    conf.write_text(cfg, encoding="utf-8")

    if node == 1:
        shutil.copyfile(ROOT / "temp/z_trace.h", src / "z_trace.h")
        for name, before, after in (
            ("common.h", "  int64_t encoder_counts;", "  int64_t encoder_counts;\n"
             "  int32_t trace_iq_ref, trace_iq, trace_vq;\n  uint32_t trace_bus;"),
            ("motor_control.c", "  *out = Drive.state;", "  *out = Drive.state;\n"
             "  out->trace_iq_ref = (int32_t)Drive.control.current.q.targetValue;\n"
             "  out->trace_iq = (int32_t)Drive.control.current.q.currentValue;\n"
             "  out->trace_vq = (int32_t)Drive.control.current.q.out;\n"
             "  out->trace_bus = current_sense_get_bus_mv();"),
            ("main.c", "#include \"mt6816.h\"", "#include \"mt6816.h\"\n#include \"z_trace.h\""),
            ("main.c", "    CAN_protocol_process(&motor, now);",
             "    CAN_protocol_process(&motor, now);\n    z_trace(&motor, now);"),
        ):
            path = src / name
            path.write_text(replace_once(path.read_text(encoding="utf-8"), before, after), encoding="utf-8")

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

    def link(name, files, base, length):
        ld = work / f"{name}.ld"
        ld.write_text(replace_once(original_ld, "0x08000000, LENGTH = 124K",
                                   f"0x{base:08x}, LENGTH = {length}"), encoding="ascii")
        objects = []
        for index, file in enumerate(files):
            obj = work / f"{name}-{index}.o"
            flags = ["-x", "assembler-with-cpp"] if file.suffix.lower() == ".s" else ["-std=c11"]
            run([gcc, *cpu, *opts, *flags, "-c", file, "-o", obj])
            objects.append(obj)
        elf = work / f"{name}.elf"
        run([gcc, *cpu, *objects, "-T", ld, "-Wl,--gc-sections",
             "-Wl,--defsym=__required_stack_bytes=4096", "--specs=nano.specs",
             "--specs=nosys.specs", "-Wl,--start-group", "-lc", "-lm",
             "-Wl,--end-group", "-o", elf])
        bin_path = work / f"{name}.bin"
        objcopy = gcc.with_name(gcc.name.replace("gcc", "objcopy"))
        run([objcopy, "-O", "binary", elf, bin_path])
        shutil.copyfile(elf, output / elf.name)
        return bin_path.read_bytes()

    peripheral = sorted((vendor / "drivers/src").glob("*.c"))
    app_peripheral = [p for p in peripheral if not p.stem.endswith("_usart")]
    system = ds / "system_at32m412_416.c"
    boot_files = [boot, src / "CAN.c", system, startup]
    boot_files += [p for p in peripheral if p.stem.endswith(("_can", "_gpio", "_crm", "_flash"))]
    bootstrap = link(f"J{node}-boot", boot_files, 0x08000000, APP - 0x08000000)
    system.write_text(replace_once(original_system, "0x0 /*!< vector table",
                                   "0x4000 /*!< vector table"), encoding="ascii")
    dsp_sources = [next(dsp.rglob(file)) for file in
                   ("arm_sin_f32.c", "arm_cos_f32.c", "arm_common_tables.c")]
    application = link(f"J{node}-app", [p for p in sorted(src.glob("*.c")) if p != boot] +
                       sorted(drivers.glob("*.c")) + dsp_sources + app_peripheral + [system, startup],
                       APP, META - APP)
    ota = package(node, application)
    _, crc, padded = unpack(ota)
    (output / f"J{node}.ota").write_bytes(ota)
    ihex(output / f"J{node}-first-install.hex",
         [(0x08000000, bootstrap), (APP, padded),
          (META, struct.pack("<3I", MAGIC, len(padded), crc))])
    manifest = dict(node=node, build_id=tag.hexdigest()[:8], profile=profile,
                    app_bytes=len(application), app_crc32=f"{crc:08x}",
                    files={p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                           for p in sorted(output.glob(f"J{node}*")) if p.suffix in ('.elf','.hex','.ota')})
    (output / f"J{node}-manifest.json").write_text(json.dumps(manifest,indent=2)+"\n",encoding='utf-8')
    print(f"J{node}：boot {len(bootstrap)} B，app {len(application)} B，CRC {crc:08x}", flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--gcc", required=True, type=Path)
    parser.add_argument("--node", required=True, choices=("1", "2", "3", "4", "all"))
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--work-root", type=Path, default=Path(tempfile.gettempdir()))
    args = parser.parse_args()
    output, gcc, work_root = args.output.resolve(), args.gcc.resolve(), args.work_root.resolve()
    if not str(work_root).isascii():
        parser.error("--work-root 需要 ASCII 路径，例如 C:/Temp")
    output.mkdir(parents=True, exist_ok=True)
    work_root.mkdir(parents=True, exist_ok=True)
    profiles = json.loads(PROFILES.read_text(encoding="utf-8"))
    for node in profiles if args.node == "all" else (args.node,):
        with tempfile.TemporaryDirectory(prefix="arm4-", dir=work_root) as directory:
            build(Path(directory), gcc, output, int(node), profiles[node])


if __name__ == "__main__":
    main()

