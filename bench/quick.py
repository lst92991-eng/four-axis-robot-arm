"""一次被动 CAN 采样。独立检查与实机测试共用，返回给调用方的仍是同一连接。"""

from time import monotonic
from drive import BUILDS, NODES


# ---------被动采样区---------
def sample(drive, four=False):
    end = monotonic() + 1.0
    while monotonic() < end:
        drive.poll(active=False)
        if all(n in drive.states for n in NODES) and (not four or drive.wrist_status):
            break
    # 三轴状态来自同一 CAN 连接，若收到 J4 的状态则一同返回。
    states = dict(drive.states)
    if drive.wrist_status:
        states[4] = drive.wrist_status
    for n, s in sorted(states.items()):
        print(
            f"J{n}  {s['build']}  {s['bus_mv'] / 1000:.2f} V  位置 {s['position']:.3f}°  "
            f"参考 {int(s['referenced'])}  故障 {s['fault']}/{s['detail']}"
        )
    return states


# ---------状态核对区---------
# 通信检查核对在线和故障，准备运动时还要核对版本、电角配置与机械参考。
def check(states, four=False, motion=False):
    issues = []
    for n in (1, 2, 3, 4) if four else NODES:
        s = states.get(n)
        if s is None:
            issues.append(f"J{n} 无状态帧")
        elif s['fault']:
            issues.append(f"J{n} 故障 {s['fault']}/{s['detail']}")
        elif motion:
            if n in BUILDS and s['build'] != BUILDS[n]:
                issues.append(f"J{n} 版本 {s['build']}，当前台架需要 {BUILDS[n]}")
            elif not s['configured'] or not s['encoder_valid']:
                issues.append(f"J{n} 电角标定或编码器反馈无效")
            elif not s['referenced']:
                issues.append(f"J{n} 需要接续机械参考")
    if issues:
        raise RuntimeError("，".join(issues))


# ---------在已知姿态接续机械参考区---------
def set_reference(drive, states, angles, log):
    """在已知实物标记处接续三轴参考。只发 PING/ZERO，不使能。"""
    import math
    from drive import ZERO

    check(states)
    for n, angle in zip(NODES, angles):
        s = states[n]
        if s['build'] != BUILDS[n]:
            raise RuntimeError(f"J{n} 实板版本与该站点的轴配置不符")
        if s['power'] or s['state'] != 0 or not s['encoder_valid'] or not s['configured']:
            raise RuntimeError(f"J{n} 需要先失能并取得有效电角标定和编码器反馈")
        if not math.isfinite(angle):
            raise ValueError("参考角度需要有限数值")
        # 先核对每个目标角换算成协议整数后的范围，再开始逐轴写入。
        wire = round(angle * 1000)
        if n == 1:
            from drive import Z_ORIGIN

            wire -= round(Z_ORIGIN * 1000)
        if abs(wire) > 2_000_000_000:
            raise ValueError(f"J{n} 参考角超出协议范围")
    for n, angle in zip(NODES, angles):
        drive.sync_sequence(n, check=False)
        drive.command(n, ZERO, round(angle * 1000), check=False)
        # 等下一份新状态回读参考角，同时确认驱动仍失能。
        stamp = states[n]['uptime']
        drive.wait(
            lambda: (
                drive.states[n]['uptime'] != stamp
                and drive.states[n]['referenced']
                and not drive.states[n]['power']
                and abs(drive.states[n]['position'] - angle) <= 0.1
            ),
            1,
            check=False,
        )
        log('reference', node=n, actual=drive.states[n])
        print(f"J{n} 已回读参考 {drive.states[n]['position']:.3f}°")
