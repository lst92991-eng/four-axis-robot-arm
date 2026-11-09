"""Decode passive driver telemetry; these packets never authorize motion."""
import math
import struct


def diagnostic(identifier, data):
    if len(data) != 64:
        raise ValueError('诊断帧长度错误')
    if 0x301 <= identifier <= 0x304:
        v = struct.unpack_from('<IIqf6i3I', data)
        if v[0] != 0x3147545a or not math.isfinite(v[3]):
            raise ValueError('Z轴诊断帧格式错误')
        return identifier-0x300, 'trace', dict(board_ms=v[1], encoder_counts=v[2],
            speed_counts_s=v[3], position_deg=v[4]/1000, target_deg=v[5]/1000,
            iq_ref_ma=v[6], iq_ma=v[7], vq_request_mv=v[8], bus_mv=v[9],
            power=bool(v[10]), fault=v[11], encoder_valid=bool(v[12]))
    if 0x381 <= identifier <= 0x384:
        v = struct.unpack_from('<I12i', data)
        if v[0] != 0x31464349:
            raise ValueError('保护现场诊断帧格式错误')
        names = ['cause', 'board_ms', 'phase_a_counts', 'phase_b_counts', 'calibrating',
                 'reference_ma', 'previous_id_ma', 'previous_iq_ma', 'previous_ua_mv',
                 'previous_ub_mv', 'encoder_counts', 'sample_window_valid']
        return identifier-0x380, 'current_fault', dict(zip(names,v[1:]))
    raise ValueError('未知诊断帧')
