"""比较本地 J1 升降日志。无参数复核旧基准，传入目录分析新一轮。"""
import argparse
import json
from math import sqrt
from pathlib import Path
from statistics import mean, median


HERE = Path(__file__).resolve().parent
ARCHIVE = HERE / 'j1-current-antiwindup' / 'previous-live-comparison.json'
BASE_BUILD = 'a5693d3f'


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def measure(folder):
    report = read(folder / 'report.json')
    trace = read(folder / 'trace.json')
    columns = trace['columns']
    samples = [dict(zip(columns, row)) for row in trace['samples']]
    results = []
    for trial in report['trials']:
        direction = trial['name'].split('_')[0]
        if direction not in ('up', 'down'):
            raise ValueError(f"Unsupported stage {trial['name']} in {folder}")
        segments = []
        for row in samples:
            if row['stage'] != trial['name'] or not row['power']:
                continue
            if not segments or not 0 < row['uptime_ms'] - segments[-1][-1]['uptime_ms'] <= 80:
                segments.append([])
            segments[-1].append(row)
        if not segments:
            raise ValueError(f"No powered measurements for {trial['name']} in {folder}")
        # 切换工况时可能带入上段保持帧。沿用基准，仅保留本段最长的连续使能记录。
        rows = max(segments, key=len)
        sign = 1 if direction == 'up' else -1
        times = [(row['uptime_ms'] - rows[0]['uptime_ms']) * .001 for row in rows]
        positions = [sign * (row['encoder_counts'] - rows[0]['encoder_counts']) * 360 / 16384
                     for row in rows]
        steps = [(tb - ta, (ta + tb) / 2, pb - pa)
                 for ta, tb, pa, pb in zip(times, times[1:], positions, positions[1:])]
        raw = [(t, d / dt) for dt, t, d in steps if .015 <= dt <= .027]
        smooth = [(raw[i][0], mean(v for _, v in raw[i-2:i+3])) for i in range(2, len(raw)-2)]
        peak = max(range(len(smooth)), key=lambda i: smooth[i][1]) if smooth else None
        peak_t = smooth[peak][0] if peak is not None else None
        saturated = [row for row in rows if row['clipped']]
        conditions = dict(requested_speed_deg_s=report['requested_speed_deg_s'],
                          acceleration_deg_s2=trial['acceleration_deg_s2'],
                          speed_kp_milli=trial['speed_kp_milli'],
                          speed_ki_milli=trial.get('speed_ki_milli',report['speed_ki_milli']),
                          voltage_limit_mv=trial['limit_mv'], iq_limit_ma=report['iq_limit_ma'],
                          test_low_deg=report.get('test_low_deg'), test_high_deg=report.get('test_high_deg'))
        result = dict(folder=folder.name, build=report['build'], direction=direction, conditions=conditions,
                      active_brake_deg_s2=trial['brake_deg_s2'], stroke_deg=trial['stroke_deg'],
                      seconds=round(trial['seconds'], 3), peak_deg_s=None, completed=trial['completed'],
                      main_motion_s=None, tail_s=None, tail_speed_deg_s=None, residue_deg=None,
                      peak_window_iq_range_ma=None, clipped_samples=len(saturated),
                      mean_iq_mismatch_in_saturation_ma=None,
                      around_peak_encoder_velocity_ripple_rms_deg_s=None,
                      phase_a_peak_a=round(trial['phase_a_peak_ma']/1000, 3),
                      phase_b_peak_a=round(trial['phase_b_peak_ma']/1000, 3),
                      bus_peak_v=round(trial['bus_peak_mv']/1000, 3),
                      skipped_velocity_intervals=len(steps)-len(raw))
        if trial['speed_peak_deg_s'] is not None:
            result['peak_deg_s'] = round(trial['speed_peak_deg_s'], 1)
        if peak_t is not None:
            # 速度波动：峰速后0.35秒内，20 ms编码器差分相对其居中5点均值的RMS。
            residuals = [raw[i+2][1] - v for i, (t, v) in enumerate(smooth)
                         if peak_t <= t <= peak_t + .35]
            if smooth[-1][0] >= peak_t + .35:
                result['around_peak_encoder_velocity_ripple_rms_deg_s'] = round(
                    sqrt(mean(value * value for value in residuals)), 1)
            window = [row for t, row in zip(times, rows) if peak_t-.14 <= t <= peak_t+.25]
            clipped = [row for row in window if row['clipped']]
            if window:
                currents = [row['iq_ma'] for row in window]
                result['peak_window_iq_range_ma'] = [min(currents), max(currents)]
            if clipped:
                result['mean_iq_mismatch_in_saturation_ma'] = round(
                    mean(abs(row['iq_ref_ma'] - row['iq_ma']) for row in clipped))
            # 主体结束点是峰后首次持续低于45°/s的位置，末段耗时含实际补位及到位确认。
            last_fast = max((i for i in range(peak, len(smooth)) if abs(smooth[i][1]) >= 45),
                            default=peak-1)
            if trial['completed'] and last_fast+1 < len(smooth):
                slow_t = smooth[last_fast+1][0]
                tail = [v for t, v in smooth if slow_t+.10 <= t <= trial['seconds']-.40]
                first_slow_pos = next(p for t, p in zip(times, positions) if t >= slow_t)
                result.update(main_motion_s=round(slow_t, 2), tail_s=round(trial['seconds']-slow_t, 2),
                              tail_speed_deg_s=round(median(tail), 1) if tail else None,
                              residue_deg=round(positions[-1]-first_slow_pos, 2))
        results.append(result)
    return results


def stored_baselines():
    # 每次都从原始遥测复算，指标或时间基准发生改变时立即报告字段差异。
    reference = read(ARCHIVE)
    measured = {}
    for folder in dict.fromkeys(item['folder'] for item in reference):
        for item in measure(HERE / folder):
            measured[folder, item['direction']] = item
    for expected in reference:
        current = measured[expected['folder'], expected['direction']]
        for key, value in expected.items():
            if current[key] != value:
                raise ValueError(f"Baseline {expected['folder']} {expected['direction']} {key}: "
                                 f"measured {current[key]} vs archived {value}")
    return [item for item in measured.values() if item['build'] == BASE_BUILD]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folders', nargs='*', type=Path, help='测试目录或 report.json，可传多轮')
    parser.add_argument('--json', action='store_true', help='逐轮原值及对基准差值输出为 JSON')
    args = parser.parse_args()
    baselines = stored_baselines()
    rows = []
    for path in args.folders:
        if not path.exists():
            path = HERE / path
        rows.extend(measure(path.parent if path.name == 'report.json' else path))
    if not rows:
        rows = baselines
    for row in rows:
        baseline = next((m for m in baselines if m['direction'] == row['direction']
                         and m['active_brake_deg_s2'] == row['active_brake_deg_s2']), None)
        row['baseline_folder'] = baseline['folder'] if baseline else None
        row['conditions_changed'] = [key for key,value in row['conditions'].items()
                                     if baseline and baseline['conditions'][key] != value]
        row['delta'] = {}
        if baseline and row['completed']:
            for key in ('seconds', 'peak_deg_s', 'tail_s', 'residue_deg', 'clipped_samples',
                        'around_peak_encoder_velocity_ripple_rms_deg_s', 'phase_a_peak_a', 'phase_b_peak_a'):
                if row[key] is not None and baseline[key] is not None:
                    row['delta'][key] = round(row[key]-baseline[key], 3)
    if args.json:
        print(json.dumps(rows, indent=2, ensure_ascii=False))
    else:
        print('build direction brake done total_s peak_deg_s main_s tail_s residue_deg '
              'clip_frames ripple_RMS_deg_s phaseA_A phaseB_A bus_V delta_s delta_RMS')
        for row in rows:
            print(*(row[key] for key in (
                'build','direction','active_brake_deg_s2','completed','seconds','peak_deg_s',
                'main_motion_s','tail_s','residue_deg','clipped_samples',
                'around_peak_encoder_velocity_ripple_rms_deg_s','phase_a_peak_a','phase_b_peak_a','bus_peak_v')),
                row['delta'].get('seconds'), row['delta'].get('around_peak_encoder_velocity_ripple_rms_deg_s'))
            if row['conditions_changed']:
                print('  changed:', ', '.join(row['conditions_changed']))


if __name__ == '__main__':
    main()
