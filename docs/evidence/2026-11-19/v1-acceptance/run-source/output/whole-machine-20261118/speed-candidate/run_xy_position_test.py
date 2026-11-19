"""Compare old and absolute-target XY on the same path, with the Z axis untouched."""
import argparse
from datetime import datetime
import json
import math
import time
from urllib.request import urlopen

from run_empty_step import HERE, LIVE, Drive, angles
from drive import BUILDS, CLEAR, PACKET


def read_json(path):
    return json.loads(path.read_text(encoding='utf8'))


def xy_mm(arm, shoulder, motor):
    l1, l2 = arm['arm_lengths_mm']
    q2 = arm['shoulder_ref_rad'] + math.radians(
        (shoulder - arm['j2_reference_deg']) * arm['j2_scale'] * arm['j2_sign'])
    q3 = arm['elbow_ref_rad'] + math.radians(
        (motor - arm['j3_reference_motor_deg']) * arm['j3_sign'] / arm['j3_ratio'])
    return l1 * math.cos(q2) + l2 * math.cos(q2 + q3), l1 * math.sin(q2) + l2 * math.sin(q2 + q3)


def main():
    args = argparse.ArgumentParser()
    args.add_argument('--direct-only', action='store_true')
    args.add_argument('--shoulder-out', type=int, choices=(18,22), default=22)
    args.add_argument('--elbow-speed', type=int, choices=(22,24,30), default=30)
    args.add_argument('--j3-ki', type=float, choices=(7.5,35))
    opt = args.parse_args()
    with urlopen('http://127.0.0.1:8121/state', timeout=2) as response:
        assert not json.load(response)['active'], 'The game controller is active'
    arm = read_json(LIVE/'arm-live.json')
    layout = read_json(LIVE/'layout-live.json')['points']['data']
    high = read_json(LIVE/'heights-live.json')['z']['travel_deg']
    last_park = read_json(HERE/'four-z-brakes-950-report.json')['last_axes']
    b2 = angles(arm, layout, 1, 2)
    stamp = datetime.now().strftime('%Y%m%d-%H%M%S')
    prefix = HERE/f'xy-position-{stamp}'
    print('LOG', prefix.name, flush=True)
    report = {'builds': BUILDS, 'at_b2_deg': b2, 'stages': []}
    trace = []
    leg = 'preflight'
    initial = None
    armed = False
    d = Drive()
    receive = d.poll
    sampled = {}
    clock = time.monotonic()
    received = d.can.recv
    gains = []

    def read_frame(timeout):
        frame = received(timeout)
        if frame and not frame.echo and frame.identifier == 0x483 and len(frame.data) == 64:
            kind, _, result, _, sequence, *values = PACKET.unpack(frame.data)
            if kind == (10 | 128) and result == 0:
                gains[:] = values[:3]
        return frame

    d.can.recv = read_frame

    def poll(active=True):
        receive(active=active)
        for n, a in d.states.items():
            if sampled.get(n) != a['uptime']:
                sampled[n] = a['uptime']
                trace.append((round(time.monotonic() - clock, 5), leg, n, a.copy()))

    d.poll = poll

    def hold_sample(goal):
        d.wait(lambda: False, .8, elapsed_ok=True)
        axes = {n:d.states[n].copy() for n in (2,3)}
        assert all(a['state'] == 1 and a['power'] and a['stationary']
                   and abs(a['position'] - goal[n - 2]) <= .101 for n,a in axes.items())
        return axes

    def measure(name, method, goal, shoulder_speed):
        nonlocal leg
        leg = name
        elbow_speed = opt.elbow_speed if method == 'absolute' else 30
        record = {'name': name, 'start_deg': [d.states[n]['position'] for n in (2,3)],
                  'goal_deg': goal, 'speed_limits_deg_s': [shoulder_speed,elbow_speed]}
        report['stages'].append(record)
        start = time.monotonic()
        if method == 'old':
            d.xy(*goal, shoulder_speed, park=True)
            d.elbow(goal[1], tolerance=1.2)
        else:
            d.xy_position(*goal, shoulder_speed, elbow_speed)
        record['seconds'] = time.monotonic() - start
        actual = tuple(d.states[n]['position'] for n in (2,3))
        record['error_deg'] = [a - b for a,b in zip(actual, goal)]
        record['estimated_xy_error_mm'] = math.dist(xy_mm(arm,*actual),xy_mm(arm,*goal))
        record['arrived_axes'] = {n:d.states[n].copy() for n in (2,3)}
        if method == 'absolute':
            record['after_hold_axes'] = hold_sample(goal)
        assert d.states[1]['stationary'] and abs(d.states[1]['position']-initial['1']['position'])<=.15
        print(name, round(record['seconds'],3), [round(v,3) for v in record['error_deg']],
              round(record['estimated_xy_error_mm'],3), flush=True)

    try:
        d.wait(lambda: len(d.states)==3,1.5,check=False)
        d.wait(lambda: False,.35,check=False,elapsed_ok=True)
        assert all(a['state'] in (0,1) and a['stationary'] and a['referenced']
                   and a['encoder_valid'] and not a['fault'] and a['build']==BUILDS[n]
                   and abs(a['position']-last_park[str(n)]['position'])<=.25
                   for n,a in d.states.items())
        assert high-4.5<=d.states[1]['position']<=high+4.5
        for n in (2,3):
            if d.states[n]['interlocked']:
                assert d.states[n]['state']==1
                d.command(n,CLEAR,check=False)
                d.wait(lambda: not d.states[n]['interlocked'],.8,check=False)
        d.healthy()
        initial = {str(n):a.copy() for n,a in d.states.items()}
        report['initial_axes'] = initial
        park = tuple(initial[str(n)]['position'] for n in (2,3))
        for n in (2,3):
            d.sync_sequence(n)
        if opt.j3_ki is not None:
            if d.states[3]['power']:
                d.idle(3)
            d.command(3,10,1000,round(opt.j3_ki*1000))
            assert gains == [1000,round(opt.j3_ki*1000),1000], gains
            report['j3_speed_gains'] = gains.copy()
        armed = True
        if not opt.direct_only:
            measure('old_out', 'old', b2, 22)
            measure('old_back', 'old', park, 12)
        measure('absolute_out', 'absolute', b2, opt.shoulder_out)
        measure('absolute_back', 'absolute', park, 12)
        report['returned'] = True
    except BaseException as error:
        report['error'] = str(error)
        report['failed_leg'] = leg
        report['fault_axes'] = {n:a.copy() for n,a in d.states.items()}
        if armed:
            report['stopped'] = d.stop()
        raise
    finally:
        report['final_axes'] = d.states
        prefix.with_suffix('.json').write_text(json.dumps(report,indent=2),encoding='utf8')
        prefix.with_suffix('.jsonl').write_text(
            ''.join(json.dumps(row)+'\n' for row in trace),encoding='utf8')
        d.close()


if __name__ == '__main__':
    main()
