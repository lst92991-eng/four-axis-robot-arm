"""Time one real, empty b2-to-f2 move; no H-bridge commands."""
from datetime import datetime
import argparse
import json
import math
from pathlib import Path
import struct
import sys
import time
from urllib.request import urlopen

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
LIVE = ROOT / 'output/whole-machine-20261117/usb-game'
sys.path[:0] = [str(LIVE.parent / 'readonly-usb'), str(ROOT / 'manual_control')]
from drive import Drive, PROFILE, SET_PROFILE
from drive import BUILDS, CLEAR, MOVE

def angles(arm, xy, col, rank):
    a, b = arm['arm_lengths_mm']
    x = xy[2] + (xy[6] - xy[2]) * col / 8 + (xy[10] - xy[2]) * rank / 9
    y = xy[3] + (xy[7] - xy[3]) * col / 8 + (xy[11] - xy[3]) * rank / 9
    elbow = math.copysign(math.acos((x*x+y*y-a*a-b*b)/(2*a*b)), arm['elbow_ref_rad'])
    shoulder = math.atan2(y, x) - math.atan2(b*math.sin(elbow), a+b*math.cos(elbow))
    dq2 = math.degrees(math.remainder(shoulder - arm['shoulder_ref_rad'], 2*math.pi))
    dq3 = math.degrees(math.remainder(elbow - arm['elbow_ref_rad'], 2*math.pi))
    return (arm['j2_reference_deg'] + dq2/(arm['j2_scale']*arm['j2_sign']),
            arm['j3_reference_motor_deg'] + dq3*arm['j3_ratio']/arm['j3_sign'])

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--acceleration', type=int, choices=(120, 180, 240, 300, 360), default=120)
    p.add_argument('--speed', type=int, choices=(360, 380, 450, 500), default=450)
    p.add_argument('--rise-speed', type=int, choices=(360, 450, 500, 600, 660, 720, 950, 1000))
    p.add_argument('--rise-accel', type=int, choices=(120, 180, 240, 300, 360, 400))
    p.add_argument('--rise-brake', type=int, choices=(360, 480, 600, 720), default=360)
    p.add_argument('--rise-voltage', type=int, choices=(6000, 7000, 7500, 8000), default=6000)
    p.add_argument('--xy-speed', type=int, choices=(22, 26, 30))
    p.add_argument('--position-hold', action='store_true')
    clearance_mode = p.add_mutually_exclusive_group()
    clearance_mode.add_argument('--verify-clearance-mm', type=float, choices=(16.5,))
    clearance_mode.add_argument('--clearance-proof', type=Path)
    args = p.parse_args()
    clearance_trial = bool(args.verify_clearance_mm or args.clearance_proof)
    if clearance_trial and not args.position_hold:
        p.error('Clearance tests require --position-hold')
    PROFILE['acceleration_deg_s2'] = args.acceleration
    PROFILE['brake_deg_s2'] = 360
    with urlopen('http://127.0.0.1:8121/state', timeout=2) as response:
        assert not json.load(response)['active'], 'stop the live game before the empty timing run'
    z = json.loads((LIVE/'heights-live.json').read_text(encoding='utf8'))['z']
    cfg = json.loads((LIVE/'game-live.json').read_text(encoding='utf8'))
    arm = json.loads((LIVE/'arm-live.json').read_text(encoding='utf8'))
    layout = json.loads((LIVE/'layout-live.json').read_text(encoding='utf8'))['points']['data']
    park = cfg['park_joints_deg']
    b2, f2 = angles(arm, layout, 1, 2), angles(arm, layout, 5, 2)
    high, pick = z['travel_deg'], z['pickup_deg']
    degrees_per_mm = (high-pick)/z['operator_estimated_rise_mm']
    drop = pick+degrees_per_mm*z['release_offset_mm']
    clearance = {}
    if args.verify_clearance_mm:
        clearance['test_deg'] = pick+degrees_per_mm*args.verify_clearance_mm
    if args.clearance_proof:
        proof = json.loads(args.clearance_proof.read_text(encoding='utf8'))
        assert proof['move'] == 'empty b2-f2 clearance validation'
        assert proof['goals']['b2'] == list(b2) and proof['goals']['f2'] == list(f2)
        assert abs(proof['goals']['pickup_z']-pick)<.001
        test_deg = proof['clearance']['test_deg']
        assert abs(test_deg-(pick+16.5*degrees_per_mm))<.001
        clearance.update(proof=args.clearance_proof.name,test_deg=test_deg,
                         floor_deg=proof['clearance']['verified_minimum_deg'],
                         trigger_deg=test_deg+degrees_per_mm)
    d = Drive()
    samples = []
    trace = []
    trace_uptime = {}
    gain_ack = []
    recv = d.can.recv
    voltage = None if args.position_hold else 6000
    leg = 'preflight'
    clock = time.monotonic()
    start_pose = {}
    active = False
    z_phases = {}
    stamp = datetime.now().strftime('%Y%m%d-%H%M%S')
    prefix = ('clearance-static' if args.verify_clearance_mm else
              'empty-step-overlap' if args.clearance_proof else 'empty-step-hold')
    current_run = HERE/f'{prefix}-{stamp}'
    clearance_floor = None
    commands = []
    send = d.send

    def record_command(n, kind, a=0, b=0, **fields):
        if kind == MOVE:
            commands.append(dict(stage=leg,node=n,seconds=time.monotonic()-clock,
                                 target_deg=a/1000,speed_deg_s=b/1000))
        return send(n,kind,a,b,**fields)

    if clearance_trial:
        d.send = record_command

    def inspect(timeout):
        f = recv(timeout)
        if f and not f.error and not f.echo and f.identifier==0x481 and len(f.data)==64:
            if f.data[0]==(10|128) and f.data[2]==0:
                gain_ack[:] = struct.unpack_from('<4i',f.data,8)
        return f

    d.can.recv = inspect

    def sample():
        if clearance_floor is not None and d.states[1]['position'] < clearance_floor:
            raise RuntimeError('J1 dropped below the verified empty-path clearance')
        if args.position_hold:
            for n,a in d.states.items():
                if trace_uptime.get(n) != a['uptime']:
                    trace_uptime[n] = a['uptime']
                    trace.append((round(time.monotonic()-clock,5),leg,n,a.copy()))
            return
        a = d.states.get(1)
        if a and (not trace or trace[-1][0] != a['uptime']):
            trace.append([a[k] for k in ('uptime', 'counts', 'position', 'velocity', 'bus_mv', 'state', 'fault', 'detail')])

    d.on_poll = sample

    def stage(name, work):
        nonlocal leg
        leg = name
        start = time.monotonic()
        work()
        if clearance_trial:
            sample()
        result = dict(stage=name,seconds=time.monotonic()-start)
        if args.position_hold:
            result.update(z_phases.pop(name,{}),axes={n:a.copy() for n,a in d.states.items()})
        samples.append(result)
        print(name, f'{samples[-1]["seconds"]:.2f}s', flush=True)

    def move_z(goal, on_rise=None):
        nonlocal voltage
        started = time.monotonic()
        ramp = (args.rise_accel or args.acceleration) if goal == high else args.acceleration
        brake = args.rise_brake if goal == high else 360
        cap = args.rise_voltage if goal == high else 6000
        if cap != voltage:
            if d.states[1]['power']:
                d.idle(1)
            d.command(1,10,1600,200000,c=cap)
            assert gain_ack == [1600,200000,1400,cap], gain_ack
            voltage = cap
        if ramp != PROFILE['acceleration_deg_s2'] or brake != PROFILE['brake_deg_s2']:
            d.command(1, SET_PROFILE, brake * 1000, ramp * 1000)
            PROFILE['acceleration_deg_s2'] = ramp
            PROFILE['brake_deg_s2'] = brake
        begin = time.monotonic()
        d.z(goal, high=high, speed_limit=(args.rise_speed or args.speed) if goal == high else args.speed,
            on_rise=on_rise)
        if args.position_hold:
            z_phases[leg] = dict(setup_seconds=begin-started,motion_seconds=time.monotonic()-begin)

    def move_xy(goal, returning=False):
        if args.position_hold:
            d.xy_position(*goal,12 if returning else 18,30)
        else:
            d.xy(*goal, args.xy_speed or cfg['joint_velocity_deg_s'][1], park=returning)

    def verify_corridor():
        nonlocal clearance_floor
        clearance_floor = clearance['test_deg']-4.5
        try:
            move_xy(f2)
        finally:
            clearance_floor = None
        clearance['verified_minimum_deg'] = min(a['position'] for _,k,n,a in trace if k==leg and n==1)
        clearance['verified_minimum_mm'] = (clearance['verified_minimum_deg']-pick)/degrees_per_mm

    def raise_and_transport():
        nonlocal clearance_floor
        def launch(a):
            nonlocal clearance_floor
            if 'launch_deg' in clearance or a['position']<clearance['trigger_deg']:
                return
            clearance['launch_deg'] = a['position']
            clearance['launch_mm'] = (a['position']-pick)/degrees_per_mm
            clearance['launch_velocity_deg_s'] = a['velocity']
            clearance_floor = clearance['floor_deg']
            try:
                move_xy(f2)
            finally:
                clearance_floor = None
        move_z(high,on_rise=launch)
        assert 'launch_deg' in clearance

    def finish_park():
        if not args.position_hold:
            d.finish_park()
            return
        if d.states[1]['power']:
            d.idle(1)
        assert abs(d.states[1]['position']-high)<=4.5
        assert all(d.states[n]['state']==1 and d.states[n]['power'] and d.states[n]['stationary']
                   and abs(d.states[n]['position']-park[n-2])<=.101 for n in (2,3))

    try:
        if args.position_hold:
            last = json.loads((HERE/'xy-position-summary.json').read_text(encoding='utf8'))['axes_after_host_disconnected']
            d.wait(lambda:len(d.states)==3,1.2,check=False)
            d.wait(lambda:False,.35,check=False,elapsed_ok=True)
            assert all(s['build']==BUILDS[n] and s['uptime']>=last[str(n)]['uptime'] and s['stationary']
                       and s['referenced'] and s['configured'] and s['encoder_valid'] and not s['fault']
                       and (n==1 or abs(s['position']-last[str(n)]['position'])<=.25)
                       for n,s in d.states.items())
            assert all(d.states[n]['state']==1 and d.states[n]['power'] for n in (2,3))
            assert d.states[1]['state']==0 and not d.states[1]['power']
            for n in (2,3):
                if d.states[n]['interlocked']:
                    d.command(n,CLEAR,check=False)
                    d.wait(lambda:not d.states[n]['interlocked'],.8,check=False)
            park = tuple(last[str(n)]['target'] for n in (2,3))
        start_pose = {n: s.copy() for n,s in d.read_initial().items()}
        if not args.position_hold:
            assert all(a['state']==0 and a['stationary'] and not a['power'] for a in start_pose.values())
        assert high-4.5<=start_pose[1]['position']<=high+12
        assert abs(start_pose[2]['position']-park[0])<=.5 and abs(start_pose[3]['position']-park[1])<=4
        active = True
        start = time.monotonic()
        stage('approach',lambda:move_xy(b2))
        if args.verify_clearance_mm:
            stage('lower_to_clearance',lambda:move_z(clearance['test_deg']))
            stage('verify_corridor',verify_corridor)
            stage('raise_from_clearance',lambda:move_z(high))
        else:
            stage('lower_to_pick',lambda:move_z(pick))
            stage('pickup_dwell',lambda:d.wait(lambda:False,.17,elapsed_ok=True))
            if args.clearance_proof:
                stage('raise_and_transport',raise_and_transport)
            else:
                stage('raise_after_pick',lambda:move_z(high))
                stage('transport',lambda:move_xy(f2))
            stage('lower_to_release',lambda:move_z(drop))
            stage('release_dwell',lambda:d.wait(lambda:False,.012,elapsed_ok=True))
            stage('raise_after_release',lambda:move_z(high))
        stage('return_to_park',lambda:move_xy(park,True))
        stage('confirm_stopped',finish_park)
        elapsed = time.monotonic()-start
        leg = 'restore_z_profile'
        if voltage != 6000:
            d.command(1,10,1600,200000,c=6000)
            assert gain_ack == [1600,200000,1400,6000], gain_ack
        if args.position_hold:
            d.command(1,SET_PROFILE,360000,120000)
        axes = {n: a.copy() for n,a in d.states.items()}
        result = dict(timestamp=datetime.now().astimezone().isoformat(),move="empty b2-f2",timing_seconds=elapsed,
                      stages=samples,profile=dict(PROFILE,acceleration_deg_s2=args.acceleration, speed_deg_s=args.speed,
                          rise_speed_deg_s=args.rise_speed or args.speed, rise_accel_deg_s2=args.rise_accel or args.acceleration,
                          rise_brake_deg_s2=args.rise_brake,rise_voltage_mv=args.rise_voltage,
                          xy_speed_deg_s=args.xy_speed),park=axes)
        suffix=f'-up{args.rise_speed}' if args.rise_speed else ''
        out = HERE / f'empty-step-{args.acceleration}-{args.speed}{suffix}.json'
        if args.position_hold:
            out = current_run.with_suffix('.json')
            result.update(start_axes=start_pose, goals={'park':park,'b2':b2,'f2':f2,
                          'pickup_z':pick,'release_z':drop,'travel_z':high},
                          xy_deg_s={'approach':[18,30],'transport':[18,30],'return':[12,30]},
                          profile_restore={'voltage_mv':6000,'acceleration_deg_s2':120,'brake_deg_s2':360})
        if clearance_trial:
            if args.verify_clearance_mm:
                result['move'] = 'empty b2-f2 clearance validation'
            result.update(clearance=clearance,move_commands=commands)
        out.write_text(json.dumps(result,indent=2)+'\n',encoding='utf8')
        print(f'EMPTY_STEP_COMPLETE {elapsed:.3f}s {out.name}',flush=True)
    except BaseException as error:
        stopped=d.stop() if active else False
        out = current_run.with_suffix('.interrupted.json') if args.position_hold else HERE/'empty-step-interrupted.json'
        out.write_text(json.dumps(dict(timestamp=datetime.now().astimezone().isoformat(),
            stages=samples,error=str(error),failed_leg=leg,start_axes=start_pose,clearance=clearance,
            move_commands=commands,
            stopped=stopped,axes=d.states),indent=2),encoding='utf8')
        print(f'EMPTY_STEP_INTERRUPTED {out.name}',flush=True)
        raise
    finally:
        d.close()
        trace_path = current_run.with_suffix('.trace.json') if args.position_hold else HERE/f'j1-frames-{args.acceleration}-{args.speed}.json'
        trace_path.write_text(json.dumps(trace),encoding='utf8')

if __name__=='__main__':
    main()
