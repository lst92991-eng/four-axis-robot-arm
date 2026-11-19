"""Screen J1 unloaded speed at the real taught stroke; magnet stays unpowered."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import struct
import time
from urllib.request import urlopen

from run_empty_step import HERE, LIVE, Drive, PROFILE, angles
from drive import CLEAR, SET_PROFILE

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--up-only', action='store_true')
    p.add_argument('--at-empty-b2', action='store_true')
    p.add_argument('--single-up', type=int, choices=(500, 600, 660, 720, 800, 900, 1000))
    p.add_argument('--down', type=int, choices=(360, 380, 400), default=360)
    p.add_argument('--down-accel', type=int, choices=(120, 180), default=120)
    p.add_argument('--up-accel', type=int, choices=(120, 240, 300, 360), default=120)
    p.add_argument('--up-brake', type=int, choices=(360, 480, 540, 600, 720), default=360)
    p.add_argument('--up-ki', type=int, choices=(140, 160, 180, 200, 220))
    p.add_argument('--up-voltage', type=int, choices=(6000, 6500, 7000, 7500, 8000))
    args = p.parse_args()
    with urlopen('http://127.0.0.1:8121/state', timeout=2) as r:
        assert not json.load(r)['active']
    z = json.loads((LIVE/'heights-live.json').read_text(encoding='utf8'))['z']
    config = json.loads((LIVE/'game-live.json').read_text(encoding='utf8'))
    arm = json.loads((LIVE/'arm-live.json').read_text(encoding='utf8'))
    layout = json.loads((LIVE/'layout-live.json').read_text(encoding='utf8'))['points']['data']
    b2, park = angles(arm, layout, 1, 2), config['park_joints_deg']
    high, pick = z['travel_deg'], z['pickup_deg']
    PROFILE['acceleration_deg_s2'] = args.down_accel
    d = Drive()
    read = d.can.recv
    trace, tested, gain_reply, tested_gain = [], [], [], []
    outcome = 'started'

    def recv(seconds):
        f = read(seconds)
        if f and not f.error and not f.echo and f.identifier == 0x301 and len(f.data) == 64:
            row = struct.unpack('<16I', f.data)
            if row[0] == 0x3147545a:
                vd = ((row[13] >> 16) ^ 32768) - 32768
                trace.append([row[i] for i in (1, 10, 11, 12, 14, 15)]
                             + [struct.unpack_from('<q', f.data, 8)[0]]
                             + list(struct.unpack_from('<3i', f.data, 28)) + [vd, bool(row[13]&2)])
        if f and not f.error and not f.echo and f.identifier == 0x481 and len(f.data) == 64:
            if f.data[0] == (10 | 0x80) and f.data[2] == 0:
                gain_reply[:] = struct.unpack_from('<4i', f.data, 8)
        return f

    d.can.recv = recv

    def leg(label, goal, speed, fallback=False):
        if label == 'up' and (args.up_ki or args.up_voltage) and not fallback:
            if d.states[1]['power']:
                d.idle(1)
            ki = (args.up_ki or 200) * 1000
            d.command(1, 10, 1600, ki, c=args.up_voltage or 0)
            assert gain_reply[:2] == [1600,ki], gain_reply
            assert not args.up_voltage or gain_reply[3] == args.up_voltage, gain_reply
            tested_gain[:] = gain_reply
            print('J1 KP KI I_LIMIT V_LIMIT ACK',gain_reply,flush=True)
        ramp = 120 if fallback else args.up_accel if label == 'up' else args.down_accel
        brake = args.up_brake if label == 'up' and not fallback else 360
        if PROFILE['acceleration_deg_s2'] != ramp or PROFILE['brake_deg_s2'] != brake:
            d.command(1, SET_PROFILE, brake * 1000, ramp * 1000)
            PROFILE['acceleration_deg_s2'] = ramp
            PROFILE['brake_deg_s2'] = brake
        first = len(trace)
        start = time.monotonic()
        d.z(goal, high=high, speed_limit=speed)
        d.wait(lambda: False, .055, elapsed_ok=True)
        captured = trace[first:]
        assert len(captured) >= 3 and captured[-1][2] and 19000 <= captured[-1][5] <= 30000
        rates = [abs((b[6] - a[6]) * 360000 / (16384 * (b[0] - a[0])))
                 for a,b in zip(captured,captured[5:]) if a[2] and b[2] and 90<=b[0]-a[0]<=140]
        info = dict(direction=label, speed_deg_s=speed, peak_mv=captured[-1][5], seconds=time.monotonic()-start,
                    max_mean_speed_deg_s=max(rates,default=None), peak_iq_ma=max(abs(a[8]) for a in captured),
                    peak_vq_mv=max(abs(a[9]) for a in captured), peak_v_dq_mv=max(
                        round((a[9]**2+a[10]**2)**.5) for a in captured),
                    voltage_clipped_frames=sum(a[11] for a in captured))
        print(info, flush=True)
        return info

    try:
        if args.at_empty_b2:
            d.wait(lambda: len(d.states)==3 and all(a['stationary'] and a['referenced'] and not a['fault']
                for a in d.states.values()),1.5,check=False)
            for n in (2,3):
                if d.states[n]['interlocked']:
                    assert d.states[n]['state']==1
                    d.command(n,CLEAR,check=False)
                    d.wait(lambda: not d.states[n]['interlocked'],.6,check=False)
        a = d.read_initial()
        assert a[1]['state']==0 and not a[1]['power'] and high-4.5<=a[1]['position']<=high+12
        assert 22000 <= a[1]['bus_mv'] <= 26000
        if args.at_empty_b2:
            assert abs(a[2]['position']-b2[0])<=.18 and abs(a[3]['position']-b2[1])<=4
        else:
            assert all(x['state']==0 and x['stationary'] and not x['power'] for x in a.values())
            assert abs(a[2]['position']-park[0])<=.5 and abs(a[3]['position']-park[1])<=4.5
            d.xy(*b2, config['joint_velocity_deg_s'][1], park=True)
        speeds = (args.single_up,) if args.single_up else (
            (420,450,480,500) if args.up_only else (360,380,400,420,440))
        for speed in speeds:
            pair=dict(speed=speed)
            tested.append(pair)
            pair['down']=leg('down',pick,args.down if args.up_only or args.single_up else speed)
            if (args.up_only or args.single_up) and pair['down']['peak_mv']>=28500:
                pair['up']=leg('up',high,360,True)
                outcome='baseline descent reached 28.5V screening margin'
                break
            climb=360 if pair['down']['peak_mv']>=28500 else speed
            pair['up']=leg('up',high,climb)
            pair['same_speed_complete']=climb==speed
            if max(pair['down']['peak_mv'],pair['up']['peak_mv'])>=28500:
                outcome='stopped increasing at 28.5V screening margin'
                break
        else:
            outcome=f'screened up to {speeds[-1]} deg/s'
        d.xy(*park,config['joint_velocity_deg_s'][1],park=True)
        d.finish_park()
        if args.up_voltage and args.up_voltage != 6000:
            d.command(1,10,1600,200000,c=6000)
            assert gain_reply == [1600,200000,1400,6000], gain_reply
        print('VOLTAGE_SCREEN_COMPLETE',outcome,flush=True)
    except BaseException as exc:
        outcome=str(exc)
        print('SCREEN_STOPPED',d.stop(),outcome,flush=True)
        raise
    finally:
        prefix=(f'feedback-speed-up-{args.single_up}' if args.single_up
                else 'voltage-speed-up' if args.up_only else 'voltage-speed')
        if args.up_ki:
            prefix+=f'-ki{args.up_ki}'
        if args.up_voltage:
            prefix+=f'-cv{args.up_voltage}'
        (HERE/f'{prefix}-screen.json').write_text(json.dumps(dict(
            timestamp=datetime.now().astimezone().isoformat(),profile=dict(PROFILE),outcome=outcome,
            tests=tested,trip=[a for a in trace if a[4]][:3],gain_ack=tested_gain or gain_reply,
            last_gain_ack=gain_reply,last_axes=d.states),indent=2),encoding='utf8')
        (HERE/f'{prefix}-trace.json').write_text(json.dumps(trace),encoding='utf8')
        d.close()

if __name__=='__main__':
    main()
