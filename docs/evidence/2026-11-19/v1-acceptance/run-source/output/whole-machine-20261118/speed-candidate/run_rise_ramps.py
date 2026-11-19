"""Four empty Z lifts on one taught stroke; no game or magnetic actuation."""
import argparse
from datetime import datetime
import json
import struct
import time
from urllib.request import urlopen

from run_empty_step import HERE, LIVE, Drive, PROFILE, SET_PROFILE, angles
from drive import CLEAR

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--at-empty-b2', action='store_true')
    p.add_argument('--brake-sweep', action='store_true')
    p.add_argument('--from-brake', type=int, choices=(780,840,900))
    p.add_argument('--rise-speed', type=int, choices=(950,1200), default=1200)
    opt = p.parse_args()
    limited_sweep = opt.rise_speed == 950
    if limited_sweep and (not opt.brake_sweep or opt.at_empty_b2 or opt.from_brake):
        p.error('950-degree/s sweep starts and finishes at the observation pose')
    profiles = [(400,b) for b in (720,780,840,900)] if opt.brake_sweep else [
        (a,720) for a in (240,300,360,400)]
    if opt.from_brake:
        assert opt.brake_sweep
        profiles = [(a,b) for a,b in profiles if b >= opt.from_brake]
    with urlopen('http://127.0.0.1:8121/state',timeout=2) as r:
        assert not json.load(r)['active']
    cfg = json.loads((LIVE/'game-live.json').read_text(encoding='utf8'))
    z = json.loads((LIVE/'heights-live.json').read_text(encoding='utf8'))['z']
    arm = json.loads((LIVE/'arm-live.json').read_text(encoding='utf8'))
    layout = json.loads((LIVE/'layout-live.json').read_text(encoding='utf8'))['points']['data']
    saved_path = 'rise-brakes-before-test.json' if opt.brake_sweep else 'rise-ramps-before-update.json'
    saved = {} if limited_sweep else json.loads((HERE/saved_path).read_text(encoding='utf8'))
    start_xy = [saved[str(n)]['position'] for n in (2,3)] if saved else []
    start_z = saved['1']['position'] if saved else None
    b2 = angles(arm,layout,1,2)
    high, pick = z['travel_deg'], z['pickup_deg']
    PROFILE.update(acceleration_deg_s2=180,brake_deg_s2=360)
    d = Drive()
    original = d.can.recv
    trace, legs, ramps_ack, gains_ack = [], [], [], []
    outcome = 'interrupted'
    interrupted_leg = return_error = None
    interrupted_axes = None
    at_test_point = False
    returned = False
    at_height = lambda: abs(d.states[1]['position']-high)<=4.5

    def recv(seconds):
        f=original(seconds)
        if f and not f.echo and not f.error and len(f.data)==64:
            if f.identifier==0x301:
                words=struct.unpack('<16I',f.data)
                if words[0]==0x3147545a:
                    vd=((words[13]>>16)^32768)-32768
                    iq, vq=struct.unpack_from('<2i',f.data,32)
                    count=struct.unpack_from('<q',f.data,8)[0]
                    trace.append([words[1],words[10],words[11],words[12],words[14],words[15],
                                  count,iq,vq,vd,bool(words[13]&2)])
            elif f.identifier==0x481 and f.data[2]==0:
                if f.data[0]==(10|128):
                    gains_ack[:]=struct.unpack_from('<4i',f.data,8)
                elif f.data[0]==(13|128):
                    ramps_ack[:]=struct.unpack_from('<2i',f.data,8)
        return f

    d.can.recv=recv

    def tune(voltage, accel, brake):
        if d.states[1]['power']:
            d.idle(1)
        d.command(1,10,1600,200000,c=voltage)
        assert gains_ack == [1600,200000,1400,voltage], gains_ack
        d.command(1,SET_PROFILE,brake*1000,accel*1000)
        assert ramps_ack == [brake*1000,accel*1000], ramps_ack
        PROFILE.update(acceleration_deg_s2=accel,brake_deg_s2=brake)

    def measure(name, goal, voltage, accel, brake, speed):
        nonlocal interrupted_leg
        t0=time.monotonic()
        interrupted_leg=dict(name=name,goal=goal,voltage_mv=voltage,accel=accel,brake=brake,speed_cmd=speed)
        from_position = d.states[1]['position']
        idx=len(trace)
        tune(voltage,accel,brake)
        begin=time.monotonic()
        d.z(goal,high=high,speed_limit=speed)
        end=time.monotonic()
        d.wait(lambda:False,.055,elapsed_ok=True)
        rows=trace[idx:]
        assert len(rows)>=3 and rows[-1][2] and rows[-1][5]>19000
        speeds=[(b[6]-a[6])*360000/(16384*(b[0]-a[0])) for a,b in zip(rows,rows[5:])
                if a[2] and b[2] and 90<=b[0]-a[0]<=140]
        result=dict(name=name,seconds=end-t0,motion_seconds=end-begin,stroke_deg=abs(goal-from_position),
                    peak_speed=max(map(abs,speeds),default=None),
                    bus_peak_mv=rows[-1][5],measured_iq_peak_ma=max(abs(a[7]) for a in rows),
                    clip_sample_count=sum(a[10] for a in rows),powered_samples=sum(a[2] for a in rows),
                    voltage_mv=voltage,accel=accel,brake=brake,speed_cmd=speed)
        legs.append(result)
        interrupted_leg = None
        print(name, result,flush=True)
        return result

    def return_to_start():
        nonlocal returned
        assert at_height()
        tune(6000,120,360)
        d.xy(*start_xy,12,park=True)
        d.elbow(start_xy[1],tolerance=1.2)
        d.move(1,start_z,60,tolerance=.12)
        d.finish_park()
        tune(6000,120,360)
        d.wait(lambda:False,.4,elapsed_ok=True)
        returned = all(a['state']==0 and a['stationary'] and not a['power']
                       and a['referenced'] for a in d.states.values())

    try:
        a=d.read_initial()
        assert all(s['state']==0 and s['stationary'] and not s['power'] for s in a.values())
        assert at_height() and 22000 <= a[1]['bus_mv'] <= 26000
        if limited_sweep:
            saved = {str(n):s.copy() for n,s in a.items()}
            start_xy = [saved[str(n)]['position'] for n in (2,3)]
            start_z = saved['1']['position']
            (HERE/'four-z-brakes-950-start.json').write_text(json.dumps(saved,indent=2),encoding='utf8')
            prior = json.loads((HERE/'four-z-brakes-restored.json').read_text(encoding='utf8'))['final_axes']
            assert all(abs(saved[str(n)]['position']-prior[str(n)]['position'])<=.25
                       for n in (1,2,3))
        if opt.at_empty_b2:
            assert abs(a[2]['position']-b2[0])<=.5 and abs(a[3]['position']-b2[1])<=4
        else:
            assert abs(a[2]['position']-start_xy[0])<=.5 and abs(a[3]['position']-start_xy[1])<=4
            d.xy(*b2,cfg['joint_velocity_deg_s'][1],park=True)
        at_test_point = True
        for accel, brake in profiles:
            label = brake if opt.brake_sweep else accel
            down=measure(f'down_before_{label}',pick,6000,180,360,380)
            if down['bus_peak_mv']>=28500:
                tune(6000,120,360)
                d.z(high,high=high,speed_limit=180)
                outcome='screening margin reached on unchanged descent'
                break
            up=measure(f'rise_{label}',high,8000,accel,brake,opt.rise_speed)
            if up['bus_peak_mv']>=28500:
                outcome='screening margin reached on rise'
                break
        else:
            outcome='four lifts completed'
        return_to_start()
        print('FINISHED',outcome,'prior working profile restored',flush=True)
    except BaseException as e:
        outcome=str(e)
        stopped = d.stop()
        interrupted_axes = {n:a.copy() for n,a in d.states.items()}
        print('STOPPED',stopped,outcome,flush=True)
        fault = d.states.get(1,{})
        recoverable = (limited_sweep and isinstance(e,Exception) and stopped and at_test_point
                       and interrupted_leg and interrupted_leg['name'].startswith('rise_')
                       and (fault.get('fault'),fault.get('detail'))==(5,4))
        if recoverable:
            try:
                d.wait(lambda:False,.5,check=False,elapsed_ok=True)
                assert all(s['stationary'] and s['referenced'] and s['encoder_valid']
                           and 22000<=s['bus_mv']<=26000 for s in d.states.values())
                assert all(time.monotonic()-d.last_update[n]<.18 for n in (1,2,3))
                assert all(not d.states[n]['fault'] and not d.states[n]['interlocked']
                           for n in (2,3))
                assert abs(d.states[2]['position']-b2[0])<=.5
                assert abs(d.states[3]['position']-b2[1])<=4
                z_stopped = d.states[1]['position']
                assert pick < z_stopped < high and not d.states[1]['power']
                d.command(1,CLEAR,check=False)
                d.wait(lambda:d.states[1]['state']==0 and not d.states[1]['fault']
                       and d.states[1]['stationary'] and d.states[1]['referenced']
                       and abs(d.states[1]['position']-z_stopped)<=.15,.9,check=False)
                tune(6000,60,360)
                d.z(high,high=high,speed_limit=120)
                return_to_start()
                print('RETURNED_AFTER_TRIP',returned,flush=True)
            except Exception as recovery:
                return_error = str(recovery)
                print('RETURN_STOPPED',d.stop(),return_error,flush=True)
        raise
    finally:
        result=dict(build=PROFILE['build_id'],start_pose_deg=dict(zip(('1','2','3'),[start_z,*start_xy])),
                    requested=dict(profiles=[dict(accel=a,brake=b) for a,b in profiles],speed=opt.rise_speed,voltage=8000),
                    taught_stroke_deg=high-pick,outcome=outcome,interrupted_leg=interrupted_leg,legs=legs,last_axes=d.states,
                    interrupted_axes=interrupted_axes,returned=returned,return_error=return_error)
        prefix='four-z-brakes' if opt.brake_sweep else 'four-z-lifts'
        if limited_sweep:
            prefix+='-950'
        if opt.from_brake:
            prefix+=f'-resume{opt.from_brake}'
        (HERE/f'{prefix}-report.json').write_text(json.dumps(result,indent=2),encoding='utf8')
        (HERE/f'{prefix}-trace.json').write_text(json.dumps(trace),encoding='utf8')
        d.close()

if __name__=='__main__':
    main()
