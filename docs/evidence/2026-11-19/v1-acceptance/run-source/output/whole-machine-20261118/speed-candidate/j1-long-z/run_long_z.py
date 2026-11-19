"""One guarded, off-board long Z round trip at the operator-confirmed height."""
import argparse
import json
import math
from pathlib import Path
import struct
import sys
import time
from urllib.request import urlopen

FOLDER = Path(__file__).resolve().parent
BENCH = FOLDER.parent
sys.path.insert(0, str(BENCH))
from run_empty_step import Drive, PROFILE
from drive import BUILDS, CLEAR, SET_PROFILE, PACKET, ZERO

LIVE = BENCH.parents[1] / 'whole-machine-20261117/usb-game'


def load(path):
    return json.loads(path.read_text(encoding='utf-8'))

class RelativeZDrive(Drive):
    def healthy(self):
        now=time.monotonic()
        for n in (1,2,3):
            a=self.states.get(n)
            if not a or now-self.last_update[n]>.25:
                raise RuntimeError(f'J{n} status expired during Z test')
            if a['build']!=BUILDS[n] or not a['encoder_valid'] or a['fault']:
                raise RuntimeError(f'J{n} state unavailable: {a}')
        z=self.states[1]
        upper=PROFILE['running_bus_max_mv'] if z['power'] else 30000
        if not z['referenced'] or z['interlocked'] or not 10000<=z['bus_mv']<=upper:
            raise RuntimeError(f'J1 relative reference unavailable: {z}')
        if any(self.states[n]['power'] or self.states[n]['state']!=0 for n in (2,3)):
            raise RuntimeError('The two rotary axes must be disabled for this relative Z test')


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--upper-cycles',type=int,choices=(3,))
    parser.add_argument('--height30',action='store_true')
    parser.add_argument('--resume-approach',action='store_true')
    parser.add_argument('--relative-cycles',type=int,choices=(10,))
    parser.add_argument('--resume-relative',action='store_true')
    args=parser.parse_args()
    if args.height30 and not args.upper_cycles:
        parser.error('--height30 requires the three upper cycles')
    if args.resume_approach and not args.height30:
        parser.error('--resume-approach requires the interrupted 30 cm approach')
    if args.relative_cycles and args.upper_cycles:
        parser.error('choose either an upper height or a relative test')
    if args.resume_relative and not args.relative_cycles:
        parser.error('--resume-relative requires a relative test')
    build_folder=BENCH/'j1-long-z30' if args.height30 or args.relative_cycles else FOLDER
    out=BENCH/'j1-after-repair-z43' if args.relative_cycles else BENCH/'j1-upper-z30' if args.height30 else BENCH/'j1-upper-z' if args.upper_cycles else FOLDER
    out.mkdir(exist_ok=True)
    with urlopen('http://127.0.0.1:8121/state', timeout=2) as response:
        assert not json.load(response)['active']
    firmware = load(build_folder/'J1-manifest.json')
    envelope = load(build_folder/'test-envelope.json')
    assert firmware['build_id'] == PROFILE['build_id']
    previous = load((BENCH/'j1-upper-z' if args.height30 else FOLDER)/'report.json') if args.upper_cycles else None
    if previous:
        assert previous['returned'] and len(previous['trials'])==(7 if args.height30 else 2)
    original = previous['final_axes'] if previous else load(BENCH/'j1-brake-pair-20261119-150018/report.json')['final_axes']
    interrupted = load(out/'interrupted-approach-report.json') if args.resume_approach else None
    if interrupted:
        restored=load(out/'encoder-recovery.json')
        assert interrupted['failed_stage']=='approach_up' and not interrupted['trials']
        assert interrupted['build']==firmware['build_id'] and restored['board_fault_detail']==8
        original = {**original,'1':restored['states']['1']}
    heights = load(LIVE/'heights-live.json')['z']
    deg_per_mm = (heights['travel_deg']-heights['pickup_deg'])/heights['operator_estimated_rise_mm']
    report = dict(build=firmware['build_id'],voltage_limit_mv=40000,requested_speed_deg_s=4800,
                  experiment_stroke_mm=envelope['study_stroke_mm'],deg_per_mm=deg_per_mm,
                  envelope=envelope,wall_clock_s=time.time(),monotonic_at_wall_clock_s=time.monotonic(),trials=[])
    stage = 'preflight'
    trace = []
    phases = []
    rotary = []
    feedback_tune = feedback_ramps = None
    d = RelativeZDrive() if args.relative_cycles else Drive()
    recv = d.can.recv
    active = False

    def record(wait):
        nonlocal feedback_tune,feedback_ramps
        f = recv(wait)
        if not f or f.echo or f.error or len(f.data)!=64:
            return f
        if f.identifier in (0x202,0x203) and f.data[0]==0x40:
            fields=PACKET.unpack(f.data)[5:]
            rotary.append([stage,f.identifier-0x200,fields[0],fields[1]/1000,fields[4]/1000])
        if f.identifier==0x301:
            words = struct.unpack('<16I',f.data)
            if words[0]==0x3147545a:
                counts = struct.unpack_from('<q',f.data,8)[0]
                iq_ref,iq,vq = struct.unpack_from('<3i',f.data,28)
                flags = words[13]
                error = 4*((((flags>>2)&16383)^8192)-8192)
                vd = (flags>>16 ^ 32768)-32768
                trace.append([stage,words[1],counts,words[10],bool(words[11]),words[12],error,
                             iq_ref,iq,vd,vq,bool(flags&2),words[14],words[15]])
            elif words[0]==0x31434850:
                phases.append([stage,words[1],words[4],words[5],bool(words[6])])
        if f.identifier==0x481 and f.data[2]==0:
            if f.data[0]==(10|0x80):
                feedback_tune=struct.unpack_from('<4i',f.data,8)
            elif f.data[0]==(13|0x80):
                feedback_ramps=struct.unpack_from('<3i',f.data,8)
        return f

    d.can.recv = record

    def configure(kp,ki,voltage,accel,brake,down_ki=0):
        if d.states[1]['power']:
            d.idle(1)
        d.command(1,10,kp,ki,c=voltage)
        assert feedback_tune==(kp,ki,2000,voltage),feedback_tune
        d.command(1,SET_PROFILE,brake*1000,accel*1000,c=down_ki)
        assert feedback_ramps==(brake*1000,accel*1000,down_ki),feedback_ramps
        PROFILE.update(acceleration_deg_s2=accel,brake_deg_s2=brake)

    def traverse(direction,goal,brake,down_ki,label=None):
        nonlocal stage
        stage=label or direction
        configure(1480,100000,40000,5050,brake,down_ki)
        initial=d.states[1]['position']
        t0=time.monotonic()
        print('Z',stage,'start',initial,'goal',goal,flush=True)
        d.move(1,goal,4800,tolerance=.12)
        t1=time.monotonic()
        d.wait(lambda:False,.055,elapsed_ok=True)
        rows=[r for r in trace if r[0]==stage and r[4] and not r[5]]
        phase_rows=[r for r in phases if r[0]==stage and r[4]]
        velocity=[abs(b[2]-a[2])*360000/(16384*(b[1]-a[1]))
                  for a,b in zip(rows,rows[5:]) if 90<=b[1]-a[1]<=140]
        assert velocity and phase_rows
        result=dict(name=stage,direction=direction,brake_deg_s2=brake,down_ki_permille=down_ki or 1000,
                    seconds=t1-t0,host_begin_s=t0,host_end_s=t1,activation_pose_deg=initial,
                    goal_deg=goal,settled_pose_deg=d.states[1]['position'],
                    peak_deg_s=max(velocity),applied_vector_peak_mv=round(max(math.hypot(r[9],r[10]) for r in rows)),
                    clipped_samples=sum(r[11] for r in rows),phase_a_peak_ma=max(r[2] for r in phase_rows)*3300/819,
                    phase_b_peak_ma=max(r[3] for r in phase_rows)*3300/819,
                    bus_peak_mv=max(max(r[3],r[13]) for r in rows),powered_samples=len(rows))
        report['trials'].append(result)
        print('LEG',json.dumps(result),flush=True)

    def relative_rounds():
        nonlocal stage,active
        saved=load(out/'interrupted-relative-report.json') if args.resume_relative else None
        recovery=load(out/'encoder-recovery.json') if saved else None
        #只检查静止计数和断开功率，不把另外两轴的读数写成棋盘坐标。
        d.wait(lambda:len(d.states)==3,2,check=False)
        baseline={n:a['counts'] for n,a in d.states.items()}
        d.wait(lambda:False,.45,check=False,elapsed_ok=True)
        for n,a in d.states.items():
            assert a['build']==BUILDS[n] and a['configured'] and a['encoder_valid']
            assert a['state']==0 and not a['power'] and not a['fault'] and 22000<=a['bus_mv']<=26000
            assert abs(a['counts']-baseline[n])<=3
            if n!=1:
                assert not a['referenced']
        home=PROFILE['saved_travel_deg']
        d.sync_sequence(1,check=False)
        if recovery:
            assert saved['relative_only'] and saved['home_deg']==home and saved['build']==firmware['build_id']
            assert recovery['board_fault_detail']==8 and recovery['states']['1']['build']==firmware['build_id']
            assert abs(d.states[1]['position']-recovery['position'])<.12
        elif d.states[1]['referenced']:
            last=load(out/'report.json')
            assert last['failed_stage']=='preflight' and not last['trials']
            assert abs(d.states[1]['target']-home)<.001
        else:
            d.command(1,ZERO,round(home*1000),check=False)
        d.wait(lambda:d.states[1]['referenced'] and d.states[1]['stationary'],2,check=False)
        d.healthy()
        if not recovery:
            assert abs(d.states[1]['position']-home)<.1
        report.update(relative_only=True,home_deg=home,upper_stroke_mm=43,
                      requested_cycles=args.relative_cycles,start={n:a.copy() for n,a in d.states.items()})
        active=True
        top=home+43*deg_per_mm
        completed=0
        if saved:
            completed=len(saved['trials'])//2
            assert 0<completed<args.relative_cycles
            report.update(start=saved['start'],reference_anchor=d.states[1].copy(),
                          wall_clock_s=saved['wall_clock_s'],monotonic_at_wall_clock_s=saved['monotonic_at_wall_clock_s'],
                          resumed=True)
            report['trials'].extend(saved['trials'][:2*completed])
            valid={leg['name'] for leg in report['trials']}
            earlier=load(out/'interrupted-relative-trace.json')
            trace.extend(row for row in earlier['samples'] if row[0] in valid)
            phases.extend(row for row in earlier['phases'] if row[0] in valid)
            rotary.extend(row for row in earlier['rotary'] if row[0] in valid)
            stage='relative_slow_resume'
            configure(1350,100000,6000,120,360)
            d.move(1,home,300,tolerance=.12)
        for cycle in range(completed+1,args.relative_cycles+1):
            traverse('up',top,4400,0,f'relative_up_{cycle}')
            stage='relative_hold'
            d.wait(lambda:False,.35,elapsed_ok=True)
            traverse('down',home,4800,950,f'relative_down_{cycle}')
            stage='relative_hold'
            d.wait(lambda:False,.35,elapsed_ok=True)
        stage='relative_return'
        configure(1350,100000,20000,120,360)
        difference=d.states[1]['position']-home
        report.update(returned=abs(difference)<=.5,return_error_deg=difference)
        assert report['returned']
        print('Relative cycles complete, returned to start',difference,flush=True)

    try:
        if args.relative_cycles:
            relative_rounds()
            return
        d.wait(lambda:len(d.states)==3,2,check=False)
        for n,a in d.states.items():
            assert a['build']==BUILDS[n] and a['configured'] and a['referenced']
            assert a['encoder_valid'] and a['stationary'] and not a['fault'] and 22000<=a['bus_mv']<=26000
            assert abs(a['position']-original[str(n)]['position'])<=.5
        assert d.states[1]['state']==0 and not d.states[1]['power'] and not d.states[1]['interlocked']
        for n in (2,3):
            assert d.states[n]['state']==1 and d.states[n]['power']
            if d.states[n]['interlocked']:
                d.sync_sequence(n,check=False)
                d.command(n,CLEAR,check=False)
                d.wait(lambda:not d.states[n]['interlocked'],1,check=False)
        start={n:a.copy() for n,a in d.read_initial().items()}
        report['start']=start
        active=True
        home=interrupted['start']['1']['position'] if interrupted else start[1]['position']
        report['home_deg']=home
        if interrupted:
            report['registered_resume_position_deg']=start[1]['position']
        raised=home+envelope['study_stroke_mm']*deg_per_mm
        if previous and not args.height30:
            raised=min(raised,previous['trials'][0]['goal_deg'])
        board_goal=raised-PROFILE['saved_travel_deg']
        assert 7210 < board_goal <= envelope['board_max_deg']-deg_per_mm
        approach='approach_resume' if interrupted else 'approach_up' if args.upper_cycles else None
        traverse('up',raised,4400,0,approach)
        stage='top_hold'
        d.wait(lambda:False,1.2,elapsed_ok=True)
        assert abs(d.states[1]['position']-raised)<=.12
        if args.upper_cycles:
            lower=raised-43*deg_per_mm
            report.update(upper_cycles=args.upper_cycles,upper_stroke_mm=43,
                          high_goal_deg=raised,upper_lower_goal_deg=lower)
            assert home<lower<raised
            for cycle in range(1,args.upper_cycles+1):
                traverse('down',lower,4800,950,f'upper_down_{cycle}')
                stage='upper_hold'
                d.wait(lambda:False,.55,elapsed_ok=True)
                traverse('up',raised,4400,0,f'upper_up_{cycle}')
                stage='upper_hold'
                d.wait(lambda:False,.55,elapsed_ok=True)
            stage='slow_return'
            configure(1350,100000,12000,450,600)
            print('Completed upper cycles, slow return',flush=True)
            d.move(1,home,900,tolerance=.12)
        else:
            traverse('down',home,4800,950)
        stage='restore_profile'
        configure(1350,100000,20000,120,360)
        d.wait(lambda:False,.35,elapsed_ok=True)
        errors={n:d.states[n]['position']-a['position'] for n,a in start.items()}
        errors[1]=d.states[1]['position']-home
        report.update(returned=abs(errors[1])<=.5 and all(abs(errors[n])<=.12 for n in (2,3)),
                      return_errors_deg=errors)
        assert report['returned']
        print('RETURNED',errors,flush=True)
    except BaseException as exc:
        report.update(failed_stage=stage,error=str(exc),failed_axes={n:a.copy() for n,a in d.states.items()})
        if active:
            report['stopped']=d.stop()
        raise
    finally:
        report['final_axes']=d.states
        (out/'report.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
        (out/'trace.json').write_text(json.dumps(dict(samples=trace,phases=phases,rotary=rotary)),encoding='utf-8')
        d.close()


if __name__=='__main__':
    main()
