"""Compare J1 rise voltage limits or speed on one taught stroke."""
import argparse
import json
import math
import struct
import time
from statistics import mean
from urllib.request import urlopen

from run_empty_step import HERE, LIVE, Drive, PROFILE, angles
from drive import BUILDS, CLEAR, SET_PROFILE, OFF, ZERO, MOVE, STOP

def read_json(path):
    return json.loads(path.read_text(encoding='utf8'))

STAIR_BLOCKS = (
    ('down_brake', (1700,2100,2500,2900,3300)),
    ('down_ki_permille', (850,725,600,475,350)),
    ('kp_milli', (1100,1230,1360,1490,1620)),
    ('accel', (2450,2800,3150,3500,3850)),
)
VOLTAGE_STAIR_BLOCKS = (
    ('accel', (4350,4700,5050,5400,5750)),
    ('down_brake', (2800,3300,3800,4300,4800)),
    ('down_ki_permille', (950,800,650,500,350)),
    ('kp_milli', (1180,1280,1380,1480,1580)),
)


def braking_current_rms(rows):
    speeds = [(b[2]-a[2])*360000/(16384*(b[1]-a[1])) for a,b in zip(rows,rows[1:])
              if 15<=b[1]-a[1]<=27]
    samples = [b for a,b in zip(rows,rows[1:]) if 15<=b[1]-a[1]<=27]
    smoothed = [abs(mean(speeds[i-2:i+3])) for i in range(2,len(speeds)-2)]
    peak = max(range(len(smoothed)),key=smoothed.__getitem__)
    changes = [(samples[i+2][8]-samples[i+1][8])**2 for i in range(peak,len(smoothed))
               if 250<=smoothed[i]<1100]
    if len(changes)<5:
        raise RuntimeError('Not enough braking telemetry in the 250–1100 deg/s band')
    return math.sqrt(mean(changes))


def main():
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--speed-test', action='store_true')
    mode.add_argument('--limit-test', action='store_true')
    mode.add_argument('--limit-refine', action='store_true')
    mode.add_argument('--follow20', action='store_true')
    mode.add_argument('--follow20-brake', action='store_true')
    mode.add_argument('--follow20-baseline', action='store_true')
    mode.add_argument('--speed1080', action='store_true')
    mode.add_argument('--down1440', action='store_true')
    mode.add_argument('--up2400', action='store_true')
    mode.add_argument('--up-kp-sweep', nargs='*', type=int)
    mode.add_argument('--brake-pair', type=int, choices=(2100,2400,4800))
    mode.add_argument('--peak-sweep', action='store_true')
    mode.add_argument('--kp-check', nargs='?', const=1350, type=int)
    parser.add_argument('--brake-rate', type=int, choices=(2000,3000,4000), default=3000)
    parser.add_argument('--pair-start', type=int, choices=(3000,4000), default=3000)
    parser.add_argument('--pair-kp', type=int, choices=(1000,1350,1550), default=1350)
    parser.add_argument('--pair-ki', type=int, choices=(100,150,200), default=200)
    parser.add_argument('--pair-repeats', type=int, choices=(1,2,3), default=1)
    parser.add_argument('--gain-series', action='store_true')
    parser.add_argument('--slope-comparison', action='store_true')
    parser.add_argument('--descent-brake-comparison', action='store_true')
    parser.add_argument('--descent-ki-comparison', action='store_true')
    parser.add_argument('--stop-comparison', action='store_true')
    parser.add_argument('--staircase', action='store_true')
    parser.add_argument('--high-accel-pilot', action='store_true')
    parser.add_argument('--voltage-probe', action='store_true')
    parser.add_argument('--voltage-staircase', action='store_true')
    parser.add_argument('--stair-history', action='append', default=[], metavar='PRIOR_RUN')
    parser.add_argument('--resume-at-b2', metavar='INTERRUPTED_RUN')
    parser.add_argument('--brake-image', choices=('j1-smooth-brake','j1-current-antiwindup',
                        'j1-tail-speed','j1-stop-ease','j1-descent-ki','j1-4800-pilot','j1-40v'),
                        default='j1-smooth-brake')
    parser.add_argument('--observe', action='store_true')
    args = parser.parse_args()
    if args.kp_check is not None and not 100<=args.kp_check<=20000:
        parser.error('--kp-check uses thousandths of Kp, from 100 to 20000')
    up_kp_sweep = args.up_kp_sweep is not None
    if up_kp_sweep and not all(100<=kp<=20000 for kp in args.up_kp_sweep):
        parser.error('--up-kp-sweep uses thousandths of Kp, from 100 to 20000')
    paired = args.speed1080
    down_only = args.down1440 or args.peak_sweep or args.kp_check
    high_rise = args.up2400 or up_kp_sweep
    brake_pair = args.brake_pair is not None
    voltage_run = args.voltage_probe or args.voltage_staircase
    staircase = args.staircase or args.voltage_staircase
    fixed_pose = args.high_accel_pilot or voltage_run
    if args.gain_series and not brake_pair:
        parser.error('gain series requires a paired Z trial')
    if args.slope_comparison and (not brake_pair or args.gain_series or args.brake_image!='j1-tail-speed'):
        parser.error('slope comparison requires a paired test with j1-tail-speed')
    if args.descent_brake_comparison and (args.brake_pair!=2100 or args.gain_series
                                        or args.brake_image!='j1-stop-ease'):
        parser.error('descent comparison requires 2100 deg/s with j1-stop-ease')
    if args.descent_ki_comparison and (args.brake_pair!=2400 or args.gain_series
                                     or args.brake_image!='j1-descent-ki'):
        parser.error('descent Ki comparison requires 2400 deg/s with j1-descent-ki')
    if args.stop_comparison and (args.brake_pair!=2100 or args.brake_image!='j1-descent-ki'
                                 or args.resume_at_b2 or args.slope_comparison or args.gain_series
                                 or args.descent_brake_comparison or args.descent_ki_comparison):
        parser.error('STOP comparison requires the 2100 deg/s paired setup and j1-descent-ki')
    if args.staircase and (args.brake_pair!=2400 or args.brake_image!='j1-descent-ki'
                          or args.gain_series or args.slope_comparison or args.stop_comparison
                          or args.descent_brake_comparison or args.descent_ki_comparison):
        parser.error('staircase requires 2400 deg/s and the j1-descent-ki application')
    if args.stair_history and not staircase:
        parser.error('stair history is only for resuming a staircase')
    if args.high_accel_pilot!=(args.brake_pair==4800 and args.brake_image=='j1-4800-pilot'):
        parser.error('4800 deg/s uses the dedicated staged comparison')
    if voltage_run!=(args.brake_pair==4800 and args.brake_image=='j1-40v') or (
            args.voltage_probe and args.voltage_staircase):
        parser.error('choose one 40 V mode with the j1-40v build and 4800 deg/s command')
    positioned = read_json(HERE/args.resume_at_b2/'report.json') if args.resume_at_b2 else None
    encoder_recovery = None
    if positioned:
        assert args.slope_comparison or args.descent_ki_comparison or staircase
        if staircase:
            assert args.resume_at_b2 in args.stair_history and positioned.get('staircase')
            recovery_path = HERE/args.resume_at_b2/'encoder-recovery.json'
            if recovery_path.is_file():
                encoder_recovery = read_json(recovery_path)
            else:
                assert positioned.get('stopped') and positioned['final_axes']['1']['referenced']
        elif positioned['failed_stage']=='up_2400_E1' and args.descent_ki_comparison:
            assert {run['name'] for run in positioned['trials'] if run['completed']}=={
                'up_2400_D1','down_2400_D1','up_2400_D2','down_2400_D2'}
            encoder_recovery = read_json(HERE/args.resume_at_b2/'encoder-recovery.json')
        else:
            assert positioned.get('stopped') and not positioned['trials']
            assert positioned['failed_stage'] in ('to_empty_b2','position_at_buffered_bottom')
    if brake_pair and max(args.pair_start,args.brake_rate)>3000 and args.brake_image!='j1-tail-speed':
        parser.error('4000 deg/s2 requires the new J1 profile')
    phase_test = paired or down_only or high_rise or brake_pair
    test_speed = args.brake_pair if brake_pair else 2400 if high_rise else 1440 if down_only else 1080
    descent_voltage = 20000 if brake_pair else 15000 if down_only else 6000
    up_voltage = 20000 if high_rise or brake_pair else 8000
    follow20 = args.follow20 or args.follow20_brake or args.follow20_baseline
    speed_mode = args.speed_test or args.limit_test or args.limit_refine or follow20
    image = HERE/('j1-follow20' if follow20 else 'rise-voltage-15v' if speed_mode else 'rise-voltage-10v')
    if brake_pair:
        image = HERE/args.brake_image
    elif high_rise:
        image = HERE/'j1-2400-voltage20'
    elif down_only:
        image = HERE/'j1-motion-15v'
    elif paired:
        image = HERE/'j1-2a-1080'
    results = HERE/'j1-rise-limit' if args.limit_test else HERE/'j1-rise-limit-refine' if args.limit_refine else image
    if args.follow20_brake:
        results = HERE/'j1-follow20-brake'
    if args.follow20_baseline:
        results = HERE/'j1-follow20-baseline'
    if args.peak_sweep:
        results = HERE/f'j1-down-peak-{time.strftime("%Y%m%d-%H%M%S")}'
    if args.kp_check:
        results = HERE/f'j1-down-kp{args.kp_check}-{time.strftime("%Y%m%d-%H%M%S")}'
    if up_kp_sweep:
        results = HERE/f'j1-up-kp-{time.strftime("%Y%m%d-%H%M%S")}'
    if brake_pair:
        results = HERE/f'j1-brake-pair-{time.strftime("%Y%m%d-%H%M%S")}'
    results.mkdir(exist_ok=True)
    previous_up = read_json(HERE/'j1-2400-ramp3000/report.json') if args.up2400 else None
    if previous_up:
        assert previous_up['stopped'] and previous_up['failed_stage']=='up_2400'
    down_profiles = ((460,840),(460,840),(500,840),(540,1080),(540,1080)) if args.peak_sweep else ((460,840),)
    if args.kp_check:
        down_profiles = ((460,840),(460,840))
    interrupted = None
    if phase_test:
        trials = []
    elif args.follow20_baseline:
        trials = [(8000,1200,460,720)]
    elif args.follow20_brake:
        trials = [(8000,1200,460,840)]
    elif args.follow20:
        trials = [(8000,1200,accel,720) for accel in (480,520,540)]
        trials += [(8000,1200,540,brake) for brake in (840,960,1080)]
    elif args.limit_refine:
        interrupted = read_json(HERE/'j1-rise-limit/report.json')
        assert interrupted['stopped'] and interrupted['failed_stage']=='up_accel_480'
        trials = [(8000,1200,460,720)]
    elif args.limit_test:
        trials = [(8000,1200,accel,720) for accel in (400,440,480,520,540)]
    elif args.speed_test:
        trials = [(15000,speed,400,720) for speed in (700,850,950,1000)]
    else:
        trials = [(limit,950,400,720) for limit in (8000,9000,10000)]
    with urlopen('http://127.0.0.1:8121/state', timeout=2) as response:
        assert not json.load(response)['active']
    new_build = read_json(image/'J1-manifest.json')['build_id']
    assert PROFILE['build_id'] == BUILDS[1] == new_build
    arm = read_json(LIVE/'arm-live.json')
    layout = read_json(LIVE/'layout-live.json')['points']['data']
    heights = read_json(LIVE/'heights-live.json')['z']
    previous = read_json(HERE/'empty-step-overlap-20261118-190627.json')
    high, pick = heights['travel_deg'], heights['pickup_deg']
    trial_high, trial_low = high,pick
    b2 = angles(arm,layout,1,2)
    PROFILE.update(acceleration_deg_s2=180,brake_deg_s2=360)
    kp_milli = PROFILE.get('speed_kp_milli',1600)
    ki_milli = PROFILE.get('speed_ki_milli',200000)
    if args.kp_check:
        assert ki_milli==200000
        kp_milli = args.kp_check
    rise_gains = tuple(args.up_kp_sweep or (1350,1250,1350,1250)) if up_kp_sweep else (1350,)
    if high_rise:
        kp_milli,ki_milli = rise_gains[0],200000
    if brake_pair:
        kp_milli,ki_milli = args.pair_kp,args.pair_ki*1000
        pair_accel, pair_brake = args.pair_start,args.brake_rate
    report = {'build':new_build,'stroke_deg':high-pick,
              'rise':{'following_limit_deg':20 if follow20 else 2.5},
              'descent':{'speed_deg_s':380,'start_deg_s2':180,'brake_deg_s2':360,'limit_mv':6000},
              'planned_trials':[{'limit_mv':v,'speed_deg_s':s,'start_deg_s2':a,'brake_deg_s2':b}
                                for v,s,a,b in trials],
              'trials':[]}
    report.update(speed_kp_milli=kp_milli,speed_ki_milli=ki_milli)
    if voltage_run:
        report.update(wall_clock_s=time.time(),monotonic_at_wall_clock_s=time.monotonic(),
                      offboard=True)
    if phase_test:
        report.update(iq_limit_ma=2000, acceleration_deg_s2=down_profiles[0][0],
                      brake_deg_s2=down_profiles[0][1],
                      requested_speed_deg_s=test_speed, down_voltage_mv=descent_voltage,
                      planned_trials=[f'down_{test_speed}'])
        if paired:
            report.update(up_voltage_mv=8000)
            report['planned_trials'].append('up_1080')
        if high_rise:
            report.update(acceleration_deg_s2=3000,brake_deg_s2=3000,up_voltage_mv=up_voltage,
                          setup_speed_deg_s=120,planned_trials=['up_2400'])
        if up_kp_sweep:
            report['planned_trials'] = [
                {'trial':i,'speed_kp_milli':kp} for i,kp in enumerate(rise_gains,1)
            ]
        if down_only or high_rise:
            report.update(bus_limit_mv=45000, command_speed_limit_deg_s=12000,
                          motor_speed_limit_deg_s=2400 if high_rise else 1440)
        if brake_pair:
            report.update(acceleration_deg_s2=pair_accel, brake_deg_s2=pair_brake,
                          up_voltage_mv=up_voltage, bus_limit_mv=45000,
                          acceleration_transition_s=.060,
                          terminal_brake_transition_s=.060 if args.brake_image in
                          ('j1-smooth-brake','j1-current-antiwindup') else .150,
                          planned_trials=['up_2400', 'down_2400'])
        if args.peak_sweep or args.kp_check:
            assert kp_milli==(args.kp_check or 1450) and ki_milli==200000
            report['planned_trials'] = [
                {'trial':i,'acceleration_deg_s2':a,'brake_deg_s2':b}
                for i,(a,b) in enumerate(down_profiles,1)
            ]
        report.pop('rise')
        report.pop('descent')
    stage = 'preflight'
    trace = []
    phase_trace = []
    gain_ack, ramp_ack = (), ()
    brake_ki_scale = 0
    active = False
    d = Drive()
    recv = d.can.recv

    def observe_frame(timeout):
        nonlocal gain_ack, ramp_ack
        f = recv(timeout)
        if f and not f.echo and not f.error and len(f.data)==64:
            words = struct.unpack('<16I',f.data)
            if f.identifier==0x301 and words[0]==0x3147545a:
                flags = words[13]
                following = (((flags >> 2) & 16383) ^ 8192) - 8192
                if follow20 or phase_test:
                    following *= 4
                vd = ((flags >> 16) ^ 32768) - 32768
                count = struct.unpack_from('<q',f.data,8)[0]
                iq_ref, iq, vq = struct.unpack_from('<3i',f.data,28)
                trace.append([stage,words[1],count,words[10],bool(words[11]),words[12],
                              following,iq_ref,iq,vd,vq,bool(flags&2),words[14],words[15]])
            if f.identifier==0x301 and words[0]==0x31434850:
                phase_a, phase_b = struct.unpack_from('<2i',f.data,8)
                phase_trace.append([stage,words[1],phase_a,phase_b,words[4],words[5],bool(words[6])])
            if f.identifier==0x481 and f.data[2]==0:
                if f.data[0]==(10|0x80):
                    gain_ack = struct.unpack_from('<4i',f.data,8)
                if f.data[0]==(13|0x80):
                    ramp_ack = struct.unpack_from('<3i',f.data,8)
        return f

    d.can.recv = observe_frame

    def configure(cap, accel, brake):
        if d.states[1]['power']:
            d.idle(1)
        d.command(1,10,kp_milli,ki_milli,c=cap)
        assert gain_ack==(kp_milli,ki_milli,2000 if phase_test else 1400,cap), gain_ack
        d.command(1,SET_PROFILE,brake*1000,accel*1000,c=brake_ki_scale)
        assert ramp_ack==(brake*1000,accel*1000,brake_ki_scale), ramp_ack
        PROFILE.update(acceleration_deg_s2=accel,brake_deg_s2=brake)

    def move_with_early_stop(goal, speed, first, info):
        deg_per_mm = (high-pick)/heights['operator_estimated_rise_mm']
        fallback = goal - 4*deg_per_mm
        trigger = goal + 16*deg_per_mm
        first_state = d.states[1].copy()
        assert fallback>pick+6*deg_per_mm and first_state['position']>trigger
        assert first_state['state']==0 and not first_state['power'] and first_state['stationary']
        print(f'J1 STOP test {first_state["position"]:.3f} -> {goal:.3f}, trigger {trigger:.3f}',flush=True)
        d.command(1,MOVE,round(fallback*1000),round(speed*1000))

        def position_at_trigger():
            rows = [r for r in trace[first:] if r[4] and not r[5]]
            if len(rows)<2:
                return False
            a,b = rows[-2:]
            dt = b[1]-a[1]
            if not 10<=dt<=60:
                return False
            position = first_state['position']+(b[2]-first_state['counts'])*360/16384
            #提前半帧采样距离，在20ms采样网格上靠近16mm触发点。
            velocity = (b[2]-a[2])*360000/(16384*dt)
            if position>trigger+min(.5*deg_per_mm,max(0,-velocity*.010)):
                return False
            info.update(stop_trigger_board_ms=b[1],stop_trigger_pos_deg=position,
                        stop_trigger_lead_mm=(position-goal)/deg_per_mm,
                        stop_trigger_speed_deg_s=velocity,fallback_deg=fallback)
            return True

        d.wait(position_at_trigger,1.4)
        info['stop_command_host_s'] = time.monotonic()
        before = d.states[1]['uptime']
        d.command(1,STOP)
        d.wait(lambda:d.states[1]['uptime']!=before and d.states[1]['state']==1
               and d.states[1]['power'] and d.states[1]['stationary'],4)

    def traverse(label, goal, cap, accel, brake, speed, early_stop=False):
        nonlocal stage
        stage = label
        configure(cap,accel,brake)
        if args.observe and high_rise and label.startswith('up_'):
            print('OBSERVE_UP_IN_10_SECONDS', flush=True)
            d.wait(lambda:False,10,elapsed_ok=True)
        begin = time.monotonic()
        first = len(trace)
        initial = d.states[1]['position']
        trip = None
        host_timeout = False
        stop_info = {}
        try:
            if early_stop:
                move_with_early_stop(goal,speed,first,stop_info)
            else:
                d.z(goal,high=high,speed_limit=speed)
        except TimeoutError:
            if not (high_rise or brake_pair):
                raise
            trip = d.states[1].copy()
            host_timeout = True
        except RuntimeError:
            a = d.states[1].copy()
            known_fault = (a['fault'],a['detail'])
            recoverable = known_fault in ((3,0),(5,4)) or (high_rise or brake_pair) and known_fault==(5,3)
            if not phase_test or not recoverable:
                raise
            trip = a
        elapsed = time.monotonic()-begin
        if trip:
            assert d.stop()
            d.wait(lambda:False,.35,check=False,elapsed_ok=True)
        else:
            d.wait(lambda:False,.055,elapsed_ok=True)
        rows = [r for r in trace[first:] if r[4]]
        assert rows and (trip or len(rows)>10 and all(r[5]==0 for r in rows))
        speed_samples = [abs(b[2]-a[2])*360000/(16384*(b[1]-a[1]))
                         for a,b in zip(rows,rows[5:]) if 90<=b[1]-a[1]<=140]
        result = {'name':label,'limit_mv':cap,'seconds':elapsed,'speed_kp_milli':kp_milli,
                  'host_begin_s':begin,'host_end_s':begin+elapsed,
                  'stroke_deg':abs(goal-initial),'speed_peak_deg_s':max(speed_samples,default=None),
                  'clipped_samples':sum(r[11] for r in rows),'powered_samples':len(rows),
                  'max_following_mdeg':max(abs(r[6]) for r in rows),
                  'voltage_vector_peak_mv':round(max(math.hypot(r[9],r[10]) for r in rows)),
                  'iq_peak_ma':max(abs(r[8]) for r in rows),
                  'bus_peak_mv':max(max(r[3],r[13]) for r in rows)}
        result.update(stop_info)
        if (staircase or fixed_pose) and trip is None:
            result['braking_iq_step_rms_ma'] = braking_current_rms(rows)
        if phase_test:
            phases = [r for r in phase_trace if r[0]==label and r[1]>=rows[0][1]]
            stroke_frames = [r for r in trace[first:] if r[1]>=rows[0][1]]
            assert phases
            result.update(phase_a_peak_ma=max(r[4] for r in phases)*3300/819,
                          phase_b_peak_ma=max(r[5] for r in phases)*3300/819,
                          bus_trip_mv=max(r[12] for r in stroke_frames),
                          bus_peak_mv=max(max(r[3],r[13]) for r in stroke_frames),
                          trip=trip,completed=trip is None,
                          stopped_position_deg=d.states[1]['position'])
            if host_timeout:
                result.update(host_timeout=True,post_stop=d.states[1].copy())
        print(label, json.dumps(result),flush=True)
        return result

    def recover_z(goal=None):
        d.wait(lambda:len(d.states)==3 and all(
            time.monotonic()-d.last_update[n]<.2 and s['referenced'] and s['encoder_valid']
            and s['stationary'] and 22000<=s['bus_mv']<=26000 for n,s in d.states.items()),
            1.5,check=False)
        a = d.states[1].copy()
        assert a['state']==4 and not a['power'] and a['stationary'] and pick<a['position']<=trial_high+4.5
        known_fault = (a['fault'],a['detail'])
        assert known_fault in ((5,4),(3,0)) or (high_rise or brake_pair) and known_fault==(5,3)
        assert all(not d.states[n]['fault'] and d.states[n]['power']
                   and abs(d.states[n]['position']-b2[n-2])<=.15 for n in (2,3))
        d.sync_sequence(1,check=False)
        d.command(1,CLEAR,check=False)
        d.wait(lambda:not d.states[1]['fault'] and not d.states[1]['power'] and d.states[1]['referenced']
               and abs(d.states[1]['position']-a['position'])<=.15,.9,check=False)
        d.read_initial()
        configure(6000,60,360)
        d.z(high if goal is None else goal,high=high,speed_limit=120)

    def return_park():
        nonlocal stage
        stage = 'return_to_park'
        assert abs(d.states[1]['position']-high)<=4.5
        configure(6000,120,360)
        if not fixed_pose:
            d.xy_position(start[2]['position'],start[3]['position'],12,30)
        d.move(1,start[1]['position'],60,tolerance=.12)
        if d.states[1]['power']:
            d.idle(1)
        if down_only or high_rise or brake_pair:
            configure(up_voltage if high_rise or brake_pair else 15000,120,360)
            report['restored_voltage_mv'] = gain_ack[3]
        d.wait(lambda:False,.35,elapsed_ok=True)
        report['return_errors_deg'] = {n:round(a['position']-start[n]['position'],3) for n,a in d.states.items()}
        report['returned'] = all(abs(e)<=(.5 if n==1 else .101)
                                 for n,e in report['return_errors_deg'].items())
        print('RETURNED',report['returned'],report['return_errors_deg'],flush=True)

    def run_staircase():
        nonlocal kp_milli,ki_milli,brake_ki_scale
        prior_rounds = {}
        for folder in args.stair_history:
            saved = read_json(HERE/folder/'report.json')
            assert saved['build']==new_build and saved.get('staircase')
            for run in saved['trials']:
                if run.get('completed') and 'stair_round' in run:
                    prior_rounds[run['stair_round'],run['direction']] = run
        report['staircase'] = True
        report['voltage_staircase'] = args.voltage_staircase
        report['stair_history'] = args.stair_history
        report['block_choices'] = []
        config = dict(kp_milli=1350,ki_milli=100000,accel=3300,up_brake=3100,
                      down_brake=3000,down_ki_permille=1000)
        blocks = VOLTAGE_STAIR_BLOCKS if args.voltage_staircase else STAIR_BLOCKS
        active_cap = 40000 if args.voltage_staircase else 20000
        if args.voltage_staircase:
            config.update(accel=4200,up_brake=4400,down_brake=4400)
        for block,(field,levels) in enumerate(blocks):
            entries = []
            for step,level in enumerate(levels,1):
                number = block*5+step
                setting = dict(config,**{field:level})
                entry = {}
                both_prior = all((number,direction) in prior_rounds for direction in ('up','down'))
                if not both_prior:
                    print('STAIR',number,field,level,json.dumps(setting),flush=True)
                for direction,goal in (('up',trial_high),('down',trial_low)):
                    if both_prior:
                        run = prior_rounds[number,direction]
                        assert run['setting']==setting
                    else:
                        kp_milli,ki_milli = setting['kp_milli'],setting['ki_milli']
                        brake_ki_scale = setting['down_ki_permille'] if direction=='down' else 0
                        brake = setting['up_brake'] if direction=='up' else setting['down_brake']
                        label = f'{direction}_{test_speed}_R{number:02d}'
                        run = traverse(label,goal,active_cap,setting['accel'],brake,test_speed)
                        run.update(stair_round=number,stair_field=field,stair_level=level,
                                   direction=direction,setting=setting,
                                   descent_brake_ki_permille=brake_ki_scale or 1000,
                                   acceleration_deg_s2=setting['accel'],brake_deg_s2=brake)
                        report['trials'].append(run)
                        if run['trip']:
                            raise RuntimeError(f'Staircase interrupted in {label}: {run["trip"]}')
                        d.wait(lambda:False,.30,elapsed_ok=True)
                    entry[direction] = run
                entries.append((number,level,entry))
            quickest = min(e['up']['seconds']+e['down']['seconds'] for _,_,e in entries)
            eligible = [(n,v,e) for n,v,e in entries if e['up']['seconds']+e['down']['seconds']<=quickest+.8]
            chosen = min(eligible,key=lambda a:a[2]['down']['braking_iq_step_rms_ma']
                                             +.5*a[2]['up']['braking_iq_step_rms_ma'])
            report['block_choices'].append(dict(field=field, value=chosen[1], stair_round=chosen[0]))
            config[field] = chosen[1]
            print('STAIR_CHOICE',field,chosen[1],chosen[0],flush=True)
        report['selected_stair_setting'] = config
        kp_milli,ki_milli = 1350,100000
        brake_ki_scale = 0

    def run_high_accel_pilot(voltage_comparison=False):
        nonlocal kp_milli,ki_milli,brake_ki_scale
        kp_milli,ki_milli = 1350,100000
        report['high_accel_pilot'] = True
        probes = ((4200,20000),(4200,40000)) if voltage_comparison else (
            (4200,20000),(4800,20000),(5400,20000),(6000,20000))
        report['voltage_probe'] = voltage_comparison
        for step,(rate,cap) in enumerate(probes,1):
            print('ACCEL PILOT',step,rate,cap,flush=True)
            for direction,goal in (('up',trial_high),('down',trial_low)):
                brake_ki_scale = 0
                run = traverse(f'{direction}_4800_P{step}',goal,cap,rate,rate,4800)
                run.update(direction=direction,acceleration_deg_s2=rate,brake_deg_s2=rate,
                           speed_ki_milli=ki_milli,descent_brake_ki_permille=1000)
                report['trials'].append(run)
                if run['trip']:
                    raise RuntimeError(f'4800 pilot interrupted on {direction} at {rate}: {run["trip"]}')
                d.wait(lambda:False,.35,elapsed_ok=True)

    try:
        d.wait(lambda:len(d.states)==3,1.2,check=False)
        d.wait(lambda:False,.35,check=False,elapsed_ok=True)
        assert all(s['build']==BUILDS[n] and s['configured'] and s['encoder_valid']
                   and s['referenced'] and s['stationary']
                   and (not s['fault'] or interrupted and n==1 and (s['fault'],s['detail'])==(5,4))
                   for n,s in d.states.items())
        assert 22000<=d.states[1]['bus_mv']<=26000
        if positioned:
            resume_z = encoder_recovery['position'] if encoder_recovery else positioned.get('test_low_deg',high)
            if staircase and not encoder_recovery:
                resume_z = positioned['final_axes']['1']['position']
            assert resume_z>pick and positioned['build']==new_build and abs(d.states[1]['position']-resume_z)<.75
            resume_xy = tuple(positioned['start'][str(n)]['position'] for n in (2,3)) if fixed_pose else b2
            assert all(abs(d.states[n]['position']-goal)<.12 for n,goal in zip((2,3),resume_xy))
            d.sync_sequence(1,check=False)
            if d.states[1]['state']==1 and d.states[1]['power']:
                if d.states[1]['interlocked']:
                    d.command(1,CLEAR,check=False)
                    d.wait(lambda:not d.states[1]['interlocked'],1,check=False)
                d.command(1,OFF,check=False)
                d.wait(lambda:d.states[1]['state']==0 and not d.states[1]['power'] and d.states[1]['stationary'],2,check=False)
            if d.states[1]['state']==0 and d.states[1]['interlocked']:
                d.wait(lambda:False,.35,check=False,elapsed_ok=True)
                z=d.states[1]
                assert z['referenced'] and z['stationary'] and not z['power'] and not z['fault']
                assert abs(z['position']-resume_z)<.75
                current_z=z['position']
                d.command(1,ZERO,round(current_z*1000),check=False)
                d.wait(lambda:not d.states[1]['interlocked'] and d.states[1]['referenced'] and abs(d.states[1]['position']-current_z)<.10,1,check=False)
        assert not d.states[1]['power']
        if interrupted:
            assert all(abs(s['position']-interrupted['final_axes'][str(n)]['position'])<=.25
                       for n,s in d.states.items())
        elif phase_test:
            readback = 'speed-kp-1450-readback.json' if args.peak_sweep else 'ota-readback.json'
            saved_z = high+read_json(image/readback)['after']['position']
            if args.kp_check:
                candidates = [*HERE.glob('j1-down-peak-*/report.json'),*HERE.glob('j1-down-kp*/report.json')]
                prior = read_json(max(candidates,key=lambda path:path.stat().st_mtime))
                assert prior['returned'] and prior['build']==new_build
                saved_z = prior['final_axes']['1']['position']
            if up_kp_sweep:
                candidates = [image/'report.json',*HERE.glob('j1-up-kp-*/report.json')]
                prior = read_json(max(candidates,key=lambda q:q.stat().st_mtime))
                assert prior['returned'] and prior['build']==new_build
                saved_z = prior['final_axes']['1']['position']
            if brake_pair:
                for path in sorted(HERE.glob('j1-brake-pair-*/report.json'), reverse=True):
                    prior = read_json(path)
                    if prior.get('returned') and 'test_high_deg' in prior:
                        trial_high = prior['test_high_deg']
                        if prior['build']==new_build:
                            saved_z = prior['final_axes']['1']['position']
                        break
            min_z = pick if previous_up or positioned else high-4.5
            assert d.states[1]['state']==0 and d.states[1]['position']>=min_z
            if not positioned:
                assert abs(d.states[1]['position']-saved_z)<=.5
        else:
            assert d.states[1]['state']==0 and abs(d.states[1]['position']-high)<=4.5
        initial_xy = b2 if not fixed_pose and (interrupted or previous_up or positioned) else previous['goals']['park']
        for n,goal in zip((2,3),initial_xy):
            a = d.states[n]
            parked = a['state']==1 and a['power']
            cold_start = brake_pair and a['state']==0 and not a['power']
            assert (parked or cold_start) and abs(a['position']-goal)<=.25
            if a['interlocked']:
                d.sync_sequence(n,check=False)
                d.command(n,CLEAR,check=False)
                d.wait(lambda:not d.states[n]['interlocked'],.8,check=False)
        if interrupted:
            start = {int(n):a for n,a in interrupted['start'].items()}
            report['start'] = start
            active = True
            stage = 'recover_before_460'
            recover_z()
        elif previous_up or positioned:
            d.read_initial()
            if staircase:
                report['reference_anchor'] = d.states[1].copy()
            last_start = (previous_up or positioned)['start']
            start = {int(n):a for n,a in last_start.items()}
            report['start'] = start
            active = True
        else:
            start = {n:a.copy() for n,a in d.read_initial().items()}
            report['start'] = start
            active = True
            if brake_pair and start[3]['state']==0:
                d.command(3,10,1000,35000)
            if phase_test and abs(start[1]['position']-high)>4.5:
                stage = 'align_to_taught_high'
                configure(6000,60,360)
                d.z(high,high=high,speed_limit=120)
            if not fixed_pose:
                stage = 'to_empty_b2'
                d.xy_position(*b2,18,30)
        if brake_pair:
            #取已经到过的高点，缩短试跑段以给棋盘留出净空。
            deg_per_mm = (high-pick)/heights['operator_estimated_rise_mm']
            if trial_high==high:
                trial_high = start[1]['position']
            trial_low = trial_high-36*deg_per_mm
            assert trial_high>=high and trial_low>pick
            report.update(stroke_deg=trial_high-trial_low, test_low_deg=trial_low,
                          test_high_deg=trial_high, buffer_above_pickup_mm=(trial_low-pick)/deg_per_mm)
            stage = 'position_at_buffered_bottom'
            if (not encoder_recovery or staircase) and not args.stop_comparison:
                configure(6000,60,360)
                d.z(trial_low,high=high,speed_limit=120)
            base_gains = (kp_milli,ki_milli)
            if args.voltage_staircase and not args.stair_history:
                run_high_accel_pilot(voltage_comparison=True)
            if staircase:
                run_staircase()
            if args.high_accel_pilot or args.voltage_probe:
                run_high_accel_pilot(voltage_comparison=args.voltage_probe)
            groups = [] if staircase or fixed_pose else [('A',*base_gains)]
            if args.stop_comparison:
                groups = [('MOVE',1350,100000),('STOP',1350,100000)]
            if args.gain_series:
                groups = [('A',1350,200000),('B',1350,150000),('C',1000,150000)]
            if args.slope_comparison:
                groups = [('D',1350,100000),('E',1350,100000)]
                #先录几秒静止画面作为光学对照。
                d.wait(lambda:False,4,elapsed_ok=True)
            if args.descent_brake_comparison or args.descent_ki_comparison:
                groups = [('D',1350,100000),('E',1350,100000)]
                if encoder_recovery:
                    groups = groups[1:]
                if not positioned:
                    d.wait(lambda:False,4,elapsed_ok=True)
            repeats = 2 if args.gain_series else args.pair_repeats
            legs = [(group,kp,ki,repeat,direction,goal) for group,kp,ki in groups
                    for repeat in range(1,repeats+1)
                    for direction,goal in (('up',trial_high),('down',trial_low))]
            report['planned_trials'] = [dict(group=g,kp_milli=kp,ki_milli=ki,repeat=i,direction=s)
                                        for g,kp,ki,i,s,_ in legs]
            for group,kp_milli,ki_milli,repeat,direction,goal in legs:
                if args.slope_comparison:
                    pair_accel = pair_brake = 3000 if group=='D' else 4000
                if args.descent_brake_comparison:
                    pair_accel = 4000 if direction=='down' else 3000
                    pair_brake = 2200 if direction=='down' and group=='E' else 3000
                if args.descent_ki_comparison:
                    brake_ki_scale = 500 if direction=='down' and group=='E' else 0
                if direction=='up' and repeat==1:
                    print('GROUP',group,kp_milli,ki_milli,flush=True)
                    if group!='A' and not args.stop_comparison:
                        d.wait(lambda:False,5,elapsed_ok=True)
                label = f'{direction}_{test_speed}'
                if len(groups)>1 or repeats>1:
                    label += f'_{group}{repeat}'
                if (encoder_recovery or args.stop_comparison) and direction=='up':
                    stage = 'slow_return_to_trial_top'
                    configure(6000,120,360)
                    d.z(trial_high,high=high,speed_limit=300)
                    continue
                run = traverse(label,goal,20000,pair_accel,pair_brake,test_speed,
                               early_stop=args.stop_comparison and group=='STOP')
                run.update(group=group, repeat=repeat, speed_ki_milli=ki_milli,
                           descent_brake_ki_permille=brake_ki_scale or 1000,
                           direction=direction,acceleration_deg_s2=pair_accel,brake_deg_s2=pair_brake)
                if args.stop_comparison:
                    run.update(stop_strategy=group,stop_error_mm=(run['stopped_position_deg']-goal)/deg_per_mm)
                report['trials'].append(run)
                if args.stop_comparison and not run['trip']:
                    assert d.states[1]['position']>pick+6*deg_per_mm
                    if group=='STOP' and not 14<=run['stop_trigger_lead_mm']<=18:
                        report['comparison_stopped']='STOP trigger outside the 16 mm measurement window'
                        break
                if run['trip'] or (results/'stop-after-trial').is_file():
                    stage = 'recover_after_brake_test_stop'
                    if d.states[1]['fault']:
                        recover_z()
                    else:
                        assert d.states[1]['stationary'] and pick<d.states[1]['position']<=trial_high+4.5
                    break
                if args.slope_comparison or args.descent_brake_comparison or args.descent_ki_comparison:
                    d.wait(lambda:False,2,elapsed_ok=True)
            brake_ki_scale = 0
            kp_milli,ki_milli = base_gains
            if abs(d.states[1]['position']-high)>4.5:
                stage = 'restore_travel_height'
                configure(6000,60,360)
                d.z(high,high=high,speed_limit=120)
        elif high_rise:
            for i,kp_milli in enumerate(rise_gains,1):
                stage = 'position_for_up_2400'
                configure(6000,60,360)
                d.z(pick,high=high,speed_limit=120)
                assert abs(d.states[1]['position']-pick)<=4.5
                label = f'up_2400_{i}' if up_kp_sweep else 'up_2400'
                up = traverse(label,high,up_voltage,3000,3000,2400)
                up.update(acceleration_deg_s2=3000,brake_deg_s2=3000)
                report['trials'].append(up)
                if up['trip']:
                    stage = 'recover_after_up_trip'
                    if d.states[1]['fault']:
                        recover_z()
                    elif abs(d.states[1]['position']-high)>4.5:
                        assert d.states[1]['stationary'] and pick<d.states[1]['position']<high
                        configure(6000,60,360)
                        d.z(high,high=high,speed_limit=120)
                    break
                if (results/'stop-after-trial').is_file():
                    break
        elif phase_test:
            for i,(accel,brake) in enumerate(down_profiles,1):
                assert abs(d.states[1]['position']-high)<=4.5
                label = f'down_{test_speed}_{i}' if args.peak_sweep or args.kp_check else f'down_{test_speed}'
                down = traverse(label,pick,descent_voltage,accel,brake,test_speed)
                down.update(acceleration_deg_s2=accel,brake_deg_s2=brake)
                report['trials'].append(down)
                if paired:
                    if down['trip']:
                        stage = 'position_after_down_trip'
                        recover_z(pick)
                    assert abs(d.states[1]['position']-pick)<=4.5
                    up = traverse('up_1080',high,8000,460,840,1080)
                    report['trials'].append(up)
                    if up['trip']:
                        stage = 'recover_after_up_trip'
                        recover_z()
                elif down['trip']:
                    stage = 'recover_after_down_trip'
                    recover_z()
                    break
                else:
                    stage = 'raise_to_travel_height'
                    configure(6000,60,360)
                    d.z(high,high=high,speed_limit=120)
        for cap,speed,accel,brake in trials:
            entry = {'limit_mv':cap,'speed_deg_s':speed,'start_deg_s2':accel,'brake_deg_s2':brake}
            report['trials'].append(entry)
            label = f'accel_{accel}' if args.limit_test or args.limit_refine else f'speed_{speed}' if args.speed_test else str(cap)
            if follow20:
                label = f'accel_{accel}_brake_{brake}'
            entry['down'] = traverse(f'down_before_{label}',pick,6000,180,360,380)
            if entry['down']['bus_peak_mv']>=28500:
                entry['descent_above_previous_screening_mv'] = 28500
            entry['up'] = traverse(f'up_{label}',high,cap,accel,brake,speed)
            if speed_mode and entry['up']['bus_peak_mv']>=28500:
                report['speed_screening'] = 'rise reached the 28.5 V screening margin'
                break
        return_park()
    except BaseException as exc:
        report.update(error=str(exc),failed_stage=stage,failed_axes={n:a.copy() for n,a in d.states.items()})
        if active:
            report['stopped'] = d.stop()
        recoverable_stage = args.limit_refine and stage=='up_accel_460' or follow20 and stage.startswith('up_')
        if recoverable_stage and report.get('stopped'):
            try:
                d.wait(lambda:False,.5,check=False,elapsed_ok=True)
                stage = 'recover_after_rise'
                recover_z()
                return_park()
            except Exception as recovery_error:
                report['recovery_error'] = str(recovery_error)
                d.stop()
        raise
    finally:
        report['final_axes'] = d.states
        (results/'report.json').write_text(json.dumps(report,indent=2),encoding='utf8')
        (results/'trace.json').write_text(json.dumps({
            'columns':['stage','uptime_ms','encoder_counts','bus_mv','power','fault','following_mdeg',
                       'iq_ref_ma','iq_ma','vd_mv','vq_mv','clipped','bus_trip_mv','bus_peak_mv'],
            'samples':trace,
            'phase_columns':['stage','uptime_ms','a_counts','b_counts','a_peak_counts',
                             'b_peak_counts','power'],
            'phase_samples':phase_trace},separators=(',',':')),encoding='utf8')
        d.close()


if __name__=='__main__':
    main()
