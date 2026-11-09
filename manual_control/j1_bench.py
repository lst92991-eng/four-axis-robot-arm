"""Sequential J1 bench tests. Owns CAN; JSON commands arrive on stdin.

No MOVE on launch. Each run accelerates to its requested speed and continues
until its time budget, then STOP holds. Fault evidence is saved before CLEAR.
"""
import argparse
from datetime import datetime
import json
from pathlib import Path
import queue
import statistics
import sys
import threading
import time

from calibrate_j1 import Session
from telemetry import diagnostic


class Bench(Session):
    def __init__(self, serial, out, expected, acceleration=30, travel_budget=35, max_speed=30):
        super().__init__(serial,out)
        self.out=out;self.expected=expected;self.traces=[];self.attempt=0
        self.acceleration=acceleration
        self.travel_budget=travel_budget
        self.max_speed=max_speed;self.recovering=False
        self.brake_rate=360
        self.control_queue=None;self.quit_requested=False
        original=self.adapter.recv
        self.rx_queue=queue.Queue(maxsize=4096);self.rx_stop=threading.Event();self.rx_error=None
        def receiver():
            try:
                while not self.rx_stop.is_set():
                    f=original(.02)
                    if f is not None:self.rx_queue.put_nowait((f,time.monotonic()))
            except Exception as exc:self.rx_error=exc
        self.rx_thread=threading.Thread(target=receiver,name='bench-can-rx')
        self.rx_thread.start()
        def tap(timeout):
            if self.rx_error is not None:raise self.rx_error
            try:frame,received=self.rx_queue.get(timeout=max(.001,timeout))
            except queue.Empty:return None
            if time.monotonic()-received>.25 and (getattr(self,'running',False) or (self.status and self.status['power'])):
                raise RuntimeError('Delayed CAN frame is not live feedback')
            if frame and not frame.echo and not frame.error and frame.identifier==0x301:
                _,_,v=diagnostic(frame.identifier,frame.data)
                self.traces.append(dict(v,host=received,velocity=v['speed_counts_s']*360/16384))
            return frame
        self.adapter.recv=tap
        self.wait(lambda v:True,3)
        if self.status['build']!=expected:raise RuntimeError('Unexpected firmware build')
        if not self.status['configured'] or not self.status['stored']:raise RuntimeError('Electrical calibration missing')
        self.origin=round(self.status['counts']-self.status['position']*16384/360) if self.status['referenced'] else self.status['counts']
        self.running=False
        self.sync()
        if self.status['fault']:self.recover('initial_fault')
        elif not self.status['referenced']:
            if self.status['power']:raise RuntimeError('Unreferenced powered axis')
            self.command(2,0)
            self.wait(lambda v:v['referenced'] and v['stationary'],2,True)
        # Communication qualification has no motion commands.
        self.qualifying=True
        started=time.monotonic();begin_commands=len(self.commands)
        while time.monotonic()-started<1.5:
            self.pump()
        self.report('ready',pings=sum(c['kind']==7 for c in self.commands[begin_commands:]),status=self.status)
        self.qualifying=False

    def close(self):
        self.rx_stop.set();self.rx_thread.join(1)
        if self.rx_thread.is_alive():raise RuntimeError('Receiver did not exit before USB close')
        super().close()

    def report(self,event,**kw):
        record=dict(event=event,time=datetime.now().astimezone().isoformat(),**kw)
        print(json.dumps(record,ensure_ascii=False),flush=True)
        with (self.out/(self.stamp+'-bench-events.jsonl')).open('a',encoding='utf-8') as f:
            f.write(json.dumps(record,ensure_ascii=False)+'\n')

    def pump(self):
        self.receive(.005)
        now=time.monotonic()
        if not self.running and self.status['fault'] and not self.recovering:
            self.recovering=True
            try:self.recover('idle_fault')
            finally:self.recovering=False
        if now-self.seen>.25 and (self.running or self.status['power']):
            raise RuntimeError('Stale J1 feedback')
        if (getattr(self,'qualifying',False) or self.status['state'] in (1,2,3,5)) and now-self.last_ping>=.08:
            try:self.command(7,timeout=.15)
            except TimeoutError:
                if time.monotonic()-self.seen>.15:raise
                self.report('ping_reply_missing',status=self.status)
                self.command(7,timeout=.15)  # New sequence; never retry a MOVE.
            self.last_ping=time.monotonic()
        return self.status

    def recover(self,tag):
        # One diagnostic frame period before clearing, while output is OFF.
        self.wait(lambda v:not v['power'] and v['encoder_valid'],1)
        end=time.monotonic()+.06
        while time.monotonic()<end:self.receive(.005)
        evidence=dict(status=dict(self.status),current_fault=self.fault_evidence,
                      traces=self.traces[-80:])
        file=self.out/(self.stamp+f'-{self.attempt:02d}-{tag}.json')
        file.write_text(json.dumps(evidence,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        if not 10000<=self.status['bus_mv']<=30000:raise RuntimeError('Supply outside recovery range')
        self.command(6)
        self.wait(lambda v:v['state']==0 and not v['power'] and v['encoder_valid'] and v['fault']==0,1,True)
        # CLEAR reinitializes the driver's multi-turn counter. Retain the
        # logical position using its last unwrapped count and the new raw angle.
        old=evidence['status']['counts'];new=self.status['counts']
        delta=((new-(old&16383)+8192)&16383)-8192
        if abs(delta)>512:raise RuntimeError('Unexpected movement during fault recovery')
        self.origin=new-((old-self.origin)+delta)
        degrees=(self.status['counts']-self.origin)*360/16384
        if abs(degrees)>self.travel_budget:raise RuntimeError('Cumulative travel exceeds this bench session budget')
        self.command(2,round(degrees*1000))
        self.wait(lambda v:v['referenced'] and v['stationary'],1,True)
        self.report('fault_cleared',evidence=str(file),status=self.status)

    def run_test(self,direction,seconds,speed=10,velocity_mode=False):
        if direction not in (-1,1) or not .2<=seconds<=(25 if velocity_mode else 3) or not 0<speed<=self.max_speed:
            raise ValueError('Invalid direction, duration or reviewed speed')
        if self.status['fault']:self.recover('pre_run_fault')
        self.wait(lambda v:v['state'] in (0,1) and v['stationary'] and not v['interlocked'],2,True)
        self.attempt+=1;start=time.monotonic();index=len(self.traces);initial=dict(self.status)
        self.running=True
        self.report('start',attempt=self.attempt,direction=direction,speed=speed,seconds=seconds,initial=initial)
        if velocity_mode:self.command(12,round(direction*speed*1000))
        else:self.command(3,180000*direction,round(speed*1000))
        failed=False
        while time.monotonic()-start<seconds:
            v=self.pump()
            if self.stop_requested():break
            if v['fault']:failed=True;break
            if abs((v['counts']-self.origin)*360/16384)>self.travel_budget-5:
                self.command(4);raise RuntimeError('Reached cumulative bench travel boundary')
        if not failed:
            stop_at=time.monotonic()
            self.command(4,rejection_ok=True)
            end=time.monotonic()+speed/self.brake_rate+2
            while time.monotonic()<end:
                v=self.pump()
                if v['fault']:failed=True;break
                if v['state']==1 and v['stationary']:break
            else:raise TimeoutError('STOP did not settle within 3 seconds')
        rows=self.traces[index:]
        moving=[v for v in rows if v['power']]
        cruise=[v for v in moving if start+max(.4,speed/self.acceleration+.1)<=v['host']<=start+seconds-.05]
        summary=dict(attempt=self.attempt,direction=direction,seconds=seconds,speed=speed,
            velocity_mode=velocity_mode,brake_rate=self.brake_rate,
            stop_confirm_s=round(time.monotonic()-stop_at,3) if not failed else None,
            fault=self.status['fault'],initial=initial,final=dict(self.status),samples=len(rows),
            current_peak_ma=max((abs(v['iq_ref_ma']) for v in moving),default=0),
            velocity_peak_deg_s=max((abs(v['velocity']) for v in moving),default=0),
            cruise_mean_deg_s=statistics.mean(v['velocity'] for v in cruise) if cruise else None,
            saturation_samples=sum(abs(v['iq_ref_ma'])>=950 for v in moving))
        (self.out/(self.stamp+f'-trial-{self.attempt:02d}.json')).write_text(
            json.dumps(dict(summary=summary,trace=rows),ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        self.report('result',**summary)
        self.running=False
        if failed:self.recover('fault')

    def stop_requested(self):
        if self.control_queue is None:return False
        try:q=self.control_queue.get_nowait()
        except queue.Empty:return False
        if q['op'] in ('stop','exit'):
            self.quit_requested=q['op']=='exit'
            if self.status['state'] in (1,2,3,5):self.command(4,rejection_ok=True)
            return True
        self.control_queue.put(q)
        return False

    def travel(self,target,speed=10):
        if not 0<speed<=self.max_speed or abs(target)>self.travel_budget-5:
            raise ValueError('Long test requires reviewed speed and authorized travel envelope')
        if self.status['fault']:self.recover('pre_travel_fault')
        self.wait(lambda v:v['state'] in (0,1) and v['stationary'] and not v['interlocked'],2,True)
        self.attempt+=1;self.running=True;index=len(self.traces);start=time.monotonic();initial=dict(self.status)
        duration=abs(target-initial['position'])/speed+2*speed/self.acceleration+8
        direction=1 if target>initial['position'] else -1
        self.report('travel_start',attempt=self.attempt,target=target,speed=speed,initial=initial)
        self.command(3,round(target*1000),round(speed*1000))
        progress_at=start;failed=False;aborted=False
        while time.monotonic()-start<duration:
            v=self.pump()
            if self.stop_requested():
                aborted=True
                self.wait(lambda v:v['state'] in (0,1,4) and (v['stationary'] or not v['power']),3,True)
                break
            if v['fault']:failed=True;break
            if abs((v['counts']-self.origin)*360/16384)>self.travel_budget:
                self.command(4);raise RuntimeError('Long-travel envelope exceeded')
            now=time.monotonic()
            if now-progress_at>=20:
                self.report('travel_progress',attempt=self.attempt,elapsed_s=round(now-start,1),
                    turns=round(abs(v['position']-initial['position'])/360,3),position=v['position'],velocity=v['velocity'],fault=v['fault'])
                progress_at=now
            if v['state']==1 and v['stationary'] and abs(v['position']-target)<=.1:
                hold_until=time.monotonic()+1
                while time.monotonic()<hold_until:
                    v=self.pump()
                    if v['fault']:failed=True;break
                break
        else:
            self.command(4);raise TimeoutError('Long travel did not finish')
        self.running=False
        rows=self.traces[index:]
        cruise=[r for r in rows if r['power'] and start+speed/self.acceleration+.5<r['host']<time.monotonic()-speed/self.acceleration-.5]
        summary=dict(attempt=self.attempt,direction=direction,speed=speed,target=target,
            completed=not aborted and not failed,aborted=aborted,
            elapsed_s=round(time.monotonic()-start,3),fault=self.status['fault'],initial=initial,final=dict(self.status),
            motor_turns=(self.status['counts']-initial['counts'])/16384,
            cruise_mean_deg_s=statistics.mean(r['velocity'] for r in cruise) if cruise else None,
            current_peak_ma=max((abs(r['iq_ref_ma']) for r in rows),default=0),
            saturation_samples=sum(abs(r['iq_ref_ma'])>=950 for r in rows))
        with (self.out/(self.stamp+f'-travel-{self.attempt:02d}.json')).open('w',encoding='utf-8') as f:
            json.dump(dict(summary=summary,trace=rows),f)
        self.report('travel_result',**summary)
        if failed:self.recover('long_fault')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--build',required=True)
    p.add_argument('--acceleration',type=float,default=30)
    p.add_argument('--travel-budget',type=float,default=35)
    p.add_argument('--max-speed',type=float,default=30)
    p.add_argument('--out',type=Path,default=Path(__file__).resolve().parents[1]/'output/j1-bench-20261109')
    args=p.parse_args();commands=queue.Queue()
    def read_stdin():
        for line in sys.stdin:
            try:commands.put(json.loads(line))
            except ValueError:print('Invalid JSON input',flush=True)
    threading.Thread(target=read_stdin,daemon=True).start()
    b=None
    try:
        b=Bench('37813482333551062',args.out,args.build,args.acceleration,args.travel_budget,args.max_speed)
        b.control_queue=commands
        while True:
            if b.quit_requested:break
            b.pump()
            try:q=commands.get_nowait()
            except queue.Empty:continue
            if q['op']=='run':b.run_test(q['direction'],q['seconds'],q.get('speed',10))
            elif q['op']=='velocity':b.run_test(q['direction'],q['seconds'],q['speed'],True)
            elif q['op']=='brake':
                b.command(13,round(q['rate']*1000))
                if b.last_reply_values[0]!=round(q['rate']*1000):raise RuntimeError('Brake rate readback mismatch')
                b.brake_rate=q['rate'];b.report('brake_rate',rate=b.brake_rate)
            elif q['op']=='travel':b.travel(q['target'],q.get('speed',10))
            elif q['op']=='tune':
                b.command(10,round(q['kp']*1000),round(q['ki']*1000))
                if b.last_reply_values[:2]!=[round(q['kp']*1000),round(q['ki']*1000)]:
                    raise RuntimeError('Gain readback mismatch')
                b.report('tuned',kp=q['kp'],ki=q['ki'],current_limit_ma=b.last_reply_values[2])
            elif q['op']=='status':b.report('status',status=b.status)
            elif q['op']=='stop':
                if b.status['state'] in (1,2,3,5):b.command(4,rejection_ok=True)
                b.report('stopped',status=b.status)
            elif q['op']=='exit':break
    except BaseException as exc:
        if b:b.report('error',error=str(exc),status=b.status)
        else:print(str(exc),flush=True)
        raise
    finally:
        if b:
            if b.status and b.status['state'] in (1,2,3,5):
                try:b.command(4,rejection_ok=True)
                except Exception:pass
            b.close()  # No OFF: the driver retains holding torque if already active.


if __name__=='__main__':main()
