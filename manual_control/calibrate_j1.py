"""Basic electrical calibration using v3 command 9.

The default node is J1. --run calibrates and saves the record. For J1, --run
also sets mechanical zero, as before. For J2–J4, mechanical referencing is
a separate operation.
"""
import argparse
from datetime import datetime
import json
from pathlib import Path
import struct
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'temp/transport'))
from stepfoc_candlelight import Candlelight, Frame, is_candlelight_status_report, describe_bus_error
from core import PACKET, packet

STATES = ['idle', 'precheck', 'align', 'direction', 'return', 'forward', 'reverse', 'done', 'failed', 'aborted', 'static_ok']
ERRORS = ['ok', 'sensor', 'current', 'timeout', 'travel', 'unstable', 'no_motion', 'hysteresis', 'map', 'link', 'user', 'power']


def decode(data, node=1):
    k,n,result,version,seq,*v=PACKET.unpack(data)
    if (k,n,result,version)!=(0x40,node,0,3) or v[13]!=0x43414c31:
        raise ValueError(f'J{node} 尚未运行本次标定固件')
    bits=(v[9]&0xffffffff)>>16
    return dict(build=f'{v[10]&0xffffffff:08x}',uptime=v[0]&0xffffffff,
        state=v[9]&255,fault=(v[9]>>8)&255,power=bool(bits&64),encoder_valid=bool(bits&32),
        referenced=bool(bits&2),configured=bool(bits&16),position=v[1]/1000,
        velocity=v[4]/1000,target=v[2]/1000,stationary=bool(bits&256),interlocked=bool(bits&8),
        cal_state=v[3]&255,cal_error=(v[3]>>8)&255,ready=bool(v[3]&(1<<16)),stored=bool(v[3]&(1<<17)),
        motion_fault_detail=(v[3]>>24)&255,
        counts=v[5],raw_zero=v[6],direction=v[7],current_ma=v[8],progress=v[11],bus_mv=v[12])


class Session:
    def __init__(self, serial, out, node=1):
        if node not in range(1, 5):
            raise ValueError('标定节点必须为 1–4')
        self.node = node
        out.mkdir(parents=True,exist_ok=True)
        self.stamp=datetime.now().strftime('%Y%m%d-%H%M%S-%f')
        self.path=out/(self.stamp+f'-j{node}-calibration.jsonl')
        self.log=self.path.open('x',encoding='utf-8',buffering=1)
        try:
            self.adapter=Candlelight(serial=serial,fd=True)
        except BaseException:
            self.log.close()
            raise
        self.seq=0;self.status=None;self.seen=0.;self.last_ping=0.;self.replies={};self.commands=[]
        self.boot_ms=None
        self.fault_evidence=None
        self.reply_values={};self.last_reply_values=None

    def receive(self,timeout=.01):
        f=self.adapter.recv(timeout)
        if not f or f.echo:return
        if f.error:
            if not is_candlelight_status_report(f):raise RuntimeError(describe_bus_error(f))
            return
        base_id = f.identifier - self.node
        if base_id not in (0x200,0x480,0x300,0x380):return
        self.log.write(json.dumps(dict(t=time.monotonic(),direction='rx',id=f.identifier,data=f.data.hex()))+'\n')
        if base_id==0x300:return
        if base_id==0x380:
            if len(f.data)==64 and struct.unpack_from('<I',f.data)[0]==0x31464349:
                fields=struct.unpack_from('<12i',f.data,4)
                self.fault_evidence=dict(zip(['cause','board_ms','phase_a_counts','phase_b_counts',
                    'calibrating','reference_ma','previous_id_ma','previous_iq_ma','previous_ua_mv',
                    'previous_ub_mv','encoder_counts','sample_window_valid'],fields))
            return
        if not f.fd or not f.brs or f.extended or len(f.data)!=64:
            raise RuntimeError(f'Invalid J{self.node} frame')
        k,n,res,ver,seq,*v=PACKET.unpack(f.data)
        if n!=self.node or ver!=3:raise RuntimeError('Wrong node/version')
        if base_id==0x480:
            self.replies[seq]=(k,res)
            self.reply_values[seq]=v
        else:
            value=decode(f.data, self.node)
            if self.status and ((value['uptime']-self.status['uptime'])&0xffffffff)>0x80000000:
                raise RuntimeError(f'J{self.node} restarted during session')
            self.status=value;self.seen=time.monotonic()

    def command(self,kind,a=0,b=0,rejection_ok=False,timeout=.35):
        self.seq=(self.seq+1)&0xffffffff
        identifier = 0x400 + self.node
        data=packet(kind,self.node,self.seq,a,b)
        self.log.write(json.dumps(dict(t=time.monotonic(),direction='tx',id=identifier,data=data.hex()))+'\n')
        self.commands.append(dict(kind=kind,a=a,b=b,seq=self.seq))
        self.adapter.send(Frame(identifier,data,fd=True,brs=True))
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            self.receive(.01)
            if self.seq in self.replies:
                k,res=self.replies.pop(self.seq)
                self.last_reply_values=self.reply_values.pop(self.seq,None)
                if k!=(kind|0x80) or res not in (0,1):raise RuntimeError('Invalid ACK')
                if res and not rejection_ok:raise RuntimeError(f'J{self.node} rejected command {kind}, op {a}')
                return res==0
        raise TimeoutError(f'J{self.node} ACK timeout, command {kind}; no retry')

    def sync(self):
        for _ in range(4):
            if self.command(7,rejection_ok=True):return
            self.seq=(self.seq+0x3fffffff)&0xffffffff
        raise RuntimeError('PING sequence sync failed')

    def wait(self,predicate,seconds,heartbeat=False):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            self.receive(.01)
            if heartbeat and time.monotonic()-self.last_ping>.1:
                self.command(7);self.last_ping=time.monotonic()
            if self.status and time.monotonic()-self.seen<.25 and predicate(self.status):return self.status
        raise TimeoutError(f'Expected fresh J{self.node} state did not arrive')

    def close(self):
        self.adapter.close();self.log.close()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--node', type=int, choices=range(1, 5), default=1)
    p.add_argument('--run',action='store_true')
    p.add_argument('--zero-only',action='store_true',help='仅使用已经保存的电角标定，将当前位置记为机械0度')
    p.add_argument('--current-ma',type=int,default=300)
    p.add_argument('--serial',default='37813482333551062')
    p.add_argument('--expected-build',required=True)
    p.add_argument('--out',type=Path,default=ROOT/'output/manual-control')
    args=p.parse_args()
    if args.run and args.zero_only:p.error('--run 与 --zero-only 不能同时使用')
    if not 100<=args.current_ma<=1000:p.error('current must be 100..1000 mA')
    s=Session(args.serial,args.out,node=args.node)
    report=dict(node=args.node,completed=False,run=args.run,commands=s.commands,raw_log=str(s.path))
    started=False
    try:
        initial=s.wait(lambda v:True,3)
        print('INITIAL '+json.dumps(initial),flush=True);report['initial']=initial
        if initial['build']!=args.expected_build:raise RuntimeError('Build ID does not match reviewed firmware')
        if not (args.run or args.zero_only):return
        if initial['power'] or initial['fault'] or initial['state']!=0 or not initial['encoder_valid']:
            raise RuntimeError(f'J{args.node} must be idle, unpowered, fault-free with valid encoder')
        s.sync()
        if args.zero_only:
            if not initial['stored'] or not initial['ready'] or not initial['configured']:
                raise RuntimeError('No saved electrical calibration; mechanical zero alone is insufficient')
            s.command(2,0)
            final=s.wait(lambda v:v['referenced'] and not v['power'] and abs(v['position'])<=.1,2,True)
            report.update(completed=True,zero_only=True,final=final)
            print('SUCCESS '+json.dumps(final),flush=True)
            return
        started=True
        s.command(9,1,args.current_ma)
        # Demand a new PRECHECK/etc frame; never reuse a previous DONE frame.
        s.wait(lambda v:1<=v['cal_state']<=6,1,True)
        end=time.monotonic()+15;last_phase=None
        while time.monotonic()<end:
            s.receive(.01)
            if time.monotonic()-s.last_ping>.1:
                s.command(7);s.last_ping=time.monotonic()
            v=s.status
            if time.monotonic()-s.seen>.25:raise TimeoutError('STATUS stale during calibration')
            if v['fault']:raise RuntimeError(f"Drive fault {v['fault']}")
            if v['cal_state']!=last_phase:
                print('PHASE '+json.dumps(v),flush=True);last_phase=v['cal_state']
            if v['cal_state'] in (8,9):
                raise RuntimeError('Calibration failed: '+ERRORS[v['cal_error']])
            if v['cal_state']==7 and not v['power']:
                if not v['ready'] or not 0<=v['raw_zero']<16384 or v['direction'] not in (-1,1):
                    raise RuntimeError('Invalid calibration result')
                break
        else:raise TimeoutError('Calibration did not finish')
        calibrated = dict(s.status)
        report['calibrated'] = calibrated
        s.command(9,3)
        final = s.wait(lambda v: v['uptime'] != calibrated['uptime'] and
                       v['stored'] and v['ready'] and v['configured'] and not v['power'] and
                       not v['fault'] and v['raw_zero'] == calibrated['raw_zero'] and
                       v['direction'] == calibrated['direction'], 2, True)
        if args.node == 1:
            s.command(2,0)  # Preserve J1's existing mechanical-reference workflow.
            final=s.wait(lambda v:v['configured'] and v['referenced'] and not v['power'] and abs(v['position'])<=.1,2,True)
        report.update(completed=True,final=final)
        print('SUCCESS '+json.dumps(final),flush=True)
    except BaseException as exc:
        report['error']=str(exc)
        if started:
            try:s.command(9,2)  # Only abort calibration; never MOVE or a generic OFF.
            except Exception as cancel:report['abort_error']=str(cancel)
        end=time.monotonic()+.1
        try:
            while time.monotonic()<end:s.receive(.01)
        except Exception:pass
        raise
    finally:
        report['last_status']=s.status
        report['fault_evidence']=s.fault_evidence
        s.close()
        (args.out/(s.stamp+f'-j{args.node}-calibration-summary.json')).write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')


if __name__=='__main__':main()
