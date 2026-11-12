"""Repeat two opening moves and their reverse, checking each placement."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import sys
import time

import cv2
import numpy as np

from first_pick import Trial

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'temp/yolo_live'))
from host_preview import Capture

MOVES = (('h2','e2'), ('b9','c7'), ('c7','b9'), ('e2','h2'))
START = {'h2':'red','e2':None,'b9':'black','c7':None}
PARK = 'c4'
ANCHORS = ((375,109,65,70), (900,109,60,70))
SERIAL = 'abc8f01242c327ec'


def read(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def square_map(data):
    if not data['geometry_valid'] or not data['orientation_valid']:
        raise ValueError('棋盘格线和红黑方向需要确认')
    points = {f"{chr(97+p['column'])}{p['row']}":p for p in data['points']}
    for square in (*START,PARK):
        if not points[square]['base_xy_valid']:
            raise ValueError(f'{square} 缺少基座坐标')
    return points


def disk(frame, pixel):
    u,v=map(round,pixel)
    left,top=u-42,v-42
    crop=frame[top:v+43,left:u+43]
    gray=cv2.cvtColor(crop,cv2.COLOR_BGR2GRAY)
    found=cv2.HoughCircles(cv2.GaussianBlur(gray,(5,5),0),cv2.HOUGH_GRADIENT,1,35,
                           param1=85,param2=14,minRadius=18,maxRadius=30)
    if found is None:
        return None
    candidates=sorted(found[0],key=lambda p:math.hypot(float(p[0])-42,float(p[1])-42))
    for x,y,r in candidates:
        if math.hypot(float(x)-42,float(y)-42)>22:
            continue
        yy,xx=np.ogrid[:gray.shape[0],:gray.shape[1]]
        b,g,red=crop[(xx-x)**2+(yy-y)**2 < (.75*r)**2].astype(np.int16).T
        if np.count_nonzero((red-g>32)&(red-b>25))>25:
            return ('red',[float(x)+left,float(y)+top])
        if np.count_nonzero((b<95)&(g<95)&(red<95))>24:
            return ('black',[float(x)+left,float(y)+top])
    return None


class Demo:
    def __init__(self, work, adb, reference='initial', lift_extra_mm=0, cannon_only=False):
        self.work=work
        self.adb=[adb,'-s',SERIAL]
        self.grid=square_map(read(work/f'{reference}-grid.json'))
        self.reference=cv2.imdecode(np.frombuffer((work/f'{reference}.png').read_bytes(),np.uint8),cv2.IMREAD_COLOR)
        calibration=ROOT/'output/xy-registration-20261111'
        # Use the same calibrated plane that produced the native grid report.
        plane=cv2.FileStorage((calibration/'plane-map.yml').read_text(encoding='utf-8'),
                              cv2.FILE_STORAGE_READ|cv2.FILE_STORAGE_MEMORY)
        self.h=plane.getNode('pixel_to_base').mat()
        plane.release()
        plan=read(ROOT/'output/first-pick-c3-c4-20261111/trial-plan.json')
        z=plan['z']
        rise=z['operator_estimated_rise_mm']
        z['travel_deg']+=(z['travel_deg']-z['pickup_deg'])*lift_extra_mm/rise
        z['operator_estimated_rise_mm']=rise+lift_extra_mm
        self.trial=Trial(plan,adb,work/'events.jsonl')
        self.anchors=[(x,y,cv2.cvtColor(self.reference[y:y+h,x:x+w],cv2.COLOR_BGR2GRAY))
                      for x,y,w,h in ANCHORS]
        self.moves=(('e2','h2'),('h2','e2')) if cannon_only else MOVES
        self.expected=dict(START,h2=None,e2='red') if cannon_only else dict(START)
        self.cannon_only=cannon_only
        self.next=0
        self.cycles=0
        self.capture=None
        self.port=None

    def target(self, square, pixel=None):
        if pixel is None:
            x,y=self.grid[square]['base_xy_mm']
        else:
            X,Y,W=self.h@np.array([*pixel,1])
            x,y=float(X/W),float(Y/W)
        arm=self.trial.arm
        l1,l2=arm['arm_lengths_mm']
        cosine=(x*x+y*y-l1*l1-l2*l2)/(2*l1*l2)
        if not -1<=cosine<=1:
            raise ValueError(f'{square} 超出两段臂长可达范围')
        elbow=math.copysign(math.acos(cosine),arm['elbow_ref_rad'])
        shoulder=math.atan2(y,x)-math.atan2(l2*math.sin(elbow),l1+l2*cosine)
        j2=arm['j2_reference_deg']+math.degrees(math.remainder(shoulder-arm['shoulder_ref_rad'],2*math.pi))/(arm['j2_scale']*arm['j2_sign'])
        j3=arm['j3_reference_motor_deg']+math.degrees(math.remainder(elbow-arm['elbow_ref_rad'],2*math.pi))*arm['j3_ratio']/arm['j3_sign']
        return dict(grid_id=square,base_xy_mm=[x,y],j2_deg=j2,j3_motor_deg=j3)

    def observe(self, frame, squares=START):
        if frame.shape!=self.reference.shape:
            raise RuntimeError('相机分辨率变化')
        for x,y,template in self.anchors:
            h,w=template.shape
            crop=cv2.cvtColor(frame[y-8:y+h+8,x-8:x+w+8],cv2.COLOR_BGR2GRAY)
            _,score,_,point=cv2.minMaxLoc(cv2.matchTemplate(crop,template,cv2.TM_CCOEFF_NORMED))
            if score<.76 or math.dist(point,(8,8))>4:
                raise RuntimeError('棋盘基准位置变化')
        return {square:disk(frame,self.grid[square]['pixel_geometric']) for square in squares}

    def check_initial(self, carried=None):
        found=self.observe(self.reference,(*START,'f2','g2','b8','d7','e7','b2','h9'))
        colors={k:(v[0] if v else None) for k,v in found.items()}
        expected=dict(self.expected,f2=None,g2=None)
        if carried is not None:
            expected[carried]=None
        if not self.cannon_only:
            expected.update(b8=None,d7=None)
        if any(colors[k]!=v for k,v in expected.items()):
            raise RuntimeError(f'开局位置与演示走法不匹配 {colors}')
        for k in ('b2','h9','e7'):
            if colors[k] is not None:
                self.expected[k]=colors[k]
        return {k:self.target(k,v[1] if v else None) for k,v in found.items() if k in START}

    def load_checkpoint(self):
        marker=self.work/('cannon-checkpoint.json' if self.cannon_only else 'checkpoint.json')
        if marker.exists():
            state=read(marker)
            self.expected,self.next,self.cycles=state['expected'],state['next'],state['cycles']

    def fresh_view(self):
        # Ignore frames captured while the arm was clearing the board.
        began=time.monotonic()
        end=began+3
        last=None
        colors=None
        frame=None
        last_error=None
        while time.monotonic()<end:
            self.trial.sample()
            if self.capture.error:
                raise RuntimeError(self.capture.error)
            frame, stamp, number=self.capture.latest()
            if frame is not None and stamp>=began+.25 and number!=last:
                last=number
                try:
                    seen=self.observe(frame,self.expected)
                except RuntimeError as exc:
                    last_error=str(exc)
                    time.sleep(.035)
                    continue
                colors={k:(v[0] if v else None) for k,v in seen.items()}
                if colors==self.expected:
                    return seen, frame
            time.sleep(.035)
        if frame is not None:
            self.save_view(frame,'last-check-failed.jpg')
        if colors is None and last_error:
            raise RuntimeError(last_error)
        raise RuntimeError(f'相机核对未通过，预期 {self.expected}，实拍 {colors}')

    def save_view(self, frame, name):
        ok,data=cv2.imencode('.jpg',frame,[cv2.IMWRITE_JPEG_QUALITY,91])
        if ok:
            (self.work/name).write_bytes(data.tobytes())

    def run(self, resume_held=False, resume=False, black_pick=None, black_release=None):
        if resume or resume_held:
            self.load_checkpoint()
        self.check_initial(self.moves[self.next][0] if resume_held else None)
        r=subprocess.run(self.adb+['forward','tcp:0','tcp:554'],capture_output=True,text=True,check=True)
        self.port=int(r.stdout.strip())
        self.capture=Capture(f'rtsp://127.0.0.1:{self.port}/live/0')
        self.trial.acquire()
        self.trial.note('demo_travel_height',angle=self.trial.high,
                        rise_mm=self.trial.span/self.trial.degrees_per_mm)
        if not resume_held:
            self.trial.xy(self.target(PARK))
            seen,frame=self.fresh_view()
            self.save_view(frame,'verified-start.jpg')
        while not self.cannon_only or self.cycles==0:
            source,dest=self.moves[self.next]
            color=self.expected[source]
            if not resume_held:
                _,center=seen[source]
                self.trial.note('demo_move',cycle=self.cycles+1,source=source,destination=dest,piece=color)
                self.trial.pick_piece(self.target(source,center),black_pick if color=='black' else None)
            else:
                self.trial.note('demo_resume_held',source=source,destination=dest,piece=color)
                resume_held=False
            self.trial.xy(self.target(PARK))
            self.expected[source]=None
            _,lifted=self.fresh_view()
            self.save_view(lifted,'latest-pick-verified.jpg')
            self.trial.note('demo_pick_verified',source=source,piece=color)
            self.trial.place_piece(self.target(dest),black_release if color=='black' else None)
            self.trial.xy(self.target(PARK))
            self.expected[source],self.expected[dest]=None,color
            seen,frame=self.fresh_view()
            self.trial.note('demo_verified',cycle=self.cycles+1,source=source,destination=dest,piece=color)
            self.save_view(frame,'latest-verified.jpg')
            if self.cycles==0:
                self.save_view(frame,f'first-{source}-{dest}.jpg')
            self.next=(self.next+1)%len(self.moves)
            if self.next==0:
                self.cycles+=1
                self.trial.note('demo_cycle',cycles=self.cycles)
            marker='cannon-checkpoint.json' if self.cannon_only else 'checkpoint.json'
            (self.work/marker).write_text(json.dumps(dict(expected=self.expected,next=self.next,
                                                          cycles=self.cycles),indent=2)+'\n')

    def close(self):
        try:
            self.trial.stop()
        finally:
            if self.capture:
                self.capture.close()
            if self.port is not None:
                subprocess.run(self.adb+['forward','--remove',f'tcp:{self.port}'],timeout=3,check=False)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('work',type=Path)
    p.add_argument('--adb',required=True)
    p.add_argument('--reference',default='initial')
    p.add_argument('--lift-extra-mm',type=float,default=0)
    p.add_argument('--cannon-only',action='store_true')
    p.add_argument('--run',action='store_true')
    p.add_argument('--resume-held',action='store_true')
    p.add_argument('--resume',action='store_true')
    p.add_argument('--black-pick-deg',type=float)
    p.add_argument('--black-release-deg',type=float)
    args=p.parse_args()
    cv2.setNumThreads(1)
    demo=Demo(args.work,args.adb,args.reference,args.lift_extra_mm,args.cannon_only)
    if not args.run:
        if args.resume or args.resume_held:
            demo.load_checkpoint()
        carried=demo.moves[demo.next][0] if args.resume_held else None
        print(json.dumps(demo.check_initial(carried),indent=2))
        return
    try:
        demo.run(args.resume_held,args.resume,args.black_pick_deg,args.black_release_deg)
    except KeyboardInterrupt:
        demo.trial.note('demo_stopped',reason='keyboard')
    except Exception as exc:
        demo.trial.note('demo_stopped',reason=str(exc),cycles=demo.cycles,move_index=demo.next)
        raise
    finally:
        demo.close()


if __name__=='__main__':
    main()
