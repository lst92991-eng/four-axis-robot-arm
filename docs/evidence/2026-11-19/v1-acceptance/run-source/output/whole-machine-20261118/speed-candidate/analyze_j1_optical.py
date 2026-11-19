"""Measure tool-board lateral image motion against the fixed board; produce a 1/4-speed replay."""
import argparse
import csv
import json
from itertools import groupby
from pathlib import Path
import shutil
import subprocess

import cv2 as cv
import numpy as np
from PIL import Image, ImageDraw, ImageFont


# 原始1280x720图像坐标。目标是吸头正上方的末端驱动板。
TOOL = (405, 92, 490, 176)
FIXED = [(345, 366, 572, 627), (723, 502, 925, 631)]
FIXED_CROP = [(343, 183, 392, 254), (337, 86, 387, 177)]
REPLAY = (320, 24, 610, 270)


def read(path):
    return json.loads(path.read_text(encoding='utf8'))


def feature_points(gray, regions, origin, count):
    mask = np.zeros_like(gray)
    ox,oy = origin
    for x1,y1,x2,y2 in regions:
        x1,y1 = max(0,x1-ox),max(0,y1-oy)
        x2,y2 = min(gray.shape[1],x2-ox),min(gray.shape[0],y2-oy)
        if x1<x2 and y1<y2:
            mask[y1:y2,x1:x2] = 255
    points = cv.goodFeaturesToTrack(gray,count,.025,4,mask=mask,blockSize=5)
    if points is None or len(points)<8:
        raise ValueError('Insufficient texture in optical ROI')
    return points


def displacement(reference, frame, points, at):
    opts = dict(winSize=(21,21), maxLevel=2,
                criteria=(cv.TERM_CRITERIA_COUNT|cv.TERM_CRITERIA_EPS,25,.02))
    found,ok,_ = cv.calcOpticalFlowPyrLK(reference,frame,points,None,**opts)
    back,ok2,_ = cv.calcOpticalFlowPyrLK(frame,reference,found,None,**opts)
    good = (ok.ravel()!=0) & (ok2.ravel()!=0) & (np.linalg.norm(back-points,axis=2).ravel()<.65)
    src,dst = points[good].reshape(-1,2),found[good].reshape(-1,2)
    if len(src)<8:
        return np.full(2,np.nan),0
    transform,mask = cv.estimateAffinePartial2D(src,dst,method=cv.RANSAC,ransacReprojThreshold=.85)
    if transform is None or int(mask.sum())<8:
        return np.full(2,np.nan),0
    return transform[:,:2]@at+transform[:,2]-at, int(mask.sum())


def read_segment(cap, times, relative_to, first, last):
    start = int(np.searchsorted(times,first))
    end = int(np.searchsorted(times,last,'right'))
    cap.set(cv.CAP_PROP_POS_FRAMES,start)
    frames=[]
    for _ in range(start,end):
        ok,frame=cap.read()
        if not ok:
            break
        frames.append(frame)
    return times[start:start+len(frames)]-relative_to,frames


def analyze(t,frames,origin,terminal_window=None):
    if len(frames)<20:
        raise ValueError('Insufficient camera overlap with trial')
    grays = [cv.cvtColor(f,cv.COLOR_BGR2GRAY) for f in frames]
    ref = grays[max(0,int(np.searchsorted(t,-.15))-1)]
    cropped = frames[0].shape[1]<640
    tip = feature_points(ref,[TOOL],origin,55)
    bg = feature_points(ref,FIXED_CROP if cropped else FIXED,origin,100)
    center=np.mean(tip.reshape(-1,2),axis=0)
    tool_path=[]; bg_path=[]; tip_inliers=[]; bg_inliers=[]
    for frame in grays:
        tool,n = displacement(ref,frame,tip,center)
        fixed,m = displacement(ref,frame,bg,center)
        tool_path.append(tool); bg_path.append(fixed)
        tip_inliers.append(n); bg_inliers.append(m)
    track=np.asarray(tool_path)-np.asarray(bg_path)
    valid=np.isfinite(track).all(axis=1)
    if valid.mean()<.90:
        raise ValueError(f'Tracking valid in {valid.mean():.1%} of frames')
    track=track[valid]; tt=t[valid]
    # 相机与Z轴夹角使正常升降产生沿一个方向的像素位移，取其垂直分量。
    first=np.median(track[:5],axis=0); last=np.median(track[-5:],axis=0)
    travel=np.linalg.norm(last-first)
    if travel<4:
        raise ValueError(f'Visible optical stroke too short ({travel:.2f} px)')
    axis=(last-first)/travel
    position=(track-first)@axis
    lateral=(track-first)@np.array([-axis[1],axis[0]])
    radius=2
    ripple=lateral[radius:-radius]-np.convolve(lateral,np.ones(5)/5,mode='valid')
    tm=tt[radius:-radius]
    active=(position[radius:-radius]>=.05*travel)&(position[radius:-radius]<=.95*travel)
    if active.sum()<8:
        raise ValueError('Insufficient tracked samples during the stroke')
    # 前置和落稳位置两个静止窗口，剔除命令切换和运动边缘。
    static=((tm<=-.06) | (tm>=tt[-1]-.45)) & ~active
    static &= (np.abs(np.diff(position,n=1,prepend=position[0]))[radius:-radius]<.22)
    def rms(a):
        return round(float(np.sqrt(np.mean(a*a))),3)
    measures=dict(image_travel_px=round(float(travel),3),ripple_rms_px=rms(ripple[active]),
                ripple_p95_px=round(float(np.percentile(abs(ripple[active]),95)),3),
                measured_stationary_rms_px=rms(ripple[static]) if static.sum()>=5 else None,
                total_frames=len(t),valid_frames=int(valid.sum()),
                stroke_samples=int(active.sum()),stationary_samples=int(static.sum()),
                median_tool_inliers=int(np.median(tip_inliers)),median_fixed_inliers=int(np.median(bg_inliers)),
                median_frame_gap_ms=round(float(np.median(np.diff(tt))*1000),1),
                max_frame_gap_ms=round(float(np.max(np.diff(tt))*1000),1))
    if terminal_window:
        near = (tm>=terminal_window[0]) & (tm<=terminal_window[1])
        measures['terminal_ripple_rms_px'] = rms(ripple[near]) if near.sum()>=5 else None
        measures['terminal_frames'] = int(near.sum())
    return measures


def terminal_window(trace,name):
    powered = [r for r in trace['samples'] if r[0]==name]
    rows = max((list(g) for on,g in groupby(powered,key=lambda r:r[4]) if on),key=len)
    dt = np.diff([r[1] for r in rows])
    speeds = abs(np.diff([r[2] for r in rows])*360000/(16384*dt))
    times = (np.array([r[1] for r in rows[1:]])-rows[0][1])*.001
    fine = (dt>=15)&(dt<=27)
    smooth = np.convolve(speeds[fine],np.ones(5)/5,mode='valid')
    times = times[fine][2:-2]
    peak = int(np.argmax(smooth))
    begin = next((i for i in range(peak,len(smooth)-2) if max(smooth[i:i+3])<=250),None)
    if begin is None:
        return None
    end = next((i for i in range(begin,len(smooth)-4) if max(smooth[i:i+5])<=5),None)
    if end is None:
        return None
    return (float(times[begin]-.12),float(times[end]+.18))


def replay(writer,frames,t,origin,setting,direction,playback):
    x,y,x2,y2=REPLAY
    ox,oy=origin
    label='上升' if direction=='up' else '下降'
    font=ImageFont.truetype('C:/Windows/Fonts/msyh.ttc',22)
    for f,ts in zip(frames,t):
        crop=f[y-oy:y2-oy,x-ox:x2-ox]
        zoom=cv.resize(crop,(580,492),interpolation=cv.INTER_LINEAR)
        image=Image.fromarray(cv.cvtColor(cv.copyMakeBorder(zoom,0,78,0,0,cv.BORDER_CONSTANT),
                                          cv.COLOR_BGR2RGB))
        draw=ImageDraw.Draw(image)
        draw.text((12,500),f'J1 {label}   {setting}   ·   {playback}',font=font,fill='white')
        draw.text((12,535),f'距指令 {ts:+.3f} 秒    同一机位与裁切',font=font,fill=(195,205,210))
        writer.stdin.write(cv.cvtColor(np.asarray(image),cv.COLOR_RGB2BGR).tobytes())


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('camera',type=Path)
    parser.add_argument('reports',type=Path,nargs='+')
    parser.add_argument('--replay',action='store_true')
    parser.add_argument('--descents',action='store_true')
    parser.add_argument('--staircase',action='store_true')
    args=parser.parse_args(); folder=args.camera
    roi=read(folder/'camera-roi.json'); origin=roi['crop_xyxy'][:2]; fps=roi['nominal_fps']
    with (folder/'camera-b2-time.csv').open(encoding='ascii',newline='') as stream:
        times=np.array([float(x['host_monotonic_s']) for x in csv.DictReader(stream)])
    cap=cv.VideoCapture(str(folder/'camera-b2.avi'))
    results=[]; seen=set(); writer=None
    rendered_frames=0
    target=folder/'j1-slow-replay.mp4'; staging=folder/'j1-slow-replay.partial.mp4'
    if args.replay:
        ffmpeg=shutil.which('ffmpeg')
        writer=subprocess.Popen([ffmpeg,'-hide_banner','-loglevel','error','-y','-f','rawvideo',
             '-pix_fmt','bgr24','-s','580x570','-r',str(fps if args.staircase else fps/4),'-i','pipe:0','-an',
             '-c:v','libx264','-crf','17','-preset','veryfast','-pix_fmt','yuv420p',
             '-movflags','+faststart',str(staging)],stdin=subprocess.PIPE)
    complete=False
    try:
        for p in args.reports:
            report=read(p/'report.json' if p.is_dir() else p)
            trace=read((p if p.is_dir() else p.parent)/'trace.json') if args.staircase else None
            for trial in report['trials']:
                if not trial.get('completed',False):
                    continue
                direction=trial.get('direction') or trial['name'].split('_')[0]
                if args.descents and direction!='down':
                    continue
                begin,end=trial['host_begin_s'],trial['host_end_s']
                if times[0]>begin-.28 or times[-1]<end-.1:
                    continue
                # 停稳至 H 桥或下一支运动最多只留少量余量，按每程单独跟踪。
                t,frames=read_segment(cap,times,begin,begin-.38,end+.035)
                slope=trial['brake_deg_s2']
                brake_ki=trial.get('descent_brake_ki_permille')
                setting=f'制动 {slope}°/s²'
                if trial.get('stop_strategy'):
                    setting='提前发 STOP' if trial['stop_strategy']=='STOP' else '定位减速 MOVE'
                elif args.descents and brake_ki:
                    setting=f"制动 Ki {trial['speed_ki_milli']*brake_ki/1e6:g}"
                if 'stair_round' in trial:
                    field={'down_brake':'下制动','down_ki_permille':'下制动 Ki‰',
                           'kp_milli':'Kp×1000','accel':'起动斜率'}[trial['stair_field']]
                    setting=f"第 {trial['stair_round']:02d} 轮  {field} {trial['stair_level']}"
                name=trial['name']
                try:
                    window=terminal_window(trace,name) if trace else None
                    m=analyze(t,frames,origin,window)
                except ValueError as e:
                    m=dict(unavailable=str(e))
                m.update(trial=name,folder=p.name,direction=direction,slope=slope)
                if args.staircase:
                    m['stair_round']=trial['stair_round']
                print(json.dumps(m),flush=True); results.append(m)
                if writer and (setting,direction) not in seen:
                    m['replay_start_s']=round(rendered_frames/(fps if args.staircase else fps/4),3)
                    replay(writer,frames,t,origin,setting,direction,'原速' if args.staircase else '1/4 速')
                    rendered_frames+=len(frames)
                    seen.add((setting,direction))
        if not results:
            raise RuntimeError('No complete trials matching the recording clock')
        complete=True
    finally:
        cap.release()
        if writer:
            writer.stdin.close()
            rc=writer.wait()
            if rc: raise RuntimeError(f'ffmpeg returned {rc}')
    if writer and complete:
        staging.replace(target)
    (folder/'j1-optical-results.json').write_text(json.dumps(dict(tool_roi=TOOL, fixed_roi=FIXED if origin==[0,0] else FIXED_CROP,
        metric='Image-plane motion normal to the endpoint axis; 5-frame centered moving-average residual during 5-95% of optical travel. Stationary RMS is estimated from nearly stationary edges. Times are host reception times. No depth or absolute Z measurement.',
        results=results),ensure_ascii=False,indent=2),encoding='utf8')


if __name__=='__main__':
    main()
