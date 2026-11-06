"""Same-frame YOLO (real RV1106 NPU) + native C++ OpenCV 5 geometry browser preview.
Python transports/displays frames; grid detection runs in scara_geometry_tool, not Python.
No robot connections. No calibration matrix is supplied or generated here.
"""
import argparse
from datetime import datetime
import http.client
import json
from pathlib import Path
import subprocess
import time
import cv2
import numpy as np
import host_preview as hp


def publish_pair(board,jpeg,frame_id,geometry):
    conn=http.client.HTTPConnection(board,8088,timeout=10)
    try:
        conn.request('POST','/dual/preview',body=jpeg,headers={
            'Content-Type':'image/jpeg','X-Frame-ID':frame_id,
            'X-Geometry-Valid':str(int(bool(geometry.get('geometry_valid')))),
            'X-Orientation-Valid':str(int(bool(geometry.get('orientation_valid'))))})
        reply=conn.getresponse();body=reply.read()
        if reply.status!=200:raise RuntimeError(f'Dual publish HTTP {reply.status}: {body[:120]!r}')
    finally:conn.close()


def inspect_geometry(exe,reference,out,source,frame_id):
    try:
        hp.write_image(out/'current-source.png',source)
        cfg={'camera':{'id':'sc3336-1280x720-fixed-focus-v1','width':source.shape[1],'height':source.shape[0]},
             'image':str(out/'current-source.png'),'frame_id':frame_id,'mode':'auto',
             'orientation':'reference','orientation_reference':str(reference),'plane_file':'','detections_file':'','scene_clear':0}
        (out/'inspect.json').write_text(json.dumps(cfg,ensure_ascii=False),encoding='utf-8')
        result=subprocess.run([str(exe),'inspect',str(out/'inspect.json'),str(out/'geometry.json')],
            capture_output=True,text=True,encoding='utf-8',errors='replace',timeout=8,
            creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
        if result.returncode:
            raise RuntimeError((result.stderr or f'Geometry exit {result.returncode}')[-500:])
        geometry=json.loads((out/'geometry.json').read_text(encoding='utf-8'))
        if geometry['frame_id']!=frame_id:raise RuntimeError('Mismatched geometry frame')
        overlay=cv2.imdecode(np.frombuffer((out/'geometry.json.png').read_bytes(),np.uint8),1)
        if overlay is None:raise RuntimeError('Cannot decode geometry overlay')
        return geometry,overlay
    except (OSError,ValueError,RuntimeError,subprocess.SubprocessError,cv2.error) as exc:
        # A failed geometry frame must not stop YOLO or reuse old grid points.
        return {'geometry_valid':0,'orientation_valid':0,'error':str(exc)[-500:]},source.copy()


def display_detections(detections,inference_roi,display_roi):
    dx,dy=inference_roi[0]-display_roi[0],inference_roi[1]-display_roi[1]
    return [dict(d,bbox=[d['bbox'][0]+dx,d['bbox'][1]+dy,d['bbox'][2]+dx,d['bbox'][3]+dy])
            for d in detections]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--board',required=True)
    p.add_argument('--geometry-exe',type=Path,required=True)
    p.add_argument('--orientation-reference',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--roi',type=int,nargs=4,metavar=('X','Y','WIDTH','HEIGHT'),
                   help='Fixed camera crop for both views; defaults to the full camera frame')
    p.add_argument('--inference-roi',type=int,nargs=4,metavar=('X','Y','WIDTH','HEIGHT'),
                   help='Fixed source region sent to YOLO; independent of the display viewport')
    p.add_argument('--expected-sha',default='ce9c91f4668cb0ec58f836432546b7eb2aebfc675ba696ce9788968560dcbdd2')
    p.add_argument('--seconds',type=float,default=0)
    args=p.parse_args();out=args.out.resolve();out.mkdir(parents=True,exist_ok=True)
    exe=args.geometry_exe.resolve();reference=args.orientation_reference.resolve()
    if not exe.is_file() or not reference.is_file():raise ValueError('Geometry executable/reference missing')
    health=json.loads(hp.request(args.board,'/health'))
    if health['model_sha256']!=args.expected_sha:raise ValueError('Unexpected NPU model')
    (out/'npu-health.json').write_text(json.dumps(health,indent=2),encoding='utf-8')
    capture=hp.Capture(f'rtsp://{args.board}/live/0')
    session=datetime.now().strftime('%Y%m%d-%H%M%S');start=time.monotonic();last_number=-1;processed=0;last_report=start;last_begin=None;periods=[]
    roi=None;inference_roi=None
    try:
        with (out/'frames.jsonl').open('w',encoding='utf-8') as log:
            while not args.seconds or time.monotonic()-start<args.seconds:
                if capture.error:raise RuntimeError(capture.error)
                source,received,number=capture.latest()
                if source is None or number==last_number:time.sleep(.01);continue
                if time.monotonic()-received>1:raise RuntimeError('Camera frame stale')
                if roi is None:
                    roi=list(args.roi) if args.roi is not None else [0,0,source.shape[1],source.shape[0]]
                    x,y,w,h=roi
                    if x<0 or y<0 or w<=0 or h<=0 or x+w>source.shape[1] or y+h>source.shape[0]:
                        raise ValueError('Fixed ROI must be inside the camera frame')
                    inference_roi=list(args.inference_roi) if args.inference_roi is not None else list(roi)
                    ix,iy,iw,ih=inference_roi
                    if ix<x or iy<y or iw<=0 or ih<=0 or ix+iw>x+w or iy+ih>y+h:
                        raise ValueError('Inference ROI must be inside the fixed display ROI')
                last_number=number;begin=time.monotonic();frame_id=f'{session}-{number}-{processed}'
                if last_begin is not None:periods.append(begin-last_begin)
                last_begin=begin
                geometry,overlay=inspect_geometry(exe,reference,out,source,frame_id)
                geometry_ms=(time.monotonic()-begin)*1000
                # Keep the viewport independent of grid detection and its failure state.
                # Recomputing this crop per frame changes MJPEG aspect ratio and zoom.
                frame=hp.transform(source,roi,0)
                inference_frame=hp.transform(source,inference_roi,0)
                rgb,ratio,left,top=hp.letterbox(inference_frame)
                detections,meta=hp.decode(hp.request(args.board,'/infer',rgb,frame_id),ratio,left,top,inference_frame.shape,frame_id)
                detections=display_detections(detections,inference_roi,roi)
                fps=1/(sum(periods[-20:])/len(periods[-20:])) if periods else 0
                age=(time.monotonic()-received)*1000
                left_view=hp.render(frame,detections,meta,fps,frame_id,roi,0,age)
                x,y,w,h=roi;right_view=overlay[y:y+h,x:x+w].copy()
                right_view=cv2.copyMakeBorder(right_view,70,35,0,0,cv2.BORDER_CONSTANT,value=(24,32,45))
                label='90 grid points' if geometry.get('geometry_valid') else 'Board not found'
                direction='direction OK' if geometry.get('orientation_valid') else 'direction unconfirmed'
                cv2.putText(right_view,f'OpenCV 5 | {label} | {geometry_ms:.0f} ms',(12,26),cv2.FONT_HERSHEY_SIMPLEX,.52,(240,240,240),1,cv2.LINE_AA)
                cv2.putText(right_view,f'frame {processed} | {direction}',(12,54),cv2.FONT_HERSHEY_SIMPLEX,.47,(160,210,255),1,cv2.LINE_AA)
                cv2.putText(right_view,'Grid preview only - robot XY calibration not applied',(12,right_view.shape[0]-12),cv2.FONT_HERSHEY_SIMPLEX,.43,(180,200,215),1,cv2.LINE_AA)
                combined=np.hstack((left_view,np.full((left_view.shape[0],8,3),48,np.uint8),right_view))
                ok,jpg=cv2.imencode('.jpg',combined,[cv2.IMWRITE_JPEG_QUALITY,90]);assert ok
                ok,single=cv2.imencode('.jpg',left_view,[cv2.IMWRITE_JPEG_QUALITY,90]);assert ok
                hp.request(args.board,'/preview',single.tobytes(),frame_id)
                publish_pair(args.board,jpg.tobytes(),frame_id,geometry)
                record={'frame_id':frame_id,'source_size':[source.shape[1],source.shape[0]],'roi':roi,'geometry_valid':bool(geometry.get('geometry_valid')),
                        'inference_roi':inference_roi,'coordinates':'roi',
                        'orientation_valid':bool(geometry.get('orientation_valid')),'grid_points':len(geometry.get('points',[])),
                        'npu_ms':meta['npu_ms'],'geometry_ms':geometry_ms,'fps':fps,'detections':detections,
                        'geometry_error':geometry.get('error'),
                        'preview_size':[combined.shape[1],combined.shape[0]],'motor_commands':False}
                log.write(json.dumps(record,ensure_ascii=False)+'\n');log.flush()
                try:
                    (out/'latest-dual.jpg').write_bytes(jpg.tobytes())
                except OSError as exc:
                    print(json.dumps({'snapshot_error':str(exc)},ensure_ascii=True),flush=True)
                processed+=1
                if processed==1 or time.monotonic()-last_report>=15:
                    print(json.dumps({k:v for k,v in record.items() if k!='detections'}),flush=True);last_report=time.monotonic()
    finally:capture.close()


if __name__=='__main__':main()
