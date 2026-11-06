"""Temporary PC pre/postprocessing + real RV1106 NPU preview. No training/CAN.

Uses the supplied model's RGB letterbox, sigmoid heads and class-agnostic NMS.
The HTTP preview is served by board_npu.py for viewing in the Ubuntu VM.
"""
import argparse
from collections import Counter
from datetime import datetime
import http.client
import json
import os
from pathlib import Path
import struct
import threading
import time

os.environ.setdefault('OPENCV_FFMPEG_CAPTURE_OPTIONS', 'rtsp_transport;tcp')
import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

FONT_PATH = Path('C:/Windows/Fonts/msyh.ttc')
CHINESE_FONT = ImageFont.truetype(str(FONT_PATH),16) if FONT_PATH.exists() else None

SYMBOLS = 'kabnrcpKABNRCP'
NAMES = ['将', '士', '象', '马', '车', '炮', '卒', '帅', '仕', '相', '马', '车', '炮', '兵']
ANCHORS = np.array([[10,13,16,30,33,23], [30,61,62,45,59,119],
                    [116,90,156,198,373,326]], dtype=np.float32).reshape(3,3,2)


def write_image(path, image):
    ok, encoded = cv2.imencode(path.suffix, image)
    if not ok:
        raise RuntimeError('Image encoding failed: ' + str(path))
    path.write_bytes(encoded.tobytes())


def request(board, path, data=None, frame_id=''):
    conn = http.client.HTTPConnection(board, 8088, timeout=10)
    try:
        conn.request('GET' if data is None else 'POST', path, body=data,
                     headers={'Content-Type':'application/octet-stream', 'X-Frame-ID':str(frame_id)})
        reply = conn.getresponse()
        body = reply.read()
        if reply.status != 200:
            raise RuntimeError(f'{path}: HTTP {reply.status}: {body[:300]!r}')
        return body
    finally:
        conn.close()


def letterbox(bgr):
    h, w = bgr.shape[:2]
    ratio = min(640/w, 640/h)
    nw, nh = round(w*ratio), round(h*ratio)
    dx, dy = (640-nw)/2, (640-nh)/2
    left, top = round(dx-.1), round(dy-.1)
    right, bottom = round(dx+.1), round(dy+.1)
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    rgb = cv2.resize(rgb, (nw, nh), interpolation=cv2.INTER_LINEAR)
    rgb = cv2.copyMakeBorder(rgb, top, bottom, left, right,
                            cv2.BORDER_CONSTANT, value=(114,114,114))
    assert rgb.shape == (640,640,3) and rgb.dtype == np.uint8
    return rgb.tobytes(), ratio, left, top


def nms(boxes, scores, iou_limit=.45):
    order = np.argsort(-scores, kind='stable')
    keep = []
    while order.size:
        i = int(order[0]); keep.append(i)
        rest = order[1:]
        if not rest.size:
            break
        lo = np.maximum(boxes[i,:2], boxes[rest,:2])
        hi = np.minimum(boxes[i,2:], boxes[rest,2:])
        intersection = np.maximum(hi-lo, 0).prod(axis=1)
        area_i = np.maximum(boxes[i,2:]-boxes[i,:2], 0).prod()
        area_r = np.maximum(boxes[rest,2:]-boxes[rest,:2], 0).prod(axis=1)
        iou = intersection/np.maximum(area_i+area_r-intersection, 1e-9)
        order = rest[iou <= iou_limit]
    return keep


def decode(packet, ratio, left, top, shape, expected_frame_id):
    header_len = struct.unpack_from('<I', packet)[0]
    assert 0 < header_len < 16384
    meta = json.loads(packet[4:4+header_len])
    assert meta['frame_id'] == str(expected_frame_id)
    offset = 4+header_len
    found_boxes, found_scores, found_classes = [], [], []
    for head, attr in enumerate(meta['outputs']):
        size = (80,40,20)[head]
        assert attr['dims'] == [1,size,size,57] and attr['type'] == 2 and attr['fmt'] == 1
        assert attr['w_stride'] in (0,size)
        count = size*size*57
        data = np.frombuffer(packet, dtype=np.int8, count=count, offset=offset)
        offset += count
        values = (data.astype(np.float32)-attr['zero_point'])*attr['scale']
        values = values.reshape(size,size,3,19)
        classes = values[...,5:].argmax(axis=-1)
        scores = values[...,4]*values[...,5:].max(axis=-1)
        y, x, anchor = np.nonzero(scores >= .50)
        if not len(x):
            continue
        chosen = values[y,x,anchor]
        xy = (chosen[:,:2]*2-.5 + np.stack((x,y),axis=1))*(8,16,32)[head]
        wh = (chosen[:,2:4]*2)**2 * ANCHORS[head,anchor]
        found_boxes.append(np.concatenate((xy-wh/2,xy+wh/2),axis=1))
        found_scores.append(scores[y,x,anchor])
        found_classes.append(classes[y,x,anchor])
    assert offset == len(packet)
    detections = []
    if found_boxes:
        boxes = np.concatenate(found_boxes)
        scores, classes = np.concatenate(found_scores), np.concatenate(found_classes)
        for i in nms(boxes,scores):
            b = boxes[i].copy()
            b[[0,2]] = (b[[0,2]]-left)/ratio
            b[[1,3]] = (b[[1,3]]-top)/ratio
            b[[0,2]] = b[[0,2]].clip(0,shape[1])
            b[[1,3]] = b[[1,3]].clip(0,shape[0])
            if b[2] <= b[0] or b[3] <= b[1]:
                continue
            cls = int(classes[i])
            detections.append(dict(class_id=cls,symbol=SYMBOLS[cls],
                name=('黑' if cls<7 else '红')+NAMES[cls],score=float(scores[i]),bbox=b.tolist()))
    return detections, meta


def transform(frame, roi, rotate):
    if roi:
        x,y,w,h = roi
        if min(x,y)<0 or min(w,h)<=0 or x+w>frame.shape[1] or y+h>frame.shape[0]:
            raise ValueError('ROI outside source frame')
        frame = frame[y:y+h,x:x+w]
    if rotate:
        rotations = {90:cv2.ROTATE_90_CLOCKWISE,180:cv2.ROTATE_180,270:cv2.ROTATE_90_COUNTERCLOCKWISE}
        frame = cv2.rotate(frame, rotations[rotate])
    return frame.copy()


def render(frame, detections, meta, fps, frame_id, roi, rotate, age):
    image = frame.copy()
    for d in detections:
        x1,y1,x2,y2 = map(round,d['bbox'])
        color = (65,85,245) if d['class_id']>=7 else (255,180,65)
        cv2.rectangle(image,(x1,y1),(x2,y2),color,2)
        if not CHINESE_FONT:
            label = f"{d['symbol']} {d['score']:.2f}"
            cv2.rectangle(image,(x1,max(0,y1-21)),(x1+82,y1),color,-1)
            cv2.putText(image,label,(x1+2,max(16,y1-5)),cv2.FONT_HERSHEY_SIMPLEX,.48,(255,255,255),1,cv2.LINE_AA)
    if CHINESE_FONT:
        pil = Image.fromarray(cv2.cvtColor(image,cv2.COLOR_BGR2RGB))
        draw = ImageDraw.Draw(pil)
        for d in detections:
            x,y = map(round,d['bbox'][:2])
            x,y = max(0,min(x,image.shape[1]-120)),max(0,y-25)
            color = (245,85,65) if d['class_id']>=7 else (65,180,255)
            draw.rectangle((x,y,x+120,y+25),fill=color)
            draw.text((x+2,y+1),f"{d['name']} {d['score']:.2f}",font=CHINESE_FONT,fill='white')
        image = cv2.cvtColor(np.asarray(pil),cv2.COLOR_RGB2BGR)
    image = cv2.copyMakeBorder(image,70,35,0,0,cv2.BORDER_CONSTANT,value=(24,32,45))
    text1 = f"RV1106 NPU | {len(detections)} detections | {fps:.1f} FPS | NPU {meta['npu_ms']:.1f} ms"
    text2 = f"frame {str(frame_id).split('-')[-1]} | host age {age:.0f} ms | score .50 / NMS .45 | rotation {rotate}"
    cv2.putText(image,text1,(12,26),cv2.FONT_HERSHEY_SIMPLEX,.53,(240,240,240),1,cv2.LINE_AA)
    cv2.putText(image,text2,(12,54),cv2.FONT_HERSHEY_SIMPLEX,.47,(160,210,255),1,cv2.LINE_AA)
    cv2.putText(image,'VISUAL TEST ONLY - lowercase BLACK / uppercase RED - no robot commands',
                (12,image.shape[0]-12),cv2.FONT_HERSHEY_SIMPLEX,.43,(180,200,215),1,cv2.LINE_AA)
    return image


class Capture:
    def __init__(self,url):
        self.lock,self.stopping = threading.Lock(),threading.Event()
        self.frame,self.timestamp,self.number,self.error = None,0,0,None
        self.cap = cv2.VideoCapture(url,cv2.CAP_FFMPEG,
            [cv2.CAP_PROP_OPEN_TIMEOUT_MSEC,5000,cv2.CAP_PROP_READ_TIMEOUT_MSEC,5000])
        if not self.cap.isOpened():
            raise RuntimeError('RTSP open failed')
        self.thread = threading.Thread(target=self.read,daemon=True)
        self.thread.start()

    def read(self):
        while not self.stopping.is_set():
            ok,frame = self.cap.read()
            if not ok:
                self.error='RTSP read failed';return
            with self.lock:
                self.frame,self.timestamp = frame,time.monotonic()
                self.number += 1

    def latest(self):
        with self.lock:
            return (None if self.frame is None else self.frame.copy(),self.timestamp,self.number)

    def close(self):
        self.stopping.set()
        self.thread.join(timeout=6)
        if not self.thread.is_alive():
            self.cap.release()


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--board',default='192.168.31.233')
    parser.add_argument('--rtsp',default='rtsp://192.168.31.233/live/0')
    parser.add_argument('--image',type=Path)
    parser.add_argument('--roi',type=int,nargs=4)
    parser.add_argument('--rotate',type=int,choices=(0,90,180,270),default=0)
    parser.add_argument('--seconds',type=float,default=0,help='0 means run until stopped')
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--publish',action='store_true')
    args=parser.parse_args();args.out.mkdir(parents=True,exist_ok=True)
    description=json.loads(request(args.board,'/health'))
    (args.out/'npu-health.json').write_text(json.dumps(description,indent=2),encoding='utf-8')
    session=datetime.now().strftime('%Y%m%d-%H%M%S')
    capture=None;started=time.monotonic();processed=0;last_frame=-1
    durations=[];npu_times=[];counts=[];last_report=started;error=None
    previous_begin=None;intervals=[]
    try:
        if args.image:
            source=cv2.imdecode(np.frombuffer(args.image.read_bytes(),dtype=np.uint8),cv2.IMREAD_COLOR)
            if source is None:raise ValueError('Cannot decode input image')
        else:
            capture=Capture(args.rtsp)
        with (args.out/'detections.jsonl').open('w',encoding='utf-8') as log:
            while True:
                if args.seconds and time.monotonic()-started>=args.seconds:break
                if capture:
                    if capture.error:raise RuntimeError(capture.error)
                    source,ts,number=capture.latest()
                    if source is None or number==last_frame:
                        time.sleep(.01);continue
                    if time.monotonic()-ts>1:raise RuntimeError('Source frame stale')
                    last_frame=number
                else:ts=time.monotonic();number=0
                begin=time.monotonic();frame_id=f'{session}-{number}-{processed}'
                if previous_begin is not None:intervals.append(begin-previous_begin)
                previous_begin=begin
                frame=transform(source,args.roi,args.rotate)
                rgb,ratio,left,top=letterbox(frame)
                response=request(args.board,'/infer',rgb,frame_id)
                detections,meta=decode(response,ratio,left,top,frame.shape,frame_id)
                age=(time.monotonic()-ts)*1000
                dt=time.monotonic()-begin
                durations.append(dt);npu_times.append(meta['npu_ms']);counts.append(len(detections))
                fps=1/(sum(intervals[-20:])/len(intervals[-20:])) if intervals else 0
                annotated=render(frame,detections,meta,fps,frame_id,args.roi,args.rotate,age)
                ok,encoded=cv2.imencode('.jpg',annotated,[cv2.IMWRITE_JPEG_QUALITY,90])
                if not ok:raise RuntimeError('JPEG encode failed')
                if args.publish:request(args.board,'/preview',encoded.tobytes(),frame_id)
                row=dict(frame_id=frame_id,source_size=[source.shape[1],source.shape[0]],
                    roi=args.roi,rotate_clockwise=args.rotate,detector_image_size=[frame.shape[1],frame.shape[0]],
                    npu_ms=meta['npu_ms'],processing_ms=dt*1000,host_age_ms=age,detections=detections)
                log.write(json.dumps(row,ensure_ascii=False)+'\n');log.flush()
                if processed==0:
                    write_image(args.out/'source.png',source)
                    write_image(args.out/'detector-input.png',frame)
                    write_image(args.out/'first-preview.jpg',annotated)
                    (args.out/'first-output.bin').write_bytes(response)
                    print('FIRST '+json.dumps(row,ensure_ascii=False),flush=True)
                processed+=1
                if time.monotonic()-last_report>=15:
                    print(json.dumps(dict(frames=processed,last_count=len(detections),fps=round(fps,2),
                        npu_ms=round(meta['npu_ms'],2),classes=dict(Counter(d['symbol'] for d in detections)))),flush=True)
                    write_image(args.out/'latest-preview.jpg',annotated)
                    last_report=time.monotonic()
                if args.image:break
    except KeyboardInterrupt:pass
    except Exception as exc:
        error=repr(exc);print('ERROR '+error,flush=True)
    finally:
        if capture:capture.close()
        result=dict(frames=processed,elapsed_s=time.monotonic()-started,error=error,
                    effective_fps=processed/(time.monotonic()-started),
                    npu_mean_ms=sum(npu_times)/len(npu_times) if npu_times else None,
                    npu_max_ms=max(npu_times) if npu_times else None,
                    detections_min=min(counts) if counts else None,detections_max=max(counts) if counts else None,
                    opencv_version=cv2.__version__,pipeline='PC preprocessing/postprocessing, real RV1106 NPU',
                    training=False,motor_commands=False,roi=args.roi,rotate_clockwise=args.rotate)
        (args.out/'run-summary.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
        print(json.dumps(result),flush=True)
    return 1 if error or not processed else 0


if __name__=='__main__':
    raise SystemExit(main())
