"""Record the fixed camera and timestamp each decoded frame on the bench clock."""
import argparse
import csv
import json
import os
import time
from pathlib import Path

import cv2 as cv


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('out', type=Path)
    parser.add_argument('--url', required=True)
    parser.add_argument('--seconds', type=float, default=150)
    parser.add_argument('--crop-b2', action='store_true')
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    movie = args.out/'camera-b2.avi'
    stamp = args.out/'camera-b2-time.csv'
    if movie.exists() or stamp.exists():
        raise SystemExit('Recording already exists in this folder')
    os.environ['OPENCV_FFMPEG_CAPTURE_OPTIONS'] = 'rtsp_transport;tcp'
    cap = cv.VideoCapture(args.url, cv.CAP_FFMPEG, [cv.CAP_PROP_OPEN_TIMEOUT_MSEC, 5000,
                                                   cv.CAP_PROP_READ_TIMEOUT_MSEC, 3000])
    if not cap.isOpened():
        raise SystemExit('Camera open failed')
    writer = None
    frames = 0
    begin = time.monotonic()
    try:
        with stamp.open('x', newline='', encoding='ascii') as stream:
            table = csv.writer(stream)
            table.writerow(['frame','host_monotonic_s','stream_pts'])
            while time.monotonic()-begin < args.seconds and not (args.out/'stop-camera').exists():
                ok, frame = cap.read()
                host = time.monotonic()
                if not ok:
                    raise RuntimeError('Camera read failed')
                if writer is None:
                    height, width = frame.shape[:2]
                    if (width,height)!=(1280,720):
                        raise RuntimeError(f'Expected 1280x720; got {width}x{height}')
                    fps = cap.get(cv.CAP_PROP_FPS) or 30
                    x1,y1,x2,y2=(320,24,610,270) if args.crop_b2 else (0,0,width,height)
                    writer = cv.VideoWriter(str(movie),cv.VideoWriter_fourcc(*'MJPG'),fps,(x2-x1,y2-y1))
                    if not writer.isOpened():
                        raise RuntimeError('Could not open video writer')
                    (args.out/'camera-roi.json').write_text(json.dumps(dict(crop_xyxy=[x1,y1,x2,y2],
                                                                          nominal_fps=fps)))
                writer.write(frame[y1:y2,x1:x2])
                table.writerow([frames, f'{host:.6f}', cap.get(cv.CAP_PROP_PTS)])
                frames += 1
                if frames==1 or frames%30==0:
                    stream.flush()
                if frames==1:
                    if not cv.imwrite(str(args.out/'preflight.jpg'),frame):
                        raise RuntimeError('Could not save the initial camera view')
                    print('CAMERA_READY',args.out,host,flush=True)
    finally:
        if writer is not None:
            writer.release()
        cap.release()
    print('CAMERA_CLOSED',frames,flush=True)


if __name__=='__main__':
    main()
