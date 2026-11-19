"""Compare saved C++ preprocessing with the supplied v3 ONNX model on CPU.

This does not execute the RV1106 RKNN runtime or change/convert model weights.
The 64 manually read piece centers are a spot check, not an mAP benchmark.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import onnxruntime as ort

ROOT = Path(__file__).resolve().parents[1]
SYMBOLS = 'kabnrcpKABNRCP'
ANCHORS = np.array([[10,13,16,30,33,23], [30,61,62,45,59,119],
                    [116,90,156,198,373,326]],np.float32).reshape(3,3,2)
MODEL_SHA = 'fdf31b53da7bc2350936bb51fb6e53ddf0d5958aa36830a6493a187332458047'


def decode(outputs, roi):
    boxes, scores, classes = [], [], []
    for index,output in enumerate(outputs):
        size = (80,40,20)[index]
        values = output[0].transpose(1,2,0).reshape(size,size,3,19)
        class_ids = values[...,5:].argmax(axis=-1)
        confidence = values[...,4]*values[...,5:].max(axis=-1)
        y,x,anchor = np.nonzero(confidence>=.50)
        chosen = values[y,x,anchor]
        xy = (chosen[:,:2]*2-.5+np.stack((x,y),axis=1))*(8,16,32)[index]
        wh = (chosen[:,2:4]*2)**2*ANCHORS[index,anchor]
        boxes.extend(np.concatenate((xy-wh/2,xy+wh/2),axis=1))
        scores.extend(confidence[y,x,anchor])
        classes.extend(class_ids[y,x,anchor])
    boxes,scores = np.asarray(boxes),np.asarray(scores)
    order = np.argsort(-scores,kind='stable')
    keep = []
    while order.size:
        i = int(order[0]); keep.append(i)
        rest = order[1:]
        if not rest.size: break
        lo = np.maximum(boxes[i,:2],boxes[rest,:2])
        hi = np.minimum(boxes[i,2:],boxes[rest,2:])
        intersection = np.maximum(hi-lo,0).prod(axis=1)
        area = (boxes[i,2:]-boxes[i,:2]).prod()+(boxes[rest,2:]-boxes[rest,:2]).prod(axis=1)
        order = rest[intersection/np.maximum(area-intersection,1e-9)<=.45]
    x,y,w,h = roi
    ratio = min(640/w,640/h)
    left,top = (640-round(w*ratio))//2,(640-round(h*ratio))//2
    result = []
    for i in keep:
        b = boxes[i].copy()
        b[[0,2]] = ((b[[0,2]]-left)/ratio).clip(0,w)+x
        b[[1,3]] = ((b[[1,3]]-top)/ratio).clip(0,h)+y
        if b[2]>b[0] and b[3]>b[1]:
            result.append(dict(class_id=int(classes[i]),score=float(scores[i]),bbox=b.tolist()))
    return result


def grade(detections, truth, radius):
    pairs = []
    for i,(symbol,x,y) in enumerate(truth):
        for j,d in enumerate(detections):
            b = d['bbox']
            distance = ((.5*(b[0]+b[2])-x)**2+(.5*(b[1]+b[3])-y)**2)**.5
            if distance<=radius: pairs.append((distance,i,j))
    used_truth,used_predictions = set(),set()
    details = []
    for distance,i,j in sorted(pairs):
        if i in used_truth or j in used_predictions: continue
        used_truth.add(i); used_predictions.add(j)
        actual = SYMBOLS[detections[j]['class_id']]
        details.append(dict(truth=truth[i],prediction=actual,correct=actual==truth[i][0],distance_px=distance))
    return dict(expected=len(truth),detected=len(detections),matched=len(details),
                correctly_identified=sum(d['correct'] for d in details),
                missing=[truth[i] for i in range(len(truth)) if i not in used_truth],
                extra_predictions=len(detections)-len(used_predictions),details=details)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('model',type=Path)
    parser.add_argument('before',type=Path)
    parser.add_argument('after',type=Path)
    parser.add_argument('output',type=Path)
    args = parser.parse_args()
    digest = hashlib.sha256(args.model.read_bytes()).hexdigest()
    if digest!=MODEL_SHA: raise RuntimeError('Expected the supplied v3 ONNX; weights must not change during review')
    options = ort.SessionOptions()
    options.intra_op_num_threads = 2
    options.inter_op_num_threads = 1
    session = ort.InferenceSession(str(args.model),sess_options=options,providers=['CPUExecutionProvider'])
    corpus = json.loads((ROOT/'tests/fixtures/vision/review-corpus.json').read_text(encoding='utf-8'))['cases']
    manual = json.loads((ROOT/'tests/fixtures/vision/visual-checks.json').read_text(encoding='utf-8'))
    truth = {c['id']:c['pieces'] for c in manual['cases']}
    all_results,comparisons,grades = {},[],[]
    args.output.mkdir(parents=True,exist_ok=True)
    for c in corpus:
        ident = c['id']; result = {}
        input_hashes = []
        for label,directory in [('before',args.before),('after',args.after)]:
            raw = (directory/(ident+'.rgb')).read_bytes()
            input_hashes.append(hashlib.sha256(raw).hexdigest())
            tensor = np.frombuffer(raw,np.uint8).reshape(640,640,3).transpose(2,0,1)[None].astype(np.float32)/255
            result[label] = decode(session.run(None,{'rgb':tensor}),c['roi'])
        unchanged = result['before']==result['after'] and input_hashes[0]==input_hashes[1]
        if not unchanged: raise RuntimeError('CPU inference changed: '+ident)
        comparisons.append(dict(id=ident,identical=True,detections=len(result['after'])))
        if ident in truth:
            grades.append(dict(id=ident,backend='v3 ONNX CPU',
                before=grade(result['before'],truth[ident],manual['matching_radius_px']),
                after=grade(result['after'],truth[ident],manual['matching_radius_px'])))
        all_results[ident] = result
    npu = json.loads((args.after/'npu-v3.json').read_text(encoding='utf-8'))['detections']
    report = dict(model_sha256=digest,provider=session.get_providers(),
        comparisons=comparisons,visual_spot_checks=grades,
        recorded_v3_npu_spot_check=grade(npu,truth['npu-v3'],manual['matching_radius_px']),
        limitations=['Only 64 visually reviewed pieces in two saved scenes; no bounding-box mAP.',
            'CPU inference uses the supplied floating-point ONNX, not the RV1106 INT8 runtime.',
            'Recorded NPU tensors verify host processing and do not validate a new NPU execution.'])
    for name,data in [('accuracy.json',report),('cpu-predictions.json',all_results)]:
        (args.output/name).write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(frames=len(comparisons),detections=sum(c['detections'] for c in comparisons),
        identical=True,visual_correct=sum(g['after']['correctly_identified'] for g in grades),
        visual_total=sum(g['after']['expected'] for g in grades),
        recorded_npu_correct=report['recorded_v3_npu_spot_check']['correctly_identified'])))


if __name__=='__main__':
    main()
