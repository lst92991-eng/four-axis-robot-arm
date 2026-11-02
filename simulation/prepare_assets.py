"""离线渲染素材：只从已有训练集人工标签裁棋子，不参与识别决策。"""
import hashlib,json,zipfile
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw
ROOT=Path(__file__).resolve().parent
archive=zipfile.ZipFile(ROOT/'assets/yolo14-v1.zip')
names='kabnrcpKABNRCP';candidates=[[] for _ in names]
for label in archive.namelist():
    if not label.startswith('labels/train/') or not label.endswith('.txt'):continue
    image='images/train/'+Path(label).stem+'.jpg'
    if image not in archive.namelist():continue
    import io
    picture=Image.open(io.BytesIO(archive.read(image))).convert('RGB');w,h=picture.size
    for line in archive.read(label).decode().splitlines():
        cls,x,y,bw,bh=map(float,line.split())
        # 人工核对：这一处标签写帅，实际图片是相。只排除渲染素材，不修改原数据集。
        if Path(image).name=='92.rf.b1ff3d95f5ac210d5ccd91f87c838b91.jpg' and int(cls)==7:continue
        bw*=w;bh*=h;x*=w;y*=h
        if min(bw,bh)<35 or max(bw,bh)/min(bw,bh)>1.25:continue
        score=min(bw,bh)**2/(1+4*abs(bw/bh-1))
        crop=picture.crop((round(x-bw/2),round(y-bh/2),round(x+bw/2),round(y+bh/2)))
        candidates[int(cls)].append((score,crop,image,line))
manifest=[]
for cls,values in enumerate(candidates):
    for index,(_,crop,image,line) in enumerate(sorted(values,key=lambda a:a[0],reverse=True)[:5]):
        crop=crop.resize((128,128),Image.LANCZOS).convert('RGBA')
        alpha=Image.new('L',(128,128));ImageDraw.Draw(alpha).ellipse((3,3,124,124),fill=255)
        crop.putalpha(alpha);path=ROOT/'assets'/f'{names[cls]}{index}.png';crop.save(path)
        manifest.append({'class':names[cls],'file':path.name,'source':image,'annotation':line,'sha256':hashlib.sha256(path.read_bytes()).hexdigest()})
assert len(manifest)==70,'Need five training-set sprites per class'
(ROOT/'assets/sprites.json').write_text(json.dumps({'scope':'训练集人工标注照片的渲染贴图；不是新增独立测试数据，也不是模型识别输出','excluded':'92.rf.b1ff3d95f5ac210d5ccd91f87c838b91.jpg class 7: picture is Elephant_red, manually checked','sprites':manifest},ensure_ascii=False,indent=2),encoding='utf-8')
data=np.load(ROOT/'assets/cad-collision-scan.npz');mask=data['connected'].astype(np.uint8)
assert mask.shape==(91,91)
(ROOT/'assets/workspace.bin').write_bytes(mask.tobytes())
print("已导出70张训练贴图和旧机CAD采样，config.json由用户单独修改，不覆盖。")
