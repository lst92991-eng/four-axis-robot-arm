"""从用户已导入的STEP复制板框/主要器件实体；省略小SMD，仅作装配参考。"""
import json
from pathlib import Path
import build_bridge as b

ROOT=b.ROOT
info=json.loads((ROOT/'pcb-import-evidence.json').read_text('utf-8'))
sw=b.SW
doc=next(d for d in b.g(sw,'GetDocuments') if b.g(d,'GetTitle')=='3D_PCB_20260622_2026-10-09')
records=info['components'];names={r['name'] for r in records}
selected=[]
for r in records:
    if any(n.startswith(r['name']+'/') for n in names):continue
    a=r['box_mm'];area=(a[3]-a[0])*(a[4]-a[1])
    if '/Board~' in r['name'] or area>=40 or a[2]<-3:selected.append(r)
d,planes=b.new_part('reference_pcb')
count=0
for r in selected:
    parent=doc
    for segment in r['name'].split('/'):
        component=parent.GetComponentByName(segment)
        if component is None:raise RuntimeError('引用解析失败：'+r['name'])
        parent=b.g(component,'GetModelDoc2')
    for body in b.g(parent,'GetBodies2',0,False) or []:
        cp=b.g(body,'Copy')
        if not cp.ApplyTransform(b.g(component,'Transform2')):raise RuntimeError('实体变换失败')
        # 本机PIA已回读签名：object CreateFeatureFromBody3(object,bool,int)。
        f=d.CreateFeatureFromBody3(cp,False,0)
        if f is None:raise RuntimeError('实体导入失败')
        count+=1;f.Name='参考器件实体_'+str(count)
    print('REFERENCE',count,r['name'].split('/')[-1][:45],flush=True)
path=ROOT/'reference/pcb_major_components.SLDPRT'
path.parent.mkdir(exist_ok=True)
if d.SaveAs3(str(path),0,1)!=0:raise RuntimeError('参考主板保存失败')
(ROOT/'pcb-reference.json').write_text(json.dumps({'source_sha256':info['sha256'],'copied_components':[r['name'] for r in selected],'solid_bodies':count,'scope':'从用户STEP原实体复制板框及主要接插件/器件；省略小SMD，仅用于安装参考。完整包络校核仍使用pcb-import-evidence.json全部组件。'},ensure_ascii=False,indent=2),encoding='utf-8')
