import sys,json
from pathlib import Path
import numpy as np
import win32com.client,pythoncom
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'research'))
from audit_workspace_constraints import get,walk,safe
OUT=ROOT/'hardware/arm_equal_275/v1';sw=win32com.client.GetActiveObject('SldWorks.Application')
source=json.loads((OUT/'source-before-copy.json').read_text('utf-8'))['source']
copy=str(OUT/'native/EQ275_SCARA四轴机械臂总成.SLDASM')
result={}
for label,path in [('original',source),('copy',copy)]:
    doc=sw.GetOpenDocumentByName(path);data={'errors':[],'part_features':[]}
    c=next(c for c in doc.GetComponents(False) if Path(get(c,'GetPathName')).name in ['J1机械臂.SLDPRT','EQ275_J1机械臂.SLDPRT'])
    part=get(c,'GetModelDoc2');t=np.array(c.Transform2.ArrayData);r=t[:9].reshape(3,3);tr=t[9:12]
    data['transform']=t.tolist()
    for model in [doc,part]:
        for f,depth in walk(model):
            isw=win32com.client.VARIANT(pythoncom.VT_BYREF|pythoncom.VT_BOOL,False)
            code=f.GetErrorCode2(isw)
            if code:data['errors'].append({'doc':get(model,'GetTitle'),'feature':f.Name,'code':code,'warning':isw.value})
            if model is part and depth==0:
                row={'name':f.Name,'type':get(f,'GetTypeName2'),'cylinders':[]}
                faces=safe(f,'GetFaces') or []
                for face in faces:
                    s=get(face,'GetSurface')
                    if get(s,'IsCylinder'):
                        p=s.CylinderParams;axis=np.array(p[3:6])@r
                        if abs(axis[2])>.999:
                            xyz=(np.array(p[:3])@r*t[12]+tr)*1000
                            row['cylinders'].append([*xyz[:2],p[6]*1000])
                if row['cylinders']:data['part_features'].append(row)
    result[label]=data
(OUT/'feature-comparison-before-fix.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
sys.stdout.reconfigure(encoding='utf-8')
for label,d in result.items():
    print(label,'errorcount',len(d['errors']));print(json.dumps(d['errors'],ensure_ascii=False));print(json.dumps(d['part_features'],ensure_ascii=False))
