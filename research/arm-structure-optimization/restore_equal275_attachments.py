import sys,json
from pathlib import Path
import numpy as np
import win32com.client,pythoncom
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'research'))
from audit_workspace_constraints import get
OUT=ROOT/'hardware/arm_equal_275/v1';sw=win32com.client.GetActiveObject('SldWorks.Application')
def iv():return win32com.client.VARIANT(pythoncom.VT_BYREF|pythoncom.VT_I4,0)
def main():
    a=sw.GetOpenDocumentByName(str(OUT/'native/EQ275_SCARA四轴机械臂总成.SLDASM'))
    old=json.loads((ROOT/'output/constrained-workspace-2026-10-09/constraint-audit.json').read_text('utf-8'));om={c['name']:c for c in old['components']}
    m=get(sw,'GetMathUtility');m._FlagAsMethod('CreateTransform');rows=[]
    for c in a.GetComponents(False):
        n=c.Name2.replace('EQ275_','')
        if n=='光轴固定环-1' or n.startswith('M3x4mm紧固螺丝-'):
            t=list(om[n]['transform'])
            if n=='光轴固定环-1' or int(n.rsplit('-',1)[1])>=5:t[10]-=.080
            c.Transform2=m.CreateTransform(win32com.client.VARIANT(pythoncom.VT_ARRAY|pythoncom.VT_R8,t))
            rows.append(n)
    a._FlagAsMethod('ForceRebuild3');a.ForceRebuild3(False)
    belt=[]
    for c in a.GetComponents(False):
        if '皮带1-2' in c.Name2:
            p=get(c,'GetModelDoc2');p._FlagAsMethod('ForceRebuild3');p.ForceRebuild3(False)
            belt.append({'name':c.Name2,'bbox_mm':[v*1000 for v in c.GetBox(False,False)]})
    a.ForceRebuild3(False);assert a.Save3(1|4,iv(),iv())
    (OUT/'attachment-and-belt-check.json').write_text(json.dumps({'restored_attachments':rows,'belt':belt},ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({'restored_attachments':rows,'belt':belt},ensure_ascii=False),flush=True)
if __name__=='__main__':sys.stdout.reconfigure(encoding='utf-8');main()
