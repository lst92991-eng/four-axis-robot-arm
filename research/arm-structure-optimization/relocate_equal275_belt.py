"""同步更新副本内的虚拟皮带总成，大同步轮中心随肘轴前移80mm。"""
import sys,json
from pathlib import Path
import numpy as np
import win32com.client,pythoncom
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'research'))
from audit_workspace_constraints import get
OUT=ROOT/'hardware/arm_equal_275/v1';NATIVE=OUT/'native';sw=win32com.client.GetActiveObject('SldWorks.Application')
def iv():return win32com.client.VARIANT(pythoncom.VT_BYREF|pythoncom.VT_I4,0)
def tolocal(world,parent):
    w,p=np.array(world),np.array(parent);rp=p[:9].reshape(3,3);rw=w[:9].reshape(3,3)
    return [*(rw@rp.T).ravel(),*((w[9:12]-p[9:12])@rp.T/p[12]),w[12]/p[12],0,0,0]
def main():
    asm=sw.GetOpenDocumentByName(str(NATIVE/'EQ275_SCARA四轴机械臂总成.SLDASM'));cs=list(asm.GetComponents(False))
    parent=next(c for c in cs if '装配体1' in c.Name2 and '/' not in c.Name2);sub=get(parent,'GetModelDoc2')
    assert 'EQ275_' in get(sub,'GetPathName')
    old=json.loads((ROOT/'output/constrained-workspace-2026-10-09/constraint-audit.json').read_text('utf-8'));orig={c['name']:c for c in old['components']}
    pname=parent.Name2.replace('EQ275_','');pt=orig[pname]['transform']
    mathutil=get(sw,'GetMathUtility');mathutil._FlagAsMethod('CreateTransform')
    def matrix(values):return mathutil.CreateTransform(win32com.client.VARIANT(pythoncom.VT_ARRAY|pythoncom.VT_R8,list(values)))
    feature=sub.FeatureByName('皮带1');definition=get(feature,'GetDefinition')
    before={'driving_length':definition.DrivingLength,'length_m':definition.BeltLength}
    null=win32com.client.VARIANT(pythoncom.VT_DISPATCH,None)
    assert definition.AccessSelections(sub,null)
    definition.DrivingLength=False
    assert feature.ModifyDefinition(definition,sub,null)
    parent.Transform2=matrix(pt)
    changes=[]
    for c in sub.GetComponents(False):
        stem=c.Name2.replace('EQ275_','')
        if stem not in ['GT20同步轮-1','GT60同步轮-1']:continue
        world=list(orig[pname+'/'+stem]['transform'])
        if stem=='GT60同步轮-1':world[10]-=.080
        local=tolocal(world,pt);c.Transform2=matrix(local)
        changes.append({'name':stem,'target_world_transform':world,'target_local_transform':local})
    sub._FlagAsMethod('ForceRebuild3');sub.ForceRebuild3(False)
    asm._FlagAsMethod('ForceRebuild3');asm.ForceRebuild3(False)
    after=[]
    for c in asm.GetComponents(False):
        if any(c.Name2.endswith('EQ275_'+n) for n in ['GT20同步轮-1','GT60同步轮-1']):after.append({'name':c.Name2,'transform':list(c.Transform2.ArrayData)})
    # 所有外部引用已验证是独立副本；同时保存其内部虚拟皮带零件。
    e,w=iv(),iv();assert asm.Save3(1|4,e,w)
    after_definition=get(feature,'GetDefinition')
    result={'belt_before':before,'belt_after':{'driving_length':after_definition.DrivingLength,'length_m':after_definition.BeltLength},'changes':changes,'root_context_after':after,'save_errors':e.value,'save_warnings':w.value}
    (OUT/'belt-relocation.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False),flush=True)
if __name__=='__main__':sys.stdout.reconfigure(encoding='utf-8');main()
