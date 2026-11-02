"""直接读取完成后的等臂装配副本，计算4度构型网格的实体碰撞工作区。"""
import sys,json,math,time,itertools,ast,gc,ctypes
from pathlib import Path
import numpy as np
import win32com.client,pythoncom
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'research'))
from audit_workspace_constraints import get
OUT=ROOT/'hardware/arm_equal_275/v1/analysis';OUT.mkdir(exist_ok=True)
MODEL=OUT.parent/'native/EQ275_SCARA四轴机械臂总成.SLDASM'
SW=win32com.client.GetActiveObject('SldWorks.Application');DOC=SW.GetOpenDocumentByName(str(MODEL))
assert DOC and Path(get(DOC,'GetPathName')).resolve()==MODEL.resolve()
MATH=get(SW,'GetMathUtility');MATH._FlagAsMethod('CreateTransform');MODELER=get(SW,'GetModeler')
verify=json.loads((OUT.parent/'final-model-verification.json').read_text('utf-8'))
assert not verify['part_errors'] and not verify['new_assembly_errors'] and not verify['component_position_mismatches']
S=np.r_[np.array(verify['joint_axes_cad_mm']['shoulder'])/1000,0]
GROUPS=json.loads((ROOT/'output/constrained-workspace-2026-10-09/rigid-groups.json').read_text('utf-8'))['groups']

def mt(rot,tr):return MATH.CreateTransform(win32com.client.VARIANT(pythoncom.VT_ARRAY|pythoncom.VT_R8,[*rot.ravel(),*tr,1,0,0,0]))
_ids={}
def raw_copy_transform(b,t=None):
    raw=b._oleobj_ if hasattr(b,'_oleobj_') else b
    if not _ids:_ids.update({n:raw.GetIDsOfNames(n) for n in ('Copy','ApplyTransform')})
    cp=raw.InvokeTypes(_ids['Copy'],0,1,(9,0),())
    if t is not None:assert cp.InvokeTypes(_ids['ApplyTransform'],0,1,(11,0),((9,1),),t._oleobj_)
    return cp
def clash(a,b):
    f1=win32com.client.VARIANT(pythoncom.VT_BYREF|pythoncom.VT_VARIANT,None)
    f2=win32com.client.VARIANT(pythoncom.VT_BYREF|pythoncom.VT_VARIANT,None)
    bs=win32com.client.VARIANT(pythoncom.VT_BYREF|pythoncom.VT_VARIANT,None)
    MODELER.CheckInterference(a,b,False,f1,f2,bs)
    return {'bodies':len(bs.value) if bs.value else 0}

# 复用已验证的碰撞扫描类，去除其旧原装配的导入和初始化副作用。
tree=ast.parse((ROOT/'research/arm-structure-optimization/compare_cad_variants.py').read_text('utf-8'))
nodes=[n for n in tree.body if isinstance(n,(ast.FunctionDef,ast.ClassDef)) and n.name in ['rz','overlap','record','Scene']]
exec(compile(ast.Module(body=nodes,type_ignores=[]),'shared_collision_scene','exec'),globals())

def memory_available_mb():
    class Status(ctypes.Structure):
        _fields_=[('length',ctypes.c_ulong),('load',ctypes.c_ulong)]+[(n,ctypes.c_ulonglong) for n in ['total_phys','avail_phys','total_page','avail_page','total_virtual','avail_virtual','extended']]
    s=Status();s.length=ctypes.sizeof(s);ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(s))
    return s.avail_page/1048576

def main():
    start=time.time();records=[];unassigned=[];renamed=[]
    components=list(DOC.GetComponents(False));names={c.Name2.replace('EQ275_','') for c in components}
    old=json.loads((ROOT/'output/constrained-workspace-2026-10-09/constraint-audit.json').read_text('utf-8'))['components']
    used=set()
    for c in components:
        n=c.Name2.replace('EQ275_','')
        if get(c,'IsSuppressed'):continue
        p=get(c,'GetModelDoc2')
        if not p or get(p,'GetType')!=1:continue
        if n in GROUPS:g=GROUPS[n]
        elif '/GT20同步轮-' in n:g=1
        elif '/GT60同步轮-' in n:g=2
        elif '/皮带1-2' in n:g=1
        else:
            # 原生阵列重建会重新编号实例；按同一零件、消失的旧实例与实际包围盒对应。
            bb=np.array(c.GetBox(False,False))*1000;choices=[]
            stem=Path(get(c,'GetPathName')).name.replace('EQ275_','')
            for oc in old:
                on=oc['name']
                if on in names or on in used or on not in GROUPS or Path(oc['path']).name!=stem:continue
                target=np.array(oc['box'])*1000;og=GROUPS[on]
                if og==2 or (og==1 and (-155.215865757977-(target[1]+target[4])/2)>140):target[[1,4]]-=80
                choices.append((float(np.max(np.abs(bb-target))),on,og))
            if not choices or min(choices)[0]>.02:unassigned.append(n);continue
            error,on,g=min(choices);used.add(on);renamed.append({'new_name':n,'original_instance':on,'group':g,'bbox_error_mm':error})
        for b in p.GetBodies2(0,False) or []:
            cp=get(b,'Copy');assert cp.ApplyTransform(c.Transform2)
            records.append(record(n,cp,g))
    assert not unassigned,unassigned
    (OUT/'pattern-instance-mapping.json').write_text(json.dumps(renamed,ensure_ascii=False,indent=2),encoding='utf-8')
    allowed={frozenset(pair) for pair in [('61mm光轴-2','FF丝杆支撑座[FF10]-2'),('61mm光轴-2','SKF_608-2ZZ-1'),('61mm光轴-2','8x10x3mm轴套-1'),('DLE42-L2BJ-长款50-2','方键条销-1'),('DLE42-L2BJ-长款50-2','8mm键槽法兰-1')]}
    excluded=set();initial=[];unknown=[]
    for i,a in enumerate(records):
        for j,b in enumerate(records[:i]):
            if a['group']==b['group'] or not overlap(a['box'],b['box']):continue
            if clash(a['body'],b['body'])['bodies']:
                pair=[a['name'],b['name']]
                known=frozenset(pair) in allowed or (any('/皮带1-2' in n for n in pair) and any('/GT60同步轮-' in n for n in pair))
                initial.append({'pair':pair,'accepted_joint_contact':known})
                if known:excluded.add((i,j))
                else:unknown.append(pair)
    (OUT/'initial-contact-audit.json').write_text(json.dumps({'body_count':len(records),'contacts':initial,'unknown':unknown},ensure_ascii=False,indent=2),encoding='utf-8')
    print('bodies',len(records),'initial contacts',json.dumps(initial,ensure_ascii=False),flush=True)
    assert not unknown,unknown
    scene=Scene('equal275',records,excluded,275,275,0,[])
    angles=np.arange(-180,181,4);good=np.zeros((len(angles),len(angles)),bool)
    for i,a in enumerate(angles):
        if memory_available_mb()<1800:
            np.savez_compressed(OUT/'checkpoint.npz',angles=angles,good=good,completed_rows=i)
            raise RuntimeError('内存余量不足，已保存检查点，停止继续分配临时实体')
        for j,b in enumerate(angles):good[i,j]=scene.pose(float(a),float(b))['free']
        if i%10==0:print('q1',int(a),'elapsed',round(time.time()-start),'checks',scene.calls,'memory_MB',round(memory_available_mb()),flush=True)
        gc.collect()
    visited=np.zeros_like(good);stack=[(45,45)]
    while stack:
        i,j=stack.pop()
        if not 0<=i<len(angles) or visited[i,j] or not good[i,j]:continue
        visited[i,j]=True;stack.extend([(i-1,j),(i+1,j),(i,(j-1)%len(angles)),(i,(j+1)%len(angles))])
    np.savez_compressed(OUT/'workspace.npz',angles=angles,good=good,connected=visited)
    summary={'model':str(MODEL),'links_mm':[275,275],'step_deg':4,'configurations':int(good.size),'collision_free':int(good.sum()),'connected':int(visited.sum()),
        'body_count':len(records),'body_pair_checks':scene.calls,'elapsed_seconds':time.time()-start,'initial_contacts':initial,
        'scope':'当前装配Z高度与末端R相对姿态，含修正后的同步轮与皮带外形；4度离散构型连通性，不是连续碰撞或实机安全验收。'}
    (OUT/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(summary,ensure_ascii=False),flush=True)
if __name__=='__main__':sys.stdout.reconfigure(encoding='utf-8');main()
