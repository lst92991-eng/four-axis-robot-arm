"""只读审计总装与子装配的所有配合，提取真实限位定义及引用。"""
import json,sys,math
from pathlib import Path
import win32com.client
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'output/constrained-workspace-2026-10-09'
OUT.mkdir(parents=True,exist_ok=True)
def get(o,n,*args):
    a=getattr(o,n)
    return a(*args) if callable(a) and not hasattr(a,'_oleobj_') else a
def safe(o,n,*args):
    try:return get(o,n,*args)
    except Exception:return None
def walk(doc):
    def children(f,depth):
        yield f,depth
        sf=safe(f,'GetFirstSubFeature')
        while sf:
            yield from children(sf,depth+1)
            sf=safe(sf,'GetNextSubFeature')
    f=get(doc,'FirstFeature')
    while f:
        yield from children(f,0)
        f=safe(f,'GetNextFeature')
def audit():
    sw=win32com.client.GetActiveObject('SldWorks.Application')
    ref=json.loads((ROOT/'research/scara-kinematics-geometry.json').read_text('utf-8'))
    d=sw.GetOpenDocumentByName(ref['assembly'])
    if not d:raise RuntimeError('原总装未打开')
    docs=[d]; comps=[]
    for c in d.GetComponents(False):
        p=get(c,'GetModelDoc2')
        comps.append({'name':c.Name2,'path':safe(c,'GetPathName'),'fixed':safe(c,'IsFixed'),
            'suppressed':safe(c,'IsSuppressed'),'transform':list(c.Transform2.ArrayData),
            'box':list(c.GetBox(False,False) or []),'doc_type':safe(p,'GetType') if p else None})
        if p and get(p,'GetType')==2 and get(p,'GetPathName') not in [get(x,'GetPathName') for x in docs]:docs.append(p)
    result={'root':ref['assembly'],'save_before':get(d,'GetSaveFlag'),'documents':[],'components':comps}
    for doc in docs:
        rows=[]; seen=set()
        for f,depth in walk(doc):
            t=safe(f,'GetTypeName2'); name=f.Name
            if not str(t).startswith('Mate') or (name,t) in seen:continue
            seen.add((name,t))
            row={'name':name,'type':t,'depth':depth,'suppressed':safe(f,'IsSuppressed')}
            definition=safe(f,'GetDefinition')
            if definition:
                vals={}
                for field in ['IsAdvancedMate','MinimumAngle','MaximumAngle','Angle','MinimumDistance','MaximumDistance','Distance','MateAlignment','FlipDimension','LockRotation']:
                    val=safe(definition,field)
                    if val is not None and isinstance(val,(str,int,float,bool)):vals[field]=val
                row['definition']=vals
            m=safe(f,'GetSpecificFeature2')
            if m:
                row['mate_type']=safe(m,'Type');row['alignment']=safe(m,'Alignment')
                n=safe(m,'GetMateEntityCount')
                if isinstance(n,int):
                    es=[]
                    for i in range(n):
                        e=m.MateEntity(i);c=safe(e,'ReferenceComponent');p=safe(e,'EntityParams')
                        es.append({'component':c.Name2 if c else None,'reference_type':safe(e,'ReferenceType'),'params':list(p) if p is not None else None})
                    row['entities']=es
            rows.append(row)
        result['documents'].append({'title':get(doc,'GetTitle'),'path':get(doc,'GetPathName'),'mates':rows})
    result['save_after']=get(d,'GetSaveFlag')
    (OUT/'constraint-audit.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    for dd in result['documents']:
        print(dd['title'],len(dd['mates']))
        for row in dd['mates']:
            if 'Angle' in row['type'] or 'Limit' in row['type'] or row.get('definition',{}).get('LockRotation'):
                print(json.dumps(row,ensure_ascii=False))
    print('saved:',result['save_before'],result['save_after'])
if __name__=='__main__':
    sys.stdout.reconfigure(encoding='utf-8');audit()
