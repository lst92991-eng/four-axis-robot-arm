import sys,json,hashlib,zipfile,shutil
from pathlib import Path
from PIL import Image
ROOT=Path(__file__).resolve().parents[2];OUT=ROOT/'hardware/xiangqi30-old195275-2026-10-12'
DELIVERY=OUT/'delivery-verified';DEST=DELIVERY/'SolidWorks';NATIVE=OUT/'native';HOLD=OUT/'native_reopen_hold'
sys.path.insert(0,'C:/Users/lst/.codex/tools/solidworks-automation-mcp/scripts')
from sw_connect import win32com_client,get_com_member as g,open_document
sw=win32com_client.GetActiveObject('SldWorks.Application')
sw=win32com_client.dynamic.DumbDispatch(sw._oleobj_,'SldWorks.Application')
owned=[]
for d in g(sw,'GetDocuments'):
    p=g(d,'GetPathName')
    if p and Path(p).is_relative_to(NATIVE):owned.append(g(d,'GetTitle'))
for title in owned:sw.CloseDoc(title)
sw._FlagAsMethod('SetCurrentWorkingDirectory')
assert sw.SetCurrentWorkingDirectory(str(DEST))
assert NATIVE.resolve().parent==OUT.resolve() and HOLD.resolve().parent==OUT.resolve()
assert not HOLD.exists()
NATIVE.rename(HOLD)
try:
    target=DEST/'SCARA_195_275_CHESS30.SLDASM'
    m=open_document(sw,str(target),silent=True,read_only=True,raise_on_error=True)
    top=list(g(m,'GetComponents',True) or [])
    assert len(top)==38,('top component count',len(top))
    rows=[];outside=[];missing=[]
    for c in g(m,'GetComponents',False):
        p=g(c,'GetPathName');virtual=bool(g(c,'IsVirtual'))
        suppressed=g(c,'GetSuppression')==0
        rows.append({'name':g(c,'Name2'),'path':p,'virtual':virtual,'suppressed':suppressed})
        if not virtual and p:
            if not Path(p).is_relative_to(DEST):outside.append(p)
            if not Path(p).exists():missing.append(p)
        if not suppressed:assert g(c,'GetModelDoc2') is not None,('unresolved',p)
    assert not outside and not missing,(outside,missing)
    report=json.loads((DELIVERY/'validation.json').read_text('utf-8'))
    review=json.loads((OUT/'preview/assembly_review_report.json').read_text('utf-8'))
    new=[x for x in review['interference']['items'] if any('/native/chess/' in c.get('path','').replace('\\','/') for c in x['components'])]
    assert not new
    report.update({'relocated_reopen':True,'original_native_folder_hidden_during_check':True,'top_components':len(top),
                   'all_components':len(rows),'outside_package_references':outside,'missing_files':missing,
                   'new_chess_interferences':0,'original_arm_interferences':review['interference']['interference_count'],
                   'original_arm_warning':'reference warning code 1; inherited internal assembly, not silently removed',
                   'component_references':rows})
    (DELIVERY/'validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),'utf-8')
finally:
    assert HOLD.resolve().parent==OUT.resolve() and not NATIVE.exists()
    HOLD.rename(NATIVE)
for bmp in (DELIVERY/'preview').glob('*.bmp'):
    Image.open(bmp).save(bmp.with_suffix('.png'))
# 更新显示用薄油墨的0.1mm间隔，和实际保存的变换一致。
p=DELIVERY/'assembly-layout.json';layout=json.loads(p.read_text('utf-8'))
for c in layout['components']:
    if '/piece_' in c['file'].replace('\\','/'):c['transform'][11]=1.0081
layout['board']['piece_display_clearance_mm']=.1
p.write_text(json.dumps(layout,ensure_ascii=False,indent=2),'utf-8')
shutil.copy2(OUT/'piece-dimensions.json',DELIVERY/'piece-dimensions.json')
files=[p for p in DELIVERY.rglob('*') if p.is_file() and not p.name.startswith('~$') and p.suffix.lower()!='.bmp' and p.name!='SHA256SUMS.json']
manifest={p.relative_to(DELIVERY).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(files)}
(DELIVERY/'SHA256SUMS.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),'utf-8')
files.append(DELIVERY/'SHA256SUMS.json')
archive=ROOT/'output/SCARA_195-275_30mm_chess_2026-10-12.zip'
with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
    for f in files:z.write(f,'SCARA_30mm_chess/'+f.relative_to(DELIVERY).as_posix())
with zipfile.ZipFile(archive) as z:assert z.testzip() is None
print(json.dumps({'zip':str(archive),'bytes':archive.stat().st_size,'files':len(files),
                  'sha256':hashlib.sha256(archive.read_bytes()).hexdigest(),'reopen':True},ensure_ascii=False),flush=True)
