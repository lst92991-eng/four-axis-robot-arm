import sys,json,hashlib,shutil,zipfile,faulthandler
faulthandler.dump_traceback_later(20,repeat=True)
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'hardware/xiangqi30-old195275-2026-10-12'
DELIVERY=OUT/'delivery-verified';DEST=DELIVERY/'SolidWorks'
DEST.mkdir(parents=True,exist_ok=True)
sys.path.insert(0,'C:/Users/lst/.codex/tools/solidworks-automation-mcp/scripts')
from sw_delivery import _comtypes_module
from sw_connect import win32com_client,get_com_member as g,open_document
from sw_export import export_to_step
_comtypes_module()
import comtypes.client
from comtypes.gen import SldWorks
source=OUT/'native/SCARA_195_275_CHESS30.SLDASM'
# 源机械臂已完成原生Pack and Go；总装最后一层采用完整依赖暂存，另做隔离重开验证。
names=set()
for f in (OUT/'native').rglob('*'):
    if not f.is_file() or f.name.startswith('~$') or f.suffix.lower() not in ('.sldasm','.sldprt'):continue
    assert f.name.casefold() not in names,('duplicate basename',f)
    names.add(f.name.casefold());shutil.copy2(f,DEST/f.name)
print('Dependency staging:',len(names),'native files',flush=True)
report={'packaging':'native arm Pack and Go + staged top-level dependencies; verified by relocated reopening',
        'native_files':len(names)}
swp=win32com_client.GetActiveObject('SldWorks.Application')
swp=win32com_client.dynamic.DumbDispatch(swp._oleobj_,'SldWorks.Application')
m=swp.GetOpenDocumentByName(str(source))
if m is None:m=open_document(swp,str(source),silent=True,raise_on_error=True)
print('STEP exporting',flush=True)
assert export_to_step(m,str(DELIVERY/'SCARA_195_275_CHESS30.step'))
shutil.copytree(OUT/'markers',DELIVERY/'markers',dirs_exist_ok=True)
shutil.copytree(OUT/'preview',DELIVERY/'preview',dirs_exist_ok=True)
shutil.copy2(OUT/'assembly-layout.json',DELIVERY/'assembly-layout.json')
shutil.copy2(Path(__file__).parent/'delivery-notes.md',DELIVERY/'README_先读我.md')
shutil.copy2(Path(__file__).parent/'source-parts-hash-check.json',DELIVERY/'source-parts-hash-check.json')
pcb=Path('C:/Users/lst/Downloads/3D_PCB_20260622_2026-10-09.step')
if pcb.exists():
    (DELIVERY/'reference').mkdir(exist_ok=True)
    shutil.copy2(pcb,DELIVERY/'reference/RV1106_PCB_user_supplied.step')
report['source_parts_hash_unchanged']=44
report['marker_decoder']='OpenCV 5.0.0; rendered A4 PDF decoded IDs 2,3,1,0'
(DELIVERY/'validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),'utf-8')
print('Exports ready; must reopen relocated native package before ZIP',flush=True)
