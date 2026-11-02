import sys,shutil,json
from pathlib import Path
import pythoncom
from win32com.client import VARIANT
ROOT=Path(__file__).resolve().parents[2]
D=ROOT/'hardware/xiangqi30-old195275-2026-10-12/delivery-front-2026-10-13/SolidWorks'
sys.path.insert(0,'C:/Users/lst/.codex/tools/solidworks-automation-mcp/scripts')
from sw_connect import win32com_client,get_com_member as g,open_document,save_document,create_empty_dispatch_variant
s=win32com_client.GetActiveObject('SldWorks.Application');s=win32com_client.dynamic.DumbDispatch(s._oleobj_,'SldWorks.Application')
sources={
 'OLD_DRIVE_ASSEMBLY.SLDASM':Path('C:/Users/lst/AppData/Local/Temp/swx45652/VC~~/SCARA四轴机械臂总成/装配体1^SCARA四轴机械臂总成.SLDASM'),
 'OLD_BELT.SLDPRT':Path('C:/Users/lst/AppData/Local/Temp/swx46404/VC~~/装配体1_SCARA四轴机械臂总成/皮带1-2^装配体1_SCARA四轴机械臂总成.SLDPRT')}
for name,p in sources.items():
    assert p.exists() and p.stat().st_size>0
    if not (D/name).exists():shutil.copy2(p,D/name)
drive=D/'OLD_DRIVE_ASSEMBLY.SLDASM'
raw=list(s.GetDocumentDependencies2(str(drive),False,False,False) or [])
for i in range(1,len(raw),2):
    old=raw[i]
    if '^' not in Path(old).name:assert s.ReplaceReferencedDocument(str(drive),old,str(D/Path(old).name))
parent=s.GetOpenDocumentByName(str(D/'OLD_SCARA四轴机械臂总成.SLDASM'))
assert parent is not None
parent.SetReadOnlyState(False)
s.ActivateDoc3(str(D/'OLD_SCARA四轴机械臂总成.SLDASM'),False,0,VARIANT(pythoncom.VT_BYREF|pythoncom.VT_I4,0))
c=next(c for c in g(parent,'GetComponents',True) if g(c,'IsVirtual') and Path(g(c,'GetPathName')).suffix.lower()=='.sldasm')
before=list(g(g(c,'Transform2'),'ArrayData'))
parent.ClearSelection2(True);assert c.Select4(False,create_empty_dispatch_variant(),False)
assert parent.ReplaceComponents2(str(drive),'',False,0,True),'Replace virtual drive'
print('Replaced virtual drive with packaged native assembly',flush=True)
assert save_document(parent)
main=s.GetOpenDocumentByName(str(D/'SCARA_FRONT_195_275_CHESS30.SLDASM'));assert save_document(main)
driveDoc=s.GetOpenDocumentByName(str(drive)) or open_document(s,str(drive),silent=True)
driveDoc.SetReadOnlyState(False)
s.ActivateDoc3(str(drive),False,0,VARIANT(pythoncom.VT_BYREF|pythoncom.VT_I4,0))
belts=[c for c in g(driveDoc,'GetComponents',True) if g(c,'IsVirtual')]
assert len(belts)==1
driveDoc.ClearSelection2(True);assert belts[0].Select4(False,create_empty_dispatch_variant(),False)
assert driveDoc.ReplaceComponents2(str(D/'OLD_BELT.SLDPRT'),'',False,0,True),'Replace virtual belt'
assert save_document(driveDoc);assert save_document(parent);assert save_document(main)
print('Drive and belt are now external package files',flush=True)
