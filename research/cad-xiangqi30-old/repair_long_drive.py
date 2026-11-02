import sys,json,shutil
from pathlib import Path
import pythoncom
from win32com.client import VARIANT
ROOT=Path(__file__).resolve().parents[2]
D=ROOT/'hardware/xiangqi30-long275275-2026-10-13/SolidWorks'
sys.path.insert(0,'C:/Users/lst/.codex/tools/solidworks-automation-mcp/scripts')
from sw_connect import win32com_client,get_com_member as g,open_document,save_document,create_empty_dispatch_variant
s=win32com_client.GetActiveObject('SldWorks.Application');s=win32com_client.dynamic.DumbDispatch(s._oleobj_,'SldWorks.Application')
parent=s.GetOpenDocumentByName(str(D/'EQ275_SCARA四轴机械臂总成.SLDASM'));assert parent is not None
components=list(g(parent,'GetComponents',False))
drive_comp=next(c for c in components if bool(g(c,'IsVirtual')) and Path(g(c,'GetPathName')).suffix.lower()=='.sldasm')
belt_comp=next(c for c in components if bool(g(c,'IsVirtual')) and Path(g(c,'GetPathName')).suffix.lower()=='.sldprt')
drive=D/'EQ275_DRIVE_ASSEMBLY.SLDASM';belt=D/'EQ275_BELT.SLDPRT'
for c,target in ((drive_comp,drive),(belt_comp,belt)):
    source=Path(g(c,'GetPathName'));assert source.exists();shutil.copy2(source,target)
deps=list(s.GetDocumentDependencies2(str(drive),False,False,False) or [])
for prior in deps[1::2]:
    if '^' in Path(prior).name:continue
    target=D/('LONG_'+Path(prior).name);shutil.copy2(prior,target)
    assert s.ReplaceReferencedDocument(str(drive),prior,str(target))
parent.SetReadOnlyState(False)
s.ActivateDoc3(str(D/'EQ275_SCARA四轴机械臂总成.SLDASM'),False,0,VARIANT(pythoncom.VT_BYREF|pythoncom.VT_I4,0))
pose=list(g(g(drive_comp,'Transform2'),'ArrayData'))
parent.ClearSelection2(True);assert drive_comp.Select4(False,create_empty_dispatch_variant(),False)
assert parent.ReplaceComponents2(str(drive),'',False,0,True)
print('Replaced long-arm virtual drive',flush=True)
assert save_document(parent)
dd=s.GetOpenDocumentByName(str(drive)) or open_document(s,str(drive),silent=True)
dd.SetReadOnlyState(False)
s.ActivateDoc3(str(drive),False,0,VARIANT(pythoncom.VT_BYREF|pythoncom.VT_I4,0))
bc=next(c for c in g(dd,'GetComponents',True) if bool(g(c,'IsVirtual')))
dd.ClearSelection2(True);assert bc.Select4(False,create_empty_dispatch_variant(),False)
assert dd.ReplaceComponents2(str(belt),'',False,0,True)
assert save_document(dd);assert save_document(parent)
main=s.GetOpenDocumentByName(str(D/'SCARA_FRONT_275_275_CHESS30.SLDASM'))
assert save_document(main)
print('Repaired drive and belt in long-arm delivery copy',flush=True)
