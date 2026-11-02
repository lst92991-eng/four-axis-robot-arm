import sys
from pathlib import Path
import pythoncom
from win32com.client import VARIANT
ROOT=Path(__file__).resolve().parents[2]
D=ROOT/'hardware/xiangqi30-old195275-2026-10-12/delivery-front-2026-10-13/SolidWorks'
sys.path.insert(0,'C:/Users/lst/.codex/tools/solidworks-automation-mcp/scripts')
from sw_connect import win32com_client,get_com_member as g,save_document
s=win32com_client.GetActiveObject('SldWorks.Application');s=win32com_client.dynamic.DumbDispatch(s._oleobj_,'SldWorks.Application')
m=s.GetOpenDocumentByName(str(D/'SCARA_FRONT_195_275_CHESS30.SLDASM'));assert m is not None
m.ResolveAllLightWeightComponents(False)
assert m.SetReadOnlyState(False)
virtual=[c for c in g(m,'GetComponents',False) if g(c,'IsVirtual') and Path(g(c,'GetPathName')).suffix.lower()=='.sldasm']
assert len(virtual)==1
c=virtual[0];parent=g(g(c,'GetParent'),'GetModelDoc2')
assert Path(g(parent,'GetPathName')).is_relative_to(D)
assert parent.SetReadOnlyState(False)
s.ActivateDoc3(g(parent,'GetPathName'),False,0,VARIANT(pythoncom.VT_BYREF|pythoncom.VT_I4,0))
c=next(v for v in g(parent,'GetComponents',True) if g(v,'IsVirtual') and Path(g(v,'GetPathName')).suffix.lower()=='.sldasm')
print('Virtual resolve',c.SetSuppression2(2),bool(g(c,'GetModelDoc2')),flush=True)
assert c.SaveVirtualComponent(str(D/'OLD_DRIVE_ASSEMBLY.SLDASM'))
print('Virtual drive saved as independent packaged assembly',flush=True)
assert save_document(parent);assert save_document(m)
titles=[g(d,'GetTitle') for d in (g(s,'GetDocuments') or []) if g(d,'GetPathName') and Path(g(d,'GetPathName')).is_relative_to(D)]
for title in titles:s.CloseDoc(title)
drive=D/'OLD_DRIVE_ASSEMBLY.SLDASM'
raw=list(s.GetDocumentDependencies2(str(drive),False,False,False) or [])
for i in range(1,len(raw),2):
    old=raw[i];new=D/Path(old).name
    assert new.exists()
    if Path(old)!=new:assert s.ReplaceReferencedDocument(str(drive),old,str(new))
print('Drive references rewritten inside package',flush=True)
