import sys,json
from pathlib import Path
from PIL import Image
import numpy as np,pythoncom
from win32com.client import VARIANT
ROOT=Path(__file__).resolve().parents[2];OUT=ROOT/'hardware/xiangqi33-assemblies-2026-10-13'
label,kind=sys.argv[1:3];stage=next(r for r in json.loads((OUT/'stages.json').read_text('utf-8')) if r['label']==label)
folder=OUT/label;dest=folder/'preview';dest.mkdir(exist_ok=True)
sys.path.insert(0,'C:/Users/lst/.codex/tools/solidworks-automation-mcp/scripts')
from sw_connect import win32com_client,get_com_member as g
sw=win32com_client.GetActiveObject('SldWorks.Application');sw=win32com_client.dynamic.DumbDispatch(sw._oleobj_,'SldWorks.Application')
m=sw.GetOpenDocumentByName(stage['main']);assert m is not None
assert m.SaveBMP(str(dest/(kind+'.bmp')),1800,1400)
im=Image.open(dest/(kind+'.bmp')).convert('RGB');nonwhite=float(np.mean(np.min(np.asarray(im),axis=2)<220));assert nonwhite>.02
im.save(dest/(kind+'.png'));print('PREVIEW',label,kind,round(nonwhite,3),flush=True)
if kind=='isometric':
    mu=g(sw,'GetMathUtility');mu._FlagAsMethod('CreateTransform')
    arr=[-1.,0.,0.,0.,-1.,0.,0.,0.,1.,0.,0.,0.,1.,0.,0.,0.]
    m.ActiveView.Orientation3=mu.CreateTransform(VARIANT(pythoncom.VT_ARRAY|pythoncom.VT_R8,arr));m.ViewZoomtofit2();m.GraphicsRedraw2()
