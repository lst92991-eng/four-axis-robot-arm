import sys,json,math,faulthandler
faulthandler.dump_traceback_later(20,repeat=True)
from pathlib import Path
import pythoncom
from win32com.client import VARIANT
ROOT=Path(__file__).resolve().parents[2];OUT=ROOT/'hardware/xiangqi30-old195275-2026-10-12'
sys.path.insert(0,'C:/Users/lst/.codex/tools/solidworks-automation-mcp/scripts')
from sw_connect import win32com_client,get_com_member as g,save_document,create_empty_dispatch_variant
from sw_review import run_review
sw=win32com_client.GetActiveObject('SldWorks.Application')
sw=win32com_client.dynamic.DumbDispatch(sw._oleobj_,'SldWorks.Application')
asm=sw.GetOpenDocumentByName(str(OUT/'native/SCARA_195_275_CHESS30.SLDASM'))
assert asm is not None
seen=set();metrics=[]
for c in g(asm,'GetComponents',True):
    p=Path(g(c,'GetPathName'))
    if not p.name.startswith('piece_'):continue
    if str(p) not in seen:
        seen.add(str(p));m=g(c,'GetModelDoc2')
        print('Editing',p.name,flush=True)
        sw.ActivateDoc3(str(p),False,0,VARIANT(pythoncom.VT_BYREF|pythoncom.VT_I4,0))
        m.ClearSelection2(True);assert m.FeatureByName('草图2').Select2(False,0)
        print('Opening sketch',flush=True);m.EditSketch()
        sketch=g(m.FeatureByName('草图2'),'GetSpecificFeature2')
        texts=g(sketch,'GetSketchTextSegments');assert len(texts)==1
        t=texts[0];fmt=g(t,'GetTextFormat');fmt.CharHeight=.012
        print('Formatting',flush=True);assert t.SetTextFormat(False,fmt)
        assert t.SetCoordinates(-.009,-.0085,0)
        print('Exiting sketch',flush=True);m.SketchManager.InsertSketch(True)
        assert m.ForceRebuild3(False)
        box=list(m.GetPartBox(True))
        assert max(abs(box[0]),abs(box[1]),abs(box[3]),abs(box[4]))<=.015001,(p,box)
        print('Saving',flush=True);assert save_document(m)
        metrics.append({'file':p.name,'bbox_mm':[v*1000 for v in box]})
    sw.ActivateDoc3(str(OUT/'native/SCARA_195_275_CHESS30.SLDASM'),False,0,VARIANT(pythoncom.VT_BYREF|pythoncom.VT_I4,0))
    asm.ClearSelection2(True);assert c.Select4(False,create_empty_dispatch_variant(),False)
    asm.UnfixComponent();tr=g(c,'Transform2');a=list(g(tr,'ArrayData'));a[11]=1.0081
    tr.ArrayData=VARIANT(pythoncom.VT_ARRAY|pythoncom.VT_R8,a);c.Transform2=tr
    asm.FixComponent()
assert len(seen)==14
asm.ClearSelection2(True);assert asm.ForceRebuild3(False);assert save_document(asm)
(OUT/'piece-dimensions.json').write_text(json.dumps(metrics,indent=2),'utf-8')
review,path=run_review(asm,str(OUT/'preview'),'assembly',expected_outputs=[str(OUT/'native/SCARA_195_275_CHESS30.SLDASM')])
print('Revised review',review['evaluation'],flush=True)
f=[v/math.sqrt(2.64) for v in (1,-1,.8)];r=[2**-.5,2**-.5,0]
u=[f[1]*r[2]-f[2]*r[1],f[2]*r[0]-f[0]*r[2],f[0]*r[1]-f[1]*r[0]]
arr=[x for k in range(3) for x in (r[k],u[k],f[k])]+[0.,0.,0.,1.,0.,0.,0.]
mu=g(sw,'GetMathUtility');mu._FlagAsMethod('CreateTransform')
asm.ActiveView.Orientation3=mu.CreateTransform(VARIANT(pythoncom.VT_ARRAY|pythoncom.VT_R8,arr))
asm.ViewZoomtofit2();asm.GraphicsRedraw2()
assert asm.SaveBMP(str(OUT/'preview/assembly_z_up.bmp'),1800,1200)
assert save_document(asm)
