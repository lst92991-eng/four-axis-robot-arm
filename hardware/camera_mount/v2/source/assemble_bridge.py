"""创建独立桥架装配及包含原机械臂/主板的参考场景，不保存原机械资料。"""
import hashlib
import json
import math
from pathlib import Path
import build_bridge as b
import pythoncom
from PIL import Image

ROOT=b.ROOT;SW=b.SW
MATH=b.g(SW,'GetMathUtility');MATH._FlagAsMethod('CreateTransform')
NULL=b.win32com_client.VARIANT(pythoncom.VT_DISPATCH,None)
I=[1,0,0,0,1,0,0,0,1]
B=[0,1,0,0,0,1,1,0,0]
placed=[]

def transform(values):return MATH.CreateTransform(b.win32com_client.VARIANT(pythoncom.VT_ARRAY|pythoncom.VT_R8,values))

def assembly():
    d=SW.NewDocument(r'C:\ProgramData\SOLIDWORKS\SOLIDWORKS 2026\templates\gb_assembly.asmdot',0,0.,0.)
    if d is None:raise RuntimeError('新建装配失败')
    d._FlagAsMethod('UnfixComponent','FixComponent','ViewZoomtofit2','GraphicsRedraw2')
    return d

def add(d,path,xyz,rotation=I,label=None):
    if SW.GetOpenDocumentByName(str(path)) is None:
        from sw_connect import open_document
        open_document(SW,str(path),read_only=True,silent=True,raise_on_error=True)
    error=b.win32com_client.VARIANT(pythoncom.VT_BYREF|pythoncom.VT_I4,0)
    SW.ActivateDoc3(b.g(d,'GetTitle'),False,0,error)
    c=d.AddComponent5(str(path),0,'',False,'',0,0,0)
    if c is None:raise RuntimeError('插入失败：'+str(path))
    values=rotation+[v/1000 for v in xyz]+[1,0,0,0]
    d.ClearSelection2(True);c.Select4(False,NULL,False);d.UnfixComponent()
    c.Transform2=transform(values)
    d.ClearSelection2(True);c.Select4(False,NULL,False);d.FixComponent()
    if label:c.Name2=label
    actual=list(b.g(b.g(c,'Transform2'),'ArrayData'))
    if max(abs(actual[k]-values[k]) for k in range(13))>1e-7:raise RuntimeError('变换未生效')
    placed.append({'path':str(path),'label':label,'transform':actual})
    return c

def save(d,name,native=True):
    d.ClearSelection2(True);d.ForceRebuild3(True)
    for ext in ((['.SLDASM'] if native else [])+['.step']):
        if d.SaveAs3(str(ROOT/(name+ext)),0,1)!=0:raise RuntimeError('保存失败：'+name+ext)
    f=[v/math.sqrt(2.64) for v in (1,-1,.8)];r=[2**-.5,2**-.5,0]
    u=[f[1]*r[2]-f[2]*r[1],f[2]*r[0]-f[0]*r[2],f[0]*r[1]-f[1]*r[0]]
    values=[x for k in range(3) for x in (r[k],u[k],f[k])]+[0.,0.,0.,1.,0.,0.,0.]
    d.ActiveView.Orientation3=transform(values)
    d.ViewZoomtofit2();d.GraphicsRedraw2()
    bmp=ROOT/(name+'.bmp')
    if not d.SaveBMP(str(bmp),2800,1800):raise RuntimeError('预览导出失败')
    Image.open(bmp).save(ROOT/(name+'.png'))
    if native and d.SaveAs3(str(ROOT/(name+'.SLDASM')),0,1)!=0:raise RuntimeError('保存视图失败')

def bridge():
    d=assembly()
    for hand,name in ((1,'01_clamp_right'),(-1,'02_clamp_left')):
        add(d,b.PARTS/(name+'.SLDPRT'),[60*hand,-130,524],B,name)
    for x in (-68.6,64.6):
        for y in (-130,50):add(d,b.PARTS/'03_truss_200.SLDPRT',[x,y,605],B,'镂空侧梁')
    add(d,b.PARTS/'04_camera_B_crossbar.SLDPRT',[0,250,650],I,'相机横梁')
    for y in (146,204):add(d,b.PARTS/'05_pcb_support.SLDPRT',[0,y,650],I,'主板横条')
    save(d,'camera_bridge')
    (ROOT/'assembly-layout.json').write_text(json.dumps({'units':'mm','components':placed,'camera_lens_front_reference_mm':[0,250,626.4],'pcb_bottom_z_mm':665},ensure_ascii=False,indent=2),encoding='utf-8')
    return d

def scene():
    project=ROOT.parents[2]
    info=json.loads((project/'research/scara-kinematics-geometry.json').read_text('utf-8-sig'))
    original=Path(info['assembly']);before=hashlib.sha256(original.read_bytes()).hexdigest()
    d=assembly();placed.clear()
    add(d,ROOT/'camera_bridge.SLDASM',[0,0,0],I,'省料双梁桥架')
    add(d,original,[.508123,-155.215866,-986],[-1,0,0,0,-1,0,0,0,1],'原机械臂_引用')
    old=ROOT.parent/'v1/parts'
    add(d,old/'reference_board.SLDPRT',[0,250,0],I,'参考棋盘_摆位待验收')
    add(d,old/'reference_camera_B.SLDPRT',[0,250,644],[1,0,0,0,-1,0,0,0,-1],'相机示意_非精确原厂模型')
    add(d,ROOT/'reference/pcb_major_components.SLDPRT',[-24.4811506223,204.0616573533,665],[0,-1,0,1,0,0,0,0,1],'用户主板_省略小SMD参考')
    # 整机场景仅导出STEP和预览，避免原装配CAM插件拦截新原生总装保存。
    save(d,'layout_check',native=False)
    after=hashlib.sha256(original.read_bytes()).hexdigest()
    if before!=after:raise RuntimeError('原文件字节变化')
    (ROOT/'scene-evidence.json').write_text(json.dumps({'original_sha256_before':before,'original_sha256_after':after,'components':placed,'scope':'静态参考场景；不是运动扫掠验收。主板保留原板框和主要器件，省略小SMD。'},ensure_ascii=False,indent=2),encoding='utf-8')

if __name__=='__main__':
    if len(b.sys.argv)>1 and b.sys.argv[1]=='scene':scene()
    else:bridge()
