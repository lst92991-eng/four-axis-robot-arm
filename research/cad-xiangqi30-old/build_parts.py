import sys,json,math
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'hardware/xiangqi30-old195275-2026-10-12/native/chess'
OUT.mkdir(parents=True,exist_ok=True)
sys.path.insert(0,'C:/Users/lst/.codex/tools/solidworks-automation-mcp/scripts')
from sw_connect import win32com_client,get_com_member as g,new_document,save_document
from sw_part import start_sketch,end_sketch,extrude_boss
from sw_sketch_quality import fully_define_sketch
from sw_appearance import set_document_appearance,set_feature_appearance,verify_appearance
from sw_review import save_preview
sw=win32com_client.GetActiveObject('SldWorks.Application')
def rectangle(m,x1,y1,x2,y2):
    m.SketchManager.AddToDB=True
    assert m.SketchManager.CreateCornerRectangle(x1/1000,y1/1000,0,x2/1000,y2/1000,0) is not None
    m.SketchManager.AddToDB=False
def extrude(m,depth,name,color=None,reverse=False,merge=True):
    sk=end_sketch(m)
    feature=extrude_boss(m,sk,depth/1000,direction=reverse,merge=merge)
    assert feature is not None,name
    feature.Name=name
    if color: set_feature_appearance(feature,color)
    return feature
def text(m,word,x,y,size):
    t=m.InsertSketchText(x/1000,y/1000,0,word,0,0,0,100,100)
    assert t is not None,'Sketch text'
    fmt=g(t,'GetTextFormat'); fmt.TypeFaceName='SimHei'; fmt.CharHeight=size/1000
    assert t.SetTextFormat(False,fmt),'Text font'
    return t
def finish(m,path):
    m.ClearSelection2(True)
    assert m.ForceRebuild3(False),'Rebuild error'
    assert save_document(m,str(path))
    sw.CloseDoc(g(m,'GetTitle'))
def disc(code,char):
    path=OUT/('piece_'+('red_' if code.isupper() else 'black_')+code+'.SLDPRT')
    if path.exists(): return
    m=new_document(sw,'part',max_documents=128)
    start_sketch(m,'Front Plane'); m.SketchManager.CreateCircleByRadius(0,0,0,.015)
    fully_define_sketch(m)
    extrude(m,8,'Diameter30_Height8',reverse=True)
    set_document_appearance(m,'#E5C99A')
    assert verify_appearance(m,'#E5C99A')['ok']
    ink='#B42420' if code.isupper() else '#171D24'
    start_sketch(m,'Front Plane'); text(m,char,-9,-8.5,12)
    extrude(m,.10,'Character_'+char,ink)
    start_sketch(m,'Front Plane')
    m.SketchManager.CreateCircleByRadius(0,0,0,.0131)
    m.SketchManager.CreateCircleByRadius(0,0,0,.0127)
    extrude(m,.08,'Ink_ring',ink)
    finish(m,path)
    print('Piece ready',code,char,flush=True)
def board():
    path=OUT/'board_315x350_pitch35.SLDPRT'
    if path.exists(): return
    m=new_document(sw,'part',max_documents=128)
    start_sketch(m,'Front Plane'); rectangle(m,-157.5,-175,157.5,175)
    fully_define_sketch(m); extrude(m,14,'Board315x350x14',reverse=True)
    set_document_appearance(m,'#E7C994')
    start_sketch(m,'Front Plane')
    for r in range(10):
        y=(r-4.5)*35; rectangle(m,-140,y-.16,140,y+.16)
    extrude(m,.05,'Ten_ranks','#222222')
    start_sketch(m,'Front Plane')
    for c in range(9):
        x=(c-4)*35
        if c in [0,8]: rectangle(m,x-.16,-157.5,x+.16,157.5)
        else:
            rectangle(m,x-.16,-157.5,x+.16,-17.5)
            rectangle(m,x-.16,17.5,x+.16,157.5)
    extrude(m,.06,'Nine_files_with_river','#222222')
    for slope in [-1,1]:
        start_sketch(m,'Front Plane')
        m.SketchManager.AddToDB=True
        for y in [-122.5,122.5]:
            a=(-35,y-slope*35); b=(35,y+slope*35)
            dx=.12; dy=-slope*.12
            ps=[(a[0]+dx,a[1]+dy),(b[0]+dx,b[1]+dy),(b[0]-dx,b[1]-dy),(a[0]-dx,a[1]-dy)]
            for i in range(4):
                p,q=ps[i],ps[(i+1)%4]; m.SketchManager.CreateLine(p[0]/1000,p[1]/1000,0,q[0]/1000,q[1]/1000,0)
        m.SketchManager.AddToDB=False
        extrude(m,.07,'Palace_diagonals_'+str(slope),'#222222')
    start_sketch(m,'Front Plane'); text(m,'楚河',-88,-8,16); text(m,'汉界',55,-8,16)
    extrude(m,.09,'River_name','#222222')
    finish(m,path); print('Board ready',flush=True)
def marker(id):
    path=OUT/f'aruco_id_{id}_60mm.SLDPRT'
    if path.exists(): return
    bits=json.loads((Path(__file__).parent/'markers.json').read_text('utf-8'))['markers'][id]
    m=new_document(sw,'part',max_documents=128)
    start_sketch(m,'Front Plane');rectangle(m,-30,-30,30,30)
    fully_define_sketch(m);extrude(m,14,'Coplanar_support60x60x14',reverse=True)
    set_document_appearance(m,'#FFFFFF')
    for r,row in enumerate(bits):
        start_sketch(m,'Front Plane')
        col=0
        while col<6:
            if row[col]: col+=1;continue
            first=col
            while col<6 and not row[col]:col+=1
            rectangle(m,-24+8*first,24-8*(r+1),-24+8*col,24-8*r)
        extrude(m,.04,'ID'+str(id)+'_row'+str(r),'#000000',merge=False)
    finish(m,path);print('Marker ready',id,flush=True)
if __name__=='__main__':
    for code,ch in zip('KABNRCPkabnrcp','帅仕相马车炮兵将士象马车炮卒'): disc(code,ch)
    board()
    for id in range(4):marker(id)
